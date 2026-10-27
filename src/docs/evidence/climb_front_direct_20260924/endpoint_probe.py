#!/usr/bin/env python3
"""Static inherited-q endpoint preflight for the front direct-climb hypotheses."""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / 'src/grasp_hexapod_control/scripts'
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / 'tools')]
from build_climb_front import (  # noqa: E402
    P0, base_points, load_triangles, solve_ik, support_margin, surface_height,
    world_from_base,
)
from kinematics import GraspKinematic, JOINT_LOWER, JOINT_UPPER, Q_STAND  # noqa: E402

CONFIG = ROOT / 'src/grasp_hexapod_control/config/climb_front_fast.json'
OUT = Path(__file__).with_name('endpoint_probe.json')
LEGS = ('LB', 'LF', 'LM', 'RB', 'RF', 'RM')
BACK, FRONT, MID = (0, 3), (1, 4), (2, 5)
GROUND_Z = .0065
SUPPORT_MIN_M = .010
IK_MAX_M = .004
YAW = -np.pi / 2.0

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def cached_z(config, leg, x, y, preferred=None):
    stages = config['stages']
    if preferred:
        stages = [next(s for s in stages if s['name'] == preferred)] + stages
    for stage in stages:
        for knot in stage['anchor_knots']:
            point = knot[leg]
            if abs(point[0] - x) < 1e-8 and abs(point[1] - y) < 1e-8:
                return float(point[2])
    triangles = load_triangles(ROOT / 'src/grasp_hexapod_description/meshes/xiaolan/base_link_xiaolan.STL')
    return float(surface_height(triangles, x, y))


def main():
    cfg_bytes = CONFIG.read_bytes()
    config = json.loads(cfg_bytes)
    by_name = {s['name']: s for s in config['stages']}
    kin = GraspKinematic()
    report = {
        'source_config': str(CONFIG.relative_to(ROOT)),
        'source_config_sha256': hashlib.sha256(cfg_bytes).hexdigest(),
        'method': 'Sequential static endpoint DLS probes seeded from the previous endpoint. No trajectory, controller playback, STL sweep, or GPU claim.',
        'stages': [],
        'stopped_on_first_failure': None,
    }
    q = Q_STAND.copy()
    pose = np.asarray(P0, dtype=float)
    anchors = np.asarray(by_name['PREP']['anchor_knots'][0], dtype=float)

    def probe(label, target_pose, target_anchors, active, source):
        nonlocal q, pose, anchors
        target_pose = np.asarray(target_pose, dtype=float)
        target_anchors = np.asarray(target_anchors, dtype=float)
        active = tuple(active)
        support = [leg for leg in range(6) if leg not in active]
        entry_com = (world_from_base(pose) @ np.r_[kin.center_of_mass_base(q), 1.0])[:3]
        entry_margin = support_margin(entry_com[:2], anchors[support])
        q0, pose0, anchors0 = q.copy(), pose.copy(), anchors.copy()
        q1, ik_residual = solve_ik(kin, target_pose, target_anchors, q0, tolerance_m=1e-5)
        fk_base = kin.hip_to_base(kin.forward(q1))
        target_base = base_points(target_pose, target_anchors)
        foot_errors = np.linalg.norm(fk_base - target_base, axis=1)
        per_leg_margin = np.min(np.minimum(q1 - JOINT_LOWER, JOINT_UPPER - q1), axis=1)
        exit_com = (world_from_base(target_pose) @ np.r_[kin.center_of_mass_base(q1), 1.0])[:3]
        exit_margin = support_margin(exit_com[:2], target_anchors[support])
        row = {
            'state': label, 'source_reference': source,
            'active_legs': [LEGS[i] for i in active],
            'body_pose_start': pose0.tolist(), 'body_pose_target': target_pose.tolist(),
            'entry_q_rad': q0.tolist(), 'exit_q_rad': q1.tolist(),
            'entry_support_legs': [LEGS[i] for i in support],
            'entry_support_com_margin_m': float(entry_margin),
            'exit_support_com_margin_m': float(exit_margin),
            'world_anchors_start_m': anchors0.tolist(),
            'world_anchors_target_m': target_anchors.tolist(),
            'ik_max_foot_residual_m': float(ik_residual),
            'per_leg_foot_residual_m': dict(zip(LEGS, foot_errors.tolist())),
            'per_leg_joint_margin_rad': dict(zip(LEGS, per_leg_margin.tolist())),
            'fixed_thigh_delta_rad_diagnostic_only': {
                LEGS[i]: float(q1[i, 0] - q0[i, 0]) for i in support
            },
        }
        reason = None
        if ik_residual > IK_MAX_M:
            reason = 'ik_residual_gt_4mm'
        elif float(np.min(per_leg_margin)) <= 0.0:
            reason = 'joint_margin_nonpositive'
        elif entry_margin < SUPPORT_MIN_M:
            reason = 'entry_support_com_below_10mm'
        elif exit_margin < SUPPORT_MIN_M:
            reason = 'exit_support_com_below_10mm'
        row['pass'] = reason is None
        row['failure_reason'] = reason
        report['stages'].append(row)
        if reason:
            report['stopped_on_first_failure'] = {'state': label, 'reason': reason}
            return False
        q, pose, anchors = q1, target_pose, target_anchors
        return True

    # First three endpoints are inherited unchanged from the old reference.
    for name in ('PREP', 'FRONT_LOW', 'BODY_A'):
        stage = by_name[name]
        if not probe(name, stage['pose_end'], stage['anchor_knots'][-1],
                     stage['active_legs'], 'current fast config unchanged prefix endpoint'):
            break
    if report['stopped_on_first_failure'] is None:
        # First M landing changes from +.04 m to -.02 m; body and all other feet stay fixed.
        stage = by_name['MID_GROUND_SHIFT']
        target = np.asarray(stage['anchor_knots'][-1], dtype=float)
        target[list(MID), 0] = -.02
        target[list(MID), 2] = GROUND_Z
        probe('M_GROUND_FIRST_MINUS02', stage['pose_end'], target, MID,
              'MID_GROUND_SHIFT endpoint with M x=-.02 m')
    if report['stopped_on_first_failure'] is None:
        stage = by_name['BODY_B_FRONT_INWARD']
        target = np.asarray(stage['anchor_knots'][-1], dtype=float)
        target[list(MID)] = anchors[list(MID)]
        probe('FRONT_TO_029', [0., -.067694497636, .15, .25, 0., YAW], target, FRONT,
              'BODY_B_FRONT_INWARD F=.29 endpoint; M inherited at -.02')
    if report['stopped_on_first_failure'] is None:
        target = anchors.copy(); target[list(BACK), 0] = -.09; target[list(BACK), 2] = GROUND_Z
        probe('BACK_TO_MINUS09', [.029, -.067694497636, .15, .15, 0., YAW], target, BACK,
              'proposed B=-.09 m with synchronized body x=.029 m')
    if report['stopped_on_first_failure'] is None:
        target = anchors.copy(); target[list(MID), 0] = .04; target[list(MID), 2] = GROUND_Z
        probe('MID_TO_040', [.059, -.067694497636, .15, .15, 0., YAW], target, MID,
              'proposed M=.04 m with synchronized body x=.059 m')
    if report['stopped_on_first_failure'] is None:
        target = anchors.copy()
        for leg in FRONT:
            target[leg, 0] = .34
            target[leg, 2] = cached_z(config, leg, .34, target[leg, 1], 'G_MID_BACK_SYNC')
        probe('FRONT_TO_034', pose, target, FRONT,
              'proposed F=.34 m at body x=.059 m; cached old STL anchor height')
    if report['stopped_on_first_failure'] is None:
        target = anchors.copy(); target[list(BACK), 0] = 0.; target[list(BACK), 2] = GROUND_Z
        probe('BACK_TO_000', [.081, -.067694497636, .15, .15, 0., YAW], target, BACK,
              'authorized first B endpoint x=0 m with body x=.081 m')
    if report['stopped_on_first_failure'] is None:
        target = anchors.copy(); target[list(MID), 0] = .08; target[list(MID), 2] = GROUND_Z
        probe('MID_TO_080', [.1017, -.067694497636, .155, .15, 0., YAW], target, MID,
              'proposed M=.08 m and body x=.1017,z=.155 m')
    if report['stopped_on_first_failure'] is None:
        target = anchors.copy()
        for leg in MID:
            target[leg, 0] = .19
            target[leg, 2] = cached_z(config, leg, .19, target[leg, 1], 'Q_MID_LOW')
        probe('Q_MID_LOW', pose, target, MID,
              'proposed M=.19 m low-surface landing at same body pose')

    OUT.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({
        'output': str(OUT), 'source_config_sha256': report['source_config_sha256'],
        'states_checked': len(report['stages']),
        'stopped_on_first_failure': report['stopped_on_first_failure'],
        'last_state': report['stages'][-1]['state'] if report['stages'] else None,
        'last_failure_detail': report['stages'][-1] if report['stages'] and not report['stages'][-1]['pass'] else None,
    }, indent=2))


if __name__ == '__main__':
    main()
