#!/usr/bin/env python3
"""Build the bounded C20--C22 LM level-and-lift candidate from frozen 6761."""
import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / 'src/grasp_hexapod_control/scripts'
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / 'tools')]
from climb_mode import ClimbMode
from control import GraspController
from kinematics import JOINT_LOWER, JOINT_UPPER
from rebuild_climb_tail import original_tangents, set_world_boundary_velocity, shared_world_velocity, stage_pose, world_from_base
from utils.climb import gravity_projected_support
from utils.climb_retime import segment_for_time, speed_report, update_speed_report

BASE = ROOT / 'src/docs/evidence/climb_concurrency_20260908/candidate_config.json'
BASE_SHA256 = '67616b7056329c0f65723962befa5166c4d162c45ca0b85fc79aad516aed9edf'
DT = 1.0 / 30.0
LM_SUPPORT = (0, 1, 3, 4, 5)


def dock_mode_defaults():
    """Load the runtime DOCK constants only for the optional audit path."""
    from dock_mode import DockMode
    return DockMode


def pose_at(stage, time_s):
    return stage_pose(stage, time_s)


def world_active(stage, base_knots):
    times = np.r_[0.0, np.cumsum(stage['segment_durations_s'])]
    world = []
    for time_s, base in zip(times, base_knots):
        transform = world_from_base(pose_at(stage, float(time_s)))
        world.append([(transform @ np.r_[point, 1.0])[:3] for point in base])
    return np.asarray(world)


def rebuild_active_world(stage, fixed_anchor):
    base = np.asarray(stage['active_base_knots_m'], dtype=np.float64)
    knots = np.repeat(np.asarray(fixed_anchor, dtype=np.float64)[None, :, :], len(base), axis=0)
    values = world_active(stage, base)
    for index, leg in enumerate(stage['active_legs']):
        knots[:, leg] = values[:, index]
    stage['anchor_knots'] = knots.tolist()


def require(condition, detail):
    if not condition:
        raise AssertionError(detail)


def solve_exact(kinematic, q_seed, desired_base):
    q = np.asarray(q_seed, dtype=np.float64).copy()
    desired_hip = kinematic.base_to_hip(desired_base)
    for _ in range(1200):
        error = desired_hip - kinematic.forward(q)
        if float(np.max(np.linalg.norm(error, axis=1))) <= 5e-8:
            break
        inverse = kinematic.damped_inverse_jacobian(q, damping=.001)
        delta = (inverse @ error[..., None]).squeeze(-1)
        q = np.clip(q + np.clip(delta, -.02, .02), JOINT_LOWER, JOINT_UPPER)
    residual = np.linalg.norm(desired_base - kinematic.forward_base(q), axis=1)
    return q, residual


def reference(mode, index, time_s):
    mode.stage_index = index
    mode.phase_time = float(time_s)
    pose, anchors, _ = mode._stage_reference()
    inverse = np.linalg.inv(ClimbMode._world_from_base(pose))
    desired = (np.c_[anchors, np.ones(6)] @ inverse.T)[:, :3]
    return pose, anchors, desired


def build(base):
    data = copy.deepcopy(base)
    stages = data['stages']
    names = [stage['name'] for stage in stages]
    indices = [names.index(name) for name in ('BODY_PRELOAD_LM', 'LM_LIFT',
                                               'BODY_ADVANCE_LM_AIR', 'LM_LEFT_FINAL_LAND')]
    if indices != list(range(indices[0], indices[0] + 4)):
        raise ValueError('expected contiguous named LM chain')
    c20, c21, c22, c23 = (stages[index] for index in indices)

    original_c21 = copy.deepcopy(c21)
    original_c22 = copy.deepcopy(c22)
    c20['pose_end'] = [.239, c20['pose_end'][1], .226, .16, -.2]
    c20['segment_durations_s'] = [.5]
    # C20 has no active legs: preserve its original fixed world anchors.
    require(np.array_equal(np.asarray(c20['anchor_knots'][0]), np.asarray(c20['anchor_knots'][-1])),
            'C20 must retain all fixed anchors')

    c21['pose_start'] = copy.deepcopy(c20['pose_end'])
    c21['pose_end'] = [.239, c21['pose_end'][1], .226, 0.0, -.2]
    c21['pose_curve'] = 'quintic_first_segment'
    # Keep original mid/end base knots; derive only the start knot from its original world anchor.
    start_inverse = np.linalg.inv(world_from_base(c21['pose_start']))
    original_start_world = np.asarray(original_c21['anchor_knots'][0][2], dtype=np.float64)
    c21_base = np.asarray(original_c21['active_base_knots_m'], dtype=np.float64).copy()
    c21_base[0, 0] = (start_inverse @ np.r_[original_start_world, 1.0])[:3]
    c21['active_base_knots_m'] = c21_base.tolist()
    rebuild_active_world(c21, c20['anchor_knots'][-1])
    original_tangents(c21)

    c22['pose_start'] = copy.deepcopy(c21['pose_end'])
    # Original C22 end, base knots, duration, and C23 boundary remain frozen.
    require(c22['pose_end'] == original_c22['pose_end'], 'C22 end must remain frozen')
    require(c22['active_base_knots_m'] == original_c22['active_base_knots_m'],
            'C22 base knots must remain frozen')
    rebuild_active_world(c22, c21['anchor_knots'][-1])

    # Preserve the C22->C23 world continuity velocity exactly; rebuild C21->C22
    # with the existing .3 world-chord rule.
    c21_world = np.asarray(c21['anchor_knots'], dtype=np.float64)[:, 2]
    c22_world = np.asarray(c22['anchor_knots'], dtype=np.float64)[:, 2]
    shared = shared_world_velocity(c21, c22, c21_world[-1] - c21_world[-2],
                                   c22_world[-1] - c22_world[0])
    set_world_boundary_velocity(c21, -1, shared)
    set_world_boundary_velocity(c22, 0, shared)
    c21['continuous_air_transition'] = True
    c22['continuous_air_transition'] = True
    require(c22['active_base_velocities_m_s'][-1] == original_c22['active_base_velocities_m_s'][-1],
            'C22 terminal velocity changed')
    require(c23['active_base_velocities_m_s'][0] == base['stages'][names.index('LM_LEFT_FINAL_LAND')]['active_base_velocities_m_s'][0],
            'C23 initial velocity changed')

    # C23 onward exact source dictionaries and terminal are frozen.
    c23_index = names.index('LM_LEFT_FINAL_LAND')
    require(data['stages'][c23_index:] == base['stages'][c23_index:], 'C23+ changed')
    require(data['terminal_q_rad'] == base['terminal_q_rad'], 'terminal q changed')
    require(data['stage_count'] == base['stage_count'], 'stage count changed')
    return data


def local_report(data):
    controller = GraspController(DT)
    q = np.asarray(data['p0']['q_rad'], dtype=np.float64)
    controller.enter_climb(q, data)
    names = [stage['name'] for stage in data['stages']]
    start = names.index('BODY_PRELOAD_LM')
    # Continuous seed comes from the actual command chain through the unchanged prefix.
    while controller.climb_mode.state == ClimbMode.RUNNING and controller.climb_mode.stage_index < start:
        q = controller.update(q, np.zeros(4))
    require(controller.climb_mode.stage_index == start, 'prefix did not reach C20')
    mode = ClimbMode(None); mode.config = data
    min_joint = (np.inf, None)
    min_support = (np.inf, None)
    max_residual = (0.0, None)
    max_speed = (0.0, None)
    stage_speed = {}
    previous_q = q.copy()
    for index in range(start, start + 4):
        stage = data['stages'][index]
        stage_speed[index] = speed_report(index, stage)
        total = float(sum(stage['segment_durations_s']))
        times = np.arange(DT, total + 1e-12, DT)
        if not np.isclose(times[-1], total):
            times = np.r_[times, total]
        for time_s in times:
            pose, anchors, desired = reference(mode, index, min(float(time_s), total))
            q, residuals = solve_exact(controller.kinematic, previous_q, desired)
            residual = float(np.max(residuals)); leg = int(np.argmax(residuals))
            if residual > max_residual[0]: max_residual = (residual, (stage['name'], float(time_s), leg))
            margins = controller.kinematic.joint_limit_margins(q)
            margin = float(np.min(margins)); leg_joint = np.unravel_index(np.argmin(margins), margins.shape)
            if margin < min_joint[0]: min_joint = (margin, (stage['name'], float(time_s), int(leg_joint[0]), int(leg_joint[1])))
            if stage['name'] in ('LM_LIFT', 'BODY_ADVANCE_LM_AIR') and time_s > 0.0:
                transform = ClimbMode._world_from_base(pose)
                com = (transform @ np.r_[controller.kinematic.center_of_mass_base(q), 1.0])[:3]
                support = gravity_projected_support(com, anchors[list(LM_SUPPORT)], (0.,0.,-1.))
                require(support.valid, {'stage':stage['name'],'time_s':float(time_s),'metric':'support_polygon'})
                value = float(support.raw_margin_m)
                if value < min_support[0]: min_support = (value, (stage['name'], float(time_s)))
            speed = np.abs(q - previous_q) / DT
            peak = float(np.max(speed)); speed_leg, speed_joint = np.unravel_index(np.argmax(speed), speed.shape)
            if peak > max_speed[0]: max_speed=(peak,(stage['name'],float(time_s),int(speed_leg),int(speed_joint)))
            semantic = segment_for_time(index, stage, float(time_s))
            update_speed_report(stage_speed[index], semantic['segment_index'], peak,
                                {'stage':stage['name'],'time_s':float(time_s),'leg':int(speed_leg),'joint':int(speed_joint),'actual':peak,'threshold':semantic['hard_gate_rad_s']})
            previous_q=q.copy(); previous_time=float(time_s)
    speed_failures=[]
    for rows in stage_speed.values():
        for row in rows:
            if row['measured_peak_rad_s'] > row['hard_gate_rad_s'] + 1e-12:
                speed_failures.append(row)
    return {'local_names':[data['stages'][i]['name'] for i in range(start,start+4)],
            'min_joint_margin_rad':min_joint[0],'min_joint_source':min_joint[1],
            'min_lm_air_support_m':min_support[0],'min_lm_air_support_source':min_support[1],
            'max_ik_residual_m':max_residual[0],'max_ik_source':max_residual[1],
            'max_command_secant_rad_s':max_speed[0],'max_speed_source':max_speed[1],
            'speed_rows':list(stage_speed.values()),'speed_failures':speed_failures,
            'passes': {'joint_ge_08':min_joint[0]>=.08,'lm_support_ge_029':min_support[0]>=.029,
                       'ik_le_1e5':max_residual[0]<=1e-5,'speed':not speed_failures}}


def build_v2(base):
    """Retain v1 geometry and lengthen only LM_LIFT's leveling segment."""
    data = build(base)
    names = [stage['name'] for stage in data['stages']]
    c21 = data['stages'][names.index('LM_LIFT')]
    c22 = data['stages'][names.index('BODY_ADVANCE_LM_AIR')]
    c23 = data['stages'][names.index('LM_LEFT_FINAL_LAND')]
    before_geometry = copy.deepcopy(c21['anchor_knots'])
    c21['segment_durations_s'][0] = 1.2
    original_tangents(c21)
    require(c21['anchor_knots'] == before_geometry, 'v2 changed LM_LIFT world geometry')
    c21_world = np.asarray(c21['anchor_knots'], dtype=np.float64)[:, 2]
    c22_world = np.asarray(c22['anchor_knots'], dtype=np.float64)[:, 2]
    shared = shared_world_velocity(c21, c22, c21_world[-1] - c21_world[-2],
                                   c22_world[-1] - c22_world[0])
    set_world_boundary_velocity(c21, -1, shared)
    set_world_boundary_velocity(c22, 0, shared)
    require(c23 == base['stages'][names.index('LM_LEFT_FINAL_LAND')], 'v2 changed C23')
    return data


def dock_continuous_report(data):
    """Sample deterministic DockMode raise and one default 20 mm search circle."""
    dock_mode = dock_mode_defaults()
    controller = GraspController(DT); k = controller.kinematic
    q = np.asarray(data['terminal_q_rad'], dtype=np.float64); feet = k.forward_base(q)
    samples=[]
    # DockMode BODY_RAISE: feet lower in base at 50 mm/s, body rises in fixed support frame.
    raise_height = dock_mode.BODY_RAISE_HEIGHT_M
    radius = dock_mode.TAG_SEARCH_RADIUS_M
    search_speed = dock_mode.TAG_SEARCH_SPEED_M_S
    for z in np.arange(search_speed * DT, raise_height + 1e-12, search_speed * DT):
        samples.append(('body_raise', min(float(z), raise_height), np.zeros(2)))
    # Search first moves to the default -y point, then advances clockwise.
    offset=np.zeros(2); target=np.array([0., -radius]); step=search_speed*DT
    while np.linalg.norm(target-offset) > 1e-12:
        delta=target-offset; offset += delta * min(step, np.linalg.norm(delta)) / np.linalg.norm(delta)
        samples.append(('search_to_circle', .04, offset.copy()))
    angle=-.5*np.pi; arc=search_speed*DT/radius
    for _ in range(int(np.ceil(2*np.pi/arc))):
        angle -= arc; offset=radius*np.array([np.cos(angle),np.sin(angle)])
        samples.append(('search_circle', .04, offset.copy()))
    anchors=feet.copy(); q_seed=q.copy(); min_margin=(np.inf,None); min_support=(np.inf,None); max_res=(0.,None); max_axis=(0.,None)
    for index,(phase,z,xy) in enumerate(samples):
        target=feet.copy(); target[:,2] -= z; target[:,:2] -= xy
        q_seed,res=solve_exact(k,q_seed,target)
        margin=float(np.min(k.joint_limit_margins(q_seed)))
        residual=float(np.max(res)); axes=k.terminal_axes_base(q_seed)
        axis=float(np.degrees(np.max(np.arccos(np.clip(np.abs(axes[:,2]),-1.,1.)))))
        base_pose=np.eye(4); base_pose[:3,3]=[xy[0],xy[1],z]
        com=(base_pose @ np.r_[k.center_of_mass_base(q_seed),1.])[:3]
        support=gravity_projected_support(com,anchors,(0.,0.,-1.))
        require(support.valid, {'sample':index,'phase':phase,'metric':'six_foot_support'})
        item=(index,phase,float(z),xy.tolist())
        if margin<min_margin[0]: min_margin=(margin,item)
        if support.raw_margin_m<min_support[0]: min_support=(float(support.raw_margin_m),item)
        if residual>max_res[0]: max_res=(residual,item)
        if axis>max_axis[0]: max_axis=(axis,item)
    return {'sample_count':len(samples),'body_raise_height_m':raise_height,'search_radius_m':radius,'search_speed_m_s':search_speed,'scope':'30 Hz deterministic DockMode BODY_RAISE plus one default search circle; TF-driven prealign/descent excluded',
            'min_joint_margin_rad':min_margin[0],'min_joint_source':min_margin[1],
            'min_true_com_six_foot_support_m':min_support[0],'min_support_source':min_support[1],
            'max_ik_residual_m':max_res[0],'max_ik_source':max_res[1],
            'max_terminal_axis_tilt_from_base_z_deg':max_axis[0],'max_axis_source':max_axis[1],
            'evidence_boundary':'base-z axis tilt is kinematic only, not a support-surface or contact angle.'}


def dock_report(data):
    controller=GraspController(DT)
    q=np.asarray(data['terminal_q_rad'],dtype=np.float64)
    k=controller.kinematic
    feet=k.forward_base(q)
    # Deterministic default Dock actions before vision are BODY_RAISE then a 20mm XY search circle.
    targets=[]
    raised=feet.copy(); raised[:,2]-=.04
    targets.append(('body_raise_z_minus_40mm',raised))
    for x,y in ((.02,0.),(-.02,0.),(0.,.02),(0.,-.02)):
        points=raised.copy(); points[:,:2] -= np.array([x,y])
        targets.append(('search_radius_20mm_{:+.0f}_{:+.0f}mm'.format(x*1000,y*1000),points))
    min_margin=(np.inf,None); max_residual=(0.,None); max_axis=(0.,None); q_seed=q.copy(); rows=[]
    for name,target in targets:
        q_seed,res=solve_exact(k,q_seed,target)
        margin=float(np.min(k.joint_limit_margins(q_seed)))
        residual=float(np.max(res)); axes=k.terminal_axes_base(q_seed)
        angle=float(np.degrees(np.max(np.arccos(np.clip(np.abs(axes[:,2]),-1.,1.)))))
        rows.append({'action':name,'min_joint_margin_rad':margin,'max_ik_residual_m':residual,'max_terminal_axis_deviation_from_base_z_deg':angle})
        if margin<min_margin[0]:min_margin=(margin,name)
        if residual>max_residual[0]:max_residual=(residual,name)
        if angle>max_axis[0]:max_axis=(angle,name)
    return {'scope':'terminal FK then deterministic BODY_RAISE and 20mm XY tag-search radius; visual prealign/descent depend on future TF and are not bounded here',
            'terminal_joint_margin_rad':float(np.min(k.joint_limit_margins(q))),
            'min_joint_margin_rad':min_margin[0],'min_joint_margin_action':min_margin[1],
            'max_ik_residual_m':max_residual[0],'max_ik_action':max_residual[1],
            'max_terminal_axis_deviation_from_base_z_deg':max_axis[0],'max_axis_action':max_axis[1],
            'rows':rows,
            'historical_lm_anchor_dependency':'none: DockMode uses current joints/FK and does not read climb anchors; reachability is driven by base-relative foot targets.'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, default=BASE)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    raw_source = args.source.read_bytes()
    require(hashlib.sha256(raw_source).hexdigest() == BASE_SHA256,
            'source must be frozen 6761 evidence')
    base = json.loads(raw_source)
    candidate = build_v2(base)
    ClimbMode(None)._validate_config(candidate)
    raw = json.dumps(candidate, indent=2) + "\n"
    args.output.write_text(raw)
    if args.report:
        report = {'baseline_sha256': BASE_SHA256,
                  'candidate_sha256': hashlib.sha256(raw.encode()).hexdigest(),
                  'stage_count': candidate['stage_count'],
                  'unchanged_c23_onward': candidate['stages'][22:] == base['stages'][22:],
                  'local_30hz': local_report(candidate),
                  'dock_default_actions': dock_report(candidate),
                  'dock_continuous': dock_continuous_report(candidate),
                  'evidence_boundary': 'CPU kinematics/support diagnostics only; no contact, load, friction, collision-continuity, hardware, or GPU evidence.'}
        args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


if __name__ == '__main__':
    main()
