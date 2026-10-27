#!/usr/bin/env python3
"""Build the bounded 33-stage post-deck candidate from the active baseline."""
import argparse
import copy
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / 'src/grasp_hexapod_control/scripts'
sys.path.insert(0, str(SCRIPTS))

from climb_mode import ClimbMode
from control import GraspController
from kinematics import JOINT_LOWER, JOINT_UPPER


TANGENT_FRACTION = .4


def world_from_base(pose):
    x, y, z, roll, pitch = map(float, pose)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    transform = np.eye(4)
    transform[:3, :3] = [[cp, sp * sr, sp * cr], [0, cr, -sr],
                         [-sp, cp * sr, cp * cr]]
    transform[:3, 3] = [x, y, z]
    return transform


def active_base(stage, pose):
    inverse = np.linalg.inv(world_from_base(pose))
    return [[(inverse @ np.r_[knot[leg], 1.0])[:3].tolist()
             for leg in stage['active_legs']]
            for knot in stage['anchor_knots']]


def pair(template, start, legs, sources, name):
    item = copy.deepcopy(template)
    item['name'] = name
    item['active_legs'] = list(legs)
    item['pose_start'] = copy.deepcopy(start['pose_end'])
    item['pose_end'] = copy.deepcopy(start['pose_end'])
    item['anchor_knots'] = [copy.deepcopy(start['anchor_knots'][-1])
                            for _ in range(4)]
    item['segment_durations_s'] = [max(a, b) for a, b in zip(
        sources[0]['segment_durations_s'], sources[1]['segment_durations_s'])]
    for knot_index in range(1, 4):
        for leg, source in zip(legs, sources):
            item['anchor_knots'][knot_index][leg] = copy.deepcopy(
                source['anchor_knots'][knot_index][leg])
    item['active_base_knots_m'] = active_base(item, item['pose_start'])
    return item


def moving_pair_base_knots(stage):
    """Express fixed-world pair knots in the synchronized body poses."""
    total = float(sum(stage['segment_durations_s']))
    knot_times = np.r_[0.0, np.cumsum(stage['segment_durations_s'])]
    output = []
    for knot, time_s in zip(stage['anchor_knots'], knot_times):
        phase = time_s / total
        weight = 6.0 * phase**5 - 15.0 * phase**4 + 10.0 * phase**3
        pose = ((1.0 - weight) * np.asarray(stage['pose_start'], float)
                + weight * np.asarray(stage['pose_end'], float))
        inverse = np.linalg.inv(world_from_base(pose))
        output.append([(
            inverse @ np.r_[np.asarray(knot[leg], float), 1.0]
        )[:3].tolist() for leg in stage['active_legs']])
    return output


def stage_pose(stage, time_s):
    """Return the exact quintic body pose at one stage-local time."""
    total = float(sum(stage['segment_durations_s']))
    if stage['pose_curve'] == 'quintic_first_segment':
        phase = time_s / float(stage['segment_durations_s'][0])
    else:
        phase = time_s / total
    phase = float(np.clip(phase, 0.0, 1.0))
    weight = 6.0 * phase**5 - 15.0 * phase**4 + 10.0 * phase**3
    return ((1.0 - weight) * np.asarray(stage['pose_start'], float)
            + weight * np.asarray(stage['pose_end'], float))


def sync_internal_active_world_knots(stage):
    """Project changed interior base knots while retaining exact endpoints."""
    times = np.r_[0.0, np.cumsum(stage['segment_durations_s'])]
    active = stage['active_legs']
    base_knots = np.asarray(stage['active_base_knots_m'], dtype=np.float64)
    for knot_index in range(1, len(times) - 1):
        world = (
            np.column_stack((base_knots[knot_index], np.ones(len(active))))
            @ world_from_base(stage_pose(stage, float(times[knot_index]))).T
        )[:, :3]
        for local_index, leg in enumerate(active):
            stage['anchor_knots'][knot_index][leg] = world[local_index].tolist()


def original_tangents(stage):
    """Reproduce the original .4-times-slower unit-bisector swing tangents."""
    knots = np.asarray(stage['active_base_knots_m'], dtype=np.float64)
    durations = np.asarray(stage['segment_durations_s'], dtype=np.float64)
    velocities = np.zeros_like(knots)
    segment_speeds = (knots[1:] - knots[:-1]) / durations[:, None, None]
    for index in range(1, len(knots) - 1):
        incoming, outgoing = segment_speeds[index - 1], segment_speeds[index]
        magnitude = TANGENT_FRACTION * np.minimum(
            np.linalg.norm(incoming, axis=1), np.linalg.norm(outgoing, axis=1)
        )
        direction = (
            incoming / np.maximum(np.linalg.norm(incoming, axis=1)[:, None], 1e-12)
            + outgoing / np.maximum(np.linalg.norm(outgoing, axis=1)[:, None], 1e-12)
        )
        velocities[index] = (
            direction / np.maximum(np.linalg.norm(direction, axis=1)[:, None], 1e-12)
            * magnitude[:, None]
        )
    stage['active_base_velocities_m_s'] = velocities.tolist()


def continuous_tangents(stage):
    """Recompute the corrected tail's endpoint-biased .4 swing tangents."""
    knots = np.asarray(stage['active_base_knots_m'], dtype=np.float64)
    durations = np.asarray(stage['segment_durations_s'], dtype=np.float64)
    velocities = np.zeros_like(knots)
    segment_speeds = (knots[1:] - knots[:-1]) / durations[:, None, None]
    for index in range(1, len(knots) - 1):
        incoming, outgoing = segment_speeds[index - 1], segment_speeds[index]
        magnitude = TANGENT_FRACTION * np.minimum(
            np.linalg.norm(incoming, axis=1), np.linalg.norm(outgoing, axis=1)
        )
        direction = (incoming if index == 1 else outgoing
                     if index == len(knots) - 2 else
                     incoming / np.maximum(np.linalg.norm(incoming, axis=1)[:, None], 1e-12)
                     + outgoing / np.maximum(np.linalg.norm(outgoing, axis=1)[:, None], 1e-12))
        velocities[index] = (
            direction / np.maximum(np.linalg.norm(direction, axis=1)[:, None], 1e-12)
            * magnitude[:, None]
        )
    stage['active_base_velocities_m_s'] = velocities.tolist()


def lower_air_base_knots(stage, knot_indices, recompute=True):
    """Lower selected active air knots and regenerate their local curve."""
    for knot_index in knot_indices:
        for local_index in range(len(stage['active_legs'])):
            stage['active_base_knots_m'][knot_index][local_index][2] -= .005
    sync_internal_active_world_knots(stage)
    if recompute:
        continuous_tangents(stage)


def set_world_boundary_velocity(stage, endpoint, world_velocity):
    """Store one world-frame boundary velocity in its local base frame."""
    pose_key = 'pose_start' if endpoint == 0 else 'pose_end'
    velocity_index = 0 if endpoint == 0 else -1
    stage['active_base_velocities_m_s'][velocity_index][0] = (
        world_from_base(stage[pose_key])[:3, :3].T
        @ np.asarray(world_velocity, dtype=np.float64)
    ).tolist()


def shared_world_velocity(previous, following, previous_chord, following_chord):
    """Build the frozen .3-times-slower shared air velocity."""
    previous_velocity = previous_chord / float(previous['segment_durations_s'][-1])
    following_velocity = following_chord / float(following['segment_durations_s'][0])
    direction = (previous_velocity / np.linalg.norm(previous_velocity)
                 + following_velocity / np.linalg.norm(following_velocity))
    direction /= np.linalg.norm(direction)
    return direction * (.3 * min(np.linalg.norm(previous_velocity),
                                  np.linalg.norm(following_velocity)))


def continuous_lm_air_boundaries(stages):
    """Carry LM's air velocity through C24/C25 and into the C26 descent."""
    c24, c25, c26 = stages[23], stages[24], stages[25]
    if (c24['name'], c25['name'], c26['name']) != (
            'LM_LIFT', 'BODY_ADVANCE_LM_AIR', 'LM_LEFT_FINAL_LAND'):
        raise ValueError('continuous swings requires frozen LM C24--C26')
    for stage in (c24, c25, c26):
        if stage['active_legs'] != [2]:
            raise ValueError('continuous swings requires LM-only air boundary')
        if 'active_base_velocities_m_s' not in stage:
            stage['active_base_velocities_m_s'] = np.zeros_like(
                np.asarray(stage['active_base_knots_m'], dtype=np.float64)
            ).tolist()
    c24_knots = np.asarray(c24['anchor_knots'], dtype=np.float64)[:, 2]
    c25_knots = np.asarray(c25['anchor_knots'], dtype=np.float64)[:, 2]
    c26_knots = np.asarray(c26['anchor_knots'], dtype=np.float64)[:, 2]
    first = shared_world_velocity(
        c24, c25, c24_knots[-1] - c24_knots[-2],
        c25_knots[-1] - c25_knots[0],
    )
    second_chord = c26_knots[-1] - c26_knots[0]
    second = second_chord / np.linalg.norm(second_chord) * (.3 * min(
        np.linalg.norm(c25_knots[-1] - c25_knots[0])
        / float(c25['segment_durations_s'][0]),
        np.linalg.norm(second_chord) / float(c26['segment_durations_s'][0]),
    ))
    set_world_boundary_velocity(c24, -1, first)
    set_world_boundary_velocity(c25, 0, first)
    set_world_boundary_velocity(c25, -1, second)
    set_world_boundary_velocity(c26, 0, second)
    c24['continuous_air_transition'] = True
    c25['continuous_air_transition'] = True


def continuous_swings(data):
    """Apply optional continuous-swing geometry to one flow32 candidate."""
    stages = data['stages']
    c2 = stages[1]
    if c2['name'] != 'RM':
        raise ValueError('continuous swings requires C2 RM')
    c2['segment_durations_s'][1] = 2.10
    for stage in stages:
        if (stage.get('anchor_curve') == 'piecewise_base_quintic'
                and len(stage['segment_durations_s']) > 1):
            original_tangents(stage)

    lower_air_base_knots(c2, (2,), recompute=False)

    c8 = stages[7]
    if c8['name'] != 'RM_HIGH_C':
        raise ValueError('continuous swings requires C8 RM_HIGH_C')
    lower_air_base_knots(c8, (1, 2), recompute=False)

    for name in ('RM_DIRECT_FINAL', 'LB_LF_DIRECT_FINAL'):
        stage = next((item for item in stages if item['name'] == name), None)
        if stage is None:
            raise ValueError('continuous swings missing ' + name)
        lower_air_base_knots(stage, (1, 2))

    continuous_lm_air_boundaries(stages)
    data['terminal_q_rad'] = continuous_terminal_q(data)
    return data


def solve_exact(kinematic, q_seed, desired_base):
    """Follow the continuous DLS branch to one exact base-foot reference."""
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
    if float(np.max(residual)) > 5e-8:
        raise ValueError('tail builder terminal IK did not converge')
    return q


def continuous_terminal_q(compact):
    """Recompute the stored terminal posture after any final-anchor change."""
    controller = GraspController(1.0 / 30.0)
    mode = ClimbMode(None)
    mode.config = compact
    q = np.asarray(compact['p0']['q_rad'], dtype=np.float64)
    for index, stage in enumerate(compact['stages']):
        duration = float(sum(stage['segment_durations_s']))
        for time_s in np.linspace(0.0, duration, 18)[1:]:
            mode.stage_index = index
            mode.phase_time = float(time_s)
            pose, anchors, _ = mode._stage_reference()
            inverse = np.linalg.inv(ClimbMode._world_from_base(pose))
            desired = (np.column_stack((anchors, np.ones(6))) @ inverse.T)[:, :3]
            q = solve_exact(controller.kinematic, q, desired)
    return q.tolist()


def build(source):
    """Remove the post-deck LM refoot from one exact 33-stage source."""
    names = [stage.get('name') for stage in source.get('stages', ())]
    required = ['BODY_ADVANCE_LM_AIR', 'LM_LEFT_FINAL_LAND',
                'RB_RF_DIRECT_FINAL', 'RM_DIRECT_FINAL', 'BODY_REPOSITION',
                'LM_DIRECT_FINAL', 'LB_LF_DIRECT_FINAL', 'BODY_DOCK_FINAL',
                'STAND_FINAL_HOLD']
    if source.get('stage_count') != 33 or names[24:] != required:
        raise ValueError('flow builder requires the frozen 33-stage source exactly once')
    data = copy.deepcopy(source)
    old = data['stages']
    lm_target = np.asarray(old[25]['anchor_knots'][-1][2], float)
    stages = copy.deepcopy(old[:24])
    c25 = copy.deepcopy(old[24])
    inverse = np.linalg.inv(world_from_base(c25['pose_end']))
    base_target = (inverse @ np.r_[lm_target, 1.0])[:3]
    base_target[2] += .030
    c25['anchor_knots'][-1][2] = (world_from_base(c25['pose_end']) @ np.r_[base_target, 1.0])[:3].tolist()
    c25['active_base_knots_m'][-1][0] = base_target.tolist()
    stages.append(c25)
    c26 = copy.deepcopy(old[25])
    c26['anchor_knots'] = [copy.deepcopy(c25['anchor_knots'][-1]), copy.deepcopy(c25['anchor_knots'][-1])]
    c26['anchor_knots'][-1][2] = lm_target.tolist()
    c26['active_base_knots_m'] = active_base(c26, c26['pose_start'])
    c26['segment_durations_s'] = [.8]
    stages.append(c26)
    # Remove old C30 LM_DIRECT_FINAL and propagate C26's bearing support.
    for source_stage in old[26:29] + old[30:]:
        stage = copy.deepcopy(source_stage)
        for knot in stage['anchor_knots']:
            knot[2] = lm_target.tolist()
        stages.append(stage)
    for stage in stages:
        if not stage['active_legs'] and stage['name'] != 'STAND_FINAL_HOLD' or stage['name'] in ('LM_LIFT', 'BODY_ADVANCE_LM_AIR'):
            stage['settle_s'] = 1.0 / 30.0
            stage['settle_persistence_s'] = 1.0 / 30.0
    data['stages'] = stages
    data['stage_count'] = len(stages)
    data['terminal_q_rad'] = continuous_terminal_q(data)
    data['visual_validation_deferred_for_sim_finish'] = [stage['name'] for stage in stages]
    return data

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--continuous-swings', action='store_true')
    args = parser.parse_args()
    data = build(json.loads(args.input.read_text()))
    if args.continuous_swings:
        data = continuous_swings(data)
    args.output.write_text(json.dumps(data, indent=2) + '\n')


if __name__ == '__main__': main()
