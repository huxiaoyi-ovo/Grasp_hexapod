#!/usr/bin/env python3
"""Build the frozen 19-stage simulation-only front-space optimisation v2.

execution_owner: Terra.  This is an offline candidate generator; it is not
contact, load, stability, or hardware evidence.
"""
import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / "src/grasp_hexapod_control/scripts"
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / "tools")]
from build_climb_front import (  # noqa: E402
    MESH, TRANSLATION, base_points, load_triangles, solve_ik, world_from_base,
)
from kinematics import GraspKinematic, JOINT_LOWER, JOINT_UPPER  # noqa: E402
from refine_climb_front_space import target_on_surface  # noqa: E402

SOURCE = ROOT / "src/grasp_hexapod_control/config/climb_front_space.json"
OUTPUT = ROOT / "src/grasp_hexapod_control/config/climb_front_optimized.json"
SOURCE_SHA = "321a1b762d99659be988defa5bfdc27ae9672fc215bb351db9260a8cfd9bc333"
CENTER_Y = -.063
FRONT_BACK_DY = .004694497636


def _stage(name, pose_start, pose_end, anchors, active, durations, pose_knots=None):
    """Create one world-anchor piecewise stage without base-relative fields."""
    anchors = [np.asarray(k, dtype=float).tolist() for k in anchors]
    data = {
        "name": name,
        "pose_start": np.asarray(pose_start, dtype=float).tolist(),
        "pose_end": np.asarray(pose_end, dtype=float).tolist(),
        "active_legs": list(active),
        "anchor_knots": anchors,
        "segment_durations_s": list(durations),
        "anchor_curve": "piecewise_quintic",
        "pose_curve": "piecewise_quintic" if pose_knots is not None else "quintic_full_stage",
        "settle_s": .10,
    }
    if pose_knots is not None:
        data["pose_knots"] = [np.asarray(p, dtype=float).tolist() for p in pose_knots]
    return data


def _move(name, pose, end_pose, anchors, active, target, air_knots, durations, pose_knots=None):
    """Append a four-knot world-foot move, preserving every inactive anchor."""
    end = anchors.copy()
    end[active] = target
    knots = [anchors.copy(), anchors.copy(), anchors.copy(), end.copy()]
    knots[1][active] = air_knots[0]
    knots[2][active] = air_knots[1]
    return _stage(name, pose, end_pose, knots, active, durations, pose_knots), end


def _base_from_world(pose, points_world):
    """Convert selected world-foot points to the moving base frame."""
    return base_points(np.asarray(pose, dtype=float), np.asarray(points_world, dtype=float))


def _hip_to_base(kinematic, leg, point_hip):
    point = np.r_[np.asarray(point_hip, dtype=float), 1.0]
    return (kinematic.base_from_hip[leg] @ point)[:3]


def _base_to_hip(kinematic, leg, point_base):
    point = np.r_[np.asarray(point_base, dtype=float), 1.0]
    return (kinematic.hip_from_base[leg] @ point)[:3]


def _world_from_active_base(pose, points_base):
    homogeneous = np.column_stack((np.asarray(points_base, dtype=float), np.ones(len(points_base))))
    return (homogeneous @ world_from_base(np.asarray(pose, dtype=float)).T)[:, :3]


def _base_stage(name, poses, fixed_anchors, active, active_base, durations, velocities):
    """Store active body-relative knots while retaining fixed world supports."""
    poses = np.asarray(poses, dtype=float)
    fixed_anchors = np.asarray(fixed_anchors, dtype=float)
    active_base = np.asarray(active_base, dtype=float)
    anchors = []
    for pose, points_base in zip(poses, active_base):
        knot = fixed_anchors.copy()
        knot[active] = _world_from_active_base(pose, points_base)
        anchors.append(knot)
    stage = _stage(
        name, poses[0], poses[-1], anchors, active, durations, poses,
    )
    stage["anchor_curve"] = "piecewise_base_quintic"
    stage["active_base_knots_m"] = active_base.tolist()
    stage["active_base_velocities_m_s"] = np.asarray(velocities, dtype=float).tolist()
    return stage, np.asarray(anchors[-1], dtype=float)


def _capped_internal_velocities(knots, durations, internal_indices, cap_m_s=.040):
    """Central finite-difference base tangents, capped per active foot."""
    knots = np.asarray(knots, dtype=float)
    times = np.r_[0.0, np.cumsum(np.asarray(durations, dtype=float))]
    velocities = np.zeros_like(knots)
    for index in internal_indices:
        velocity = (knots[index + 1] - knots[index - 1]) / (times[index + 1] - times[index - 1])
        norm = np.linalg.norm(velocity, axis=1, keepdims=True)
        velocities[index] = velocity * np.minimum(1.0, cap_m_s / np.maximum(norm, 1e-15))
    return velocities


def build(source):
    actual_sha = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    if actual_sha != SOURCE_SHA:
        raise ValueError(f"source SHA mismatch: {actual_sha}")
    original = source["stages"]
    if len(original) != 19:
        raise ValueError("front-space source must retain 19 stages")
    cfg = copy.deepcopy(source)
    stages = [copy.deepcopy(original[0]), copy.deepcopy(original[1])]

    # new3: source C3+C4, world lift/cross/land for the middle pair.
    pose = np.asarray(original[2]["pose_start"], dtype=float)
    end_pose = np.asarray(original[3]["pose_end"], dtype=float)
    anchors = np.asarray(original[2]["anchor_knots"][0], dtype=float)
    end = np.asarray(original[3]["anchor_knots"][-1], dtype=float)
    active = [2, 5]
    apex = max(anchors[active, 2].max(), end[active, 2].max()) + .020
    lift, cross = anchors.copy(), end.copy()
    lift[active, 2] = apex
    cross[active, 2] = apex
    stages.append(_stage("BODY_MID_GROUND", pose, end_pose, [anchors, lift, cross, end], active, [.55, .85, .40]))
    pose, anchors = end_pose, end

    # new4..10 retain source C5..C11 as exact dictionaries.
    stages.extend(copy.deepcopy(original[4:12]))
    # new11: restore c1e's successful body poses but describe the two active
    # feet in the moving hip/base frame.  Fixed supports remain world anchors.
    mid_source = original[11]
    mid_poses = np.asarray(mid_source["pose_knots"], dtype=float)
    mid_active = [2, 5]
    mid_fixed = np.asarray(mid_source["anchor_knots"][0], dtype=float)
    mid_final = np.asarray(mid_source["anchor_knots"][-1], dtype=float)[mid_active]
    mid_base = np.empty((6, len(mid_active), 3), dtype=float)
    mid_base[0] = _base_from_world(mid_poses[0], mid_fixed[mid_active])
    high_hip = np.array([.155, 0.0, -.010])
    mid_base[1] = np.asarray([_hip_to_base(GraspKinematic(), leg, high_hip) for leg in mid_active])
    mid_base[2] = mid_base[1].copy(); mid_base[2, :, 1] += .035
    mid_base[3] = mid_base[1].copy(); mid_base[3, :, 1] += .090
    mid_base[4] = _base_from_world(mid_poses[4], mid_final + np.array([0.0, 0.0, .025]))
    mid_base[5] = _base_from_world(mid_poses[5], mid_final)
    mid_durations = [1.20, .75, .50, .70, .40]
    mid_velocities = np.zeros_like(mid_base)
    mid_velocities[1:4, :, 1] = .040
    mid_platform, anchors = _base_stage(
        "MID_PLATFORM_285", mid_poses, mid_fixed, mid_active, mid_base,
        mid_durations, mid_velocities,
    )
    stages[-1] = mid_platform
    pose = mid_poses[-1].copy()
    final_source_anchors = np.asarray(original[-1]["anchor_knots"][-1], dtype=float)

    # new12 A: direct front landing, feet lift before body moves.
    end_pose = np.asarray(original[13]["pose_end"], dtype=float)
    active = [1, 4]
    target = final_source_anchors[active].copy()
    apex = max(anchors[active, 2].max(), target[:, 2].max()) + .020
    air_a, air_b = anchors[active].copy(), target.copy()
    air_a[:, 2] = apex; air_b[:, 2] = apex
    stage, anchors = _move("FRONT_DIRECT_FINAL_480", pose, end_pose, anchors, active, target, (air_a, air_b), [.5, .85, .4], [pose, pose, end_pose, end_pose])
    stages.append(stage); pose = end_pose

    # new13 B1: left rear ground advance first.
    end_pose = np.asarray(original[14]["pose_end"], dtype=float)
    active = [0]; target = anchors[active].copy(); target[0, 0] = .155; target[0, 2] = .0065
    apex = max(anchors[active, 2].max(), target[:, 2].max()) + .020
    air_a, air_b = anchors[active].copy(), target.copy(); air_a[:, 2] = apex; air_b[:, 2] = apex
    stage, anchors = _move("LB_GROUND_TO_155", pose, end_pose, anchors, active, target, (air_a, air_b), [.5, .7, .4])
    stages.append(stage); pose = end_pose

    # new14 B2: high RB swing is body/hip-relative through the terminal-polar
    # arc, avoiding the previous world-z knot fold.
    end_pose = np.array([.290, original[16]["pose_end"][1], .243, .24, -.25, original[16]["pose_end"][5]], dtype=float)
    active = [3]
    rb_final = final_source_anchors[active]
    rb_poses = np.asarray([pose, end_pose, end_pose, end_pose, end_pose, end_pose, end_pose])
    terminal_base = _base_from_world(end_pose, rb_final)[0]
    terminal_hip = _base_to_hip(GraspKinematic(), active[0], terminal_base)
    terminal_rho = np.linalg.norm(terminal_hip[:2])
    terminal_theta = np.arctan2(terminal_hip[1], terminal_hip[0])
    terminal_d = np.hypot(terminal_rho - .05236, terminal_hip[2])
    terminal_phi = np.arctan2(terminal_hip[2], terminal_rho - .05236)
    air_d = np.hypot(.155 - .05236, -.010)
    air_phi = np.arctan2(-.010, .155 - .05236)
    def hip_polar(theta, distance, phi):
        radius = .05236 + distance * np.cos(phi)
        return np.array([radius * np.cos(theta), radius * np.sin(theta), distance * np.sin(phi)])
    rb_base = np.empty((7, 1, 3), dtype=float)
    rb_base[0] = _base_from_world(pose, anchors[active])
    rb_base[1, 0] = _hip_to_base(GraspKinematic(), active[0], high_hip)
    rb_base[2, 0] = _hip_to_base(GraspKinematic(), active[0], hip_polar(terminal_theta, air_d, air_phi))
    for index, fraction in ((3, .35), (4, .65)):
        distance = air_d * (1.0 - fraction) + terminal_d * fraction
        phi = air_phi * (1.0 - fraction) + terminal_phi * fraction
        rb_base[index, 0] = _hip_to_base(GraspKinematic(), active[0], hip_polar(terminal_theta, distance, phi))
    normal = np.array([-.20791, 0.0, .97815], dtype=float)
    normal /= np.linalg.norm(normal)
    rb_base[5] = _base_from_world(end_pose, rb_final + .025 * normal)
    rb_base[6] = _base_from_world(end_pose, rb_final)
    rb_durations = [1.40, .55, .45, .45, .65, .45]
    rb_velocities = _capped_internal_velocities(rb_base, rb_durations, (1, 2, 3, 4))
    rb_stage, anchors = _base_stage(
        "RB_DIRECT_PLATFORM_225", rb_poses, anchors, active, rb_base,
        rb_durations, rb_velocities,
    )
    stages.append(rb_stage); pose = end_pose

    # new15 C: retain the raised world air targets, but make their path move
    # with the body rather than remaining a world-z construction.
    end_pose = np.asarray(original[-1]["pose_end"], dtype=float).copy()
    end_pose[2:5] = [.265, 0.0, 0.0]
    active = [0]; target = final_source_anchors[active].copy()
    air_a = np.array([[.155, anchors[0, 1], .210]])
    air_b = np.array([[.225, anchors[0, 1], .210]])
    lb_poses = np.asarray([pose, end_pose, end_pose, end_pose])
    lb_base = np.asarray([
        _base_from_world(lb_poses[0], anchors[active]),
        _base_from_world(lb_poses[1], air_a),
        _base_from_world(lb_poses[2], air_b),
        _base_from_world(lb_poses[3], target),
    ])
    lb_durations = [1.1, .8, .4]
    lb_velocities = _capped_internal_velocities(lb_base, lb_durations, (1, 2))
    lb_stage, anchors = _base_stage(
        "LB_PLATFORM_225", lb_poses, anchors, active, lb_base,
        lb_durations, lb_velocities,
    )
    stages.append(lb_stage); pose = end_pose

    # new16..18 D: centre the finished robot on the CAD bounding-box centre.
    local = load_triangles(MESH); world = local + TRANSLATION
    goal = anchors.copy()
    for leg in range(6):
        if leg in (0, 3):
            x, y = .282, final_source_anchors[leg, 1] + FRONT_BACK_DY
        elif leg in (1, 4):
            x, y = .618, final_source_anchors[leg, 1] + FRONT_BACK_DY
        elif leg == 2:
            x, y = .415, .110
        else:
            x, y = .425, -.215
        goal[leg] = target_on_surface(local, world, x, y)
    for name, active, xyz, foot_first in (
        ("DIAGONAL_RF_LM_ADVANCE", [4, 2], [.365, CENTER_Y, .300], False),
        ("DIAGONAL_LF_RM_ADVANCE", [1, 5], [.450, CENTER_Y, .300], True),
        ("BACK_PAIR_CENTER", [0, 3], [.450, CENTER_Y, .300], False),
    ):
        end_pose = np.array([*xyz, 0.0, 0.0, pose[5]])
        target = goal[active].copy()
        apex = max(anchors[active, 2].max(), target[:, 2].max()) + .030
        air_a, air_b = anchors[active].copy(), target.copy(); air_a[:, 2] = apex; air_b[:, 2] = apex
        knots = [pose, pose, end_pose, end_pose] if foot_first else None
        stage, anchors = _move(name, pose, end_pose, anchors, active, target, (air_a, air_b), [.65, 1.10, .45], knots)
        stages.append(stage); pose = end_pose
    stages.append(_stage("FINAL_HOLD", pose, pose, [anchors, anchors], [], [.6]))

    if len(stages) != 19:
        raise ValueError(f"expected 19 stages, got {len(stages)}")
    kin = GraspKinematic()
    terminal_q, residual = solve_ik(kin, pose, anchors, np.asarray(source["terminal_q_rad"], dtype=float), tolerance_m=1e-5)
    margin = float(np.min(np.minimum(terminal_q - JOINT_LOWER, JOINT_UPPER - terminal_q)))
    if residual > .002 or margin <= 0.0:
        raise ValueError(f"terminal IK failed: residual={residual:.9g}, margin={margin:.9g}")
    cfg["stages"] = stages
    cfg["settle_gate"].update({
        "max_foot_target_error_m": .020,
        "persistence_s": .25,
        "timeout_s": 5.0,
        "preview_time_only_stage_advance": False,
        "tracking_errors_diagnostic_only": False,
    })
    for stage in cfg["stages"]:
        stage["settle_s"] = max(float(stage["settle_s"]), .25)
    cfg["stage_count"] = 19
    cfg["terminal_q_rad"] = terminal_q.tolist()
    cfg["front_optimized_source"] = {
        "source_config": SOURCE.name,
        "source_sha256": SOURCE_SHA,
        "simulation_only": True,
        "front_hardware_rejected": True,
        "revision": 5,
        "failed_predecessor_sha256": "7ae2970e1f4ee6ec5044b5b9eee877e04b00ac40afb997222fad885ceca1cea5",
        "chosen_stance_baseline_sha256": "c1e5860618fbef4a3cc71cd7bcbfbf35c6602ce91f6673a546b815c827c8ccc2",
        "body_relative_v5": {
            "intent": "active high swings use hip/base-relative Hermite curves while fixed supports stay world-constrained",
            "source_c1e586_body_poses": True,
            "no_world_z_microtuning": True,
            "feedback_checkpoint_simulation": True,
        },
        "center_target_body_xy_m": [.450, CENTER_Y],
        "high_clearance": {
            "hip_air_rho_m": .155,
            "hip_air_z_m": -.010,
            "lb_platform_air_world_z_m": .210,
            "centre_move_apex_clearance_m": .030,
        },
        "centre_grouping_intent": "diagonal RF+LM, diagonal LF+RM, then rear pair; avoid v1 same-side fore+mid unloading",
        "stage_map": {"1": "1", "2": "2", "3": "3+4", "4-11": "5-12", "12": "13+16", "13": "15", "14": "17", "15": "18", "16": "RF+LM diagonal", "17": "LF+RM diagonal", "18": "rear pair centre", "19": "hold"},
    }
    return cfg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    source = json.loads(SOURCE.read_text())
    result = build(source)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"output": str(args.output), "stages": result["stage_count"], "final_pose": result["stages"][-1]["pose_end"]}))


if __name__ == "__main__":
    main()
