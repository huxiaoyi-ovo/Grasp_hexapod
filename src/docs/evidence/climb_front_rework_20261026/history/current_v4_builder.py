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
from build_climb_front import MESH, TRANSLATION, load_triangles, solve_ik  # noqa: E402
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

    # new4..10 retain source C5..C11 as exact dictionaries; new11 is adjusted below.
    stages.extend(copy.deepcopy(original[4:12]))
    # Lift the middle pair as high as the statically verified support posture
    # permits, while extending the planted supports to raise the body.
    mid_platform = stages[-1]
    for knot_index in (1, 2, 3):
        for leg in (2, 5):
            mid_platform["anchor_knots"][knot_index][leg][2] += .030
    for knot_index in range(1, 6):
        mid_platform["pose_knots"][knot_index][2] += .010 if knot_index == 1 else .015
        mid_platform["pose_knots"][knot_index][3] += .150
    # Keep the body at the original x while the middle pair lifts, then use a
    # local intermediate crossing knot instead of a uniform body-height raise.
    first_air_pose = mid_platform["pose_knots"][1]
    first_air_pose[0], first_air_pose[2], first_air_pose[3] = .130, .226, .450
    middle_pose = copy.deepcopy(first_air_pose)
    middle_pose[0], middle_pose[2], middle_pose[3], middle_pose[4] = .165, .238, .600, 0.0
    middle_anchor = copy.deepcopy(mid_platform["anchor_knots"][1])
    middle_anchor[2] = [.150, middle_anchor[2][1], .185]
    middle_anchor[5] = [.150, middle_anchor[5][1], .1865]
    mid_platform["pose_knots"].insert(2, middle_pose)
    mid_platform["anchor_knots"].insert(2, middle_anchor)
    mid_platform["segment_durations_s"][1:2] = [.50, .50]
    mid_platform["pose_end"] = copy.deepcopy(mid_platform["pose_knots"][-1])
    pose = np.asarray(stages[-1]["pose_end"], dtype=float)
    anchors = np.asarray(stages[-1]["anchor_knots"][-1], dtype=float)
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

    # new14 B2: RB follows the verified high-clearance four-segment crossing.
    end_pose = np.asarray(original[16]["pose_end"], dtype=float).copy()
    end_pose[2:5] = [.259, .350, -.350]
    active = [3]
    end_anchors = anchors.copy()
    end_anchors[3] = final_source_anchors[3]
    rb_y = anchors[3, 1]
    rb_air_y = rb_y - .030
    rb_knots = [anchors.copy(), anchors.copy(), anchors.copy(), anchors.copy(), end_anchors]
    rb_knots[1][3] = [.055, rb_air_y, .200]
    rb_knots[2][3] = [.120, rb_air_y, .200]
    rb_knots[3][3] = [.195, rb_air_y, .185]
    stages.append(_stage(
        "RB_DIRECT_PLATFORM_225", pose, end_pose, rb_knots, active,
        [1.4, .65, .60, .45], [pose, end_pose, end_pose, end_pose, end_pose],
    ))
    anchors, pose = end_anchors, end_pose

    # new15 C: left rear platform transfer with its proven source air knots.
    end_pose = np.asarray(original[-1]["pose_end"], dtype=float).copy()
    end_pose[1:5] = [CENTER_Y, .300, 0.0, 0.0]
    active = [0]; target = final_source_anchors[active].copy()
    air_a = np.array([[.155, anchors[0, 1], .210]])
    air_b = np.array([[.225, anchors[0, 1], .210]])
    stage, anchors = _move("LB_PLATFORM_225", pose, end_pose, anchors, active, target, (air_a, air_b), [1.1, .8, .4], [pose, end_pose, end_pose, end_pose])
    stages.append(stage); pose = end_pose

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
    cfg["stage_count"] = 19
    cfg["terminal_q_rad"] = terminal_q.tolist()
    cfg["front_optimized_source"] = {
        "source_config": SOURCE.name,
        "source_sha256": SOURCE_SHA,
        "simulation_only": True,
        "front_hardware_rejected": True,
        "revision": 4,
        "failed_predecessor_sha256": "c1e5860618fbef4a3cc71cd7bcbfbf35c6602ce91f6673a546b815c827c8ccc2",
        "mid_platform_local_fix": {
            "v2_failed_config_sha256": "35fe720ebc98184e0da5f78bac20c7cf8e9ed173376d188bc63c62b39201ef6e",
            "intent": "hold body x for the vertical middle-leg lift, then cross through the inserted local knot",
            "uniform_body_plus_30mm": False,
        },
        "rb_direct_local_fix": {
            "air_y_offset_m": -.030,
            "intent": "outward air-only RB excursion; entry and landing contacts remain unchanged",
        },
        "center_target_body_xy_m": [.450, CENTER_Y],
        "high_clearance": {
            "mid_platform_air_anchor_raise_m": .030,
            "mid_platform_pose_raise_m": {"knot_1": .010, "knots_2_to_5": .015},
            "mid_platform_roll_raise_rad": .150,
            "rb_direct_anchor_knots_m": [[.055, .200], [.120, .200], [.195, .185]],
            "lb_platform_air_z_m": .210,
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
