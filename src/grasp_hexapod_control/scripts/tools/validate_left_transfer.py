#!/usr/bin/env python3
"""Dense model-only gate for the compact left-transfer stages.

The reported IK, posture, and support-polygon values are offline model
diagnostics.  They do not establish real contact, load sharing, friction, or
hardware stability.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / "src/grasp_hexapod_control/scripts"
sys.path.insert(0, str(SCRIPTS))

from climb_mode import ClimbMode
from control import GraspController
from kinematics import JOINT_LOWER, JOINT_UPPER
from utils import package_config_path
from utils.climb import gravity_projected_support
from utils.climb_retime import (
    assert_speed_report,
    segment_for_time,
    speed_report,
    update_speed_report,
)
from validate_final_transfer import footprint_clearance, stl_triangles


DT = 1.0 / 30.0
LEFT_START = 18
NAMES = (
    "BODY_LEFT_TRANSFER_PREP",
    "LB_LOW_STEP",
    "BODY_RIGHT_BEFORE_LF",
    "LF_LOW_STEP",
    "BODY_PRELOAD_LM",
    "LM_LIFT",
    "BODY_ADVANCE_LM_AIR",
    "LM_LEFT_FINAL_LAND",
)
CRITICAL = {
    "BODY_PRELOAD_LM",
    "LM_LIFT",
    "BODY_ADVANCE_LM_AIR",
    "LM_LEFT_FINAL_LAND",
}
SUPPORT_LEGS = {
    "BODY_LEFT_TRANSFER_PREP": (0, 1, 2, 3, 4, 5),
    "LB_LOW_STEP": (1, 2, 3, 4, 5),
    "BODY_RIGHT_BEFORE_LF": (0, 1, 2, 3, 4, 5),
    "LF_LOW_STEP": (0, 2, 3, 4, 5),
    "BODY_PRELOAD_LM": (0, 1, 2, 3, 4, 5),
    "LM_LIFT": (0, 1, 3, 4, 5),
    "BODY_ADVANCE_LM_AIR": (0, 1, 3, 4, 5),
    "LM_LEFT_FINAL_LAND": (0, 1, 3, 4, 5),
}
LM_NORMAL = np.array((-0.20791, 0.0, 0.97815), dtype=np.float64)
LM_NORMAL /= np.linalg.norm(LM_NORMAL)
LOW_SURFACE_LATERAL_EDGE_Y_LOCAL_M = 0.17374
HARD_EDGE_CLEARANCE_M = 0.015
LF_LATERAL_EDGE_MARGIN_M = 0.020
LM_AIR_SUPPORT_MARGIN_M = 0.029


def require(value, message):
    if not value:
        raise AssertionError(message)


def source(stage, time_s, leg, metric, actual, threshold):
    return {
        "stage": stage,
        "time_s": float(time_s),
        "leg": None if leg is None else int(leg),
        "metric": metric,
        "actual": float(actual),
        "threshold": float(threshold),
    }


def world_points(base_pose, points_base):
    transform = ClimbMode._world_from_base(base_pose)
    return (
        np.column_stack((points_base, np.ones(len(points_base))))
        @ transform.T
    )[:, :3]


def solve_exact(kinematic, q_seed, desired_base):
    """Follow the current seeded DLS branch to one dense reference sample."""

    desired_hip = kinematic.base_to_hip(desired_base)
    q = np.asarray(q_seed, dtype=np.float64).copy()
    for _ in range(1200):
        error = desired_hip - kinematic.forward(q)
        if float(np.max(np.linalg.norm(error, axis=1))) <= 5e-8:
            break
        inverse = kinematic.damped_inverse_jacobian(q, damping=0.001)
        delta = (inverse @ error[..., np.newaxis]).squeeze(-1)
        q = np.clip(q + np.clip(delta, -0.02, 0.02), JOINT_LOWER, JOINT_UPPER)
    residuals = np.linalg.norm(
        desired_base - kinematic.forward_base(q), axis=1
    )
    return q, residuals


def dense_times(stage):
    """Return ordered samples with at least 50 or 100 intervals per segment."""

    per_segment = 100 if stage["name"] in CRITICAL else 50
    result = [0.0]
    elapsed = 0.0
    for duration in stage["segment_durations_s"]:
        segment = np.linspace(0.0, float(duration), per_segment + 1)[1:]
        result.extend((elapsed + segment).tolist())
        elapsed += float(duration)
    return np.asarray(result, dtype=np.float64)


def stage_reference(mode, index, time_s):
    mode.stage_index = index
    mode.phase_time = float(time_s)
    pose, anchors, _ = mode._stage_reference()
    inverse = np.linalg.inv(ClimbMode._world_from_base(pose))
    desired = (
        np.column_stack((anchors, np.ones(6))) @ inverse.T
    )[:, :3]
    return pose, anchors, desired


def prefix_entry(compact):
    """Replay the unchanged prefix once and return its continuous DLS branch."""

    controller = GraspController(DT)
    q = np.asarray(compact["p0"]["q_rad"], dtype=np.float64)
    controller.enter_climb(q, compact)
    while (
        controller.climb_mode.state == ClimbMode.RUNNING
        and controller.climb_mode.stage_index < LEFT_START
    ):
        q = controller.update(q, np.zeros(4))
    require(controller.climb_mode.stage_index == LEFT_START, "prefix did not reach C19")
    return controller, q


def structural_gate(compact):
    stages = compact["stages"]
    require(compact["stage_count"] == 36 == len(stages), "expected 36 stages")
    require(tuple(stage["name"] for stage in stages[18:26]) == NAMES,
            "left-transfer stage map")
    require(stages[19]["active_legs"] == [0], "LB must swing alone")
    require(stages[20]["active_legs"] == [], "right shift must be body-only")
    right_shift = stages[20]
    require(np.array_equal(right_shift["anchor_knots"][0],
                           right_shift["anchor_knots"][-1]),
            "right shift must retain all six fixed support anchors")
    require(np.array_equal(np.asarray(right_shift["pose_start"])[[1, 4]],
                           np.asarray(right_shift["pose_end"])[[1, 4]]) and
            np.isclose(right_shift["pose_start"][0], .230) and
            np.isclose(right_shift["pose_end"][0], .232) and
            right_shift["segment_durations_s"] == [.8] and
            np.isclose(right_shift["settle_s"], .15),
            "right shift contract")
    require(np.allclose(right_shift["pose_start"],
                        [.230, -.06769449763600001, .215, -.16, -.2]) and
            np.allclose(right_shift["pose_end"],
                        [.232, -.06769449763600001, .200, 0.0, -.2]),
            "right shift must return level before LF swing")
    require(stages[21]["active_legs"] == [1], "LF must swing alone")
    lb = stages[19]
    require(len(lb["anchor_knots"]) == 4 and
            lb["segment_durations_s"] == [1.4, .9, .8] and
            np.allclose(lb["pose_start"], [.230, -.06769449763600001,
                                            .201, 0.0, -.2]) and
            np.allclose(lb["pose_end"], [.230, -.06769449763600001,
                                          .215, -.16, -.2]) and
            np.allclose(np.asarray(lb["active_base_knots_m"])[:, 0],
                        [[-.17585050573741318, -.16798118089306158,
                          -.16280924307881292],
                         [-.097, -.168, -.047],
                         [-.038, -.172, -.047],
                         [-.03798522630741269, -.1719221566687401,
                          -.0832004058967234]]),
            "LB folded lift-transfer-touchdown contract")
    lf = stages[21]
    require(len(lf["anchor_knots"]) == 4 and
            lf["segment_durations_s"] == [1.4, .9, 1.0] and
            np.allclose(lf["pose_start"], [.232, -.06769449763600001,
                                            .200, 0.0, -.2]) and
            np.allclose(lf["pose_end"], [.232, -.06769449763600001,
                                          .226, .16, -.2]) and
            np.allclose(np.asarray(lf["active_base_knots_m"])[:, 0],
                        [[-.17761196956230063, .1679808553169389,
                          -.16143183783938156],
                         [-.097, .168, -.046],
                         [-.0421307112772121, .17520403403337445, -.046],
                         [-.0421307112772121, .17520403403337445,
                          -.09424771313266513]]),
            "LF folded lift-transfer-touchdown contract")
    require(stages[22]["active_legs"] == [], "LM preload must be body-only")
    require(np.allclose(stages[22]["pose_start"],
                        [.232, -.06769449763600001, .226, .16, -.2]) and
            np.allclose(stages[22]["pose_end"],
                        [.234, -.06769449763600001, .201, 0.0, -.2]) and
            stages[22]["segment_durations_s"] == [1.0],
            "LM preload contract")
    require(all(stages[index]["active_legs"] == [2] for index in (23, 24, 25)),
            "LM must lift, remain airborne, then land once")
    require(not any(set(stage["active_legs"]) == {0, 1} for stage in stages[18:26]),
            "LB/LF simultaneous swing is forbidden")
    for index in range(1, len(stages)):
        require(np.allclose(stages[index - 1]["pose_end"], stages[index]["pose_start"],
                            rtol=0.0, atol=1e-10),
                "pose boundary C{}->C{}".format(index, index + 1))
        require(np.allclose(stages[index - 1]["anchor_knots"][-1],
                            stages[index]["anchor_knots"][0],
                            rtol=0.0, atol=1e-10),
                "anchor boundary C{}->C{}".format(index, index + 1))
    require(np.array_equal(np.asarray(stages[25]["anchor_knots"][-1]),
                           np.asarray(stages[26]["anchor_knots"])[0]),
            "C26 final left-transfer state must enter the compressed tail")
    lf_target = np.asarray(stages[21]["anchor_knots"][-1][1])
    lf_local_y = lf_target[1] - compact["xiaolan_translation"][1]
    lateral_margin = LOW_SURFACE_LATERAL_EDGE_Y_LOCAL_M - lf_local_y
    require(lateral_margin >= LF_LATERAL_EDGE_MARGIN_M,
            source("LF_LOW_STEP", 0.0, 1, "lateral_edge_margin_m",
                   lateral_margin, LF_LATERAL_EDGE_MARGIN_M))


def dense_validate(compact):
    controller, q = prefix_entry(compact)
    kinematic = controller.kinematic
    mode = ClimbMode(None)
    mode.config = compact
    model = (ROOT /
             "src/grasp_hexapod_description/meshes/xiaolan/base_link_xiaolan.STL")
    triangles = stl_triangles(model, compact["xiaolan_translation"])
    reports = []
    global_sources = {}

    for index in range(18, 26):
        stage = compact["stages"][index]
        times = dense_times(stage)
        report = {
            "stage_name": stage["name"],
            "sample_count": int(len(times)),
            "min_support_margin_m": np.inf,
            "min_joint_margin_rad": np.inf,
            "min_sigma_min": np.inf,
            "max_joint_speed_rad_s": 0.0,
            "LM_min_hip_foot_xy_m": np.inf,
            "LM_max_terminal_axis_angle_deg": 0.0,
            "max_IK_residual_m": 0.0,
            "segments": speed_report(index, stage),
        }
        if stage["name"] == "LB_LOW_STEP":
            report["platform_entry_clearance_m"] = np.inf
        previous_q = None
        previous_t = None
        for time_s in times:
            pose, anchors, desired = stage_reference(mode, index, time_s)
            q, residuals = solve_exact(kinematic, q, desired)
            max_residual = float(np.max(residuals))
            if max_residual > report["max_IK_residual_m"]:
                leg = int(np.argmax(residuals))
                report["max_IK_residual_m"] = max_residual
            if ("max_IK_residual_m" not in global_sources or
                    max_residual > global_sources["max_IK_residual_m"]["actual"]):
                leg = int(np.argmax(residuals))
                global_sources["max_IK_residual_m"] = source(
                    stage["name"], time_s, leg, "IK_residual_m",
                    max_residual, 1e-5)

            margins = kinematic.joint_limit_margins(q)
            joint_margin = float(np.min(margins))
            if joint_margin < report["min_joint_margin_rad"]:
                leg, joint = np.unravel_index(np.argmin(margins), margins.shape)
                report["min_joint_margin_rad"] = joint_margin
                global_sources["min_joint_margin_rad:" + stage["name"]] = {
                    **source(stage["name"], time_s, leg, "joint_margin_rad",
                             joint_margin, 0.08),
                    "joint": int(joint),
                }

            sigma = kinematic.jacobian_min_singular_values(q)
            sigma_value = float(sigma[2])
            report["min_sigma_min"] = min(report["min_sigma_min"], sigma_value)

            transform = ClimbMode._world_from_base(pose)
            com_world = (transform @ np.append(kinematic.center_of_mass_base(q), 1.0))[:3]
            support_legs = SUPPORT_LEGS[stage["name"]]
            support_result = gravity_projected_support(
                com_world, anchors[list(support_legs)], (0.0, 0.0, -1.0))
            require(support_result.valid, source(
                stage["name"], time_s, None, "support_polygon", -np.inf, 0.03))
            report["min_support_margin_m"] = min(
                report["min_support_margin_m"], float(support_result.raw_margin_m))

            hip_world = (transform @ np.append(
                kinematic.base_from_hip[2][:3, 3], 1.0))[:3]
            foot_world = world_points(pose, kinematic.forward_base(q)[[2]])[0]
            radius = float(np.linalg.norm(foot_world[:2] - hip_world[:2]))
            report["LM_min_hip_foot_xy_m"] = min(
                report["LM_min_hip_foot_xy_m"], radius)
            axis_world = transform[:3, :3] @ kinematic.terminal_axes_base(q)[2]
            angle = float(np.degrees(np.arccos(np.clip(
                abs(np.dot(axis_world, LM_NORMAL)), -1.0, 1.0))))
            report["LM_max_terminal_axis_angle_deg"] = max(
                report["LM_max_terminal_axis_angle_deg"], angle)

            if (stage["name"] == "LB_LOW_STEP" and
                    np.isinf(report["platform_entry_clearance_m"])):
                try:
                    report["platform_entry_clearance_m"] = footprint_clearance(
                        triangles, anchors[0])
                except AssertionError:
                    pass

            if previous_q is not None:
                speed = np.abs(q - previous_q) / (float(time_s) - previous_t)
                report["max_joint_speed_rad_s"] = max(
                    report["max_joint_speed_rad_s"], float(np.max(speed)))
                semantic = segment_for_time(index, stage, time_s)
                peak = float(np.max(speed))
                leg, joint = np.unravel_index(np.argmax(speed), speed.shape)
                update_speed_report(
                    report["segments"], semantic["segment_index"], peak,
                    {**source(stage["name"], time_s, int(leg),
                               "joint_speed_rad_s", peak,
                               semantic["hard_gate_rad_s"]),
                     "joint": int(joint)},
                )
            previous_q = q.copy()
            previous_t = float(time_s)

        reports.append(report)

    by_name = {item["stage_name"]: item for item in reports}
    for item in reports:
        name = item["stage_name"]
        require(item["max_IK_residual_m"] <= 1e-5,
                global_sources.get("max_IK_residual_m"))
        require(item["min_joint_margin_rad"] >= 0.08,
                global_sources["min_joint_margin_rad:" + name])
        assert_speed_report(item["segments"], require)
        if name in ("LB_LOW_STEP", "LF_LOW_STEP", "LM_LIFT",
                    "BODY_ADVANCE_LM_AIR", "LM_LEFT_FINAL_LAND"):
            support_margin = (LM_AIR_SUPPORT_MARGIN_M
                              if name in ("LM_LIFT", "BODY_ADVANCE_LM_AIR")
                              else 0.03)
            require(item["min_support_margin_m"] >= support_margin,
                    source(name, 0.0, None, "support_margin_m",
                           item["min_support_margin_m"], support_margin))
        if name in NAMES[4:]:
            require(item["min_sigma_min"] >= 0.01,
                    source(name, 0.0, 2, "LM_sigma_min",
                           item["min_sigma_min"], 0.01))
    lb_entry_clearance = by_name["LB_LOW_STEP"]["platform_entry_clearance_m"]
    require(np.isfinite(lb_entry_clearance) and
            lb_entry_clearance >= HARD_EDGE_CLEARANCE_M,
            source("LB_LOW_STEP", 0.0, 0, "platform_entry_clearance_m",
                   lb_entry_clearance, HARD_EDGE_CLEARANCE_M))

    preload = compact["stages"][22]
    pose, anchors, desired = stage_reference(
        mode, 22, sum(preload["segment_durations_s"]))
    q, _ = solve_exact(kinematic, q, desired)
    transform = ClimbMode._world_from_base(pose)
    com_world = (transform @ np.append(kinematic.center_of_mass_base(q), 1.0))[:3]
    future = gravity_projected_support(
        com_world, anchors[[0, 1, 3, 4, 5]], (0.0, 0.0, -1.0))
    require(future.valid and future.raw_margin_m >= 0.03,
            source("BODY_PRELOAD_LM", sum(preload["segment_durations_s"]), None,
                   "future_support_margin_m", future.raw_margin_m, 0.03))

    final_stage = compact["stages"][25]
    pose, _, desired = stage_reference(
        mode, 25, sum(final_stage["segment_durations_s"]))
    q, _ = solve_exact(kinematic, q, desired)
    transform = ClimbMode._world_from_base(pose)
    hip_world = (transform @ np.append(kinematic.base_from_hip[2][:3, 3], 1.0))[:3]
    foot_world = world_points(pose, kinematic.forward_base(q)[[2]])[0]
    final_radius = float(np.linalg.norm(foot_world[:2] - hip_world[:2]))
    final_axis = transform[:3, :3] @ kinematic.terminal_axes_base(q)[2]
    final_angle = float(np.degrees(np.arccos(np.clip(
        abs(np.dot(final_axis, LM_NORMAL)), -1.0, 1.0))))
    require(final_radius >= 0.03,
            source("LM_LEFT_FINAL_LAND", sum(final_stage["segment_durations_s"]),
                   2, "LM_hip_foot_xy_m", final_radius, 0.03))
    require(final_angle <= 30.0,
            source("LM_LEFT_FINAL_LAND", sum(final_stage["segment_durations_s"]),
                   2, "LM_terminal_axis_angle_deg", final_angle, 30.0))
    return reports, {
        "future_support_margin_m": float(future.raw_margin_m),
        "LB_platform_entry_clearance_m": float(lb_entry_clearance),
        "LF_lateral_edge_margin_m": float(
            LOW_SURFACE_LATERAL_EDGE_Y_LOCAL_M -
             (compact["stages"][21]["anchor_knots"][-1][1][1] -
             compact["xiaolan_translation"][1])),
        "final_LM_hip_foot_xy_m": final_radius,
        "final_LM_terminal_axis_angle_deg": final_angle,
        "sources": global_sources,
    }


def dynamic_gate(compact):
    controller = GraspController(DT)
    q = np.asarray(compact["p0"]["q_rad"], dtype=np.float64)
    controller.enter_climb(q, compact)
    minimum = np.inf
    support = np.inf
    margin_source = None
    segments = {index: speed_report(index, compact["stages"][index])
                for index in range(18, 26)}
    ticks = 0
    while controller.climb_mode.state == ClimbMode.RUNNING:
        name = controller.climb_mode.phase
        time_s = controller.climb_mode.phase_time
        stage_index = controller.climb_mode.stage_index
        before = q.copy()
        q = controller.update(q, np.zeros(4))
        if name not in NAMES:
            ticks += 1
            continue
        semantic = segment_for_time(
            stage_index, compact["stages"][stage_index], time_s)
        speed = np.abs(q - before) / DT
        peak = float(np.max(speed))
        leg, joint = np.unravel_index(np.argmax(speed), speed.shape)
        update_speed_report(
            segments[stage_index], semantic["segment_index"], peak,
            {**source(name, time_s, int(leg), "command_speed_rad_s", peak,
                       semantic["hard_gate_rad_s"]),
             "tick": ticks, "joint": int(joint)},
        )
        margins = controller.kinematic.joint_limit_margins(q)
        value = float(np.min(margins))
        if value < minimum:
            leg, joint = np.unravel_index(np.argmin(margins), margins.shape)
            minimum = value
            margin_source = {**source(name, time_s, leg, "joint_margin_rad",
                                      value, 0.08), "joint": int(joint)}
        if name in ("LM_LIFT", "BODY_ADVANCE_LM_AIR"):
            pose = controller.climb_mode.base_pose
            transform = ClimbMode._world_from_base(pose)
            com = (transform @ np.append(
                controller.kinematic.center_of_mass_base(q), 1.0))[:3]
            value = gravity_projected_support(
                com, controller.climb_mode.anchors_world[[0, 1, 3, 4, 5]],
                (0.0, 0.0, -1.0)).raw_margin_m
            support = min(support, float(value))
        ticks += 1
    require(minimum >= 0.08, margin_source)
    require(support >= LM_AIR_SUPPORT_MARGIN_M,
            source("LM_AIR_DYNAMIC", 0.0, None, "support_margin_m", support,
                   LM_AIR_SUPPORT_MARGIN_M))
    for rows in segments.values():
        assert_speed_report(rows, require)
    return {"min_joint_margin_rad": minimum,
            "min_LM_air_support_margin_m": support,
            "min_joint_margin_source": margin_source,
            "segments": [
                {"stage_name": compact["stages"][index]["name"],
                 "segments": rows}
                for index, rows in segments.items()
            ]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path,
                        default=package_config_path("climb_compact.json"))
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    compact = json.loads(args.config.read_text(encoding="utf-8"))
    ClimbMode(None)._validate_config(compact)
    structural_gate(compact)
    stages, summary = dense_validate(compact)
    summary["dynamic_30hz"] = dynamic_gate(compact)
    report = {
        "stages": stages,
        "summary": summary,
        "evidence_boundary": (
            "Offline kinematics, support-polygon, and sampled CAD-clearance "
            "diagnostics only; not contact, load, friction, continuous "
            "clearance, stability, or hardware authorization."
        ),
    }
    if args.report:
        args.report.write_text(
            json.dumps(report, indent=2, allow_nan=False) + "\n",
            encoding="utf-8")
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
