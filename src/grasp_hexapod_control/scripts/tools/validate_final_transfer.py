#!/usr/bin/env python3
"""Validate the C25-to-final compressed six-leg transfer.

All results are offline kinematic/CAD diagnostics. They do not establish
contact, load sharing, friction, physical clearance, or hardware safety.
"""

import argparse
import json
from pathlib import Path
import struct
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
from utils.climb_collision import (
    ANKLE_VISUAL_FOOTPAD_COMPONENT_INDEX,
    default_visual_scene,
)
from utils.climb_retime import (
    assert_speed_report,
    segment_for_time,
    speed_report,
    update_speed_report,
)


DT = 1.0 / 30.0
FIRST = 25
FOOT_RADIUS_M = 0.0065
HARD_CLEARANCE_M = 0.015
PREFERRED_CLEARANCE_M = 0.030
DIRECT = (
    (25, "RB_DIRECT_FINAL", 3),
    (26, "RF_DIRECT_FINAL", 4),
    (28, "LM_DIRECT_FINAL", 2),
    (29, "RM_DIRECT_FINAL", 5),
    (31, "LB_DIRECT_FINAL", 0),
    (32, "LF_DIRECT_FINAL", 1),
)
BODY = (
    (27, "BODY_REPOSITION_1"),
    (30, "BODY_REPOSITION_2"),
    (33, "BODY_DOCK_FINAL"),
)
TAIL_NAMES = (
    "RB_DIRECT_FINAL", "RF_DIRECT_FINAL", "BODY_REPOSITION_1",
    "LM_DIRECT_FINAL", "RM_DIRECT_FINAL", "BODY_REPOSITION_2",
    "LB_DIRECT_FINAL", "LF_DIRECT_FINAL", "BODY_DOCK_FINAL",
    "STAND_FINAL_HOLD",
)
FINAL_ANCHORS = np.array([
    [0.3571737724974, -0.21963921590647986, 0.19262685293211096],
    [0.35376607048511516, 0.11059810066223177, 0.1972999976158142],
    [0.27440314554432327, -0.05747349999999976, 0.16965469007282727],
    [0.5423322377301724, -0.21962874608688565, 0.1923691075872171],
    [0.5477660704851153, 0.11059810066223177, 0.19649999761581421],
    [0.6267854252002603, -0.05747349999999976, 0.16858417926213481],
], dtype=np.float64)
FINAL_BODY = np.array([0.45, -0.058, 0.26, 0.0, 0.0], dtype=np.float64)


def require(condition, detail):
    if not condition:
        raise AssertionError(json.dumps(detail, allow_nan=False))


def source(stage, time_s, metric, actual, threshold):
    return {"stage": stage, "time_s": float(time_s), "metric": metric,
            "actual": float(actual), "threshold": float(threshold)}


def stl_triangles(path, translation):
    data = Path(path).read_bytes()
    require(len(data) >= 84, {"metric": "STL_header"})
    count = struct.unpack("<I", data[80:84])[0]
    require(84 + count * 50 == len(data), {"metric": "binary_STL_size"})
    dtype = np.dtype([
        ("normal", "<f4", 3), ("vertices", "<f4", (3, 3)),
        ("attribute", "<u2"),
    ])
    triangles = np.frombuffer(
        data, dtype=dtype, offset=84, count=count
    )["vertices"].astype(np.float64)
    return triangles + np.asarray(translation, dtype=np.float64)[None, None, :]


def surface_height(triangles, xy):
    a = triangles[:, 0, :2]
    edge_0 = triangles[:, 1, :2] - a
    edge_1 = triangles[:, 2, :2] - a
    point = np.asarray(xy, dtype=np.float64) - a
    determinant = edge_0[:, 0] * edge_1[:, 1] - edge_1[:, 0] * edge_0[:, 1]
    valid = np.abs(determinant) > 1e-12
    u = np.zeros(len(triangles), dtype=np.float64)
    v = np.zeros(len(triangles), dtype=np.float64)
    u[valid] = (
        point[valid, 0] * edge_1[valid, 1]
        - edge_1[valid, 0] * point[valid, 1]
    ) / determinant[valid]
    v[valid] = (
        edge_0[valid, 0] * point[valid, 1]
        - point[valid, 0] * edge_0[valid, 1]
    ) / determinant[valid]
    inside = valid & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1.0 + 1e-9)
    require(np.any(inside), {"metric": "CAD_projection", "xy": list(xy)})
    z = (triangles[:, 0, 2]
         + u * (triangles[:, 1, 2] - triangles[:, 0, 2])
         + v * (triangles[:, 2, 2] - triangles[:, 0, 2]))
    return float(np.max(z[inside]))


def footprint_clearance(triangles, foot):
    angles = np.linspace(0.0, 2.0 * np.pi, 17)[:-1]
    offsets = np.vstack((
        np.zeros((1, 2)),
        FOOT_RADIUS_M * np.column_stack((np.cos(angles), np.sin(angles))),
    ))
    heights = []
    for offset in offsets:
        try:
            heights.append(surface_height(triangles, foot[:2] + offset))
        except AssertionError:
            pass
    require(heights, {"metric": "CAD_footprint_projection",
                      "foot": foot.tolist()})
    height = max(heights)
    return float(foot[2] - FOOT_RADIUS_M - height)


def solve_exact(kinematic, q_seed, desired_base):
    q = np.asarray(q_seed, dtype=np.float64).copy()
    for _ in range(1200):
        error = kinematic.base_to_hip(desired_base) - kinematic.forward(q)
        if float(np.max(np.linalg.norm(error, axis=1))) <= 5e-8:
            break
        inverse = kinematic.damped_inverse_jacobian(q, damping=0.001)
        delta = (inverse @ error[..., None]).squeeze(-1)
        q = np.clip(q + np.clip(delta, -0.02, 0.02),
                    JOINT_LOWER, JOINT_UPPER)
    residual = np.linalg.norm(
        desired_base - kinematic.forward_base(q), axis=1
    )
    return q, residual


def reference(mode, index, time_s):
    mode.stage_index = index
    mode.phase_time = float(time_s)
    pose, anchors, _ = mode._stage_reference()
    transform = ClimbMode._world_from_base(pose)
    inverse = np.linalg.inv(transform)
    desired = np.column_stack((anchors, np.ones(6))) @ inverse.T
    return pose, anchors, transform, desired[:, :3]


def prefix_entry(compact):
    controller = GraspController(DT)
    q = np.asarray(compact["p0"]["q_rad"], dtype=np.float64)
    controller.enter_climb(q, compact)
    while (controller.climb_mode.state == ClimbMode.RUNNING
           and controller.climb_mode.stage_index < FIRST):
        q = controller.update(q, np.zeros(4))
    require(controller.climb_mode.stage_index == FIRST,
            {"metric": "prefix_entry", "actual": controller.climb_mode.stage_index,
             "threshold": FIRST})
    return controller, q


def structural_gate(compact):
    stages = compact["stages"]
    require(compact["stage_count"] == len(stages) == 35,
            {"metric": "stage_count", "actual": len(stages), "threshold": 35})
    require(tuple(stage["name"] for stage in stages[FIRST:]) == TAIL_NAMES,
            {"metric": "tail_stage_map"})
    for index in range(1, len(stages)):
        require(np.allclose(stages[index - 1]["pose_end"],
                            stages[index]["pose_start"], rtol=0.0, atol=1e-12),
                {"stage": stages[index]["name"], "metric": "pose_boundary"})
        require(np.allclose(stages[index - 1]["anchor_knots"][-1],
                            stages[index]["anchor_knots"][0],
                            rtol=0.0, atol=1e-12),
                {"stage": stages[index]["name"], "metric": "anchor_boundary"})
    moved = set()
    for index, name, leg in DIRECT:
        stage = stages[index]
        require(stage["name"] == name and stage["active_legs"] == [leg],
                {"stage": name, "metric": "single_active_leg"})
        knots = np.asarray(stage["anchor_knots"], dtype=np.float64)
        require(knots.shape == (4, 6, 3)
                and len(stage["segment_durations_s"]) == 3,
                {"stage": name, "metric": "direct_primitive_shape"})
        require(np.array_equal(stage["pose_start"], stage["pose_end"]),
                {"stage": name, "metric": "fixed_body_during_swing"})
        fixed = [item for item in range(6) if item != leg]
        require(np.array_equal(knots[:, fixed], np.repeat(
                    knots[0, fixed][None, ...], 4, axis=0)),
                {"stage": name, "metric": "fixed_anchor_identity"})
        require(np.array_equal(knots[-1, leg], FINAL_ANCHORS[leg]),
                {"stage": name, "metric": "direct_final_target"})
        require(leg not in moved, {"stage": name, "metric": "leg_moved_once"})
        moved.add(leg)
    require(moved == set(range(6)), {"metric": "all_legs_moved_once"})
    for index, name in BODY:
        stage = stages[index]
        require(stage["name"] == name and stage["active_legs"] == [],
                {"stage": name, "metric": "body_stage"})
        require(np.array_equal(stage["anchor_knots"][0],
                               stage["anchor_knots"][-1]),
                {"stage": name, "metric": "body_fixed_anchors"})
    for index, _, leg in DIRECT:
        require(all(np.array_equal(stage["anchor_knots"][-1][leg],
                                   FINAL_ANCHORS[leg])
                    for stage in stages[index:]),
                {"leg": leg, "metric": "no_refoot_after_final"})
    require(np.array_equal(stages[-1]["anchor_knots"][-1], FINAL_ANCHORS),
            {"metric": "final_anchor_identity"})
    require(np.array_equal(stages[-1]["pose_end"], FINAL_BODY),
            {"metric": "final_body_identity"})
    forbidden = ("TEMP_LAND", "FINAL_REFOOT", "LOW_RESET_50", "LIFT35",
                 "BODY_AIR", "AIR_REBRANCH", "BODY_P35", "BODY_P383",
                 "BODY_P39")
    require(not any(token in stage["name"] for stage in stages[FIRST:]
                    for token in forbidden),
            {"metric": "historical_tail_removed"})


def sample_times(stage, direct_samples=101):
    durations = stage["segment_durations_s"]
    if stage["active_legs"]:
        output = [0.0]
        elapsed = 0.0
        for duration in durations:
            output.extend((elapsed + np.linspace(
                0.0, duration, direct_samples + 1)[1:]).tolist())
            elapsed += duration
        return output
    duration = float(sum(durations))
    return np.linspace(0.0, duration, int(round(duration / DT)) + 1).tolist()


def dense_kinematic_gate(compact):
    controller, q = prefix_entry(compact)
    kinematic = controller.kinematic
    mode = ClimbMode(None)
    mode.config = compact
    model = ROOT / "src/grasp_hexapod_description/meshes/xiaolan/base_link_xiaolan.STL"
    triangles = stl_triangles(model, compact["xiaolan_translation"])
    reports = []
    for index, stage in enumerate(compact["stages"][FIRST:], FIRST):
        active = stage["active_legs"]
        report = {
            "stage_name": stage["name"], "sample_count": 0,
            "min_support_margin_m": np.inf,
            "min_joint_margin_rad": np.inf,
            "min_sigma": np.inf,
            "max_joint_speed_rad_s": 0.0,
            "max_IK_residual_m": 0.0,
            "segments": speed_report(index, stage),
        }
        if active:
            report["min_cross_clearance_m"] = np.inf
            report["preferred_clearance_met"] = True
            report["fixed_support_thigh_peak_rad"] = 0.0
        entry_q = q.copy()
        previous_q = None
        previous_t = None
        durations = stage["segment_durations_s"]
        for time_s in sample_times(stage):
            _, anchors, transform, desired = reference(mode, index, time_s)
            q, residual = solve_exact(kinematic, q, desired)
            report["sample_count"] += 1
            report["max_IK_residual_m"] = max(
                report["max_IK_residual_m"], float(np.max(residual)))
            report["min_joint_margin_rad"] = min(
                report["min_joint_margin_rad"],
                float(np.min(kinematic.joint_limit_margins(q))))
            report["min_sigma"] = min(
                report["min_sigma"],
                float(np.min(kinematic.jacobian_min_singular_values(q))))
            support_legs = [item for item in range(6) if item not in active]
            com = (transform @ np.append(
                kinematic.center_of_mass_base(q), 1.0))[:3]
            support = gravity_projected_support(
                com, anchors[support_legs], (0.0, 0.0, -1.0))
            require(support.valid, source(stage["name"], time_s,
                                          "support_polygon", -1.0, 0.03))
            report["min_support_margin_m"] = min(
                report["min_support_margin_m"], float(support.raw_margin_m))
            if active:
                leg = active[0]
                fixed = [item for item in range(6) if item != leg]
                report["fixed_support_thigh_peak_rad"] = max(
                    report["fixed_support_thigh_peak_rad"],
                    float(np.max(np.abs(q[fixed, 0] - entry_q[fixed, 0]))))
                if durations[0] <= time_s <= durations[0] + durations[1]:
                    report["min_cross_clearance_m"] = min(
                        report["min_cross_clearance_m"],
                        footprint_clearance(triangles, anchors[leg]))
            if previous_q is not None:
                speed = np.abs(q - previous_q) / (time_s - previous_t)
                peak = float(np.max(speed))
                report["max_joint_speed_rad_s"] = max(
                    report["max_joint_speed_rad_s"], peak)
                semantic = segment_for_time(index, stage, time_s)
                if semantic is not None:
                    leg, joint = np.unravel_index(np.argmax(speed), speed.shape)
                    update_speed_report(
                        report["segments"], semantic["segment_index"], peak,
                        {**source(stage["name"], time_s, "joint_speed_rad_s", peak,
                                   semantic["hard_gate_rad_s"]),
                         "leg": int(leg), "joint": int(joint)},
                    )
            previous_q = q.copy()
            previous_t = float(time_s)
        checks = (
            ("min_support_margin_m", 0.03, True),
            ("min_joint_margin_rad", 0.08, True),
            ("min_sigma", 0.01, True),
            ("max_IK_residual_m", 1e-5, False),
        )
        if active:
            checks += (
                ("min_cross_clearance_m", HARD_CLEARANCE_M, True),
                ("fixed_support_thigh_peak_rad", 0.08, False),
            )
            report["preferred_clearance_met"] = bool(
                report["min_cross_clearance_m"] >= PREFERRED_CLEARANCE_M)
        for metric, threshold, minimum in checks:
            actual = report[metric]
            require(actual >= threshold if minimum else actual <= threshold,
                    source(stage["name"], 0.0, metric, actual, threshold))
        assert_speed_report(report["segments"], require)
        if index >= 34:
            require(report["max_joint_speed_rad_s"] <= 2.5,
                    source(stage["name"], 0.0, "joint_speed_rad_s",
                           report["max_joint_speed_rad_s"], 2.5))
        reports.append(report)
    return reports


def visual_platform_gate(compact):
    controller, q = prefix_entry(compact)
    mode = ClimbMode(None)
    mode.config = compact
    world_from_xiaolan = np.eye(4, dtype=np.float64)
    world_from_xiaolan[:3, 3] = compact["xiaolan_translation"]
    scene = default_visual_scene(ROOT, world_from_xiaolan)
    reports = []
    for index, stage in enumerate(compact["stages"][FIRST:], FIRST):
        if not stage["active_legs"]:
            _, _, _, desired = reference(
                mode, index, float(sum(stage["segment_durations_s"])))
            q, _ = solve_exact(controller.kinematic, q, desired)
            continue
        leg = stage["active_legs"][0]
        durations = stage["segment_durations_s"]
        lift_end = float(durations[0])
        cross_end = lift_end + float(durations[1])
        report = {"stage_name": stage["name"], "sample_count": 0,
                  "cross_sample_count": 0}
        for time_s in sample_times(stage, direct_samples=61):
            _, _, transform, desired = reference(mode, index, time_s)
            q, residual = solve_exact(controller.kinematic, q, desired)
            require(float(np.max(residual)) < 1e-5,
                    source(stage["name"], time_s, "visual_IK_residual_m",
                           float(np.max(residual)), 1e-5))
            active = scene.robot_components(
                q, transform, [leg], include_body=False)
            nonfoot = scene.components_vs_xiaolan(
                active,
                exclude_components=((
                    "ankle", leg, ANKLE_VISUAL_FOOTPAD_COMPONENT_INDEX),),
            )
            require(not nonfoot.collision,
                    {"stage": stage["name"], "time_s": time_s,
                     "metric": "active_nonfoot_visual_vs_xiaolan",
                     "hit": None if nonfoot.hit is None else nonfoot.hit.__dict__})
            report["sample_count"] += 1
            if lift_end - 1e-12 <= time_s <= cross_end + 1e-12:
                full = scene.components_vs_xiaolan(active)
                feet = scene.feet_vs_xiaolan(
                    q, transform,
                    ignore_links=tuple(("foot", item)
                                       for item in range(6) if item != leg),
                )
                require(not full.collision and not feet.collision,
                        {"stage": stage["name"], "time_s": time_s,
                         "metric": "active_full_visual_vs_xiaolan",
                         "visual_hit": None if full.hit is None else full.hit.__dict__,
                         "foot_hit": None if feet.hit is None else feet.hit.__dict__})
                report["cross_sample_count"] += 1
        reports.append(report)
    return reports


def candidate_a_witness(compact):
    controller, q = prefix_entry(compact)
    pose = np.asarray(compact["stages"][FIRST - 1]["pose_end"], dtype=np.float64)
    transform = ClimbMode._world_from_base(pose)
    inverse = np.linalg.inv(transform)
    desired = (np.column_stack((FINAL_ANCHORS, np.ones(6))) @ inverse.T)[:, :3]
    q, residual = solve_exact(controller.kinematic, q, desired)
    witness = {
        "max_IK_residual_m": float(np.max(residual)),
        "min_joint_margin_rad": float(np.min(
            controller.kinematic.joint_limit_margins(q))),
        "min_sigma": float(np.min(
            controller.kinematic.jacobian_min_singular_values(q))),
    }
    witness["feasible"] = bool(
        witness["max_IK_residual_m"] < 1e-5
        and witness["min_joint_margin_rad"] >= 0.08
        and witness["min_sigma"] > 0.01)
    require(not witness["feasible"],
            {"metric": "candidate_A_expected_infeasible", **witness})
    return witness


def dynamic_gate(compact):
    controller = GraspController(DT)
    q = np.asarray(compact["p0"]["q_rad"], dtype=np.float64)
    controller.enter_climb(q, compact)
    legs = {name: leg for _, name, leg in DIRECT}
    reports = {
        index: {"stage_name": stage["name"], "min_joint_margin_rad": np.inf,
                "max_command_speed_rad_s": 0.0,
                "max_active_foot_error_m": 0.0,
                "segments": speed_report(index, stage)}
        for index, stage in enumerate(compact["stages"][FIRST:34], FIRST)
    }
    ticks = 0
    while controller.climb_mode.state == ClimbMode.RUNNING:
        name = controller.climb_mode.phase
        stage_index = controller.climb_mode.stage_index
        time_s = controller.climb_mode.phase_time
        before = q.copy()
        q = controller.update(q, np.zeros(4))
        if stage_index not in reports:
            ticks += 1
            continue
        item = reports[stage_index]
        item["min_joint_margin_rad"] = min(
            item["min_joint_margin_rad"],
            float(np.min(controller.kinematic.joint_limit_margins(q))))
        speed = np.abs(q - before) / DT
        peak = float(np.max(speed))
        item["max_command_speed_rad_s"] = max(item["max_command_speed_rad_s"],
                                                peak)
        semantic = segment_for_time(stage_index, compact["stages"][stage_index],
                                    time_s)
        speed_leg, joint = np.unravel_index(np.argmax(speed), speed.shape)
        update_speed_report(
            item["segments"], semantic["segment_index"], peak,
            {"stage": name, "time_s": time_s, "tick": ticks,
             "leg": int(speed_leg), "joint": int(joint),
             "metric": "command_speed_rad_s", "actual": peak,
             "threshold": semantic["hard_gate_rad_s"]},
        )
        if name in legs:
            leg = legs[name]
            actual = controller.kinematic.forward_base(q)
            item["max_active_foot_error_m"] = max(
                item["max_active_foot_error_m"],
                float(np.linalg.norm(
                    actual[leg] - controller.foot_desired_base[leg])))
        ticks += 1
    for item in reports.values():
        name = item["stage_name"]
        for metric, threshold, minimum in (("min_joint_margin_rad", 0.08, True),):
            actual = item[metric]
            require(actual >= threshold if minimum else actual <= threshold,
                    source(name, 0.0, metric, actual, threshold))
        assert_speed_report(item["segments"], require)
        if name in legs:
            require(item["max_active_foot_error_m"] <= 0.015,
                    source(name, 0.0, "max_active_foot_error_m",
                           item["max_active_foot_error_m"], 0.015))
    return list(reports.values())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path,
                        default=package_config_path("climb_compact.json"))
    args = parser.parse_args()
    compact = json.loads(args.config.read_text(encoding="utf-8"))
    structural_gate(compact)
    result = {
        "candidate_A_witness": candidate_a_witness(compact),
        "dense_kinematic": dense_kinematic_gate(compact),
        "visual_platform": visual_platform_gate(compact),
        "dynamic_30hz": dynamic_gate(compact),
        "hard_clearance_m": HARD_CLEARANCE_M,
        "preferred_clearance_m": PREFERRED_CLEARANCE_M,
        "evidence_boundary": (
            "Offline kinematics and sampled visual-CAD diagnostics only; not "
            "contact, load, friction, continuous physical clearance, stability, "
            "GPU PhysX evidence, or hardware authorization."
        ),
    }
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
