#!/usr/bin/env python3
"""Offline AH replay and exact STL patch audit for the terminal refinement."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / "src/grasp_hexapod_control/scripts"
TOOLS = SCRIPTS / "tools"
sys.path[:0] = [str(SCRIPTS), str(TOOLS)]

from build_climb_front import (  # noqa: E402
    MESH, TRANSLATION, foot_sphere_signed_gap, load_triangles, world_from_base,
)
from control import COLLISION_MARGIN, LINK_COLLISION_RADII, GraspController  # noqa: E402
from kinematics import FOOT_RADIUS, JOINT_LOWER, JOINT_UPPER  # noqa: E402
from optimize_climb_front import support_margin  # noqa: E402
from climb_mode import ClimbMode  # noqa: E402

OLD_CPU_AUDIT = ROOT / "src/docs/evidence/climb_front_fast_20260923/cpu_fast_41_audit.json"
PATCH_EDGE_X_M = 0.177920889854
PATCH_MIN_INTERIOR_X_M = 0.010
BROAD_SLOPE_NORMAL = np.array((-0.2079117, 0.0, 0.9781476), dtype=float)
BROAD_SLOPE_NORMAL /= np.linalg.norm(BROAD_SLOPE_NORMAL)
BROAD_SLOPE_PLANE_D_M = 0.1016
DT = 1.0 / 30.0


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def local_ah_replay(config_path, cpu_audit_path, output_path):
    config = json.loads(Path(config_path).read_text())
    cpu_audit = json.loads(Path(cpu_audit_path).read_text())
    config_sha = sha256_file(config_path)
    ah_index = next(i for i, stage in enumerate(config["stages"])
                    if stage["name"] == "AH_BACK_LOW_FIRST_SEGMENT")
    q = np.asarray(cpu_audit["boundaries"]["AH_BACK_LOW_FIRST_SEGMENT"]["entry_q_rad"], dtype=float)
    controller = GraspController(
        DT, enable_link_collision_check=False, climb_timeout_uses_wall_time=False
    )
    controller.enter_climb(
        q, config, start_stage_index=ah_index, end_stage_index=ah_index,
        hardware_execution=False,
    )
    mode = controller.climb_mode
    triangles = load_triangles(MESH) + TRANSLATION
    stage = config["stages"][ah_index]
    active = np.asarray(stage["active_legs"], dtype=int)
    fixed = np.asarray([leg for leg in range(6) if leg not in active], dtype=int)
    record = {
        "candidate_config_sha256": config_sha,
        "source_cpu_audit_sha256": sha256_file(cpu_audit_path),
        "stage": stage["name"],
        "dt_s": DT,
        "initial_q_source": "old cpu_fast_41_audit.json boundaries.AH_BACK_LOW_FIRST_SEGMENT.entry_q_rad",
        "ticks": 0,
        "state": mode.state,
        "max_foot_error_m": 0.0,
        "min_joint_margin_rad": float("inf"),
        "min_support_com_margin_m": float("inf"),
        "min_active_foot_sphere_gap_m": float("inf"),
        "minimum_rear_knee_proxy_gaps": {
            "LB_LM": {"proxy_gap_m": float("inf")},
            "RB_RM": {"proxy_gap_m": float("inf")},
        },
        "known_interference_window_start_s": 1.5,
        "known_interference_window_samples": [],
        "velocity_clips": 0,
        "capsule_warning_samples": [],
        "first_failure": None,
        "evidence_boundary": "AH-only seeded 30 Hz DLS replay, capsule proxy, support COM and active-foot/static-STL checks; not full-route or hardware evidence",
    }
    for tick in range(1000):
        phase_time = float(mode.phase_time)
        pose, anchors, _ = mode._stage_reference()
        if phase_time >= 1.5:
            record["known_interference_window_samples"].append({
                "tick": tick, "phase_time_s": phase_time,
                "q_rad": q.tolist(), "pose": pose.tolist(),
            })
        link_points = controller.kinematic.link_points_base(q)
        for pair_name, first_leg, second_leg in (("LB_LM", 0, 2), ("RB_RM", 3, 5)):
            distance = controller._segment_distance(
                link_points[first_leg, 1], link_points[first_leg, 2],
                link_points[second_leg, 1], link_points[second_leg, 2],
            )
            gap = float(distance - 2.0 * LINK_COLLISION_RADII[1] - COLLISION_MARGIN)
            current = record["minimum_rear_knee_proxy_gaps"][pair_name]
            if gap < current["proxy_gap_m"]:
                record["minimum_rear_knee_proxy_gaps"][pair_name] = {
                    "proxy_gap_m": gap,
                    "segment_distance_m": float(distance),
                    "tick": tick,
                    "phase_time_s": phase_time,
                    "q_rad": q.tolist(),
                    "pose": pose.tolist(),
                }
        capsule_mask = controller._link_collision_free(q)
        if not bool(np.all(capsule_mask)):
            record["capsule_warning_samples"].append({
                "tick": tick, "phase_time_s": phase_time,
                "q_rad": q.tolist(), "pose": pose.tolist(),
                "mask": np.asarray(capsule_mask, dtype=bool).tolist(),
            })
        for leg in active:
            gap = foot_sphere_signed_gap(triangles, anchors[leg])
            record["min_active_foot_sphere_gap_m"] = min(
                record["min_active_foot_sphere_gap_m"], gap
            )
        com = (world_from_base(pose)
               @ np.r_[controller.kinematic.center_of_mass_base(q), 1.0])[:3]
        com_margin = support_margin(com[:2], anchors[fixed])
        record["min_support_com_margin_m"] = min(
            record["min_support_com_margin_m"], com_margin
        )

        old_q = q.copy()
        q = controller.update(q, np.zeros(4, dtype=float))
        actual_feet = controller.kinematic.hip_to_base(
            controller.kinematic.forward(q)
        )
        foot_error = float(np.max(np.linalg.norm(
            actual_feet - controller.foot_desired_base, axis=1
        )))
        joint_margin = float(np.min(np.minimum(
            q - JOINT_LOWER, JOINT_UPPER - q
        )))
        record["max_foot_error_m"] = max(record["max_foot_error_m"], foot_error)
        record["min_joint_margin_rad"] = min(record["min_joint_margin_rad"], joint_margin)
        record["velocity_clips"] += int(controller.last_update_velocity_limit_clip_count)
        record["ticks"] = tick + 1
        if record["first_failure"] is None:
            reason = None
            if foot_error > .015:
                reason = "foot_error_gt_15mm"
            elif joint_margin <= 0.0:
                reason = "joint_margin_nonpositive"
            elif com_margin < .010:
                reason = "support_com_below_10mm"
            elif record["min_active_foot_sphere_gap_m"] < -.0005:
                reason = "active_foot_stl_gap_below_minus_0_5mm"
            elif controller.last_update_velocity_limit_clip_count:
                reason = "velocity_clip"
            elif record["capsule_warning_samples"]:
                reason = "capsule_proxy_warning"
            if reason:
                record["first_failure"] = {
                    "reason": reason, "tick": tick,
                    "phase_time_s": phase_time,
                    "foot_error_m": foot_error,
                    "joint_margin_rad": joint_margin,
                    "support_com_margin_m": com_margin,
                    "active_foot_stl_gap_m": record["min_active_foot_sphere_gap_m"],
                }
                break
        if mode.state != ClimbMode.RUNNING:
            break
    terminal_capsule = bool(np.all(controller._link_collision_free(q)))
    record["terminal_capsule_clear"] = terminal_capsule
    record["state"] = mode.state
    record["last_q_rad"] = q.tolist()
    record["passed"] = bool(
        mode.state == ClimbMode.DONE
        and record["first_failure"] is None
        and not record["capsule_warning_samples"]
        and terminal_capsule
        and record["velocity_clips"] == 0
        and record["max_foot_error_m"] <= .015
        and record["min_joint_margin_rad"] > 0.0
        and record["min_support_com_margin_m"] >= .010
        and record["min_active_foot_sphere_gap_m"] >= -.0005
    )
    Path(output_path).write_text(json.dumps(record, indent=2, allow_nan=False) + "\n")
    return record


def closest_points_on_triangles(world_triangles, point):
    """Return exact closest point on each triangle using Ericson region tests."""
    triangles = np.asarray(world_triangles, dtype=float)
    point = np.asarray(point, dtype=float)
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    ab, ac, ap = b - a, c - a, point - a
    d1 = np.einsum("ij,ij->i", ab, ap)
    d2 = np.einsum("ij,ij->i", ac, ap)
    closest = np.empty_like(a)
    done = np.zeros(len(a), dtype=bool)
    mask = (d1 <= 0.0) & (d2 <= 0.0)
    closest[mask] = a[mask]; done |= mask
    bp = point - b
    d3 = np.einsum("ij,ij->i", ab, bp)
    d4 = np.einsum("ij,ij->i", ac, bp)
    mask = (~done) & (d3 >= 0.0) & (d4 <= d3)
    closest[mask] = b[mask]; done |= mask
    vc = d1 * d4 - d3 * d2
    mask = (~done) & (vc <= 0.0) & (d1 >= 0.0) & (d3 <= 0.0)
    weight = np.zeros(len(a)); weight[mask] = d1[mask] / (d1[mask] - d3[mask])
    closest[mask] = a[mask] + weight[mask, None] * ab[mask]; done |= mask
    cp = point - c
    d5 = np.einsum("ij,ij->i", ab, cp)
    d6 = np.einsum("ij,ij->i", ac, cp)
    mask = (~done) & (d6 >= 0.0) & (d5 <= d6)
    closest[mask] = c[mask]; done |= mask
    vb = d5 * d2 - d1 * d6
    mask = (~done) & (vb <= 0.0) & (d2 >= 0.0) & (d6 <= 0.0)
    weight = np.zeros(len(a)); weight[mask] = d2[mask] / (d2[mask] - d6[mask])
    closest[mask] = a[mask] + weight[mask, None] * ac[mask]; done |= mask
    va = d3 * d6 - d5 * d4
    mask = (~done) & (va <= 0.0) & ((d4 - d3) >= 0.0) & ((d5 - d6) >= 0.0)
    weight = np.zeros(len(a))
    weight[mask] = (d4[mask] - d3[mask]) / (
        (d4[mask] - d3[mask]) + (d5[mask] - d6[mask])
    )
    closest[mask] = b[mask] + weight[mask, None] * (c[mask] - b[mask]); done |= mask
    mask = ~done
    denominator = va[mask] + vb[mask] + vc[mask]
    indices = np.flatnonzero(mask)
    regular = np.abs(denominator) > 1e-15
    if np.any(regular):
        face = indices[regular]
        v = vb[face] / denominator[regular]
        w = vc[face] / denominator[regular]
        closest[face] = a[face] + v[:, None] * ab[face] + w[:, None] * ac[face]
    if np.any(~regular):
        closest[indices[~regular]] = a[indices[~regular]]
    distances = np.linalg.norm(closest - point, axis=1)
    return closest, distances


def inspect_nearest_patch(world_triangles, point):
    triangles = np.asarray(world_triangles, dtype=float)
    closest, distances = closest_points_on_triangles(triangles, point)
    minimum = float(np.min(distances))
    tied = np.flatnonzero(distances <= minimum + 1e-10)
    result = []
    for index in tied:
        triangle = triangles[index]
        normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
        normal /= np.linalg.norm(normal)
        expected = BROAD_SLOPE_NORMAL
        alignment = float(np.dot(normal, expected))
        plane_value = float(np.dot(expected, closest[index]))
        interior_clearance = float(closest[index, 0] - PATCH_EDGE_X_M)
        in_broad_patch = bool(
            alignment >= .995
            and abs(plane_value - BROAD_SLOPE_PLANE_D_M) <= .001
            and interior_clearance >= PATCH_MIN_INTERIOR_X_M
        )
        result.append({
            "triangle_index": int(index),
            "closest_point_world_xyz_m": closest[index].tolist(),
            "distance_m": float(distances[index]),
            "signed_sphere_gap_m": float(distances[index] - FOOT_RADIUS),
            "face_normal_world": normal.tolist(),
            "normal_dot_expected_12deg": alignment,
            "plane_residual_m": plane_value - BROAD_SLOPE_PLANE_D_M,
            "clearance_inward_from_broad_patch_edge_m": interior_clearance,
            "inside_broad_patch_with_10mm_margin": in_broad_patch,
        })
    return result


def audit_final_patch(metrics_path, output_path):
    metrics = json.loads(Path(metrics_path).read_text())
    if metrics.get("final_state") != "DONE" or metrics.get("final_reason") != "none":
        raise ValueError("patch audit requires completed GPU metrics")
    row = metrics["per_stage"][-1]
    if row.get("runtime_name") != "FINAL_HOLD":
        raise ValueError("last metrics row is not FINAL_HOLD")
    source_field = "stage_start_actual_world_foot_xyz_m"
    points = np.asarray(row[source_field], dtype=float)
    triangles = load_triangles(MESH) + TRANSLATION
    legs = ("LB", "LF", "LM", "RB", "RF", "RM")
    items = []
    for leg, point in zip(legs, points):
        tied = inspect_nearest_patch(triangles, point)
        expected_normal = BROAD_SLOPE_NORMAL if leg in ("LB", "RB", "LM", "RM") else np.array((0., 0., 1.))
        for item in tied:
            item["normal_dot_expected_surface"] = float(np.dot(
                np.asarray(item["face_normal_world"], dtype=float), expected_normal
            ))
        normal_rows = [item for item in tied
                       if item["normal_dot_expected_surface"] >= .995]
        patch_rows = [item for item in normal_rows
                      if item["inside_broad_patch_with_10mm_margin"]]
        signed_gap = min(item["signed_sphere_gap_m"] for item in tied)
        items.append({
            "leg": leg,
            "actual_world_foot_center_m": point.tolist(),
            "nearest_triangle_points": tied,
            "nearest_distance_m": min(item["distance_m"] for item in tied),
            "signed_sphere_gap_m": signed_gap,
            "nearest_surface_normal_expected": expected_normal.tolist(),
            "nearest_surface_normal_pass": bool(normal_rows),
            "signed_sphere_gap_in_contact_tolerance": bool(-.0005 <= signed_gap <= .002),
            "rear_broad_patch_pass": (bool(patch_rows) if leg in ("LB", "RB") else None),
            "broad_patch_pass": (bool(patch_rows) if leg in ("LB", "RB", "LM", "RM") else None),
        })
    back_pass = all(item["rear_broad_patch_pass"] for item in items if item["leg"] in ("LB", "RB"))
    all_gap_pass = all(item["signed_sphere_gap_in_contact_tolerance"] for item in items)
    all_normal_pass = all(item["nearest_surface_normal_pass"] for item in items)
    all_broad_membership_pass = all(item["broad_patch_pass"] for item in items
                                    if item["leg"] in ("LB", "RB", "LM", "RM"))
    all_pass = bool(back_pass and all_gap_pass and all_normal_pass
                    and all_broad_membership_pass
                    and np.all(points[:, 2] > .0065))
    out = {
        "source_metrics": str(Path(metrics_path)),
        "source_metrics_candidate_config_sha256": metrics.get("candidate_config_sha256"),
        "candidate_config_sha256_from_prelaunch_manifest": json.loads(
            (ROOT / "src/docs/evidence/climb_front_terminal_20260924/gpu_start_manifest.json").read_text()
        )["candidate_config_sha256"],
        "source_field": source_field,
        "sample_timing": "C41 FINAL_HOLD entry, not the final simulation frame",
        "mesh": str(MESH.relative_to(ROOT)),
        "mesh_sha256": sha256_file(MESH),
        "broad_patch_definition": {
            "normal_world": BROAD_SLOPE_NORMAL.tolist(),
            "plane_normal_dot_position_equals_m": BROAD_SLOPE_PLANE_D_M,
            "outer_edge_world_x_m": PATCH_EDGE_X_M,
            "required_inward_edge_clearance_m": PATCH_MIN_INTERIOR_X_M,
            "normal_dot_minimum": .995,
            "plane_residual_tolerance_m": .001,
            "note": "Uses physical broad patch outer boundary, not internal triangle edges.",
        },
        "feet": items,
        "rear_patch_pass": back_pass,
        "all_six_signed_sphere_gaps_minus_0_5_to_plus_2_mm": all_gap_pass,
        "all_six_expected_normals_pass_dot_0_995": all_normal_pass,
        "LB_RB_LM_RM_broad_patch_membership_pass": all_broad_membership_pass,
        "all_six_above_initial_floor_z_0_0065m": bool(np.all(points[:, 2] > .0065)),
        "all_requested_hold_entry_geometry_checks_pass": all_pass,
        "evidence_boundary": "Exact nearest-point-to-STL geometry and patch membership at one hold-entry sample; no contact, load, friction, stability or continuous-motion proof.",
    }
    Path(output_path).write_text(json.dumps(out, indent=2, allow_nan=False) + "\n")
    return out



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--local-config", type=Path)
    parser.add_argument("--cpu-audit", type=Path, default=OLD_CPU_AUDIT)
    parser.add_argument("--local-output", type=Path)
    parser.add_argument("--patch-metrics", type=Path)
    parser.add_argument("--patch-output", type=Path)
    args = parser.parse_args()
    if args.local_config is not None:
        if args.local_output is None:
            parser.error("--local-config requires --local-output")
        result = local_ah_replay(args.local_config, args.cpu_audit, args.local_output)
        print(json.dumps({"local_replay": result}, indent=2))
        if not result["passed"]:
            raise SystemExit(5)
        return
    if args.patch_metrics is not None:
        if args.patch_output is None:
            parser.error("--patch-metrics requires --patch-output")
        result = audit_final_patch(args.patch_metrics, args.patch_output)
        print(json.dumps({"rear_patch_pass": result["rear_patch_pass"],
                          "output": str(args.patch_output)}, indent=2))
        if not result["rear_patch_pass"]:
            raise SystemExit(4)
        return
    parser.error("select --local-config or --patch-metrics")


if __name__ == "__main__":
    main()
