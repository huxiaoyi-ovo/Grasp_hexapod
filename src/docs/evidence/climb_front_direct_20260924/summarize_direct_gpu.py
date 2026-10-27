#!/usr/bin/env python3
"""Bind the direct visible GPU receipt and exact FINAL_HOLD-entry STL check."""
import hashlib
import argparse
import importlib.util
import json
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
SCRIPTS = ROOT / "src/grasp_hexapod_control/scripts"
TOOLS = SCRIPTS / "tools"
sys.path[:0] = [str(SCRIPTS), str(TOOLS)]
TERMINAL_AUDIT = ROOT / "src/docs/evidence/climb_front_terminal_20260924/front_terminal_audit.py"
spec = importlib.util.spec_from_file_location("terminal_patch_helpers", TERMINAL_AUDIT)
patch = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = patch
spec.loader.exec_module(patch)

from build_climb_front import MESH, TRANSLATION, load_triangles  # noqa: E402
from kinematics import FOOT_RADIUS  # noqa: E402

CONFIG = ROOT / "src/grasp_hexapod_control/config/climb_front_direct.json"
MANIFEST = HERE / "gpu_start_manifest.json"
CONSOLE = HERE / "gpu_console.log"
METRICS = HERE / "gpu_metrics.json"
SUMMARY = HERE / "gpu_summary.json"
PATCH_OUT = HERE / "final_actual_foot_stl_audit.json"
EXPECTED_CONFIG_SHA = "a7bcb4d00ff5afb5ec6b74d1270a2e151af2a9a59ba9c3e7c890e82251fac442"
CPU_AUDIT = HERE / "cpu_fast.json"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    global CONFIG, MANIFEST, CONSOLE, METRICS, SUMMARY, PATCH_OUT
    global EXPECTED_CONFIG_SHA, CPU_AUDIT
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--manifest", type=Path, default=MANIFEST)
    parser.add_argument("--console", type=Path, default=CONSOLE)
    parser.add_argument("--metrics", type=Path, default=METRICS)
    parser.add_argument("--summary", type=Path, default=SUMMARY)
    parser.add_argument("--patch-out", type=Path, default=PATCH_OUT)
    parser.add_argument("--expected-config-sha", default=EXPECTED_CONFIG_SHA)
    parser.add_argument("--cpu-audit", type=Path, default=CPU_AUDIT)
    parser.add_argument("--mid-stage", default=None)
    args = parser.parse_args()
    CONFIG, MANIFEST, CONSOLE, METRICS = args.config, args.manifest, args.console, args.metrics
    SUMMARY, PATCH_OUT = args.summary, args.patch_out
    EXPECTED_CONFIG_SHA, CPU_AUDIT = args.expected_config_sha, args.cpu_audit
    manifest = json.loads(MANIFEST.read_bytes())
    metrics = json.loads(METRICS.read_bytes())
    log = CONSOLE.read_text(errors="replace")
    config_sha = sha(CONFIG)
    if config_sha != EXPECTED_CONFIG_SHA or manifest["candidate_config_sha256"] != config_sha:
        raise ValueError("direct config changed since the frozen prelaunch manifest")
    if metrics.get("final_state") not in ("DONE", "FAILED"):
        raise ValueError("GPU metrics are missing a terminal state")

    rows = metrics["per_stage"]
    final = rows[-1]
    if final.get("runtime_name") != "FINAL_HOLD":
        raise ValueError("last GPU metrics row is not FINAL_HOLD")
    global_metrics = {
        "stage_count": len(rows),
        "total_simulated_control_time_s": float(sum(r["simulated_duration_s"] for r in rows)),
        "max_kinematic_foot_target_error_m": float(max(r["max_kinematic_foot_target_error_m"] for r in rows)),
        "max_root_position_error_m": float(max(r["max_root_position_error_m"] for r in rows)),
        "max_world_foot_anchor_error_m": float(max(r["max_world_foot_anchor_error_m"] for r in rows)),
        "max_support_foot_world_drift_m": float(max(r["max_support_foot_world_drift_m"] for r in rows)),
        "max_root_attitude_error_deg": float(max(r["max_root_attitude_error_deg"] for r in rows)),
        "min_joint_limit_margin_rad": float(min(r["min_joint_limit_margin_rad"] for r in rows)),
        "max_controller_target_joint_speed_rad_s": float(max(r["max_controller_target_joint_speed_rad_s"] for r in rows)),
        "velocity_limit_clip_count": int(sum(r["velocity_limit_clip_count"] for r in rows)),
        "FINAL_HOLD_entry_root_error_m": float(final["max_root_position_error_m"]),
        "FINAL_HOLD_entry_actual_world_foot_center_m": final["stage_start_actual_world_foot_xyz_m"],
        "FINAL_HOLD_end_foot_target_error_m": float(final["end_foot_target_error_m"]),
        "FINAL_HOLD_end_root_position_error_m": float(final["end_root_position_error_m"]),
        "FINAL_HOLD_support_drift_m": float(final["max_support_foot_world_drift_m"]),
    }
    gate_pattern = re.compile(
        r"\[climb\] preview gate ([^:]+): value=([^ ]+) threshold=([^ ]+) (PASS|FAIL)"
    )
    gates = [{"name": m.group(1), "value": m.group(2),
              "threshold": m.group(3), "result": m.group(4)}
             for m in gate_pattern.finditer(log)]
    verdict_match = re.search(r"\[climb\] PREVIEW VERDICT: ([^\n]+)", log)
    if not gates or verdict_match is None:
        raise ValueError("GPU console is missing the five-gate preview verdict")

    triangles = load_triangles(MESH) + TRANSLATION
    legs = ("LB", "LF", "LM", "RB", "RF", "RM")
    points = np.asarray(final["stage_start_actual_world_foot_xyz_m"], dtype=float)
    feet = []
    for leg, point in zip(legs, points):
        expected_normal = (patch.BROAD_SLOPE_NORMAL if leg in ("LB", "RB", "LM", "RM")
                           else np.array((0., 0., 1.)))
        tied = patch.inspect_nearest_patch(triangles, point)
        for item in tied:
            item["normal_dot_expected_surface"] = float(np.dot(
                np.asarray(item["face_normal_world"], dtype=float), expected_normal
            ))
        gap = min(item["signed_sphere_gap_m"] for item in tied)
        normal_pass = any(item["normal_dot_expected_surface"] >= .995 for item in tied)
        broad = [item for item in tied
                 if item["normal_dot_expected_surface"] >= .995
                 and item["inside_broad_patch_with_10mm_margin"]]
        feet.append({
            "leg": leg, "actual_world_foot_center_m": point.tolist(),
            "nearest_triangle_points": tied,
            "nearest_distance_m": float(min(item["distance_m"] for item in tied)),
            "signed_sphere_gap_m": float(gap),
            "nearest_surface_normal_expected": expected_normal.tolist(),
            "nearest_surface_normal_pass_dot_0_995": bool(normal_pass),
            "signed_sphere_gap_pass_minus_0_5_to_plus_2_mm": bool(-.0005 <= gap <= .002),
            "broad_patch_inside_10mm_margin": (bool(broad) if leg in ("LB", "RB", "LM", "RM") else None),
        })
    rear_pass = all(row["broad_patch_inside_10mm_margin"]
                    for row in feet if row["leg"] in ("LB", "RB"))
    all_gap = all(row["signed_sphere_gap_pass_minus_0_5_to_plus_2_mm"] for row in feet)
    all_normals = all(row["nearest_surface_normal_pass_dot_0_995"] for row in feet)
    mid_rear_patch = all(row["broad_patch_inside_10mm_margin"]
                         for row in feet if row["leg"] in ("LB", "RB", "LM", "RM"))
    patch_receipt = {
        "runtime_stage": "C21 FINAL_HOLD entry",
        "source_field": "stage_start_actual_world_foot_xyz_m",
        "sample_timing": "actual simulator foot centers at FINAL_HOLD entry; not the last frame",
        "candidate_config_sha256": config_sha,
        "gpu_start_manifest_sha256": sha(MANIFEST),
        "gpu_metrics_sha256": sha(METRICS),
        "mesh": str(MESH.relative_to(ROOT)), "mesh_sha256": sha(MESH),
        "broad_patch_definition": {
            "normal_world": patch.BROAD_SLOPE_NORMAL.tolist(),
            "plane_normal_dot_position_equals_m": patch.BROAD_SLOPE_PLANE_D_M,
            "outer_edge_world_x_m": patch.PATCH_EDGE_X_M,
            "required_inward_edge_clearance_m": patch.PATCH_MIN_INTERIOR_X_M,
            "normal_dot_minimum": .995, "plane_residual_tolerance_m": .001,
        },
        "feet": feet,
        "all_six_signed_gaps_pass": all_gap,
        "all_six_expected_normals_pass": all_normals,
        "LB_RB_rear_broad_patch_pass_10mm": bool(rear_pass),
        "LB_RB_LM_RM_broad_patch_pass_10mm": bool(mid_rear_patch),
        "all_six_above_initial_floor_z_0_0065m": bool(np.all(points[:, 2] > .0065)),
        "acceptance_pass": bool(all_gap and all_normals and rear_pass and mid_rear_patch
                                 and np.all(points[:, 2] > .0065)),
        "boundary": "Exact nearest-triangle foot geometry at one simulator hold-entry sample only; not contact/load/stability or continuous route clearance.",
    }
    if args.mid_stage:
        by_stage = {row["runtime_name"]: row for row in rows}
        if args.mid_stage not in by_stage:
            raise ValueError(f"GPU metrics lack required stage {args.mid_stage}")
        mid_metric = by_stage[args.mid_stage]
        mid_points = np.asarray(mid_metric["stage_start_actual_world_foot_xyz_m"], dtype=float)
        mid_feet = []
        for leg in ("LM", "RM"):
            point = mid_points[legs.index(leg)]
            tied = patch.inspect_nearest_patch(triangles, point)
            for item in tied:
                item["normal_dot_expected_surface"] = float(np.dot(
                    np.asarray(item["face_normal_world"], dtype=float), patch.BROAD_SLOPE_NORMAL
                ))
            gap = min(item["signed_sphere_gap_m"] for item in tied)
            normal_pass = any(item["normal_dot_expected_surface"] >= .995 for item in tied)
            broad = any(item["normal_dot_expected_surface"] >= .995
                        and item["inside_broad_patch_with_10mm_margin"] for item in tied)
            mid_feet.append({
                "leg": leg, "actual_world_foot_center_m": point.tolist(),
                "nearest_triangle_points": tied,
                "signed_sphere_gap_m": float(gap),
                "signed_sphere_gap_pass_minus_0_5_to_plus_2_mm": bool(-.0005 <= gap <= .002),
                "nearest_surface_normal_expected": patch.BROAD_SLOPE_NORMAL.tolist(),
                "nearest_surface_normal_pass_dot_0_995": bool(normal_pass),
                "broad_patch_inside_10mm_margin": bool(broad),
            })
        mid_receipt = {
            "runtime_stage": args.mid_stage,
            "source_field": "stage_start_actual_world_foot_xyz_m",
            "sample_timing": f"actual simulator LM/RM feet at {args.mid_stage} entry",
            "entry_root_position_error_m": mid_metric["max_root_position_error_m"],
            "entry_max_support_foot_drift_m": mid_metric["max_support_foot_world_drift_m"],
            "feet": mid_feet,
            "all_checked_gaps_pass": all(f["signed_sphere_gap_pass_minus_0_5_to_plus_2_mm"] for f in mid_feet),
            "all_checked_normals_pass": all(f["nearest_surface_normal_pass_dot_0_995"] for f in mid_feet),
            "LM_RM_broad_patch_pass_10mm": all(f["broad_patch_inside_10mm_margin"] for f in mid_feet),
        }
        mid_receipt["acceptance_pass"] = bool(
            mid_receipt["all_checked_gaps_pass"] and mid_receipt["all_checked_normals_pass"]
            and mid_receipt["LM_RM_broad_patch_pass_10mm"]
        )
        patch_receipt["first_mid_contact_entry"] = mid_receipt
        patch_receipt["acceptance_pass"] = bool(
            patch_receipt["acceptance_pass"] and mid_receipt["acceptance_pass"]
        )
    PATCH_OUT.write_text(json.dumps(patch_receipt, indent=2, allow_nan=False) + "\n")

    summary = {
        "candidate_config_sha256": config_sha,
        "prelaunch_manifest_sha256": sha(MANIFEST),
        "gpu_metrics_sha256": sha(METRICS), "gpu_console_sha256": sha(CONSOLE),
        "cpu_audit_sha256": sha(CPU_AUDIT),
        "pid": manifest.get("pid"), "display": manifest.get("display"),
        "command": manifest["command"], "runtime_sha256": manifest["runtime_sha256"],
        "rates_hz": manifest["rates_hz"], "preview_flags": manifest["preview_flags"],
        "final_state": metrics["final_state"], "final_reason": metrics.get("final_reason"),
        "runtime_summary": global_metrics,
        "preview_gates": gates,
        "preview_verdict": verdict_match.group(1).strip(),
        "terminal_patch_acceptance_pass": patch_receipt["acceptance_pass"],
        "per_stage_root_and_support_drift": {
            row["runtime_name"]: {
                "max_root_position_error_m": row["max_root_position_error_m"],
                "max_root_attitude_error_deg": row["max_root_attitude_error_deg"],
                "max_support_foot_world_drift_m": row["max_support_foot_world_drift_m"],
            } for row in rows
        },
        "viewer_process_status": "run_sim exited after built-in terminal hold; no duplicate run",
        "evidence_boundary": "One visible Isaac GPU simulation and static STL samples at named stage entries; diagnostics only.",
    }
    SUMMARY.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"summary": str(SUMMARY), "patch_receipt": str(PATCH_OUT),
                      "final_state": metrics["final_state"],
                      "preview_verdict": summary["preview_verdict"],
                      "patch_acceptance_pass": patch_receipt["acceptance_pass"],
                      "rear_patch_pass": rear_pass,
                      "root_end_error_m": global_metrics["FINAL_HOLD_end_root_position_error_m"]}, indent=2))


if __name__ == "__main__":
    main()
