#!/usr/bin/env python3
"""Deterministically move the front candidate's rear terminal landing inward."""
import argparse
import copy
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
    MESH,
    TRANSLATION,
    load_triangles,
    require_surface_sphere_clearance,
    solve_ik,
    surface_height,
)
from climb_mode import ClimbMode  # noqa: E402
from kinematics import GraspKinematic  # noqa: E402
from optimize_climb_front import _refresh_active_base_knots  # noqa: E402

OLD_CPU_AUDIT = ROOT / "src/docs/evidence/climb_front_fast_20260923/cpu_fast_41_audit.json"
OLD_CONFIG_SHA256 = "d15bf4d9f9a8feca09ebb1290502176e539f5b56c8b751ce9f9d0161f6601898"
BACK_LEGS = (0, 3)
BACK_LANDING_X_M = 0.205
ORIGINAL_CROSSING_X_M = 0.19


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def refine_config(input_path, output_path, cpu_audit_path=OLD_CPU_AUDIT):
    input_path, output_path = Path(input_path), Path(output_path)
    source_bytes = input_path.read_bytes()
    source_sha = hashlib.sha256(source_bytes).hexdigest()
    if source_sha != OLD_CONFIG_SHA256:
        raise ValueError("input config is not the frozen pre-terminal candidate")
    config = json.loads(source_bytes)
    if config.get("stage_count") != 41 or len(config.get("stages", ())) != 41:
        raise ValueError("expected frozen 41-stage front candidate")
    if config.get("climb_orientation") != "front" or config.get("simulation_only") is not True:
        raise ValueError("expected simulation-only front candidate")

    stages = config["stages"]
    ah_index = next((i for i, stage in enumerate(stages)
                     if stage["name"] == "AH_BACK_LOW_FIRST_SEGMENT"), None)
    if ah_index is None or ah_index != len(stages) - 2 or stages[-1]["name"] != "FINAL_HOLD":
        raise ValueError("frozen AH / FINAL_HOLD tail changed")
    ah = stages[ah_index]
    hold = stages[-1]
    if ah["active_legs"] != list(BACK_LEGS) or ah["pose_curve"] != "quintic_first_segment":
        raise ValueError("AH rear-leg / pose-curve contract changed")
    if len(ah["anchor_knots"]) != 4:
        raise ValueError("AH must retain four world anchor knots")

    cpu_audit_path = Path(cpu_audit_path)
    cpu_audit_bytes = cpu_audit_path.read_bytes()
    cpu_audit = json.loads(cpu_audit_bytes)
    if cpu_audit.get("candidate_config_sha256") != source_sha:
        raise ValueError("old CPU receipt is not bound to the frozen source config")
    if cpu_audit.get("state") != "DONE" or "FINAL_HOLD" not in cpu_audit.get("boundaries", {}):
        raise ValueError("old continuous CPU receipt has no completed terminal boundary")
    source_stages = config["stages"]
    source_ah = next(stage for stage in source_stages
                     if stage["name"] == "AH_BACK_LOW_FIRST_SEGMENT")
    source_anchors = np.asarray(source_ah["anchor_knots"], dtype=float)
    old_prefix = copy.deepcopy(stages[:ah_index])
    old_ah = copy.deepcopy(ah)
    old_hold = copy.deepcopy(hold)
    old_terminal_q = np.asarray(
        cpu_audit["boundaries"]["FINAL_HOLD"]["exit_q_rad"], dtype=float
    )

    triangles = load_triangles(MESH)
    anchors = np.asarray(ah["anchor_knots"], dtype=float)
    landing = anchors[-1, list(BACK_LEGS)].copy()
    landing[:, 0] = BACK_LANDING_X_M
    for local_index, leg in enumerate(BACK_LEGS):
        landing[local_index, 2] = surface_height(
            triangles, BACK_LANDING_X_M, float(landing[local_index, 1])
        )
        require_surface_sphere_clearance(
            triangles + TRANSLATION, landing[local_index]
        )

    # Restore the original two rear air knots, then descend diagonally forward
    # from the original x=.19 crossing to the new x=.205 landing.
    anchors[1:3, list(BACK_LEGS)] = source_anchors[1:3, list(BACK_LEGS)]
    anchors[3, list(BACK_LEGS)] = landing
    ah["anchor_knots"] = anchors.tolist()
    _refresh_active_base_knots(ah)

    final_anchors = np.asarray(hold["anchor_knots"], dtype=float)
    final_anchors[:, list(BACK_LEGS)] = landing
    hold["anchor_knots"] = final_anchors.tolist()

    kinematic = GraspKinematic()
    terminal_pose = np.asarray(hold["pose_end"], dtype=float)
    terminal_q, terminal_residual = solve_ik(
        kinematic, terminal_pose, final_anchors[-1], old_terminal_q,
        tolerance_m=1e-5,
    )
    if terminal_residual > 1e-5:
        raise ValueError("refined terminal IK residual {:.9g} m exceeds 1e-5 m".format(
            terminal_residual
        ))
    config["terminal_q_rad"] = terminal_q.tolist()
    config["terminal_refinement"] = {
        "tool": "refine_climb_front_terminal.py",
        "source_config_sha256": source_sha,
        "source_cpu_audit_sha256": hashlib.sha256(cpu_audit_bytes).hexdigest(),
        "source_cpu_geometry_sha256": cpu_audit.get("geometry_sha256"),
        "source_terminal_q": "cpu_fast_41_audit.json boundaries.FINAL_HOLD.exit_q_rad",
        "rear_active_legs": list(BACK_LEGS),
        "rear_landing_world_x_m": BACK_LANDING_X_M,
        "rear_landing_world_z_m": landing[:, 2].tolist(),
        "rear_actual_apex_world_z_m": float(np.max(anchors[1:3, list(BACK_LEGS), 2])),
        "rear_crossing_world_x_m": ORIGINAL_CROSSING_X_M,
        "rear_descent": "diagonal_forward_descent",
        "rear_descent_dx_m": BACK_LANDING_X_M - ORIGINAL_CROSSING_X_M,
        "previous_geometry_sha256": cpu_audit.get("geometry_sha256"),
        "terminal_ik_residual_m": terminal_residual,
        "classification": "new terminal geometry; original retime/source hashes above remain historical",
    }

    if stages[:ah_index] != old_prefix:
        raise RuntimeError("prefix stages changed")
    for field in ("pose_start", "pose_end", "active_legs", "segment_durations_s",
                  "settle_s", "pose_curve", "anchor_curve"):
        if ah[field] != old_ah[field]:
            raise RuntimeError("AH timing/pose/active contract changed: " + field)
    source_others = [leg for leg in range(6) if leg not in BACK_LEGS]
    if not np.array_equal(np.asarray(ah["anchor_knots"])[:, source_others],
                          np.asarray(old_ah["anchor_knots"])[:, source_others]):
        raise RuntimeError("non-rear AH anchor changed")
    if not np.array_equal(np.asarray(ah["anchor_knots"])[1:3, list(BACK_LEGS)],
                          source_anchors[1:3, list(BACK_LEGS)]):
        raise RuntimeError("original rear air knots were not restored exactly")
    if (hold["pose_start"] != old_hold["pose_start"]
            or hold["pose_end"] != old_hold["pose_end"]
            or hold["segment_durations_s"] != old_hold["segment_durations_s"]):
        raise RuntimeError("FINAL_HOLD body pose or duration changed")
    ClimbMode(None)._validate_config(config)
    output_path.write_text(json.dumps(config, indent=2, allow_nan=False) + "\n")
    return config





def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cpu-audit", type=Path, default=OLD_CPU_AUDIT)
    args = parser.parse_args()
    config = refine_config(args.input, args.output, args.cpu_audit)
    print(json.dumps({
        "output": str(args.output),
        "candidate_sha256": sha256_file(args.output),
        "source_config_sha256": config["terminal_refinement"]["source_config_sha256"],
        "stage_count": config["stage_count"],
        "rear_landing_world_z_m": config["terminal_refinement"]["rear_landing_world_z_m"],
        "rear_actual_apex_world_z_m": config["terminal_refinement"]["rear_actual_apex_world_z_m"],
        "terminal_ik_residual_m": config["terminal_refinement"]["terminal_ik_residual_m"],
    }, indent=2))


if __name__ == "__main__":
    main()
