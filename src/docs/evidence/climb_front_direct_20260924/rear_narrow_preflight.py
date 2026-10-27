#!/usr/bin/env python3
"""Check the known AH LB-LM/RB-RM knee pairs at two bounded poses."""
import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
SCRIPTS = ROOT / "src/grasp_hexapod_control/scripts"
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / "tools")]
HELPER = ROOT / "src/docs/evidence/climb_front_fast_20260923/front_back_spacing_visual_mesh_audit.py"
spec = importlib.util.spec_from_file_location("rear_narrow_mesh_helper", HELPER)
mesh = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mesh
spec.loader.exec_module(mesh)

from build_climb_front import MESH, TRANSLATION, load_triangles, solve_ik, base_points  # noqa: E402
from climb_mode import ClimbMode  # noqa: E402
from kinematics import GraspKinematic, JOINT_LOWER, JOINT_UPPER  # noqa: E402
from utils.climb_collision import default_visual_scene  # noqa: E402

CONFIG = HERE / "rear_narrow_candidate.json"
OLD_HIT = HERE / "mesh_rear_inset_audit.json"
OUTPUT = HERE / "rear_narrow_preflight.json"
CACHE = HERE / "mesh_pair_cache.json"
EXPECTED_CONFIG_SHA = "ae47e530a107d96b8a387d1b8c50abb44a9da4c0525652ed9ff701307d3618d1"
PAIRS = (({"leg": "lb", "link": "knee"}, {"leg": "lm", "link": "knee"}),
         ({"leg": "rb", "link": "knee"}, {"leg": "rm", "link": "knee"}))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    config_bytes, old_bytes = CONFIG.read_bytes(), OLD_HIT.read_bytes()
    config_sha = hashlib.sha256(config_bytes).hexdigest()
    if config_sha != EXPECTED_CONFIG_SHA:
        raise ValueError(f"candidate SHA mismatch: {config_sha}")
    config = json.loads(config_bytes)
    old = json.loads(old_bytes)
    hit = old.get("stopped_at_first_triangle_hit")
    if not hit:
        raise ValueError("preserved rear-inset mesh receipt has no known hit")
    old_q = np.asarray(hit["q_rad"], dtype=float)
    ah_idx = next(i for i, stage in enumerate(config["stages"])
                  if stage["name"] == "AH_BACK_LOW_FIRST_SEGMENT")
    mode = ClimbMode(None)
    mode.config = config
    mode.stage_index = ah_idx
    mode.phase_time = 1.9333333333333333
    mid_pose, mid_anchors, _ = mode._stage_reference()
    kin = GraspKinematic()
    mid_q, mid_residual = solve_ik(kin, mid_pose, mid_anchors, old_q, tolerance_m=1e-5)
    mid_margin = float(np.min(np.minimum(mid_q - JOINT_LOWER, JOINT_UPPER - mid_q)))
    if mid_residual > .004 or mid_margin <= 0.0:
        raise ValueError(f"first-hit-phase seeded IK failed: residual={mid_residual:.6f}m, margin={mid_margin:.6f}rad")

    hold = next(stage for stage in config["stages"] if stage["name"] == "FINAL_HOLD")
    terminal_pose = np.asarray(hold["pose_end"], dtype=float)
    terminal_anchors = np.asarray(hold["anchor_knots"][-1], dtype=float)
    terminal_seed = np.asarray(config["terminal_q_rad"], dtype=float)
    terminal_q, terminal_residual = solve_ik(
        kin, terminal_pose, terminal_anchors, terminal_seed, tolerance_m=1e-5
    )
    terminal_margin = float(np.min(np.minimum(terminal_q - JOINT_LOWER, JOINT_UPPER - terminal_q)))
    if terminal_residual > .004 or terminal_margin <= 0.0:
        raise ValueError(f"terminal seeded IK failed: residual={terminal_residual:.6f}m, margin={terminal_margin:.6f}rad")

    scene = default_visual_scene(ROOT)
    mesh.PAIR_CACHE_PATH = CACHE
    records = []
    document = {
        "method": "Two-pose seeded IK plus exact selected visual-STL knee triangle queries; no trajectory or broad scan.",
        "candidate_config_sha256": config_sha,
        "source_rear_inset_mesh_receipt_sha256": sha(OLD_HIT),
        "source_known_hit": {"stage": hit["stage"], "tick": hit["tick"],
                             "phase_before_s": hit["phase_before_s"],
                             "legs": hit["legs"], "pair_check": hit["pair_check"]},
        "poses": {
            "updated_prior_hit_phase": {
                "stage": "AH_BACK_LOW_FIRST_SEGMENT", "phase_time_s": mode.phase_time,
                "q_seed_from_prior_hit": old_q.tolist(), "q_rad": mid_q.tolist(),
                "base_pose": mid_pose.tolist(), "world_anchors_m": mid_anchors.tolist(),
                "ik_max_foot_residual_m": mid_residual, "min_joint_margin_rad": mid_margin,
            },
            "terminal": {
                "stage": "FINAL_HOLD entry", "q_rad": terminal_q.tolist(),
                "base_pose": terminal_pose.tolist(), "world_anchors_m": terminal_anchors.tolist(),
                "ik_max_foot_residual_m": terminal_residual,
                "min_joint_margin_rad": terminal_margin,
            },
        },
        "visual_meshes": {"xiaolan_sha256": hashlib.sha256(MESH.read_bytes()).hexdigest()},
        "records": [], "completed": False,
        "boundary": "Two static IK poses and specified knee pairs only; a clear result does not establish path clearance or contact/load/stability.",
    }
    OUTPUT.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
    for pose_name, q, pose in (("updated_prior_hit_phase", mid_q, mid_pose),
                               ("terminal", terminal_q, terminal_pose)):
        for left, right in PAIRS:
            check = mesh.check_pair(scene, q, pose, left, right)
            row = {"pose": pose_name, "legs": [left["leg"], right["leg"]],
                   "q_rad": q.tolist(), "base_pose": pose.tolist(), "pair_check": check}
            records.append(row)
            document["records"] = records
            document["query_count"] = len(records)
            OUTPUT.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
            print(json.dumps({"pose": pose_name, "pair": row["legs"],
                              "triangle_hit": check["triangle_hit"]}), flush=True)
            if check["mesh_collision"]:
                document["stopped_at_first_triangle_hit"] = row
                OUTPUT.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
                return
    document["completed"] = True
    document["summary"] = {"exact_queries": len(records), "triangle_hits": 0,
                           "all_two_pose_pair_checks_clear": True}
    OUTPUT.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"completed": True, "query_count": len(records),
                      "config_sha256": config_sha, "output": str(OUTPUT)}, indent=2))


if __name__ == "__main__":
    main()
