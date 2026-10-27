#!/usr/bin/env python3
"""Bounded AH knee/body exact-STL witnesses for rear body-advance v1."""
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
HELPER_PATH = ROOT / "src/docs/evidence/climb_front_fast_20260923/front_back_spacing_visual_mesh_audit.py"
spec = importlib.util.spec_from_file_location("rear_body_advance_mesh_helper", HELPER_PATH)
helper = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = helper
spec.loader.exec_module(helper)

from climb_mode import ClimbMode  # noqa: E402
from control import COLLISION_MARGIN, LINK_COLLISION_RADII, GraspController  # noqa: E402
from utils.climb_collision import default_visual_scene  # noqa: E402

CONFIG = HERE / "rear_body_advance_candidate.json"
CPU_AUDIT = HERE / "cpu_rear_body_advance.json"
PREFIX_AUDIT = HERE / "mesh_mid_inset_audit.json"
OUTPUT = HERE / "mesh_rear_body_advance_audit.json"
CACHE = HERE / "mesh_pair_cache.json"
EXPECTED_CONFIG_SHA = "4421aff133035f358e69ed33ddaa649d580750d868f2318736efc17e797bf60c"
EXPECTED_PREFIX_SHA = "cd0fc1b37898d1bcda0c0809ed7a5bb77699d9154f40585e930ba6db10b75f5b"
NAMES = ("lb", "lf", "lm", "rb", "rf", "rm")
PAIRS = ((0, 2), (3, 5))
DT = 1.0 / 30.0


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def identity(row):
    return (json.dumps(row["q_rad"], separators=(",", ":")),
            json.dumps(row["base_pose"], separators=(",", ":")))


def unique(rows):
    out, seen = [], set()
    for row in rows:
        key = identity(row)
        if key not in seen:
            seen.add(key)
            out.append(row)
    return out


def knee_gap(controller, q, left, right):
    p = controller.kinematic.link_points_base(q)
    d = controller._segment_distance(p[left, 1], p[left, 2], p[right, 1], p[right, 2])
    return float(d - 2 * LINK_COLLISION_RADII[1] - COLLISION_MARGIN)


def scene_mesh_hashes(scene):
    meshes = {"body": scene.body_mesh, **scene.link_meshes,
              "xiaolan": scene.xiaolan_mesh_local}
    out = {}
    for name, m in meshes.items():
        if len(m.triangles) == 0 or not m.components:
            raise RuntimeError(f"empty visual mesh {name}")
        out[name] = {"sha256": m.sha256, "triangle_count": int(len(m.triangles)),
                     "component_count": len(m.components), "path": str(m.path)}
    return out


def main():
    config_bytes, cpu_bytes = CONFIG.read_bytes(), CPU_AUDIT.read_bytes()
    config_sha, cpu_sha = sha(CONFIG), sha(CPU_AUDIT)
    if config_sha != EXPECTED_CONFIG_SHA:
        raise ValueError(f"candidate SHA mismatch {config_sha}")
    config, audit = json.loads(config_bytes), json.loads(cpu_bytes)
    ClimbMode(None)._validate_config(config)
    if audit.get("state") != "DONE" or audit.get("config_sha256_before") != config_sha:
        raise ValueError("successful CPU receipt does not bind candidate")
    prefix_sha = sha(PREFIX_AUDIT)
    if sha(HERE / "mid_inset_v2_candidate.json") != EXPECTED_PREFIX_SHA:
        raise ValueError("v2 prefix candidate SHA mismatch")
    prefix = json.loads(PREFIX_AUDIT.read_bytes())
    inherited = [r for r in prefix["records"] if r["label"].startswith(("A_", "B_"))]
    if len(inherited) != 47 or prefix["candidate"]["config_sha256"] != EXPECTED_PREFIX_SHA:
        raise ValueError("required inherited prefix receipt does not bind v2")

    helper.PAIR_CACHE_PATH = CACHE
    scene = default_visual_scene(ROOT)
    mesh_hashes = scene_mesh_hashes(scene)
    controller = GraspController(DT, enable_link_collision_check=False,
                                 climb_timeout_uses_wall_time=False)
    stage = next(s for s in config["stages"] if s["name"] == "AH_BACK_LOW_FIRST_SEGMENT")
    duration = float(sum(stage["segment_durations_s"]))
    ah = unique([r for r in audit["tick_samples"] if r["stage"] == stage["name"]])
    window = [r for r in ah if 1.5 <= float(r["phase_before_s"]) <= duration + 1e-9]
    if not window:
        raise ValueError("AH risk window contains no samples")

    requests = {}
    for pair in PAIRS:
        gaps = [knee_gap(controller, np.asarray(r["q_rad"]), *pair) for r in ah]
        idx_min = int(np.argmin(gaps))
        for idx in (idx_min - 1, idx_min, idx_min + 1):
            if 0 <= idx < len(ah):
                r = ah[idx]
                requests[(pair, identity(r))] = {
                    "sample": r, "proxy_gap_m": gaps[idx],
                    "source": "whole-AH proxy minimum and adjacent samples",
                    "minimum_or_neighbor": True,
                }
        for r, gap in zip(ah, gaps):
            if r not in window:
                continue
            key = (pair, identity(r))
            requests.setdefault(key, {"sample": r, "proxy_gap_m": gap,
                                      "source": "AH phase>=1.5s through landing",
                                      "minimum_or_neighbor": False})

    body_worst = None
    for r in ah:
        gaps = helper.capsule_body_gaps(controller, np.asarray(r["q_rad"]), (2, 5))
        for gap in gaps:
            if body_worst is None or gap["proxy_gap_m"] < body_worst["proxy_gap_m"]:
                body_worst = {**gap, "sample": r}
    if body_worst is None:
        raise ValueError("AH body-middle capsule witness missing")

    document = {
        "method": "Selected exact visual-STL knee pairs over the AH late window and proxy minima; plus body-to-middle links at the worst AH proxy pose.",
        "candidate": {"config_sha256": config_sha, "cpu_audit_sha256": cpu_sha,
                      "cpu_audit_state": audit["state"], "cpu_ticks": audit["ticks"]},
        "inherited_prefix": {"source_receipt_sha256": prefix_sha,
                             "source_config_sha256": EXPECTED_PREFIX_SHA,
                             "inherited_A_B_count": len(inherited),
                             "queries_repeated": 0},
        "planned_AH_active_sphere_gap_m": audit["stage_rows"][stage["name"]]["min_active_sphere_gap_m"],
        "planned_sphere_gap_gate_m": -.0005,
        "visual_meshes": mesh_hashes, "cache_path": str(CACHE),
        "body_middle_worst_proxy": {k: v for k, v in body_worst.items() if k != "sample"},
        "records": [], "completed": False,
        "boundary": "Selected discrete exact triangles only; no continuous self-collision or contact/load/stability claim.",
    }
    OUTPUT.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
    pair_names = (({"leg": "lb", "link": "knee"}, {"leg": "lm", "link": "knee"}),
                  ({"leg": "rb", "link": "knee"}, {"leg": "rm", "link": "knee"}))
    pair_index = {PAIRS[0]: 0, PAIRS[1]: 1}
    for (pair, _), item in sorted(requests.items(), key=lambda z: (z[1]["sample"]["phase_before_s"], pair_index[z[0][0]])):
        sample = item["sample"]
        left, right = pair_names[pair_index[pair]]
        check = helper.check_pair(scene, np.asarray(sample["q_rad"]),
                                  np.asarray(sample["base_pose"]), left, right)
        row = {"stage": stage["name"], "tick": sample["tick"],
               "phase_before_s": sample["phase_before_s"], "q_rad": sample["q_rad"],
               "base_pose": sample["base_pose"], "legs": [left["leg"], right["leg"]],
               "capsule_proxy_gap_m": item["proxy_gap_m"], "witness_source": item["source"],
               "proxy_minimum_or_neighbor": item["minimum_or_neighbor"], "pair_check": check}
        document["records"].append(row)
        document["query_count"] = len(document["records"])
        OUTPUT.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
        print(json.dumps({"query": document["query_count"], "phase_s": row["phase_before_s"],
                          "pair": row["legs"], "hit": check["triangle_hit"]}), flush=True)
        if check["mesh_collision"]:
            document["stopped_at_first_triangle_hit"] = row
            OUTPUT.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
            return

    sample = body_worst["sample"]
    for leg in ("lm", "rm"):
        for link in ("knee", "ankle"):
            left, right = {"leg": None, "link": "body"}, {"leg": leg, "link": link}
            check = helper.check_pair(scene, np.asarray(sample["q_rad"]),
                                      np.asarray(sample["base_pose"]), left, right)
            row = {"stage": stage["name"], "tick": sample["tick"],
                   "phase_before_s": sample["phase_before_s"], "q_rad": sample["q_rad"],
                   "base_pose": sample["base_pose"], "legs": ["body", leg],
                   "link_pair": ["body", link], "capsule_proxy_gap_m": body_worst["proxy_gap_m"],
                   "witness_source": "worst body-middle capsule proxy over entire AH stage",
                   "pair_check": check}
            document["records"].append(row)
            document["query_count"] = len(document["records"])
            OUTPUT.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
            print(json.dumps({"query": document["query_count"], "body_pair": [leg, link],
                              "phase_s": row["phase_before_s"], "hit": check["triangle_hit"]}), flush=True)
            if check["mesh_collision"]:
                document["stopped_at_first_triangle_hit"] = row
                OUTPUT.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
                return

    document["completed"] = True
    document["summary"] = {
        "AH_late_window_unique_samples": len(window),
        "knee_pair_queries": sum("link_pair" not in r for r in document["records"]),
        "body_middle_pair_queries": sum("link_pair" in r for r in document["records"]),
        "total_queries": len(document["records"]), "real_triangle_hits": 0,
        "planned_AH_active_sphere_gap_pass": document["planned_AH_active_sphere_gap_m"] >= -.0005,
    }
    OUTPUT.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"completed": True, "summary": document["summary"],
                      "config_sha256": config_sha, "cpu_sha256": cpu_sha}, indent=2), flush=True)


if __name__ == "__main__":
    main()
