#!/usr/bin/env python3
"""Query only the inherited AH knee-pair risk window for rear-inset v1."""
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
spec = importlib.util.spec_from_file_location("rear_mesh_helper", HELPER_PATH)
helper = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = helper
spec.loader.exec_module(helper)

from climb_mode import ClimbMode  # noqa: E402
from control import COLLISION_MARGIN, LINK_COLLISION_RADII, GraspController  # noqa: E402
from utils.climb_collision import default_visual_scene  # noqa: E402

CONFIG = HERE / "rear_inset_candidate.json"
CPU_AUDIT = HERE / "cpu_rear_inset.json"
PREFIX_AUDIT = HERE / "mesh_mid_inset_audit.json"
OUTPUT = HERE / "mesh_rear_inset_audit.json"
CACHE = HERE / "mesh_pair_cache.json"
EXPECTED_CONFIG_SHA = "18a85c4a502f6df7bf482a5c07c58647523b36a8762607bc04beab53c485b9c7"
EXPECTED_PREFIX_SHA = "cd0fc1b37898d1bcda0c0809ed7a5bb77699d9154f40585e930ba6db10b75f5b"
NAMES = ("lb", "lf", "lm", "rb", "rf", "rm")
PAIRS = ((0, 2), (3, 5))
DT = 1.0 / 30.0


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def identity(sample):
    return (json.dumps(sample["q_rad"], separators=(",", ":")),
            json.dumps(sample["base_pose"], separators=(",", ":")))


def unique(rows):
    result, seen = [], set()
    for row in rows:
        key = identity(row)
        if key not in seen:
            seen.add(key)
            result.append(row)
    return result


def proxy_gap(controller, q, left, right):
    points = controller.kinematic.link_points_base(q)
    d = controller._segment_distance(points[left, 1], points[left, 2],
                                     points[right, 1], points[right, 2])
    return float(d - 2.0 * LINK_COLLISION_RADII[1] - COLLISION_MARGIN)


def mesh_hashes(scene):
    meshes = {"body": scene.body_mesh, **scene.link_meshes,
              "xiaolan": scene.xiaolan_mesh_local}
    out = {}
    for name, mesh in meshes.items():
        if len(mesh.triangles) == 0 or not mesh.components:
            raise RuntimeError(f"empty visual mesh: {name}")
        out[name] = {"sha256": mesh.sha256, "triangle_count": int(len(mesh.triangles)),
                     "component_count": len(mesh.components), "path": str(mesh.path)}
    return out


def main():
    config_bytes, cpu_bytes = CONFIG.read_bytes(), CPU_AUDIT.read_bytes()
    config_sha, cpu_sha = sha(CONFIG), sha(CPU_AUDIT)
    prefix_sha = sha(PREFIX_AUDIT)
    if config_sha != EXPECTED_CONFIG_SHA:
        raise ValueError(f"candidate SHA mismatch: {config_sha}")
    if sha(HERE / "mid_inset_v2_candidate.json") != EXPECTED_PREFIX_SHA:
        raise ValueError("accepted v2 prefix source SHA mismatch")
    config, audit = json.loads(config_bytes), json.loads(cpu_bytes)
    ClimbMode(None)._validate_config(config)
    if audit.get("config_sha256_before") != config_sha or audit.get("state") != "DONE":
        raise ValueError("CPU receipt does not bind successful rear-inset config")
    prefix = json.loads(PREFIX_AUDIT.read_bytes())
    inherited = [r for r in prefix["records"] if r["label"].startswith(("A_", "B_"))]
    if len(inherited) != 47 or prefix["candidate"]["config_sha256"] != EXPECTED_PREFIX_SHA:
        raise ValueError("expected the 47 reviewed v2 prefix mesh witnesses")

    helper.PAIR_CACHE_PATH = CACHE
    scene = default_visual_scene(ROOT)
    controller = GraspController(DT, enable_link_collision_check=False,
                                 climb_timeout_uses_wall_time=False)
    ah_stage = next(s for s in config["stages"] if s["name"] == "AH_BACK_LOW_FIRST_SEGMENT")
    duration = float(sum(ah_stage["segment_durations_s"]))
    all_ah = unique([r for r in audit["tick_samples"]
                     if r["stage"] == "AH_BACK_LOW_FIRST_SEGMENT"])
    window = [r for r in all_ah
              if 1.5 <= float(r["phase_before_s"]) <= duration + 1e-9]
    if not window:
        raise ValueError("no AH samples in the frozen phase>=1.5s landing window")

    proxy_rows = {}
    for pair in PAIRS:
        gaps = [proxy_gap(controller, np.asarray(row["q_rad"], dtype=float), *pair)
                for row in all_ah]
        minimum = int(np.argmin(gaps))
        for index in (minimum - 1, minimum, minimum + 1):
            if 0 <= index < len(all_ah):
                row = all_ah[index]
                proxy_rows[(pair, identity(row))] = {
                    "sample": row, "proxy_gap_m": gaps[index],
                    "is_minimum": index == minimum,
                }

    sample_requests = {}
    for row in window:
        for pair in PAIRS:
            sample_requests[(pair, identity(row))] = {
                "sample": row, "source": "AH phase>=1.5s through landing",
                "proxy_gap_m": proxy_gap(controller, np.asarray(row["q_rad"]), *pair),
                "is_proxy_min_neighbor": False,
            }
    for (pair, key), val in proxy_rows.items():
        previous = sample_requests.get((pair, key))
        sample_requests[(pair, key)] = {
            "sample": val["sample"],
            "source": "proxy minimum and adjacent recorded samples",
            "proxy_gap_m": val["proxy_gap_m"],
            "is_proxy_min_neighbor": True,
            "is_proxy_minimum": val["is_minimum"],
            "also_in_phase_window": previous is not None,
        }

    document = {
        "method": "Exact selected visual-STL LB-LM/RB-RM knee triangle queries from the inherited CPU replay; no dense scan.",
        "candidate": {"config_sha256": config_sha, "cpu_audit_sha256": cpu_sha,
                      "cpu_audit_state": audit["state"], "cpu_ticks": audit["ticks"]},
        "inherited_prefix": {
            "source_mesh_receipt": str(PREFIX_AUDIT), "source_receipt_sha256": prefix_sha,
            "source_config_sha256": EXPECTED_PREFIX_SHA,
            "inherited_A_B_query_count": len(inherited),
            "inherited_A_B_labels": [r["label"] for r in inherited],
            "queries_repeated": 0,
        },
        "ah_planned_reference_sphere_gap": {
            "cpu_stage_min_active_sphere_gap_m": audit["stage_rows"]["AH_BACK_LOW_FIRST_SEGMENT"]["min_active_sphere_gap_m"],
            "gate_m": -.0005,
            "pass": audit["stage_rows"]["AH_BACK_LOW_FIRST_SEGMENT"]["min_active_sphere_gap_m"] >= -.0005,
            "interpretation": "Planned reference sphere gap only; not physical contact.",
        },
        "visual_meshes": mesh_hashes(scene), "cache_path": str(CACHE),
        "records": [], "completed": False,
        "boundary": "Selected discrete exact triangles only; no continuous mesh-clearance or contact/load/stability claim.",
    }
    OUTPUT.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
    pairs = (({"leg": "lb", "link": "knee"}, {"leg": "lm", "link": "knee"}),
             ({"leg": "rb", "link": "knee"}, {"leg": "rm", "link": "knee"}))
    pair_index = {PAIRS[0]: 0, PAIRS[1]: 1}
    for (pair, _), request in sorted(sample_requests.items(),
                                     key=lambda item: (float(item[1]["sample"]["phase_before_s"]), pair_index[item[0][0]])):
        sample = request["sample"]
        left, right = pairs[pair_index[pair]]
        check = helper.check_pair(scene, np.asarray(sample["q_rad"], dtype=float),
                                  np.asarray(sample["base_pose"], dtype=float), left, right)
        record = {
            "stage": sample["stage"], "tick": sample["tick"],
            "phase_before_s": sample["phase_before_s"], "q_rad": sample["q_rad"],
            "base_pose": sample["base_pose"], "legs": [left["leg"], right["leg"]],
            "proxy_gap_m": request["proxy_gap_m"],
            "witness_source": request["source"],
            "proxy_minimum_or_neighbor": request["is_proxy_min_neighbor"],
            "pair_check": check,
        }
        document["records"].append(record)
        document["query_count"] = len(document["records"])
        OUTPUT.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
        print(json.dumps({"query": document["query_count"], "stage": sample["stage"],
                          "phase_s": sample["phase_before_s"], "pair": record["legs"],
                          "triangle_hit": check["triangle_hit"]}), flush=True)
        if check["mesh_collision"]:
            document["stopped_at_first_triangle_hit"] = record
            OUTPUT.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
            print("STOP: first exact triangle hit", flush=True)
            return
    document["completed"] = True
    document["summary"] = {
        "AH_phase_window_unique_samples": len(window),
        "phase_window_pair_queries": sum(r["witness_source"] == "AH phase>=1.5s through landing" for r in document["records"]),
        "proxy_minimum_neighbor_pair_queries": sum(r["proxy_minimum_or_neighbor"] for r in document["records"]),
        "total_queries": len(document["records"]),
        "real_triangle_hits": 0,
    }
    OUTPUT.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"completed": True, "summary": document["summary"],
                      "config_sha256": config_sha, "cpu_audit_sha256": cpu_sha,
                      "inherited_prefix_receipt_sha256": prefix_sha}, indent=2), flush=True)


if __name__ == "__main__":
    main()
