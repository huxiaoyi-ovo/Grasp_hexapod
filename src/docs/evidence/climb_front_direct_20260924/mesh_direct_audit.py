#!/usr/bin/env python3
"""Run only the frozen exact visual-mesh witnesses for the direct candidate."""
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
spec = importlib.util.spec_from_file_location("front_mesh_audit_helper", HELPER_PATH)
helper = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = helper
spec.loader.exec_module(helper)

from climb_mode import ClimbMode  # noqa: E402
from control import (  # noqa: E402
    COLLISION_MARGIN, LINK_COLLISION_RADII, GraspController,
)
from utils.climb_collision import default_visual_scene  # noqa: E402

CONFIG = ROOT / "src/grasp_hexapod_control/config/climb_front_direct.json"
CPU_AUDIT = HERE / "cpu_fast.json"
OUTPUT = HERE / "mesh_direct_audit.json"
CACHE = HERE / "mesh_pair_cache.json"
EXPECTED_CONFIG_SHA = "a7bcb4d00ff5afb5ec6b74d1270a2e151af2a9a59ba9c3e7c890e82251fac442"
EXPECTED_CPU_SHA = "ba9cccf8e4c76b38b9cd56128a432da24568ed745eee3877affc22aa10731ed9"
NAMES = ("lb", "lf", "lm", "rb", "rf", "rm")
BODY_STAGE = "MID_TO_019_DIRECT"
CHANGED_STAGES = (
    "BACK_TO_MINUS09", "BACK_TO_000", "BODY_LB_0055_SYNC", "RB_TO_0055",
    "BODY_LB_013_SYNC", "RB_TO_013", "MID_TO_019_DIRECT",
    "BODY_MID_0235_SYNC",
)
KNEE_PAIRS = ((0, 2), (3, 5))  # LB-LM and RB-RM
DT = 1.0 / 30.0


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def mesh_hashes(scene):
    meshes = {"body": scene.body_mesh, **scene.link_meshes,
              "xiaolan": scene.xiaolan_mesh_local}
    result = {}
    for name, mesh in meshes.items():
        if len(mesh.triangles) == 0 or not mesh.components:
            raise RuntimeError(f"empty visual mesh: {name}")
        result[name] = {
            "sha256": mesh.sha256, "triangle_count": int(len(mesh.triangles)),
            "component_count": len(mesh.components), "path": str(mesh.path),
        }
    return result


def identity(sample):
    return (json.dumps(sample["q_rad"], separators=(",", ":")),
            json.dumps(sample["base_pose"], separators=(",", ":")))


def unique_stage_samples(audit, stage_name):
    samples = [row for row in audit["tick_samples"] if row["stage"] == stage_name]
    output, seen = [], set()
    for row in samples:
        key = identity(row)
        if key not in seen:
            seen.add(key)
            output.append(row)
    return output


def knee_proxy_gap(controller, q, left, right):
    points = controller.kinematic.link_points_base(q)
    distance = controller._segment_distance(
        points[left, 1], points[left, 2], points[right, 1], points[right, 2]
    )
    return float(distance - 2.0 * LINK_COLLISION_RADII[1] - COLLISION_MARGIN)


def save_progress(document):
    OUTPUT.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")


def main():
    config_bytes = CONFIG.read_bytes()
    config_sha = hashlib.sha256(config_bytes).hexdigest()
    if config_sha != EXPECTED_CONFIG_SHA:
        raise ValueError(f"direct config SHA mismatch: {config_sha}")
    cpu_bytes = CPU_AUDIT.read_bytes()
    cpu_sha = hashlib.sha256(cpu_bytes).hexdigest()
    audit = json.loads(cpu_bytes)
    if EXPECTED_CPU_SHA is not None and cpu_sha != EXPECTED_CPU_SHA:
        raise ValueError(f"CPU source audit SHA mismatch: {cpu_sha}")
    if audit.get("config_sha256_before") != config_sha or audit.get("state") != "DONE":
        raise ValueError("CPU audit is not the successful receipt for this config SHA")
    config = json.loads(config_bytes)
    ClimbMode(None)._validate_config(config)

    helper.PAIR_CACHE_PATH = CACHE
    scene = default_visual_scene(ROOT)
    hashes = mesh_hashes(scene)
    controller = GraspController(DT, enable_link_collision_check=False,
                                 climb_timeout_uses_wall_time=False)
    document = {
        "method": "Selected exact visual STL triangle intersection queries at recorded 30 Hz CPU witnesses; conservative pair AABB filter; no whole-robot scan.",
        "candidate": {"config_sha256": config_sha, "cpu_audit_sha256": cpu_sha,
                      "cpu_audit_state": audit["state"], "cpu_ticks": audit["ticks"]},
        "visual_meshes": hashes,
        "cache_path": str(CACHE), "records": [], "completed": False,
        "boundary": "Triangle hits establish intersection at sampled poses. Clear samples do not establish continuous clearance, containment, contact, load, stability, or hardware safety.",
    }
    save_progress(document)

    def query(label, sample, left, right, proxy_gap=None):
        pair = helper.check_pair(
            scene, np.asarray(sample["q_rad"], dtype=float),
            np.asarray(sample["base_pose"], dtype=float), left, right,
        )
        row = {
            "label": label, "stage": sample["stage"], "tick": sample["tick"],
            "phase_before_s": sample["phase_before_s"],
            "q_rad": sample["q_rad"], "base_pose": sample["base_pose"],
            "pair_proxy_gap_m": proxy_gap, "pair_check": pair,
        }
        document["records"].append(row)
        document["query_count"] = len(document["records"])
        save_progress(document)
        print(json.dumps({"completed_query": label, "count": document["query_count"],
                          "mesh_collision": pair["mesh_collision"],
                          "triangle_hit": pair["triangle_hit"]}, indent=2), flush=True)
        if pair["mesh_collision"]:
            document["stopped_at_first_triangle_hit"] = row
            document["completed"] = False
            save_progress(document)
            print("STOP: exact visual STL triangle hit; remaining witnesses were not queried.", flush=True)
            return False
        return True

    # A: the first body proxy warning plus the smallest body capsule gap in M.
    m_samples = unique_stage_samples(audit, BODY_STAGE)
    if not m_samples:
        raise ValueError(f"missing recorded {BODY_STAGE} samples")
    warning = audit["stage_rows"][BODY_STAGE]["capsule_proxy_first_warning"]
    warning_sample = (next(row for row in m_samples if row["tick"] == warning["tick"])
                      if warning is not None else None)
    gap_rows = []
    for sample in m_samples:
        for row in helper.capsule_body_gaps(controller, np.asarray(sample["q_rad"]), (2, 5)):
            gap_rows.append({**row, "sample": sample})
    worst = min(gap_rows, key=lambda row: row["proxy_gap_m"])
    a_samples, a_seen = [], set()
    body_witnesses = [("worst_capsule_body_gap", worst["sample"])]
    if warning_sample is not None:
        body_witnesses.insert(0, ("body_proxy_first_warning", warning_sample))
    for source, sample in body_witnesses:
        key = identity(sample)
        if key not in a_seen:
            a_seen.add(key)
            a_samples.append((source, sample))
    for source, sample in a_samples:
        sample_gaps = helper.capsule_body_gaps(
            controller, np.asarray(sample["q_rad"]), (2, 5)
        )
        for leg in ("lm", "rm"):
            for link in ("knee", "ankle"):
                label = f"A_{source}_{leg}_{link}"
                proxy_gap = next(
                    row["proxy_gap_m"] for row in sample_gaps
                    if row["leg"] == leg and row["link"] == link
                )
                if not query(label, sample, {"leg": None, "link": "body"},
                             {"leg": leg, "link": link}, proxy_gap):
                    return

    # B: proxy minimum and its immediate recorded neighbours, for both knee pairs.
    for stage_name in CHANGED_STAGES:
        samples = unique_stage_samples(audit, stage_name)
        if not samples:
            raise ValueError(f"missing recorded samples for {stage_name}")
        for left, right in KNEE_PAIRS:
            gaps = [knee_proxy_gap(controller, np.asarray(sample["q_rad"]), left, right)
                    for sample in samples]
            minimum = int(np.argmin(gaps))
            for index in sorted(set((minimum - 1, minimum, minimum + 1))):
                if not 0 <= index < len(samples):
                    continue
                sample = samples[index]
                label = (f"B_{stage_name}_{NAMES[left]}_{NAMES[right]}_"
                         f"{'min' if index == minimum else 'neighbor'}_tick{sample['tick']}")
                if not query(label, sample,
                             {"leg": NAMES[left], "link": "knee"},
                             {"leg": NAMES[right], "link": "knee"}, gaps[index]):
                    return

    # C: AH back-middle knee pairs from the recorded phase 1.5 s through landing.
    ah_name = "AH_BACK_LOW_FIRST_SEGMENT"
    ah_stage = next(row for row in config["stages"] if row["name"] == ah_name)
    ah_duration = sum(ah_stage["segment_durations_s"])
    ah_samples = [row for row in unique_stage_samples(audit, ah_name)
                  if 1.5 <= float(row["phase_before_s"]) <= ah_duration + 1e-9]
    if not ah_samples:
        raise ValueError("no recorded AH samples from 1.5 s through landing")
    for sample in ah_samples:
        for left, right in KNEE_PAIRS:
            proxy = knee_proxy_gap(controller, np.asarray(sample["q_rad"]), left, right)
            label = f"C_{ah_name}_{NAMES[left]}_{NAMES[right]}_tick{sample['tick']}"
            if not query(label, sample,
                         {"leg": NAMES[left], "link": "knee"},
                         {"leg": NAMES[right], "link": "knee"}, proxy):
                return

    document["completed"] = True
    document["summary"] = {
        "body_warning_and_worst_pose_samples": len(a_samples),
        "changed_stage_knee_proxy_min_neighbor_queries": sum(
            row["label"].startswith("B_") for row in document["records"]
        ),
        "AH_phase_window_samples": len(ah_samples),
        "AH_knee_pair_queries": sum(row["label"].startswith("C_")
                                     for row in document["records"]),
        "real_triangle_hits": 0,
        "capsule_proxy_clear_claim": False,
        "whole_route_clearance_claim": False,
    }
    save_progress(document)
    print(json.dumps({"completed": True, "query_count": len(document["records"]),
                      "summary": document["summary"], "cpu_audit_sha256": cpu_sha,
                      "config_sha256": config_sha}, indent=2), flush=True)


if __name__ == "__main__":
    main()
