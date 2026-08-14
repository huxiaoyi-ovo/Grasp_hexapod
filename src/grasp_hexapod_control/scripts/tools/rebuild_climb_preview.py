#!/usr/bin/env python3
"""Offline compact snapshot/rebuild tool; it does not select footholds or authorize motion."""

import argparse
import copy
import hashlib
import json
import struct
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[4]
PROFILE_SCHEMA = "COMPACT_CLIMB_REBUILD_PROFILE_V1"
THRESHOLDS = {"joint_margin_rad": .02, "fixed_thigh_preferred_rad": .08,
              "support_m": .01, "active_foot_m": .015, "cross_clearance_m": .015,
              "touchdown_m": [-.0005, .002]}


def sha256(path):
    """Return a file SHA-256 without interpreting its contents."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite(value):
    """Return whether a JSON-compatible value contains only finite numbers."""
    if isinstance(value, dict): return all(finite(v) for v in value.values())
    if isinstance(value, list): return all(finite(v) for v in value)
    return not isinstance(value, float) or np.isfinite(value)


def repo_path(path):
    """Return a repository-relative model path or reject an out-of-tree path."""
    return str(Path(path).resolve().relative_to(ROOT))


def stl_vertices(path):
    """Read STL triangle vertices for a file/geometry summary only."""
    data = Path(path).read_bytes()
    if len(data) >= 84:
        count = struct.unpack("<I", data[80:84])[0]
        if 84 + count * 50 == len(data):
            return np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), offset=84)["v"].astype(float)
    points = []
    for line in data.decode("utf-8", errors="ignore").splitlines():
        words = line.split()
        if len(words) == 4 and words[0].lower() == "vertex": points.append([float(x) for x in words[1:]])
    vertices = np.asarray(points, dtype=float).reshape(-1, 3, 3)
    if not len(vertices) or not np.all(np.isfinite(vertices)): raise ValueError("model is not a finite STL triangle mesh")
    return vertices


def mesh_summary(path, transform):
    """Build an auditable mesh file and bounds summary, not a safety claim."""
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("world_from_model must be a finite 4x4 transform")
    triangles = stl_vertices(path)
    local = triangles.reshape(-1, 3)
    homogeneous = np.c_[local, np.ones(len(local))]
    world = (homogeneous @ transform.T)[:, :3]
    return {"repo_relative_path": repo_path(path), "sha256": sha256(path),
            "world_from_model": transform.tolist(), "triangle_count": int(len(triangles)),
            "local_bounds_m": [local.min(0).tolist(), local.max(0).tolist()],
            "world_bounds_m": [world.min(0).tolist(), world.max(0).tolist()]}


def load_json(path):
    """Load JSON and reject non-finite values."""
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not finite(value): raise ValueError("JSON must not contain NaN or infinity")
    return value


def snapshot(config_path, model_path, output):
    """Write an editable endpoint/profile snapshot of one compact template."""
    compact = load_json(config_path)
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(compact["xiaolan_translation"], dtype=float)
    stages = []
    for stage in compact["stages"]:
        endpoints = [{"leg": int(leg), "world_point_m": stage["anchor_knots"][-1][leg],
                      "surface_id": "unassigned"} for leg in stage["active_legs"]]
        stages.append({"name": stage["name"], "pose_end": stage["pose_end"], "active_endpoints": endpoints})
    profile = {"schema": PROFILE_SCHEMA, "version": 1, "template_sha256": sha256(config_path),
               "template_repo_relative_path": repo_path(config_path), "model": mesh_summary(model_path, transform),
               "thresholds": THRESHOLDS, "p0": {"base": compact["p0"]["base"], "anchors_world_m": compact["p0"]["anchors_world_m"]},
               "stage_names": [stage["name"] for stage in compact["stages"]], "stages": stages,
               "evidence_boundary": "Model file/geometry summaries are diagnostics only and do not establish CAD safety, contact, load, or stability."}
    if not finite(profile): raise ValueError("profile contains non-finite data")
    Path(output).write_text(json.dumps(profile, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def validate_profile(profile, compact):
    """Validate identity, order, finite values, and editable endpoint ownership."""
    if profile.get("schema") != PROFILE_SCHEMA or profile.get("version") != 1: raise ValueError("unsupported profile schema")
    if profile.get("template_sha256") != sha256(compact[0]): raise ValueError("template SHA mismatch")
    model = profile.get("model", {}); model_path = ROOT / model.get("repo_relative_path", "")
    if not model_path.is_file() or model.get("sha256") != sha256(model_path): raise ValueError("model SHA mismatch")
    transform = np.asarray(model.get("world_from_model"), dtype=float)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("world_from_model must be a finite 4x4 transform")
    names = [stage["name"] for stage in compact[1]["stages"]]
    if profile.get("stage_names") != names or [stage.get("name") for stage in profile.get("stages", [])] != names: raise ValueError("unknown stage or stage order mismatch")
    if not finite(profile): raise ValueError("profile contains non-finite data")
    return model_path


def build(template_path, profile_path, output, scope_output):
    """Apply explicit profile endpoints only; never chooses geometry, order, or timing."""
    compact = load_json(template_path); profile = load_json(profile_path)
    model_path = validate_profile(profile, (template_path, compact))
    identity = all(
        np.array_equal(np.asarray(stage["pose_end"]), np.asarray(target["pose_end"]))
        and all(np.array_equal(np.asarray(stage["anchor_knots"][-1][item["leg"]]),
                               np.asarray(item["world_point_m"]))
                for item in target["active_endpoints"])
        for stage, target in zip(compact["stages"], profile["stages"])
    )
    if identity:
        scope = {"changed_poses": [], "stage_leg_endpoints": [],
                 "propagated_coordinates": [], "boundary_max_m": 0.0,
                 "model_summary": mesh_summary(model_path, np.asarray(profile["model"]["world_from_model"], float)),
                 "warnings_unassigned_surface": [stage["name"] + ":" + str(item["leg"])
                     for stage in profile["stages"] for item in stage["active_endpoints"]
                     if item.get("surface_id") == "unassigned"]}
        Path(output).write_text(json.dumps(compact, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        Path(scope_output).write_text(json.dumps(scope, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        return
    result = copy.deepcopy(compact); previous_pose = np.r_[compact["p0"]["base"][:3], 0.0, compact["p0"]["base"][3]]; previous = np.asarray(compact["p0"]["anchors_world_m"], float)
    changed_poses = []; changed_endpoints = []; propagated = []; warnings = []
    for index, (stage, target) in enumerate(zip(result["stages"], profile["stages"])):
        stage["pose_start"] = previous_pose.tolist(); desired_pose = np.asarray(target["pose_end"], float)
        if not np.allclose(stage["pose_end"], desired_pose): changed_poses.append(stage["name"])
        stage["pose_end"] = desired_pose.tolist(); knots = np.asarray(stage["anchor_knots"], float)
        old_start, old_end = knots[0].copy(), knots[-1].copy()
        endpoint_map = {int(item["leg"]): np.asarray(item["world_point_m"], float) for item in target["active_endpoints"]}
        if set(endpoint_map) != set(stage["active_legs"]): raise ValueError("active endpoint ownership mismatch: " + stage["name"])
        for item in target["active_endpoints"]:
            if item.get("surface_id") == "unassigned": warnings.append(stage["name"] + ":" + str(item["leg"]))
        for leg in stage["active_legs"]:
            departure_indices = [0]
            if not np.allclose(previous[leg], old_start[leg]):
                for knot_index in range(1, len(knots)):
                    if not np.allclose(old_start[leg], knots[knot_index, leg]):
                        break
                    departure_indices.append(knot_index)
            delta = endpoint_map[leg] - old_end[leg]
            for knot_index in range(len(knots)): knots[knot_index, leg] += delta * knot_index / (len(knots) - 1)
            for knot_index in departure_indices:
                knots[knot_index, leg] = previous[leg]
                if knot_index:
                    propagated.append(stage["name"] + ":" + str(leg) + ":departure")
            if not np.allclose(delta, 0.0): changed_endpoints.append(stage["name"] + ":" + str(leg))
        for leg in range(6):
            if leg not in stage["active_legs"] and not np.allclose(knots[:, leg], previous[leg]):
                knots[:, leg] = previous[leg]; propagated.append(stage["name"] + ":" + str(leg))
        stage["anchor_knots"] = knots.tolist(); previous, previous_pose = knots[-1], desired_pose
    if not finite(result): raise ValueError("build produced non-finite candidate")
    boundary_values = []
    for index in range(1, len(result["stages"])):
        left = np.asarray(result["stages"][index - 1]["anchor_knots"][-1])
        right = np.asarray(result["stages"][index]["anchor_knots"][0])
        boundary_values.append(float(np.max(np.abs(left - right))))
    boundaries = max(boundary_values or [0.0])
    scope = {"changed_poses": changed_poses, "stage_leg_endpoints": changed_endpoints, "propagated_coordinates": propagated,
             "boundary_max_m": boundaries, "model_summary": mesh_summary(model_path, np.asarray(profile["model"]["world_from_model"], float)),
             "warnings_unassigned_surface": warnings}
    Path(output).write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    Path(scope_output).write_text(json.dumps(scope, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main():
    """Dispatch snapshot or build subcommands."""
    parser = argparse.ArgumentParser(); sub = parser.add_subparsers(dest="command", required=True)
    snapshot_parser = sub.add_parser("snapshot"); snapshot_parser.add_argument("--config", type=Path, required=True); snapshot_parser.add_argument("--model", type=Path, required=True); snapshot_parser.add_argument("--output", type=Path, required=True)
    build_parser = sub.add_parser("build"); build_parser.add_argument("--template", type=Path, required=True); build_parser.add_argument("--profile", type=Path, required=True); build_parser.add_argument("--output", type=Path, required=True); build_parser.add_argument("--scope-report", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "snapshot": snapshot(args.config, args.model, args.output)
    else: build(args.template, args.profile, args.output, args.scope_report)


if __name__ == "__main__": main()
