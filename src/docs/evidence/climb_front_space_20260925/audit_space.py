#!/usr/bin/env python3
"""Bounded CPU-sample/STL audit for frozen front-space v6.

No geometry search, dense sweep, GPU run, or hardware inference is performed.
"""
import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "src/grasp_hexapod_control/scripts"
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / "tools")]
from build_climb_front import (  # noqa: E402
    MESH, TRANSLATION, foot_sphere_signed_gap, load_triangles, support_margin,
    surface_height, world_from_base,
)
from climb_mode import ClimbMode  # noqa: E402
from control import GraspController  # noqa: E402
from kinematics import GraspKinematic  # noqa: E402
from utils.climb_collision import LINK_NAMES, default_visual_scene  # noqa: E402

HERE = Path(__file__).resolve().parent
CONFIG_PATH = ROOT / "src/grasp_hexapod_control/config/climb_front_space.json"
CPU_PATH = HERE / "cpu_space_v6.json"
RUNTIME_PATH = SCRIPTS / "climb_mode.py"
OUT_PATH = HERE / "space_audit.json"
CACHE_PATH = HERE / "mesh_pair_cache.json"
PAIR_HELPER_PATH = ROOT / "src/docs/evidence/climb_front_fast_20260923/front_back_spacing_visual_mesh_audit.py"
TERMINAL_HELPER_PATH = ROOT / "src/docs/evidence/climb_front_terminal_20260924/front_terminal_audit.py"
MESH_DIR = ROOT / "src/grasp_hexapod_description/meshes"
NAMES = ("LB", "LF", "LM", "RB", "RF", "RM")
DT = 1.0 / 30.0
WINDOWS = {
    "C8-C11": ("MID_GROUND_TO_080", "FRONT_TO_034", "BODY_TO_105", "BACK_TO_055"),
    "C12-C13": ("MID_PLATFORM_285", "FRONT_TO_0400"),
    "C14-C18": ("RB_GROUND_TO_155", "LB_GROUND_TO_155", "FRONT_TO_0480", "RB_PLATFORM_225", "LB_PLATFORM_225"),
}
PAIR_DEFS = (
    ((0, 2), ({"leg": "lb", "link": "knee"}, {"leg": "lm", "link": "knee"}), "LB-LM"),
    ((3, 5), ({"leg": "rb", "link": "knee"}, {"leg": "rm", "link": "knee"}), "RB-RM"),
    ((2, 1), ({"leg": "lm", "link": "knee"}, {"leg": "lf", "link": "knee"}), "LM-LF"),
    ((5, 4), ({"leg": "rm", "link": "knee"}, {"leg": "rf", "link": "knee"}), "RM-RF"),
)
EXPECTED_CONFIG_SHA = "321a1b762d99659be988defa5bfdc27ae9672fc215bb351db9260a8cfd9bc333"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def identity(row):
    return (json.dumps(row["q_rad"], separators=(",", ":")),
            json.dumps(row["base_pose"], separators=(",", ":")))


def json_write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def mesh_hashes(scene):
    meshes = {"body": scene.body_mesh, **scene.link_meshes,
              "xiaolan": scene.xiaolan_mesh_local}
    return {name: {"sha256": mesh.sha256, "triangle_count": int(len(mesh.triangles)),
                   "component_count": len(mesh.components), "path": str(mesh.path)}
            for name, mesh in meshes.items()}


def planned_at(mode, stage_index, phase):
    mode.stage_index = stage_index
    mode.phase_time = float(phase)
    return mode._stage_reference()


def run():
    config_sha, cpu_sha = sha(CONFIG_PATH), sha(CPU_PATH)
    if config_sha != EXPECTED_CONFIG_SHA:
        raise ValueError(f"v6 config SHA mismatch: {config_sha}")
    config = json.loads(CONFIG_PATH.read_bytes())
    cpu = json.loads(CPU_PATH.read_bytes())
    ClimbMode(None)._validate_config(config)
    if cpu.get("state") != "DONE" or cpu.get("config_sha256_before") != config_sha:
        raise ValueError("DONE CPU receipt does not bind frozen v6 config")

    local_triangles = load_triangles(MESH)
    world_triangles = local_triangles + TRANSLATION
    mode = ClimbMode(None)
    mode.config = config
    kinematic = GraspKinematic()
    controller = GraspController(DT, enable_link_collision_check=False,
                                 climb_timeout_uses_wall_time=False)
    stages = config["stages"]
    stage_index = {stage["name"]: i for i, stage in enumerate(stages)}
    rows_by_stage = {}
    for row in cpu["tick_samples"]:
        rows_by_stage.setdefault(row["stage"], []).append(row)
    for rows in rows_by_stage.values():
        rows.sort(key=lambda r: (float(r["phase_before_s"]), int(r["tick"])))

    result = {
        "audit": "Bounded CPU sample and discrete visual-STL diagnostics for v6.",
        "inputs": {
            "config_sha256": config_sha, "cpu_sha256": cpu_sha,
            "runtime_sha256": sha(RUNTIME_PATH),
            "xiaolan_stl_sha256": sha(MESH),
            "cpu_state": cpu["state"], "cpu_ticks": cpu["ticks"],
        },
        "thresholds": {"minimum_crossing_sphere_gap_m": .015,
                       "xy_surface_height_delta_m_must_be_nonnegative": True},
        "entry_support_margins": [], "active_mid_back_progress": [],
        "terminal_spacing": {}, "c12_crossing": {},
        "landing_surface_coverage": [], "terminal_axis_pose_diagnostics": [],
        "visual_mesh_hashes": None, "mesh_pair_queries": [],
        "body_middle_pair_queries": [], "c12_xiaolan_queries": [],
        "exact_mesh_queries_started": False, "first_stop": None,
        "completed": False,
        "evidence_boundary": "Finite CPU references, FK, model COM, support polygon, closest-triangle distances, and selected exact visual mesh samples only; none proves contact, load, friction, stability, continuous clearance, or hardware safety.",
    }
    json_write(OUT_PATH, result)
    json_write(CACHE_PATH, {"inputs": result["inputs"], "checks": {}})

    # Entry COM margin uses that stage's fixed support set (all legs except its active legs).
    for name in sum((list(x) for x in WINDOWS.values()), []):
        rows = rows_by_stage.get(name, [])
        if not rows:
            continue
        row = rows[0]
        idx = stage_index[name]
        _, anchors, _ = planned_at(mode, idx, row["phase_before_s"])
        active = set(stages[idx]["active_legs"])
        fixed = [leg for leg in range(6) if leg not in active]
        com_base = kinematic.center_of_mass_base(np.asarray(row["q_rad"], dtype=float))
        com_world = (world_from_base(np.asarray(row["base_pose"], dtype=float)
                                     @ np.r_[com_base, 1.0])[:3])
        margin = support_margin(com_world[:2], anchors[fixed]) if fixed else None
        result["entry_support_margins"].append({
            "stage": name, "tick": int(row["tick"]), "active_legs": sorted(active),
            "support_legs": fixed, "model_com_world_xy_m": com_world[:2].tolist(),
            "incoming_support_margin_m": margin,
        })

    # Report active M/B world progress and pose-relative foot progress for every move.
    for name in sum((list(x) for x in WINDOWS.values()), []):
        stage = stages[stage_index[name]]
        active_mid_back = [leg for leg in stage["active_legs"] if leg in (0, 2, 3, 5)]
        if not active_mid_back:
            continue
        anchors = np.asarray(stage["anchor_knots"], dtype=float)
        p0, p1 = np.asarray(stage["pose_start"], dtype=float), np.asarray(stage["pose_end"], dtype=float)
        start_local = (np.column_stack((anchors[0], np.ones(6)))
                       @ np.linalg.inv(world_from_base(p0)).T)[:, :3]
        end_local = (np.column_stack((anchors[-1], np.ones(6)))
                     @ np.linalg.inv(world_from_base(p1)).T)[:, :3]
        for leg in active_mid_back:
            result["active_mid_back_progress"].append({
                "stage": name, "leg": NAMES[leg],
                "world_delta_xyz_m": (anchors[-1, leg] - anchors[0, leg]).tolist(),
                "body_relative_delta_xyz_m": (end_local[leg] - start_local[leg]).tolist(),
            })

    base_cfg = json.loads((ROOT / "src/grasp_hexapod_control/config/climb_front_direct.json").read_bytes())
    final = np.asarray(config["stages"][-1]["anchor_knots"][-1], dtype=float)
    old_final = np.asarray(base_cfg["stages"][-1]["anchor_knots"][-1], dtype=float)
    result["terminal_spacing"] = {
        "same_side_back_middle_x_spacing_m": {
            "baseline_LB_LM": abs(float(old_final[0, 0] - old_final[2, 0])),
            "v6_LB_LM": abs(float(final[0, 0] - final[2, 0])),
            "baseline_RB_RM": abs(float(old_final[3, 0] - old_final[5, 0])),
            "v6_RB_RM": abs(float(final[3, 0] - final[5, 0])),
        },
        "same_side_mid_front_x_spacing_m": {
            "v6_LM_LF": abs(float(final[2, 0] - final[1, 0])),
            "v6_RM_RF": abs(float(final[5, 0] - final[4, 0])),
        },
    }

    # C12 crossing: retain both planned-anchor and planned-root FK sphere gaps.
    c12_i = stage_index["MID_PLATFORM_285"]
    c12 = stages[c12_i]
    durations = np.asarray(c12["segment_durations_s"], dtype=float)
    cross_start, cross_end = float(durations[0]), float(np.sum(durations[:-1]))
    cross_rows = []
    for row in rows_by_stage["MID_PLATFORM_285"]:
        phase = float(row["phase_before_s"])
        if not (cross_start - 1e-9 <= phase < cross_end):
            continue
        pose, anchors, _ = planned_at(mode, c12_i, phase)
        q = np.asarray(row["q_rad"], dtype=float)
        fk_base = kinematic.forward_base(q)
        fk_world = (np.column_stack((fk_base, np.ones(6)))
                    @ world_from_base(pose).T)[:, :3]
        for leg in (2, 5):
            try:
                height_delta = float(anchors[leg, 2] - surface_height(
                    local_triangles, anchors[leg, 0], anchors[leg, 1]))
            except ValueError:
                height_delta = None
            cross_rows.append({
                "tick": int(row["tick"]), "phase_s": phase, "leg": NAMES[leg],
                "anchor_world_xyz_m": anchors[leg].tolist(),
                "world_z_minus_surface_height_m": height_delta,
                "planned_anchor_sphere_gap_m": float(foot_sphere_signed_gap(world_triangles, anchors[leg])),
                "planned_root_fk_sphere_gap_m": float(foot_sphere_signed_gap(world_triangles, fk_world[leg])),
            })
    min_anchor = min(cross_rows, key=lambda r: r["planned_anchor_sphere_gap_m"])
    min_fk = min(cross_rows, key=lambda r: r["planned_root_fk_sphere_gap_m"])
    height_rows = [r for r in cross_rows if r["world_z_minus_surface_height_m"] is not None]
    min_height = min(height_rows, key=lambda r: r["world_z_minus_surface_height_m"]) if height_rows else None
    result["c12_crossing"] = {
        "window_s": [cross_start, cross_end], "sample_count": len(cross_rows),
        "minimum_anchor_sphere_gap": min_anchor,
        "minimum_planned_root_fk_gap": min_fk,
        "minimum_surface_height_delta": min_height,
        "negative_height_samples": [r for r in height_rows if r["world_z_minus_surface_height_m"] < 0.0],
        "under_15mm_samples": [r for r in cross_rows if min(r["planned_anchor_sphere_gap_m"], r["planned_root_fk_sphere_gap_m"]) < .015],
        "samples": cross_rows,
    }
    json_write(OUT_PATH, result)
    gap_failure = bool(result["c12_crossing"]["under_15mm_samples"])
    height_failure = bool(result["c12_crossing"]["negative_height_samples"])
    if gap_failure or height_failure:
        item = (result["c12_crossing"]["under_15mm_samples"] or
                result["c12_crossing"]["negative_height_samples"])[0]
        result["first_stop"] = {"stage": "MID_PLATFORM_285", "tick": item["tick"],
                                "phase_s": item["phase_s"], "leg": item["leg"],
                                "reason": "cross_gap_below_15mm" if gap_failure else "negative_surface_height_delta",
                                "witness": item}
        json_write(OUT_PATH, result)
        print(json.dumps({"status": "STOP_BEFORE_MESH_QUERIES", "first_stop": result["first_stop"]}), flush=True)
        return

    # Surface/normal sampling at each actual surface landing and terminal feet.
    patch_spec = importlib.util.spec_from_file_location("space_terminal_patch", TERMINAL_HELPER_PATH)
    patch = importlib.util.module_from_spec(patch_spec); sys.modules[patch_spec.name] = patch; patch_spec.loader.exec_module(patch)
    landing_specs = []
    for stage_name, legs in (("MID_PLATFORM_285", (2, 5)), ("FRONT_TO_0400", (1, 4)),
                             ("FRONT_TO_0480", (1, 4)), ("RB_PLATFORM_225", (3,)),
                             ("LB_PLATFORM_225", (0,)), ("FINAL_HOLD", tuple(range(6)))):
        row = stages[stage_index[stage_name]]
        pointset = np.asarray(row["anchor_knots"][-1], dtype=float)
        for leg in legs:
            landing_specs.append((stage_name, leg, pointset[leg]))
    unique_landings = {}
    for stage_name, leg, point in landing_specs:
        unique_landings[(stage_name, leg)] = point
    axes_base = kinematic.terminal_axes_base(np.asarray(config["terminal_q_rad"], dtype=float))
    terminal_pose = np.asarray(stages[-1]["pose_end"], dtype=float)
    rotation = world_from_base(terminal_pose)[:3, :3]
    axes_world = axes_base @ rotation.T
    for (stage_name, leg), point in unique_landings.items():
        nearest = patch.inspect_nearest_patch(world_triangles, point)[0]
        normal = np.asarray(nearest["face_normal_world"], dtype=float)
        dot = float(np.clip(np.dot(axes_world[leg], normal), -1.0, 1.0))
        axis_diag = {"stage": stage_name, "leg": NAMES[leg],
                     "directed_angle_to_actual_face_normal_deg": float(np.degrees(np.arccos(dot))),
                     "axis_line_angle_to_actual_face_normal_deg": float(np.degrees(np.arccos(abs(dot)))),
                     "pose_metric_only": True}
        if stage_name == "FINAL_HOLD":
            result["terminal_axis_pose_diagnostics"].append(axis_diag)
        grid = []
        for dx in (-.008, 0.0, .008):
            for dy in (-.008, 0.0, .008):
                try:
                    h = float(surface_height(local_triangles, point[0] + dx, point[1] + dy))
                except ValueError:
                    h = None
                grid.append({"dx_m": dx, "dy_m": dy, "surface_center_height_m": h})
        result["landing_surface_coverage"].append({
            "stage": stage_name, "leg": NAMES[leg], "target_world_xyz_m": point.tolist(),
            "nearest_actual_face_normal_world": nearest["face_normal_world"],
            "nearest_triangle_index": nearest["triangle_index"],
            "surface_height_grid_3x3_pm8mm": grid,
            "coverage_count": sum(item["surface_center_height_m"] is not None for item in grid),
            "terminal_axis_pose_diagnostic": axis_diag if stage_name == "FINAL_HOLD" else None,
        })
    result["visual_mesh_hashes"] = mesh_hashes(default_visual_scene(ROOT))
    json_write(OUT_PATH, result)

    # Configure the selected exact knee-pair helper to use only the new cache.
    pair_spec = importlib.util.spec_from_file_location("space_pair_helper", PAIR_HELPER_PATH)
    pair_helper = importlib.util.module_from_spec(pair_spec); sys.modules[pair_spec.name] = pair_helper; pair_spec.loader.exec_module(pair_helper)
    pair_helper.PAIR_CACHE_PATH = CACHE_PATH
    if not CACHE_PATH.exists():
        json_write(CACHE_PATH, {"inputs": result["inputs"], "checks": {}})
    result["exact_mesh_queries_started"] = True
    json_write(OUT_PATH, result)

    pair_tasks = {}
    for window_name, names in WINDOWS.items():
        win = [row for name in names for row in rows_by_stage.get(name, [])]
        win.sort(key=lambda r: int(r["tick"]))
        if not win:
            continue
        for pair_indices, pair_links, pair_name in PAIR_DEFS:
            gaps = [pair_helper.knee_gap(controller, np.asarray(r["q_rad"], dtype=float), *pair_indices) for r in win]
            minimum_index = int(np.argmin(gaps))
            for j in (minimum_index - 1, minimum_index, minimum_index + 1):
                if 0 <= j < len(win):
                    row = win[j]
                    key = (pair_name, identity(row))
                    item = pair_tasks.setdefault(key, {"sample": row, "pair_links": pair_links,
                        "pair_name": pair_name, "proxy_gap_m": gaps[j], "labels": []})
                    item["labels"].append(f"{window_name}:proxy_min_neighbor")

    for stage_name in ("MID_PLATFORM_285", "RB_PLATFORM_225", "LB_PLATFORM_225", "FINAL_HOLD"):
        rows = rows_by_stage.get(stage_name, [])
        if not rows:
            continue
        row = rows[-1] if stage_name != "FINAL_HOLD" else rows[0]
        for _, pair_links, pair_name in PAIR_DEFS:
            key = (pair_name, identity(row))
            item = pair_tasks.setdefault(key, {"sample": row, "pair_links": pair_links,
                "pair_name": pair_name, "proxy_gap_m": None, "labels": []})
            item["labels"].append(f"{stage_name}:landing_or_entry")

    query_count = 0
    for window_name, names in WINDOWS.items():
        worst = None
        for stage_name in names:
            for row in rows_by_stage.get(stage_name, []):
                gaps = pair_helper.capsule_body_gaps(controller, np.asarray(row["q_rad"], dtype=float), (2, 5))
                for gap in gaps:
                    if worst is None or gap["proxy_gap_m"] < worst["proxy_gap_m"]:
                        worst = {**gap, "sample": row}
        if worst is not None:
            link = {"leg": worst["leg"], "link": worst["link"]}
            pair_tasks[(f"BODY-{worst['leg'].upper()}-{worst['link']}", identity(worst["sample"]))] = {
                "sample": worst["sample"], "pair_links": ({"link": "body"}, link),
                "pair_name": f"BODY-{worst['leg'].upper()}-{worst['link']}",
                "proxy_gap_m": worst["proxy_gap_m"], "labels": [f"{window_name}:worst_middle_body_proxy"],
            }

    for item in sorted(pair_tasks.values(), key=lambda x: (x["sample"]["tick"], x["pair_name"])):
        row = item["sample"]
        exact = pair_helper.check_pair(scene, np.asarray(row["q_rad"], dtype=float),
                                       np.asarray(row["base_pose"], dtype=float),
                                       *item["pair_links"])
        record = {"query_index": query_count + 1, "pair_name": item["pair_name"],
                  "labels": item["labels"], "proxy_gap_m": item["proxy_gap_m"],
                  "tick": int(row["tick"]), "stage": row["stage"],
                  "phase_s": float(row["phase_before_s"]),
                  "q_rad": row["q_rad"], "base_pose": row["base_pose"], **exact}
        result["mesh_pair_queries"].append(record)
        query_count += 1
        json_write(OUT_PATH, result)
        print(f"mesh pair query {query_count}: {record['stage']} tick={record['tick']} pair={record['pair_name']} hit={record['mesh_collision']}", flush=True)
        if exact["mesh_collision"]:
            result["first_stop"] = {"reason": "exact_visual_mesh_hit", "witness": record}
            json_write(OUT_PATH, result)
            return

    # Three worst proxy body-to-middle witnesses, checking the indicated M knee/ankle link.
    # (The body queries have already been issued above and are tagged in mesh_pair_queries.)
    for window_name in WINDOWS:
        recs = [r for r in result["mesh_pair_queries"] if f"{window_name}:worst_middle_body_proxy" in r["labels"]]
        result["body_middle_pair_queries"].extend(recs)

    # C12 keytimes: lift end, x~=178/245 mm, worst crossing gap, and pre-landing.
    c12_rows = rows_by_stage["MID_PLATFORM_285"]
    candidates = {}
    desired_times = {
        "first_vertical_lift_end": float(durations[0]),
        "feet_near_x178mm": None,
        "feet_near_x245mm": None,
        "crossing_gap_worst": min_anchor,
        "pre_landing": max(c12_rows, key=lambda r: float(r["phase_before_s"])),
    }
    for label, target_x in (("feet_near_x178mm", .178), ("feet_near_x245mm", .245)):
        selected = None
        selected_distance = float("inf")
        for row in c12_rows:
            _, anchors, _ = planned_at(mode, c12_i, row["phase_before_s"])
            leg = min((2, 5), key=lambda i: abs(float(anchors[i, 0]) - target_x))
            distance = abs(float(anchors[leg, 0]) - target_x)
            if distance < selected_distance:
                selected_distance, selected = distance, row
        desired_times[label] = selected
    lift_row = min(c12_rows, key=lambda r: abs(float(r["phase_before_s"]) - durations[0]))
    desired_times["first_vertical_lift_end"] = lift_row
    gap_row = next(r for r in c12_rows if int(r["tick"]) == int(min_anchor["tick"]))
    desired_times["crossing_gap_worst"] = gap_row
    for label, row in desired_times.items():
        key = identity(row)
        candidates.setdefault(key, {"sample": row, "labels": []})["labels"].append(label)
    for candidate in candidates.values():
        row = candidate["sample"]
        q = np.asarray(row["q_rad"], dtype=float)
        pose = np.asarray(row["base_pose"], dtype=float)
        components = scene.robot_components(q, world_from_base(pose), leg_indices=(2, 5), include_body=False)
        components = [c for c in components if c.mesh.link_name in ("knee", "ankle")]
        for leg in (2, 5):
            selected = [c for c in components if c.leg_index == leg]
            for link_name in ("knee", "ankle"):
                part = [c for c in selected if c.mesh.link_name == link_name]
                if not part:
                    raise RuntimeError(f"missing selected component: {NAMES[leg]} {link_name}")
                check = scene.components_vs_xiaolan(part)
                rec = {"labels": candidate["labels"], "tick": int(row["tick"]),
                       "stage": row["stage"], "phase_s": float(row["phase_before_s"]),
                       "leg": NAMES[leg], "link": link_name,
                       "component_indices": [c.component.component_index for c in part],
                       "q_rad": row["q_rad"], "base_pose": row["base_pose"],
                       "mesh_collision": bool(check.collision),
                       "collision_hit": None if check.hit is None else check.hit.__dict__,
                       "broad_phase_candidates": int(check.broad_phase_candidates),
                       "narrow_phase_tests": int(check.narrow_phase_tests),
                       "landing_ankle_hit_requires_sol_review": bool(link_name == "ankle" and "pre_landing" in candidate["labels"])}
                result["c12_xiaolan_queries"].append(rec)
                json_write(OUT_PATH, result)
                query_count += 1
                print(f"mesh Xiaolan query {query_count}: {row['stage']} tick={row['tick']} {NAMES[leg]}-{link_name} hit={check.collision}", flush=True)
                if check.collision:
                    result["first_stop"] = {"reason": "exact_visual_mesh_hit_vs_xiaolan", "witness": rec}
                    json_write(OUT_PATH, result)
                    return

    result["completed"] = True
    result["selected_query_count"] = query_count
    json_write(OUT_PATH, result)


if __name__ == "__main__":
    run()
