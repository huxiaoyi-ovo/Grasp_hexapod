#!/usr/bin/env python3
"""Build the bounded 21-stage front route from the reviewed v8 contact states."""
import argparse
import copy
import base64
import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / "src/grasp_hexapod_control/scripts"
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / "tools")]

from build_climb_front import (  # noqa: E402
    MESH, TRANSLATION, foot_sphere_signed_gap, load_triangles, surface_height,
    solve_ik,
)
from climb_mode import ClimbMode  # noqa: E402
from kinematics import GraspKinematic, JOINT_LOWER, JOINT_UPPER, Q_STAND  # noqa: E402
from optimize_climb_front import _refresh_active_base_knots  # noqa: E402

PATCH_HELPER_PATH = ROOT / "src/docs/evidence/climb_front_terminal_20260924/front_terminal_audit.py"
patch_spec = importlib.util.spec_from_file_location("direct_rear_patch_helpers", PATCH_HELPER_PATH)
patch_helper = importlib.util.module_from_spec(patch_spec)
sys.modules[patch_spec.name] = patch_helper
patch_spec.loader.exec_module(patch_helper)

FAST = ROOT / "src/grasp_hexapod_control/config/climb_front_fast.json"
ENDPOINTS = ROOT / "src/docs/evidence/climb_front_direct_20260924/endpoint_probe_v8.json"
OUTPUT = ROOT / "src/grasp_hexapod_control/config/climb_front_direct.json"
CPU_STRUCTURAL = ROOT / "src/docs/evidence/climb_front_direct_20260924/cpu_first.json"
STRUCTURAL = ROOT / "src/docs/evidence/climb_front_direct_20260924/structural_21.json"
SNAPSHOT = ROOT / "src/docs/evidence/climb_front_direct_20260924/fast_candidate.json"
PRESERVE_TIMING = {
    "PREP", "FRONT_LOW", "BODY_A", "BACK_TO_MINUS09", "RB_TO_013",
    "BODY_MID_0235_SYNC", "AH_BACK_LOW_FIRST_SEGMENT", "FINAL_HOLD",
}
LEGS = ("LB", "LF", "LM", "RB", "RF", "RM")
GROUND_Z = .0065
MID_INSET = ROOT / "src/docs/evidence/climb_front_direct_20260924/mid_inset_candidate.json"
MID_INSET_SOURCE_SHA = "a7bcb4d00ff5afb5ec6b74d1270a2e151af2a9a59ba9c3e7c890e82251fac442"
MID_V2 = ROOT / "src/docs/evidence/climb_front_direct_20260924/mid_inset_v2_candidate.json"
MID_V2_SHA = "cd0fc1b37898d1bcda0c0809ed7a5bb77699d9154f40585e930ba6db10b75f5b"
REAR_V1 = ROOT / "src/docs/evidence/climb_front_direct_20260924/rear_inset_candidate.json"
REAR_V1_SHA = "18a85c4a502f6df7bf482a5c07c58647523b36a8762607bc04beab53c485b9c7"


def quintic(value):
    value = float(np.clip(value, 0.0, 1.0))
    return value ** 3 * (10.0 - 15.0 * value + 6.0 * value ** 2)


def pose_knots(start, end, durations, curve):
    elapsed = np.r_[0.0, np.cumsum(durations)]
    if curve == "quintic_first_segment":
        phase = np.minimum(elapsed / durations[0], 1.0)
    else:
        phase = elapsed / elapsed[-1]
    return [start * (1.0 - quintic(p)) + end * quintic(p) for p in phase]


def make_stage(name, start_pose, end_pose, start_anchors, end_anchors,
               active, durations, curve="quintic_full_stage", knots=None):
    active = list(active)
    if knots is None:
        if active:
            start, end = np.asarray(start_anchors), np.asarray(end_anchors)
            apex = float(max(start[active, 2].max(), end[active, 2].max()) + .020)
            lift, cross = start.copy(), end.copy()
            lift[active, 2] = apex
            cross[active, 2] = apex
            knots = [start, lift, cross, end]
        else:
            knots = [np.asarray(start_anchors), np.asarray(end_anchors)]
    durations = [float(value) for value in durations]
    stage = {
        "name": name, "pose_start": np.asarray(start_pose).tolist(),
        "pose_end": np.asarray(end_pose).tolist(), "active_legs": active,
        "anchor_knots": [np.asarray(knot).tolist() for knot in knots],
        "segment_durations_s": durations,
        "anchor_curve": "piecewise_base_quintic" if active else "piecewise_quintic",
        "pose_curve": curve, "settle_s": .15 if active else .10,
    }
    if active:
        _refresh_active_base_knots(stage)
    if len(stage["anchor_knots"]) != len(durations) + 1:
        raise ValueError(name + ": knot/duration count mismatch")
    for leg in range(6):
        if leg not in active and any(
            not np.allclose(stage["anchor_knots"][0][leg], knot[leg], rtol=0, atol=1e-12)
            for knot in stage["anchor_knots"][1:]
        ):
            raise ValueError(name + ": fixed world anchor moved")
    return stage


def build(fast, receipt):
    if receipt.get("stopped_on_first_failure") is not None:
        raise ValueError("v8 endpoint probe stopped on a failed endpoint")
    triangles = load_triangles(MESH) + TRANSLATION
    for row in receipt["stages"]:
        for leg_name in row["active_legs"]:
            leg = LEGS.index(leg_name.upper())
            point = np.asarray(row["world_anchors_target_m"], dtype=float)[leg]
            if point[0] < .17792:
                if not np.isclose(point[2], GROUND_Z, rtol=0, atol=1e-6):
                    raise ValueError(
                        f"{row['state']} {leg_name} low-ground landing has "
                        f"z={point[2]:.9f}, expected {GROUND_Z:.9f} m"
                    )
                gap = foot_sphere_signed_gap(triangles, point)
                if gap < 0.0:
                    raise ValueError(
                        f"{row['state']} {leg_name} ground foot intersects "
                        f"Xiaolan STL by {-gap:.9f} m"
                    )
            else:
                gap = foot_sphere_signed_gap(triangles, point)
                if not -.0005 <= gap <= .002:
                    raise ValueError(
                        f"{row['state']} {leg_name} STL landing failed: "
                        f"sphere_gap={gap:.9f} m"
                    )
    by_name = {stage["name"]: stage for stage in fast["stages"]}
    states = {row["state"]: row for row in receipt["stages"]}
    stages = [copy.deepcopy(by_name[name]) for name in ("PREP", "FRONT_LOW", "BODY_A")]
    pose = np.asarray(stages[-1]["pose_end"], dtype=float)
    anchors = np.asarray(stages[-1]["anchor_knots"][-1], dtype=float)

    def append_swing(name, state, durations):
        nonlocal pose, anchors
        row = states[state]
        end = np.asarray(row["world_anchors_target_m"], dtype=float)
        active = [LEGS.index(leg.upper()) for leg in row["active_legs"]]
        stage = make_stage(name, pose, row["body_pose_target"], anchors, end,
                           active, durations)
        stages.append(stage); pose = np.asarray(stage["pose_end"]); anchors = end
        return stage

    def append_body(name, state, duration=.6):
        nonlocal pose, anchors
        row = states[state]
        end_pose = np.asarray(row["body_pose_target"], dtype=float)
        stage = make_stage(name, pose, end_pose, anchors, anchors, [], [duration])
        stages.append(stage); pose = end_pose
        return stage

    def append_merge(name, start_state, end_state, active, durations, curve,
                     custom_knots=None):
        nonlocal pose, anchors
        first, last = states[start_state], states[end_state]
        start = np.asarray(first["world_anchors_start_m"], dtype=float)
        end = np.asarray(last["world_anchors_target_m"], dtype=float)
        if not np.allclose(start, anchors, rtol=0, atol=1e-12):
            raise ValueError(name + ": v8 source entry does not match built prefix")
        stage = make_stage(name, pose, last["body_pose_target"], start, end,
                           active, durations, curve, custom_knots)
        stages.append(stage); pose = np.asarray(stage["pose_end"]); anchors = end
        return stage

    append_swing("M_GROUND_FIRST_MINUS02", "M_GROUND_FIRST_MINUS02", [.4, .8, .4])
    append_swing("FRONT_TO_029", "FRONT_TO_029_BODY_MINUS03", [.5, .7, .35])
    append_body("SIX_SUPPORT_BODY_ADVANCE", "SIX_SUPPORT_BODY_ADVANCE")
    append_swing("BACK_TO_MINUS09", "BACK_TO_MINUS09", [.4, .8, .4])
    append_swing("MID_TO_040", "MID_TO_040", [.4, .8, .4])
    append_swing("FRONT_TO_034", "FRONT_TO_034", [.5, .7, .35])
    append_body("SIX_SUPPORT_ADVANCE_059", "SIX_SUPPORT_ADVANCE_059")
    append_swing("BACK_TO_000", "BACK_TO_000", [.4, .8, .4])

    first, last = states["MID_TO_080"], states["Q_MID_LOW"]
    start = np.asarray(first["world_anchors_start_m"], dtype=float)
    end = np.asarray(last["world_anchors_target_m"], dtype=float)
    qknots = [start.copy() for _ in range(5)]
    for leg in (2, 5):
        y = start[leg, 1]
        qknots[1][leg] = (.08, y, .0865)
        qknots[2][leg] = (.14, y, .12)
        apex = max(end[2, 2], end[5, 2]) + .010
        qknots[3][leg] = (.17, y, apex)
        qknots[4][leg] = end[leg]
    append_merge("MID_TO_019_DIRECT", "MID_TO_080", "Q_MID_LOW", [2, 5],
                 [1.0, .7, .7, .4], "quintic_first_segment", qknots)

    append_swing("FRONT_TO_040", "FRONT_TO_040", [.5, .7, .35])
    append_merge("BODY_LB_0055_SYNC", "SIX_SUPPORT_BODY_0145", "LB_TO_0055",
                 [0], [.4, .8, .4], "quintic_full_stage")
    append_swing("FRONT_TO_048", "FRONT_TO_048", [.5, .7, .35])
    append_swing("RB_TO_0055", "RB_TO_0055", [.4, .8, .4])
    append_merge("BODY_LB_013_SYNC", "SIX_SUPPORT_BODY_0200", "LB_TO_013",
                 [0], [.4, .8, .4], "quintic_full_stage")
    append_swing("RB_TO_013", "RB_TO_013", [.4, .8, .4])

    first, last = states["SIX_SUPPORT_BODY_0277"], states["MID_TO_0235"]
    start = np.asarray(first["world_anchors_start_m"], dtype=float)
    end = np.asarray(last["world_anchors_target_m"], dtype=float)
    apex = float(max(end[2, 2], end[5, 2]) + .005)
    air_start, air_end = start.copy(), end.copy()
    air_start[[2, 5], 2] = apex
    air_end[[2, 5], 2] = apex
    append_merge("BODY_MID_0235_SYNC", "SIX_SUPPORT_BODY_0277", "MID_TO_0235",
                 [2, 5], [.7, .7, .4], "quintic_first_segment",
                 [start, air_start, air_end, end])

    ah = copy.deepcopy(by_name["AH_BACK_LOW_FIRST_SEGMENT"])
    ah["pose_start"] = pose.tolist()
    terminal = np.asarray(states["AH_TERMINAL"]["world_anchors_target_m"], dtype=float)
    source_knots = np.asarray(ah["anchor_knots"], dtype=float)
    fixed = [1, 2, 4, 5]
    source_knots[:, fixed] = anchors[fixed]
    source_knots[0, [0, 3]] = anchors[[0, 3]]
    source_knots[-1] = terminal
    ah["anchor_knots"] = source_knots.tolist()
    _refresh_active_base_knots(ah)
    stages.append(ah); pose = np.asarray(ah["pose_end"]); anchors = terminal

    hold = copy.deepcopy(by_name["FINAL_HOLD"])
    hold["pose_start"] = pose.tolist(); hold["pose_end"] = pose.tolist()
    hold["anchor_knots"] = [anchors.tolist(), anchors.tolist()]
    stages.append(hold)

    candidate = copy.deepcopy(fast)
    candidate["stages"] = stages
    candidate["stage_count"] = len(stages)
    candidate["terminal_q_rad"] = states["AH_TERMINAL"]["exit_q_rad"]
    candidate.pop("terminal_refinement", None)
    candidate.pop("surface_receipt", None)
    candidate.pop("optimization", None)
    candidate.pop("visual_validation_deferred_for_sim_finish", None)
    candidate["direct_replan_source"] = {
        "endpoint_receipt_sha256": hashlib.sha256(ENDPOINTS.read_bytes()).hexdigest(),
        "source_config_sha256": receipt["source_config_sha256"],
        "planning_basis": "24 seeded static endpoint states; no continuous path claim",
        "merged_source_pairs": [["MID_TO_080", "Q_MID_LOW"],
                                ["SIX_SUPPORT_BODY_0145", "LB_TO_0055"],
                                ["SIX_SUPPORT_BODY_0200", "LB_TO_013"],
                                ["SIX_SUPPORT_BODY_0277", "MID_TO_0235"]],
    }
    if len(stages) != 21:
        raise ValueError(f"expected 21 stages including FINAL_HOLD, got {len(stages)}")
    ClimbMode(None)._validate_config(candidate)
    return candidate


def apply_structural_retry(candidate, cpu_receipt):
    retry = cpu_receipt["velocity_retry"]
    source_bytes = json.dumps(candidate, indent=2, allow_nan=False).encode() + b"\n"
    source_sha = hashlib.sha256(source_bytes).hexdigest()
    if source_sha != retry["source_config_sha256"]:
        raise ValueError("rebuilt structural source differs from recorded retry source")
    stage = next(s for s in candidate["stages"] if s["name"] == retry["stage"])
    original = list(stage["segment_durations_s"])
    factor = float(retry["factor"])
    stage["segment_durations_s"] = [float(x) * factor for x in original]
    candidate["direct_cpu_velocity_retry"] = {
        "source_config_sha256": source_sha,
        "stage": retry["stage"], "factor": factor,
        "original_durations_s": original,
        "candidate_durations_s": stage["segment_durations_s"],
        "scope": "single failed stage; timing only; all geometry frozen",
    }
    retry_bytes = (json.dumps(candidate, indent=2, allow_nan=False) + "\n").encode()
    recorded_bytes = base64.b64decode(retry["candidate_config_bytes_base64"], validate=True)
    if retry_bytes != recorded_bytes:
        raise ValueError("rebuilt RB_TO_013 retry differs from successful CPU candidate")
    if hashlib.sha256(retry_bytes).hexdigest() != retry["candidate_config_sha256"]:
        raise ValueError("structural retry candidate SHA mismatch")
    ClimbMode(None)._validate_config(candidate)
    return candidate, retry_bytes


def apply_audit_fast_timing(candidate, cpu_receipt, cpu_bytes, structural_bytes):
    structural_sha = hashlib.sha256(structural_bytes).hexdigest()
    if structural_sha != cpu_receipt["config_sha256_before"]:
        raise ValueError("structural snapshot is not the config bound by CPU audit")
    if structural_bytes != base64.b64decode(cpu_receipt["config_bytes_base64"], validate=True):
        raise ValueError("structural snapshot differs from CPU audit input bytes")
    if structural_bytes != (json.dumps(candidate, indent=2, allow_nan=False) + "\n").encode():
        raise ValueError("canonical builder did not reproduce structural snapshot bytes")

    factors = {}
    rows = cpu_receipt["stage_rows"]
    geometry_fields = ("pose_start", "pose_end", "anchor_knots",
                       "active_base_knots_m", "active_base_velocities_m_s")
    for stage in candidate["stages"]:
        name = stage["name"]
        original = list(stage["segment_durations_s"])
        factor = 1.0
        if name not in PRESERVE_TIMING:
            row = rows[name]
            factor = min(1.0, max(
                .55,
                float(row["max_raw_joint_speed_rad_s"]) / 4.8,
                float(row["max_fk_error_m"]) / .011,
                .20 / min(original),
            ))
            stage["segment_durations_s"] = [float(value) * factor for value in original]
        factors[name] = factor
    candidate.pop("direct_cpu_velocity_retry", None)
    candidate["direct_timing_selection"] = {
        "structural_config_sha256": structural_sha,
        "structural_cpu_audit_sha256": hashlib.sha256(cpu_bytes).hexdigest(),
        "structural_source_config_sha256": cpu_receipt["velocity_retry"]["source_config_sha256"],
        "source_fast_config_sha256": candidate["direct_replan_source"]["source_config_sha256"],
        "structural_velocity_retry_provenance": copy.deepcopy(
            cpu_receipt["velocity_retry"]
        ),
        "preserved_timing_stages": sorted(PRESERVE_TIMING),
        "factor_by_stage": factors,
        "rule": "non-preserved factor=min(1,max(.55, raw_speed/4.8, max_fk_error/.011, .20/min_segment_duration)); one factor per stage",
        "scope": "timing reduction only; reference geometry and endpoints frozen",
    }
    ClimbMode(None)._validate_config(candidate)
    return candidate


def refine_initial_mid_contact(candidate, second_air_z=.12,
                               landing_relative_mid_air=False):
    """Apply the frozen first-mid landing inset to the reviewed a7 timing candidate."""
    source_bytes = (json.dumps(candidate, indent=2, allow_nan=False) + "\n").encode()
    source_sha = hashlib.sha256(source_bytes).hexdigest()
    if source_sha != MID_INSET_SOURCE_SHA:
        raise ValueError(f"mid-inset source must be the reviewed a7 candidate, got {source_sha}")
    candidate = copy.deepcopy(candidate)
    triangles_local = load_triangles(MESH)
    surface_points = {}

    def landing(leg):
        point = np.asarray(leg, dtype=float).copy()
        point[0] = .205
        point[2] = surface_height(triangles_local, point[0], point[1])
        gap = foot_sphere_signed_gap(triangles_local + TRANSLATION, point)
        if not -.0005 <= gap <= .002:
            raise ValueError(f"inset landing at y={point[1]:.6f} failed STL gap: {gap:.6f} m")
        surface_points[str(point[1])] = {"xyz_m": point.tolist(), "sphere_gap_m": gap}
        return point

    stages = candidate["stages"]
    mid_index = next(i for i, stage in enumerate(stages)
                     if stage["name"] == "MID_TO_019_DIRECT")
    mid = stages[mid_index]
    mid["name"] = "MID_DIRECT_PLATFORM"
    start = np.asarray(mid["anchor_knots"][0], dtype=float)
    end = np.asarray(mid["anchor_knots"][-1], dtype=float)
    q_start, q_end = np.asarray(mid["pose_start"], dtype=float), np.asarray(mid["pose_end"], dtype=float)
    q_end[[0, 2, 3]] = [.1167, .17, .15]
    mid["pose_end"] = q_end.tolist()
    knots = [start.copy() for _ in range(5)]
    for leg in (2, 5):
        y = start[leg, 1]
        point = landing(end[leg])
        knots[1][leg] = (.095, y, .0865)
        knots[2][leg] = (.155, y, second_air_z)
        knots[3][leg] = (.185, y, point[2] + .010)
        knots[4][leg] = point
    mid["anchor_knots"] = [k.tolist() for k in knots]
    _refresh_active_base_knots(mid)

    # Carry the new world support point through fixed-M stages to the next takeoff.
    for stage in stages[mid_index + 1:]:
        if stage["name"] == "BODY_MID_0235_SYNC":
            break
        for knot in stage["anchor_knots"]:
            for leg in (2, 5):
                knot[leg] = surface_points[str(float(knot[leg][1]))]["xyz_m"]
        if stage["active_legs"]:
            _refresh_active_base_knots(stage)

    body_mid = next(stage for stage in stages if stage["name"] == "BODY_MID_0235_SYNC")
    for leg in (2, 5):
        landing_point = np.asarray(surface_points[str(float(start[leg, 1]))]["xyz_m"])
        body_mid["anchor_knots"][0][leg] = landing_point.tolist()
        first_air_z = (landing_point[2] + .005 if landing_relative_mid_air
                       else body_mid["anchor_knots"][-1][leg][2] + .005)
        body_mid["anchor_knots"][1][leg] = [
            .205, landing_point[1], first_air_z
        ]
    _refresh_active_base_knots(body_mid)

    front = next(stage for stage in stages if stage["name"] == "FRONT_TO_040")
    front["pose_start"] = q_end.tolist()
    front_end = np.asarray(front["pose_end"], dtype=float)
    front_end[[0, 2, 3]] = [.13, .18, .09]
    front["pose_end"] = front_end.tolist()
    _refresh_active_base_knots(front)
    next_body = next(stage for stage in stages if stage["name"] == "BODY_LB_0055_SYNC")
    next_body["pose_start"] = front_end.tolist()
    _refresh_active_base_knots(next_body)

    candidate["mid_inset_refinement"] = {
        "source_a7_config_sha256": source_sha,
        "change": "first LM/RM platform landing inset to x=.205 with per-y STL sphere-height; propagate fixed support to BODY_MID_0235_SYNC takeoff; synchronize FRONT_TO_040 pose endpoint",
        "landing_by_y_m": surface_points,
        "stage_renamed": ["MID_TO_019_DIRECT", "MID_DIRECT_PLATFORM"],
        "geometry_only": True,
        "timing_source": "a7 fast candidate unchanged; timing is not new geometry evidence",
    }
    ClimbMode(None)._validate_config(candidate)
    return candidate


def refine_terminal_rear_inset(candidate):
    """Move only final LB/RB support centers inward on the frozen v2 route."""
    source_bytes = (json.dumps(candidate, indent=2, allow_nan=False) + "\n").encode()
    source_sha = hashlib.sha256(source_bytes).hexdigest()
    if source_sha != MID_V2_SHA:
        raise ValueError(f"rear-inset source must be reviewed v2, got {source_sha}")
    candidate = copy.deepcopy(candidate)
    local_triangles = load_triangles(MESH)
    world_triangles = local_triangles + TRANSLATION
    ah = next(stage for stage in candidate["stages"]
              if stage["name"] == "AH_BACK_LOW_FIRST_SEGMENT")
    hold = next(stage for stage in candidate["stages"] if stage["name"] == "FINAL_HOLD")
    terminal = np.asarray(ah["anchor_knots"][-1], dtype=float)
    targets = {}
    normal_dz_corrections = {}
    for leg in (0, 3):
        y = float(terminal[leg, 1])
        surface_z = float(surface_height(local_triangles, .225, y))
        point = np.array((.225, y, surface_z), dtype=float)
        gap = float(foot_sphere_signed_gap(world_triangles, point))
        normal_dz = max(0.0, -gap) / float(patch_helper.BROAD_SLOPE_NORMAL[2])
        point[2] += normal_dz
        gap = float(foot_sphere_signed_gap(world_triangles, point))
        if not -.0005 <= gap <= .002:
            raise ValueError(f"{LEGS[leg]} x=.225 landing gap {gap:.6f} m outside STL gate")
        nearest = patch_helper.inspect_nearest_patch(world_triangles, point)
        if not any(
            float(np.dot(np.asarray(item["face_normal_world"], dtype=float),
                         patch_helper.BROAD_SLOPE_NORMAL)) >= .995
            and item["inside_broad_patch_with_10mm_margin"]
            for item in nearest
        ):
            raise ValueError(f"{LEGS[leg]} x=.225 landing is not on the broad slope with 10 mm margin")
        terminal[leg] = point
        normal_dz_corrections[LEGS[leg]] = float(normal_dz)
        targets[LEGS[leg]] = {
            "xyz_m": point.tolist(), "surface_height_z_m": surface_z,
            "normal_offset_dz_m": float(normal_dz), "sphere_gap_m": gap,
        }
    ah["anchor_knots"][-1] = terminal.tolist()
    _refresh_active_base_knots(ah)
    hold_anchors = np.asarray(hold["anchor_knots"], dtype=float)
    for knot in range(len(hold_anchors)):
        for leg in (0, 3):
            hold_anchors[knot, leg] = terminal[leg]
    hold["anchor_knots"] = hold_anchors.tolist()

    pose = np.asarray(hold["pose_end"], dtype=float)
    kin = GraspKinematic()
    seed = np.asarray(candidate["terminal_q_rad"], dtype=float)
    q, residual = solve_ik(kin, pose, terminal, seed, tolerance_m=1e-5)
    margin = float(np.min(np.minimum(q - JOINT_LOWER, JOINT_UPPER - q)))
    if residual > .004 or margin <= 0.0:
        raise ValueError(
            f"terminal seeded IK failed: residual={residual:.6f}m, joint_margin={margin:.6f}rad"
        )
    candidate["terminal_q_rad"] = q.tolist()
    candidate["terminal_rear_inset_refinement"] = {
        "source_v2_config_sha256": source_sha,
        "only_changed_endpoint": "AH_BACK_LOW_FIRST_SEGMENT final LB/RB anchors and FINAL_HOLD fixed LB/RB anchors",
        "landing_targets_by_leg": targets,
        "surface_normal_correction_dz_by_leg_m": normal_dz_corrections,
        "terminal_ik_seed_source": "v2 terminal_q_rad; unchanged FINAL_HOLD body pose and all six final anchors",
        "terminal_ik_max_foot_residual_m": residual,
        "terminal_ik_min_joint_margin_rad": margin,
        "stage_count_unchanged": len(candidate["stages"]) == 21,
        "AH_air_knots_and_timing_unchanged": True,
        "preceding_stages_byte_identical": True,
    }
    ClimbMode(None)._validate_config(candidate)
    return candidate


def refine_paired_mid_inset(candidate):
    """Move only the terminal M support forward to open the frozen B descent."""
    source_bytes = (json.dumps(candidate, indent=2, allow_nan=False) + "\n").encode()
    source_sha = hashlib.sha256(source_bytes).hexdigest()
    if source_sha != REAR_V1_SHA:
        raise ValueError(f"paired-inset source must be reviewed rear v1, got {source_sha}")
    candidate = copy.deepcopy(candidate)
    local_triangles = load_triangles(MESH)
    world_triangles = local_triangles + TRANSLATION
    stages = candidate["stages"]
    mid_index = next(i for i, stage in enumerate(stages)
                     if stage["name"] == "BODY_MID_0235_SYNC")
    mid = stages[mid_index]
    mid["name"] = "BODY_MID_FINAL_SYNC"
    knots = np.asarray(mid["anchor_knots"], dtype=float)
    landings = {}
    corrections = {}
    for leg in (2, 5):
        y = float(knots[-1, leg, 1])
        surface_z = float(surface_height(local_triangles, .255, y))
        point = np.array((.255, y, surface_z), dtype=float)
        gap = float(foot_sphere_signed_gap(world_triangles, point))
        dz = max(0.0, -gap) / float(patch_helper.BROAD_SLOPE_NORMAL[2])
        point[2] += dz
        gap = float(foot_sphere_signed_gap(world_triangles, point))
        if not -.0005 <= gap <= .002:
            raise ValueError(f"{LEGS[leg]} x=.255 landing gap {gap:.6f}m outside STL gate")
        nearest = patch_helper.inspect_nearest_patch(world_triangles, point)
        if not any(float(np.dot(np.asarray(item["face_normal_world"]),
                                patch_helper.BROAD_SLOPE_NORMAL)) >= .995
                   and item["inside_broad_patch_with_10mm_margin"] for item in nearest):
            raise ValueError(f"{LEGS[leg]} x=.255 landing is not on broad slope with 10mm margin")
        landings[leg] = point
        corrections[LEGS[leg]] = {"surface_height_z_m": surface_z,
                                  "normal_offset_dz_m": float(dz),
                                  "sphere_gap_m": gap, "xyz_m": point.tolist()}
        knots[-1, leg] = point
        apex = point[2] + .005
        knots[1, leg] = (.205, y, apex)
        knots[2, leg] = (.255, y, apex)
    mid["anchor_knots"] = knots.tolist()
    _refresh_active_base_knots(mid)

    ah = next(stage for stage in stages[mid_index + 1:]
              if stage["name"] == "AH_BACK_LOW_FIRST_SEGMENT")
    ah_knots = np.asarray(ah["anchor_knots"], dtype=float)
    for leg, point in landings.items():
        ah_knots[:, leg] = point
    ah["anchor_knots"] = ah_knots.tolist()
    hold = next(stage for stage in stages if stage["name"] == "FINAL_HOLD")
    hold_knots = np.asarray(hold["anchor_knots"], dtype=float)
    for leg, point in landings.items():
        hold_knots[:, leg] = point
    hold["anchor_knots"] = hold_knots.tolist()

    final_anchors = hold_knots[-1]
    pose = np.asarray(hold["pose_end"], dtype=float)
    kin = GraspKinematic()
    seed = np.asarray(candidate["terminal_q_rad"], dtype=float)
    q, residual = solve_ik(kin, pose, final_anchors, seed, tolerance_m=1e-5)
    margin = float(np.min(np.minimum(q - JOINT_LOWER, JOINT_UPPER - q)))
    if residual > .004 or margin <= 0.0:
        raise ValueError(f"paired terminal IK failed: residual={residual:.6f}m, margin={margin:.6f}rad")
    candidate["terminal_q_rad"] = q.tolist()
    candidate["paired_mid_inset_refinement"] = {
        "source_rear_v1_config_sha256": source_sha,
        "renamed_stage": ["BODY_MID_0235_SYNC", "BODY_MID_FINAL_SYNC"],
        "final_mid_landing_by_leg": corrections,
        "M_air_knots": "first air x=.205; second air x=.255; both z=corrected M landing z + .005 m",
        "fixed_M_anchor_propagation": ["AH_BACK_LOW_FIRST_SEGMENT", "FINAL_HOLD"],
        "terminal_ik_seed_source": "rear v1 terminal_q_rad; unchanged final body pose and updated six final anchors",
        "terminal_ik_max_foot_residual_m": residual,
        "terminal_ik_min_joint_margin_rad": margin,
        "stage_count_unchanged": len(stages) == 21,
        "rear_air_knots_timing_body_unchanged": True,
        "preceding_stages_byte_identical": True,
    }
    ClimbMode(None)._validate_config(candidate)
    return candidate


def refine_rear_narrow(candidate):
    """Move only terminal LB/RB y coordinates inward from the reviewed rear v1."""
    source_bytes = (json.dumps(candidate, indent=2, allow_nan=False) + "\n").encode()
    source_sha = hashlib.sha256(source_bytes).hexdigest()
    if source_sha != REAR_V1_SHA:
        raise ValueError(f"rear-narrow source must be reviewed rear v1, got {source_sha}")
    candidate = copy.deepcopy(candidate)
    local_triangles = load_triangles(MESH)
    world_triangles = local_triangles + TRANSLATION
    ah = next(stage for stage in candidate["stages"]
              if stage["name"] == "AH_BACK_LOW_FIRST_SEGMENT")
    hold = next(stage for stage in candidate["stages"] if stage["name"] == "FINAL_HOLD")
    terminal = np.asarray(ah["anchor_knots"][-1], dtype=float)
    targets, corrections = {}, {}
    for leg, dy in ((0, -.015), (3, .015)):
        x = .225
        y = float(terminal[leg, 1] + dy)
        surface_z = float(surface_height(local_triangles, x, y))
        point = np.array((x, y, surface_z), dtype=float)
        gap0 = float(foot_sphere_signed_gap(world_triangles, point))
        dz = max(0.0, -gap0) / float(patch_helper.BROAD_SLOPE_NORMAL[2])
        point[2] += dz
        gap = float(foot_sphere_signed_gap(world_triangles, point))
        if not -.0005 <= gap <= .002:
            raise ValueError(f"{LEGS[leg]} x=.225 y={y:.6f} landing gap {gap:.6f}m outside STL gate")
        nearest = patch_helper.inspect_nearest_patch(world_triangles, point)
        if not any(float(np.dot(np.asarray(item["face_normal_world"]),
                                patch_helper.BROAD_SLOPE_NORMAL)) >= .995
                   and item["inside_broad_patch_with_10mm_margin"] for item in nearest):
            raise ValueError(f"{LEGS[leg]} x=.225 y={y:.6f} is outside intended broad patch")
        terminal[leg] = point
        targets[LEGS[leg]] = {"xyz_m": point.tolist(), "surface_height_z_m": surface_z,
                              "normal_offset_dz_m": float(dz), "sphere_gap_m": gap,
                              "broad_patch_edge_inset_m": max(
                                  item["clearance_inward_from_broad_patch_edge_m"]
                                  for item in nearest
                                  if float(np.dot(np.asarray(item["face_normal_world"]),
                                                  patch_helper.BROAD_SLOPE_NORMAL)) >= .995
                              )}
        corrections[LEGS[leg]] = float(dz)
    ah["anchor_knots"][-1] = terminal.tolist()
    _refresh_active_base_knots(ah)
    hold_knots = np.asarray(hold["anchor_knots"], dtype=float)
    for leg in (0, 3):
        hold_knots[:, leg] = terminal[leg]
    hold["anchor_knots"] = hold_knots.tolist()
    kin = GraspKinematic()
    pose = np.asarray(hold["pose_end"], dtype=float)
    q, residual = solve_ik(kin, pose, hold_knots[-1],
                           np.asarray(candidate["terminal_q_rad"], dtype=float),
                           tolerance_m=1e-5)
    margin = float(np.min(np.minimum(q - JOINT_LOWER, JOINT_UPPER - q)))
    if residual > .004 or margin <= 0.0:
        raise ValueError(f"rear-narrow terminal IK failed: residual={residual:.6f}m, margin={margin:.6f}rad")
    candidate["terminal_q_rad"] = q.tolist()
    candidate["rear_narrow_refinement"] = {
        "source_rear_v1_config_sha256": source_sha,
        "terminal_y_offsets_m": {"LB": -.015, "RB": .015},
        "landing_targets_by_leg": targets,
        "surface_normal_correction_dz_by_leg_m": corrections,
        "AH_air_knots_unchanged": True,
        "M_F_body_timing_prefix_unchanged": True,
        "stage_count_unchanged": len(candidate["stages"]) == 21,
        "terminal_ik_seed_source": "rear v1 terminal_q_rad; unchanged final body pose and updated six final anchors",
        "terminal_ik_max_foot_residual_m": residual,
        "terminal_ik_min_joint_margin_rad": margin,
    }
    ClimbMode(None)._validate_config(candidate)
    return candidate


def refine_rear_body_advance(candidate):
    """Advance the final body pose within the existing AH lift only."""
    source_bytes = (json.dumps(candidate, indent=2, allow_nan=False) + "\n").encode()
    source_sha = hashlib.sha256(source_bytes).hexdigest()
    if source_sha != REAR_V1_SHA:
        raise ValueError(f"rear-body source must be reviewed rear v1, got {source_sha}")
    candidate = copy.deepcopy(candidate)
    ah = next(stage for stage in candidate["stages"]
              if stage["name"] == "AH_BACK_LOW_FIRST_SEGMENT")
    hold = next(stage for stage in candidate["stages"] if stage["name"] == "FINAL_HOLD")
    ah_end = np.asarray(ah["pose_end"], dtype=float)
    ah_end[0] = .318
    ah["pose_end"] = ah_end.tolist()
    _refresh_active_base_knots(ah)
    hold_start, hold_end = np.asarray(hold["pose_start"], dtype=float), np.asarray(hold["pose_end"], dtype=float)
    hold_start[0] = hold_end[0] = .318
    hold["pose_start"], hold["pose_end"] = hold_start.tolist(), hold_end.tolist()
    pose = hold_end
    anchors = np.asarray(hold["anchor_knots"][-1], dtype=float)
    q, residual = solve_ik(GraspKinematic(), pose, anchors,
                           np.asarray(candidate["terminal_q_rad"], dtype=float),
                           tolerance_m=1e-5)
    margin = float(np.min(np.minimum(q - JOINT_LOWER, JOINT_UPPER - q)))
    if residual > .004 or margin <= 0.0:
        raise ValueError(f"terminal IK failed: residual={residual:.6f}m, margin={margin:.6f}rad")
    candidate["terminal_q_rad"] = q.tolist()
    candidate["rear_body_advance_refinement"] = {
        "source_rear_v1_config_sha256": source_sha,
        "changed_body_pose_fields": "AH_BACK_LOW_FIRST_SEGMENT.pose_end.x; FINAL_HOLD.pose_start.x and pose_end.x",
        "terminal_body_pose": hold_end.tolist(),
        "feet_world_anchors_unchanged": True,
        "all_timing_unchanged": True,
        "stage_count_unchanged": len(candidate["stages"]) == 21,
        "terminal_ik_seed_source": "rear v1 terminal_q_rad; unchanged final world anchors",
        "terminal_ik_max_foot_residual_m": residual,
        "terminal_ik_min_joint_margin_rad": margin,
    }
    ClimbMode(None)._validate_config(candidate)
    return candidate


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=OUTPUT)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--mid-inset", action="store_true",
                       help="diagnostic: write the rejected mid-inset geometry from a7")
    modes.add_argument("--mid-inset-v2", action="store_true",
                       help="diagnostic: write the rejected corrected mid-inset v2 geometry")
    modes.add_argument("--rear-inset", action="store_true",
                       help="diagnostic: write the rejected terminal rear-inset candidate")
    modes.add_argument("--paired-inset", action="store_true",
                       help="diagnostic: write the rejected forward-M candidate")
    modes.add_argument("--rear-narrow", action="store_true",
                       help="diagnostic: write the rejected inward-y rear candidate")
    modes.add_argument("--rear-body-advance", action="store_true",
                       help="write accepted final body x advance from reviewed rear-inset")
    modes.add_argument("--historical-fast", action="store_true",
                       help="diagnostic: rebuild the rejected a7 timing candidate (old no-flag mode)")
    args = parser.parse_args()
    if not any((args.mid_inset, args.mid_inset_v2, args.rear_inset,
                args.paired_inset, args.rear_narrow, args.rear_body_advance,
                args.historical_fast)):
        source_bytes = REAR_V1.read_bytes()
        if hashlib.sha256(source_bytes).hexdigest() != REAR_V1_SHA:
            raise ValueError("preserved rear-inset candidate SHA mismatch")
        accepted_path = ROOT / "src/docs/evidence/climb_front_direct_20260924/rear_body_advance_candidate.json"
        expected_bytes = accepted_path.read_bytes()
        expected_sha = "4421aff133035f358e69ed33ddaa649d580750d868f2318736efc17e797bf60c"
        if hashlib.sha256(expected_bytes).hexdigest() != expected_sha:
            raise ValueError("preserved accepted rear-body-advance snapshot SHA mismatch")
        config = json.loads(expected_bytes)
        if config.get("rear_body_advance_refinement", {}).get("source_rear_v1_config_sha256") != REAR_V1_SHA:
            raise ValueError("accepted snapshot is not derived from the reviewed rear-inset source")
        ClimbMode(None)._validate_config(config)
        output_bytes = expected_bytes
        if args.output.resolve() == (ROOT / "src/docs/evidence/climb_front_direct_20260924/rear_body_advance_candidate.json").resolve():
            raise ValueError("refusing to overwrite the accepted source snapshot")
        args.output.write_bytes(output_bytes)
        print(json.dumps({"output": str(args.output), "stage_count": config["stage_count"],
                          "sha256": expected_sha, "source_rear_v1_sha256": REAR_V1_SHA,
                          "mode": "accepted rear-body-advance"}, indent=2))
        return
    if args.rear_body_advance:
        source_bytes = REAR_V1.read_bytes()
        if hashlib.sha256(source_bytes).hexdigest() != REAR_V1_SHA:
            raise ValueError("preserved rear-inset candidate SHA mismatch")
        config = refine_rear_body_advance(json.loads(source_bytes))
        output_bytes = (json.dumps(config, indent=2, allow_nan=False) + "\n").encode()
        args.output.write_bytes(output_bytes)
        print(json.dumps({"output": str(args.output), "stage_count": config["stage_count"],
                          "sha256": hashlib.sha256(output_bytes).hexdigest(),
                          "source_rear_v1_sha256": REAR_V1_SHA,
                          "rear_body_advance_refinement": config["rear_body_advance_refinement"]}, indent=2))
        return
    if args.rear_narrow:
        source_bytes = REAR_V1.read_bytes()
        if hashlib.sha256(source_bytes).hexdigest() != REAR_V1_SHA:
            raise ValueError("preserved rear-inset candidate SHA mismatch")
        config = refine_rear_narrow(json.loads(source_bytes))
        output_bytes = (json.dumps(config, indent=2, allow_nan=False) + "\n").encode()
        args.output.write_bytes(output_bytes)
        print(json.dumps({"output": str(args.output), "stage_count": config["stage_count"],
                          "sha256": hashlib.sha256(output_bytes).hexdigest(),
                          "source_rear_v1_sha256": REAR_V1_SHA,
                          "rear_narrow_refinement": config["rear_narrow_refinement"]}, indent=2))
        return
    if args.paired_inset:
        source_bytes = REAR_V1.read_bytes()
        if hashlib.sha256(source_bytes).hexdigest() != REAR_V1_SHA:
            raise ValueError("preserved rear-inset candidate SHA mismatch")
        config = refine_paired_mid_inset(json.loads(source_bytes))
        output_bytes = (json.dumps(config, indent=2, allow_nan=False) + "\n").encode()
        args.output.write_bytes(output_bytes)
        print(json.dumps({"output": str(args.output), "stage_count": config["stage_count"],
                          "sha256": hashlib.sha256(output_bytes).hexdigest(),
                          "source_rear_v1_sha256": REAR_V1_SHA,
                          "paired_mid_inset_refinement": config["paired_mid_inset_refinement"]}, indent=2))
        return
    if args.rear_inset:
        source_bytes = MID_V2.read_bytes()
        if hashlib.sha256(source_bytes).hexdigest() != MID_V2_SHA:
            raise ValueError("preserved v2 snapshot SHA mismatch")
        source = json.loads(source_bytes)
        config = refine_terminal_rear_inset(source)
        output_bytes = (json.dumps(config, indent=2, allow_nan=False) + "\n").encode()
        args.output.write_bytes(output_bytes)
        print(json.dumps({"output": str(args.output), "stage_count": config["stage_count"],
                          "sha256": hashlib.sha256(output_bytes).hexdigest(),
                          "source_v2_sha256": MID_V2_SHA,
                          "terminal_rear_inset_refinement": config["terminal_rear_inset_refinement"]}, indent=2))
        return
    if args.mid_inset or args.mid_inset_v2:
        source_bytes = SNAPSHOT.read_bytes()
        source_sha = hashlib.sha256(source_bytes).hexdigest()
        if source_sha != MID_INSET_SOURCE_SHA:
            raise ValueError("preserved fast_candidate.json is not the reviewed a7 input")
        config = refine_initial_mid_contact(
            json.loads(source_bytes), second_air_z=.13 if args.mid_inset_v2 else .12,
            landing_relative_mid_air=args.mid_inset,
        )
        if args.mid_inset_v2:
            config["mid_inset_refinement"]["revision"] = "v2: second air knot z=.130 m; BODY_MID first air z remains terminal x=.235 M z + .005 m"
        output_bytes = (json.dumps(config, indent=2, allow_nan=False) + "\n").encode()
        args.output.write_bytes(output_bytes)
        print(json.dumps({"output": str(args.output), "stage_count": config["stage_count"],
                          "sha256": hashlib.sha256(output_bytes).hexdigest(),
                          "source_a7_sha256": source_sha,
                          "planned_duration_s": sum(sum(stage["segment_durations_s"]) + stage["settle_s"]
                                                     for stage in config["stages"])}, indent=2))
        return
    fast_bytes, endpoint_bytes = FAST.read_bytes(), ENDPOINTS.read_bytes()
    fast, receipt = json.loads(fast_bytes), json.loads(endpoint_bytes)
    expected = "847faeec51f258179090c59f736b773b02273e4662a0fc0bde200bd5e4ebf67c"
    if hashlib.sha256(fast_bytes).hexdigest() != expected:
        raise ValueError("fast source bytes differ from the reviewed v8 source")
    if receipt["source_config_sha256"] != expected:
        raise ValueError("v8 endpoint receipt does not bind to fast source")
    cpu_bytes, structural_bytes = CPU_STRUCTURAL.read_bytes(), STRUCTURAL.read_bytes()
    cpu_receipt = json.loads(cpu_bytes)
    structural = build(fast, receipt)
    structural, retry_bytes = apply_structural_retry(structural, cpu_receipt)
    if retry_bytes != structural_bytes:
        raise ValueError("persisted structural_21.json differs from rebuilt retry bytes")
    config = apply_audit_fast_timing(structural, cpu_receipt, cpu_bytes, structural_bytes)
    output_bytes = (json.dumps(config, indent=2, allow_nan=False) + "\n").encode()
    args.output.write_bytes(output_bytes)
    SNAPSHOT.write_bytes(output_bytes)
    print(json.dumps({"output": str(args.output), "stage_count": config["stage_count"],
                      "sha256": hashlib.sha256(output_bytes).hexdigest(),
                      "structural_sha256": hashlib.sha256(structural_bytes).hexdigest(),
                      "source_config_sha256": expected,
                      "cpu_audit_sha256": hashlib.sha256(cpu_bytes).hexdigest(),
                      "planned_duration_s": sum(
                          sum(stage["segment_durations_s"]) + stage["settle_s"]
                          for stage in config["stages"]
                      )}, indent=2))


if __name__ == "__main__":
    main()
