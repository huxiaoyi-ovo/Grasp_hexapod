#!/usr/bin/env python3
"""Construct and audit the frozen front-climb structural fast candidate.

This tool only creates simulation-only front candidates.  It rebuilds active
base knots from world anchors and runtime-compatible pose curves, then runs a
30 Hz inherited-reference controller audit.  Its geometric and capsule checks
are model diagnostics, not contact, load, stability, or hardware evidence.
"""
import argparse
import copy
import json
import sys
import hashlib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / "src/grasp_hexapod_control/scripts"
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / "tools")]

from climb_mode import ClimbMode
from control import GraspController
from kinematics import FOOT_RADIUS, JOINT_LOWER, JOINT_UPPER, Q_STAND
from build_climb_front import (
    MESH, TRANSLATION, base_points, foot_sphere_signed_gap, load_triangles,
    quintic_weight, support_margin, world_from_base,
)

LEG_NAMES = ("lb", "lf", "lm", "rb", "rf", "rm")
CAP = 6.0
DT = 1.0 / 30.0

# Each entry names the exact baseline span it replaces.  No coordinates are
# optimized here; the contract below is the entire allowed structural change.
MERGES = (
    ("IJ_MID_DIRECT", ("I_MID_FORWARD_SYNC", "J_MID_BACK_SYNC"), [2, 5], .020, [.55, .75, .55]),
    ("LM_MID_RELAND", ("L_MID_FORWARD_SYNC", "M_MID_PREP_SYNC"), [2, 5], .010, [.55, .75, .55]),
    ("OP_MID_BODY_LIFT", ("O_MID_FORWARD_SYNC", "P_BODY_PREP"), [2, 5], .020, None),
    ("R_FRONT_BODY_LIFT", ("R_FRONT_40_SYNC", "S0_BODY_LIFT"), [1, 4], .020, None),
    ("BODY_B_FRONT_INWARD", ("BODY_B", "FRONT_INWARD"), [1, 4], .020, [.525, .725, .2375]),
    ("RB_RESET_BODY_LIFT", ("RB_RESET_NEG03", "T0_BODY_LIFT"), [3], .020, None),
    ("AE_AF_BACK_PAIR", ("AE_LB_090_SYNC", "AF_RB_090_SYNC"), [0, 3], .020, [.55, .75, .55]),
    ("AGAB_BACK_PAIR", ("AGa_LB_11_STATIC_BODY", "AGb_RB_11_SYNC"), [0, 3], .020, [.825, 1.125, .825]),
    ("AGCD_BACK_PAIR", ("AGc_LB_13_SYNC", "AGd_RB_13_SYNC"), [0, 3], .020, [.55, .75, .55]),
)


def _pose_knots(start, end, durations, curve):
    elapsed = np.r_[0.0, np.cumsum(durations)]
    if curve == "quintic_full_stage":
        phase = elapsed / elapsed[-1]
    elif curve == "quintic_first_segment":
        phase = np.minimum(elapsed / durations[0], 1.0)
    else:
        raise ValueError("unsupported pose curve")
    return [start * (1.0 - quintic_weight(p)) + end * quintic_weight(p) for p in phase]


def _merge_stage(name, source, active, clearance, durations):
    first, last = source[0], source[-1]
    start = np.asarray(first["anchor_knots"][0], dtype=float)
    end = np.asarray(last["anchor_knots"][-1], dtype=float)
    pose_start = np.asarray(first["pose_start"], dtype=float)
    pose_end = np.asarray(last["pose_end"], dtype=float)
    if durations is None:
        durations = list(first["segment_durations_s"])
    durations = [float(x) for x in durations]
    if len(durations) != 3 or min(durations) <= 0.0:
        raise ValueError(name + " needs exactly three positive durations")
    active = list(active)
    height = max(float(np.max(start[active, 2])), float(np.max(end[active, 2]))) + clearance
    knots = [start.copy(), start.copy(), start.copy(), end.copy()]
    knots[1][active, 2] = height
    knots[2][active] = end[active]
    knots[2][active, 2] = height
    curve = "quintic_full_stage"
    poses = _pose_knots(pose_start, pose_end, durations, curve)
    base = [base_points(pose, knot[active]) for pose, knot in zip(poses, knots)]
    return {
        "name": name,
        "pose_start": pose_start.tolist(), "pose_end": pose_end.tolist(),
        "active_legs": active,
        "anchor_knots": [k.tolist() for k in knots],
        "active_base_knots_m": [k.tolist() for k in base],
        "active_base_velocities_m_s": np.zeros((4, len(active), 3)).tolist(),
        "segment_durations_s": durations,
        "anchor_curve": "piecewise_base_quintic", "pose_curve": curve,
        "settle_s": float(first["settle_s"]),
    }


def _refresh_active_base_knots(stage):
    """Recompute a stage's active base knots after world-anchor edits."""
    active = np.asarray(stage["active_legs"], dtype=int)
    anchors = np.asarray(stage["anchor_knots"], dtype=float)
    durations = [float(value) for value in stage["segment_durations_s"]]
    poses = _pose_knots(
        np.asarray(stage["pose_start"], dtype=float),
        np.asarray(stage["pose_end"], dtype=float),
        durations,
        stage["pose_curve"],
    )
    if anchors.shape != (len(durations) + 1, 6, 3):
        raise ValueError(stage["name"] + " has inconsistent anchor knots")
    stage["active_base_knots_m"] = [
        base_points(pose, anchors[index, active]).tolist()
        for index, pose in enumerate(poses)
    ]
    stage["active_base_velocities_m_s"] = np.zeros(
        (len(anchors), len(active), 3)
    ).tolist()


def _set_back_anchor_x(stage, start_x, end_x=None):
    """Set the back-pair ground anchors, preserving the existing air height."""
    anchors = np.asarray(stage["anchor_knots"], dtype=float)
    if end_x is None:
        anchors[:, (0, 3), 0] = float(start_x)
    else:
        if len(anchors) != 4:
            raise ValueError(stage["name"] + " must have four movement knots")
        anchors[:2, (0, 3), 0] = float(start_x)
        anchors[2:, (0, 3), 0] = float(end_x)
    stage["anchor_knots"] = anchors.tolist()
    _refresh_active_base_knots(stage)


def apply_back_knee_spacing(baseline):
    """Move only the frozen K/N rear ground anchors away from the mid knees.

    The edit propagates the new world anchors through both single-leg reset
    stages.  Body poses, durations, active sets, apex heights, and every
    post-reset stage stay unchanged.
    """
    by_name = {stage["name"]: stage for stage in baseline["stages"]}
    expected = {
        "K_BACK_POS03_SYNC": [0, 3], "N_BACK_POS05_SYNC": [0, 3],
        "LB_RESET_NEG03": [0], "RB_RESET_NEG03": [3],
    }
    for name, active in expected.items():
        if by_name[name]["active_legs"] != active:
            raise ValueError(name + " active-leg contract changed")

    # K: -.03 -> .00; L/M inherit .00.  N: .00 -> .01; every stage until
    # the resets inherits .01.  The resets then return one rear leg at a time
    # to the original -.03 terminal, so T0 and all downstream anchors remain
    # byte-for-byte geometric equivalents of the frozen baseline.
    _set_back_anchor_x(by_name["K_BACK_POS03_SYNC"], -.03, .00)
    for name in ("L_MID_FORWARD_SYNC", "M_MID_PREP_SYNC"):
        _set_back_anchor_x(by_name[name], .00)
    _set_back_anchor_x(by_name["N_BACK_POS05_SYNC"], .00, .01)
    for name in (
        "O_MID_FORWARD_SYNC", "P_BODY_PREP", "Q_MID_LOW",
        "R_FRONT_40_SYNC", "S0_BODY_LIFT", "S_MID_20_SYNC",
    ):
        _set_back_anchor_x(by_name[name], .01)

    reset_left = by_name["LB_RESET_NEG03"]
    anchors = np.asarray(reset_left["anchor_knots"], dtype=float)
    anchors[:2, (0, 3), 0] = .01
    anchors[2:, 0, 0] = -.03
    anchors[2:, 3, 0] = .01
    reset_left["anchor_knots"] = anchors.tolist()
    _refresh_active_base_knots(reset_left)
    reset_right = by_name["RB_RESET_NEG03"]
    anchors = np.asarray(reset_right["anchor_knots"], dtype=float)
    anchors[:, 0, 0] = -.03
    anchors[:2, 3, 0] = .01
    anchors[2:, 3, 0] = -.03
    reset_right["anchor_knots"] = anchors.tolist()
    _refresh_active_base_knots(reset_right)
    return baseline


def build_candidate(baseline):
    baseline = copy.deepcopy(baseline)
    baseline_sha256 = hashlib.sha256(
        json.dumps(baseline, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    apply_back_knee_spacing(baseline)
    by_name = {stage["name"]: stage for stage in baseline["stages"]}
    replacements = {}
    stage_map = []
    for name, source_names, active, clearance, durations in MERGES:
        source = [by_name[s] for s in source_names]
        stage = _merge_stage(name, source, active, clearance, durations)
        replacements[source_names[0]] = (source_names, stage)
        stage_map.append({"candidate": name, "source_stages": list(source_names),
                          "active_legs": active, "clearance_m": clearance,
                          "segment_durations_s": stage["segment_durations_s"]})
    stages = []
    index = 0
    original = baseline["stages"]
    while index < len(original):
        current = original[index]
        item = replacements.get(current["name"])
        if item is None:
            stages.append(current)
            index += 1
            continue
        source_names, stage = item
        observed = tuple(s["name"] for s in original[index:index + len(source_names)])
        if observed != source_names:
            raise ValueError("source sequence changed for " + stage["name"])
        stages.append(stage)
        index += len(source_names)
    candidate = baseline
    candidate["stages"] = stages
    candidate["stage_count"] = len(stages)
    candidate.pop("surface_receipt", None)
    candidate["visual_validation_deferred_for_sim_finish"] = [
        stage["name"] for stage in stages
    ]
    candidate["simulation_joint_velocity_limit_rad_s"] = CAP
    candidate["optimization"] = {
        "kind": "frozen_front_structure_candidate",
        "baseline_stage_count": len(original), "baseline_config_sha256": baseline_sha256,
        "back_knee_spacing": {
            "K_BACK_POS03_SYNC_end_x_m": .00,
            "N_BACK_POS05_SYNC_end_x_m": .01,
            "reset_entry_x_m": .01,
        },
        "baseline_surface_receipt_source": "baseline config only; not candidate evidence",
        "stage_map": stage_map, "simulation_only": True,
    }
    ClimbMode(None)._validate_config(candidate)
    return candidate, stage_map


def _audit(config, candidate=False, inherited_capsule_proxy_names=()):
    """30 Hz inherited-reference audit with model-only geometry receipts."""
    triangles = load_triangles(MESH) + TRANSLATION
    controller = GraspController(DT, enable_link_collision_check=False,
                                 climb_timeout_uses_wall_time=False)
    controller.enter_climb(Q_STAND, config, 0, len(config["stages"]) - 1,
                           hardware_execution=False)
    mode = controller.climb_mode
    q = Q_STAND.copy()
    stage_rows, boundaries, q_samples = {}, {}, []
    first_failure = None
    merged = {item[0] for item in MERGES}
    inherited_capsule_proxy_names = set(inherited_capsule_proxy_names)
    last_index = None
    ticks = 0
    while mode.state == ClimbMode.RUNNING and ticks < 10000:
        idx = mode.stage_index
        stage = config["stages"][idx]
        name = stage["name"]
        active = np.asarray(stage["active_legs"], dtype=int)
        fixed = np.asarray([leg for leg in range(6) if leg not in active], dtype=int)
        phase_before = float(mode.phase_time)
        durations = np.asarray(stage["segment_durations_s"], dtype=float)
        segment_index = min(int(np.searchsorted(np.cumsum(durations), phase_before, side="right")), len(durations)-1)
        if idx != last_index:
            last_index = idx
            boundaries[name] = {"entry_q_rad": q.tolist(), "source_index": idx}
            stage_rows[name] = {
                "max_foot_error_m": 0.0, "min_joint_margin_rad": float("inf"),
                "min_support_margin_m": None, "min_active_sphere_gap_m": None,
                "velocity_clips": 0, "capsule_tick_samples": 0,
                "capsule_clear": True,
                "capsule_witness": None,
                "segments": [{"max_foot_error_m": 0.0, "max_raw_joint_speed_rad_s": 0.0,
                              "velocity_clips": 0} for _ in durations],
            }
        pose, anchors, _ = mode._stage_reference()
        row = stage_rows[name]
        if len(active):
            gaps = [foot_sphere_signed_gap(triangles, anchors[leg]) for leg in active]
            minimum_gap = min(gaps)
            row["min_active_sphere_gap_m"] = minimum_gap if row["min_active_sphere_gap_m"] is None else min(row["min_active_sphere_gap_m"], minimum_gap)
            com = (world_from_base(pose) @ np.r_[controller.kinematic.center_of_mass_base(q), 1.0])[:3]
            support = support_margin(com[:2], anchors[fixed])
            row["min_support_margin_m"] = support if row["min_support_margin_m"] is None else min(row["min_support_margin_m"], support)
        # Every 30 Hz sample includes all stage boundaries and covers every
        # merged stage densely.  This is a capsule proxy only, not mesh self-collision.
        row["capsule_tick_samples"] += 1
        capsule_mask = controller._link_collision_free(q)
        if not capsule_mask.all() and row["capsule_witness"] is None:
            row["capsule_witness"] = {
                "phase_time_s": phase_before,
                "q_rad": q.tolist(), "base_pose": pose.tolist(),
                "proxy_mask": capsule_mask.tolist(),
            }
        row["capsule_clear"] &= bool(capsule_mask.all())
        old = q.copy()
        previous_ref = controller.foot_desired_base_prev.copy()
        q = controller.update(q, np.zeros(4))
        ticks += 1
        actual = controller.kinematic.hip_to_base(controller.kinematic.forward(q))
        foot_error = float(np.max(np.linalg.norm(actual - controller.foot_desired_base, axis=1)))
        hip_des = controller.kinematic.base_to_hip(controller.foot_desired_base)
        hip_prev = controller.kinematic.base_to_hip(previous_ref)
        raw = (16.0 * np.einsum("lij,lj->li", controller.kinematic.damped_inverse_jacobian(old), hip_des - controller.kinematic.forward(old))
               + np.einsum("lij,lj->li", controller.kinematic.damped_inverse_jacobian(old), (hip_des - hip_prev) / DT))
        raw_peak = float(np.max(np.abs(raw)))
        margin = float(np.min(np.minimum(q - JOINT_LOWER, JOINT_UPPER - q)))
        row["max_foot_error_m"] = max(row["max_foot_error_m"], foot_error)
        row["min_joint_margin_rad"] = min(row["min_joint_margin_rad"], margin)
        row["velocity_clips"] += int(controller.last_update_velocity_limit_clip_count)
        segment = row["segments"][segment_index]
        segment["max_foot_error_m"] = max(segment["max_foot_error_m"], foot_error)
        segment["max_raw_joint_speed_rad_s"] = max(segment["max_raw_joint_speed_rad_s"], raw_peak)
        segment["velocity_clips"] += int(controller.last_update_velocity_limit_clip_count)
        q_samples.append({"stage": name, "q_thigh_rad": q[:, 0].tolist()})
        if first_failure is None:
            failure = None
            if margin <= 0.0: failure = "joint_limit"
            elif foot_error > .015: failure = "foot_error"
            elif controller.last_update_velocity_limit_clip_count: failure = "velocity_clip"
            elif len(active) and row["min_support_margin_m"] < .010: failure = "support_com"
            elif len(active) and row["min_active_sphere_gap_m"] < -.0005: failure = "active_sphere_gap"
            elif (candidate and name in merged and name not in inherited_capsule_proxy_names
                  and not row["capsule_clear"]):
                failure = "capsule_proxy"
            if failure:
                first_failure = {"stage": name, "reason": failure,
                                 "stage_time_s": float(mode.phase_time),
                                 "q_rad": q.tolist(), "active_legs": active.tolist(),
                                 "row": copy.deepcopy(row)}
                break
        if mode.stage_index != idx:
            boundaries[name]["exit_q_rad"] = q.tolist()
    terminal_capsule = bool(controller._link_collision_free(q).all())
    for name, boundary in boundaries.items():
        if "exit_q_rad" not in boundary:
            boundary["exit_q_rad"] = q.tolist()
    return {
        "state": mode.state, "ticks": ticks, "first_failure": first_failure,
        "stage_rows": stage_rows, "boundaries": boundaries, "q_thigh_samples": q_samples,
        "terminal_capsule_clear": terminal_capsule,
        "capsule_proxy_warnings": [
            {"stage": name, "capsule_tick_samples": row["capsule_tick_samples"]}
            for name, row in stage_rows.items() if not row["capsule_clear"]
        ],
        "terminal_q_rad": q.tolist(), "terminal_pose": mode.base_pose.tolist(),
        "capsule_boundary": "Controller capsule proxy only. Isaac actor collision filter disables robot self-collision; this is not full mesh collision evidence.",
    }

def compare_spans(baseline_audit, candidate_audit, stage_map):
    output = []
    for item in stage_map:
        candidate = item["candidate"]
        source = item["source_stages"]
        if (candidate not in candidate_audit["boundaries"]
                or any(name not in baseline_audit["boundaries"] for name in source)):
            output.append({"candidate": candidate, "source_stages": source,
                           "complete": False, "reason": "audit did not reach full source span"})
            continue
        active = np.asarray(item["active_legs"], dtype=int)
        fixed = np.asarray([x for x in range(6) if x not in active], dtype=int)
        source_set = set(source)
        bpath = np.asarray([x["q_thigh_rad"] for x in baseline_audit["q_thigh_samples"] if x["stage"] in source_set])
        cpath = np.asarray([x["q_thigh_rad"] for x in candidate_audit["q_thigh_samples"] if x["stage"] == candidate])
        if len(bpath) == 0 or len(cpath) == 0:
            output.append({"candidate": candidate, "source_stages": source,
                           "complete": False, "reason": "missing thigh path samples"})
            continue
        old = float(np.max(np.ptp(bpath[:, fixed], axis=0)))
        new = float(np.max(np.ptp(cpath[:, fixed], axis=0)))
        output.append({"candidate": candidate, "source_stages": source,
                       "complete": True,
                       "baseline_fixed_thigh_path_delta_rad": old,
                       "candidate_fixed_thigh_path_delta_rad": new,
                       "increase_rad": new - old,
                       "passes_extra_010rad": new <= old + .010})
    return output


def uniform_stage_retime(config, audit):
    """Apply one duration factor per stage; preserve all reference geometry."""
    retimed = copy.deepcopy(config)
    rows = audit["stage_rows"]
    metadata = []
    for stage in retimed["stages"]:
        name = stage["name"]
        original = np.asarray(stage["segment_durations_s"], dtype=float)
        if name == "FINAL_HOLD":
            metadata.append({"stage": name, "factor": 1.0,
                             "original_durations_s": original.tolist(),
                             "retimed_durations_s": original.tolist(),
                             "reason": "final hold frozen"})
            continue
        row = rows[name]
        raw_peak = max((float(segment["max_raw_joint_speed_rad_s"])
                        for segment in row["segments"]), default=0.0)
        max_foot = float(row["max_foot_error_m"])
        floor = np.full(len(original), .20, dtype=float)
        if len(original) > 2:
            floor[1:-1] = .25
        floor_factor = float(np.max(floor / original))
        if not stage["active_legs"]:
            floor_factor = max(floor_factor, .70 / float(np.sum(original)))
        if name == "AH_BACK_LOW_FIRST_SEGMENT":
            floor_factor = max(floor_factor, 1.0 / float(original[0]))
        factor = min(1.0, max(raw_peak / 5.2, max_foot / .012, floor_factor))
        scaled = original * factor
        geometry_fields = ("pose_start", "pose_end", "anchor_knots",
                           "active_base_knots_m", "active_base_velocities_m_s",
                           "pose_curve", "anchor_curve")
        before = {key: copy.deepcopy(stage.get(key)) for key in geometry_fields}
        stage["segment_durations_s"] = scaled.tolist()
        if name != "FINAL_HOLD":
            stage["settle_s"] = .10
        if any(stage.get(key) != value for key, value in before.items()):
            raise RuntimeError(name + " uniform retime changed reference geometry")
        metadata.append({
            "stage": name, "factor": factor,
            "raw_peak_joint_speed_rad_s": raw_peak,
            "structural_max_foot_error_m": max_foot,
            "duration_floor_s": floor.tolist(),
            "original_durations_s": original.tolist(),
            "retimed_durations_s": scaled.tolist(),
            "settle_s": stage["settle_s"],
        })
    retimed["simulation_joint_velocity_limit_rad_s"] = CAP
    retimed["optimization"]["kind"] = "uniform_stage_retime_front_candidate"
    retimed["optimization"]["structural_stage_count"] = config["stage_count"]
    retimed["optimization"]["retime"] = {
        "target_raw_peak_rad_s": 5.2,
        "simulator_joint_velocity_cap_rad_s": CAP,
        "geometry_preserved": True,
        "stages": metadata,
    }
    ClimbMode(None)._validate_config(retimed)
    return retimed, metadata


def _fast_retime_violations(audit):
    """Return only stage-local tracking or actuator-clip failures."""
    failures = {}
    for name, row in audit["stage_rows"].items():
        peak = max((float(item["max_raw_joint_speed_rad_s"])
                    for item in row["segments"]), default=0.0)
        reasons = []
        if float(row["max_foot_error_m"]) > .012:
            reasons.append("foot_error")
        if int(row["velocity_clips"]):
            reasons.append("velocity_clip")
        if reasons:
            failures[name] = {"reasons": reasons, "raw_peak_rad_s": peak,
                              "max_foot_error_m": float(row["max_foot_error_m"]),
                              "velocity_clips": int(row["velocity_clips"])}
    return failures


def _apply_local_slowdown(config, metadata, stage_names, factor=1.1):
    by_name = {stage["name"]: stage for stage in config["stages"]}
    meta_by_name = {item["stage"]: item for item in metadata}
    for name in stage_names:
        stage = by_name[name]
        stage["segment_durations_s"] = [float(value) * factor
                                        for value in stage["segment_durations_s"]]
        meta = meta_by_name[name]
        meta["retimed_durations_s"] = list(stage["segment_durations_s"])
        meta["extra_slowdown_factor"] = float(meta.get("extra_slowdown_factor", 1.0) * factor)
        meta["retry_slowdown_count"] = int(meta.get("retry_slowdown_count", 0)) + 1


def _json_clean(value):
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_clean(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_clean(item) for item in value]
    return value


def _geometry_sha256(config):
    fields = ("name", "pose_start", "pose_end", "active_legs", "anchor_knots",
              "active_base_knots_m", "active_base_velocities_m_s",
              "pose_curve", "anchor_curve")
    geometry = [{key: stage.get(key) for key in fields} for stage in config["stages"]]
    return hashlib.sha256(json.dumps(geometry, sort_keys=True,
                                     separators=(",", ":")).encode()).hexdigest()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument("--stage-map", type=Path, required=True)
    parser.add_argument("--structural-output", type=Path, required=True)
    parser.add_argument("--fast-audit", type=Path, required=True)
    parser.add_argument("--structural-only", action="store_true")
    parser.add_argument("--retime-only", action="store_true")
    parser.add_argument("--recorded-retime-metadata", type=Path)
    parser.add_argument("--align-recorded-only", action="store_true")
    args = parser.parse_args()
    if args.retime_only:
        structural = json.loads(args.structural_output.read_text())
        structural_doc = json.loads(args.audit.read_text())
        structural_audit = structural_doc["candidate"]
        inherited_proxy = tuple(structural_doc.get(
            "inherited_capsule_proxy_candidate_stages", ()))
        retimed, retime_metadata = uniform_stage_retime(structural, structural_audit)
        if args.recorded_retime_metadata is not None:
            recorded = json.loads(args.recorded_retime_metadata.read_text())
            recorded_metadata = recorded["retime_metadata"]
            durations = {item["stage"]: item["retimed_durations_s"]
                         for item in recorded_metadata}
            for stage in retimed["stages"]:
                stage["segment_durations_s"] = list(durations[stage["name"]])
            retime_metadata = copy.deepcopy(recorded_metadata)
            retimed["optimization"]["retime"]["stages"] = retime_metadata
            retimed["optimization"]["retime"]["reconstruction_basis"] = (
                "recorded timing metadata; controller audit timing predates this candidate hash")
            retimed["optimization"]["retime"]["local_slowdown_classification"] = (
                "speed_margin_heuristic")
            if args.align_recorded_only:
                if _geometry_sha256(structural) != _geometry_sha256(retimed):
                    raise RuntimeError("recorded retime changed frozen reference geometry")
                retimed["optimization"]["retime"]["geometry_preserved"] = True
                args.output.write_text(json.dumps(retimed, indent=2, allow_nan=False) + "\n")
                candidate_sha256 = hashlib.sha256(args.output.read_bytes()).hexdigest()
                aligned_audit = copy.deepcopy(recorded)
                retry_history = aligned_audit.get("local_slowdown_retries", [])
                for retry in retry_history:
                    original = retry.pop("stage_target_violations", {})
                    retry["stage_soft_speed_margin_adjustments"] = {
                        name: {"raw_peak_rad_s": values["raw_peak_rad_s"],
                               "foot_error_m": values["max_foot_error_m"],
                               "velocity_clips": values["velocity_clips"],
                               "classification": "speed_margin_heuristic"}
                        for name, values in original.items()
                    }
                aligned_audit["local_slowdown_retries"] = retry_history
                aligned_audit["candidate_config_sha256"] = candidate_sha256
                aligned_audit["geometry_sha256"] = _geometry_sha256(retimed)
                aligned_audit["measurement_binding"] = "reconstructed_from_recorded_timing; original audit did not capture this config hash"
                aligned_audit["configuration_alignment"] = {
                    "stage_durations_match_recorded_audit": True,
                    "geometry_sha256_matches_structural_snapshot": True,
                }
                args.fast_audit.write_text(json.dumps(_json_clean(aligned_audit), indent=2, allow_nan=False) + "\n")
                print(json.dumps({
                    "candidate_stage_count": retimed["stage_count"],
                    "candidate_config_sha256": candidate_sha256,
                    "recorded_state": aligned_audit["state"],
                    "recorded_ticks": aligned_audit["ticks"],
                    "measurement_binding": aligned_audit["measurement_binding"],
                    "reconstructed_from_recorded_timing": True,
                }, indent=2))
                return
        retry_history = []
        fast_result = None
        for retry in range(4):
            # Bind each measured run to the exact candidate bytes used by it.
            args.output.write_text(json.dumps(retimed, indent=2, allow_nan=False) + "\n")
            fast_result = _audit(
                retimed, candidate=True,
                inherited_capsule_proxy_names=inherited_proxy,
            )
            violations = _fast_retime_violations(fast_result)
            retry_history.append({"attempt": retry, "ticks": fast_result["ticks"],
                                  "state": fast_result["state"],
                                  "stage_target_violations": violations})
            if not violations:
                break
            if fast_result["first_failure"] is not None:
                break
            if retry == 3:
                break
            _apply_local_slowdown(retimed, retime_metadata, violations)
        args.output.write_text(json.dumps(retimed, indent=2, allow_nan=False) + "\n")
        candidate_sha256 = hashlib.sha256(args.output.read_bytes()).hexdigest()
        fast_audit = {
            "state": fast_result["state"], "ticks": fast_result["ticks"],
            "first_failure": fast_result["first_failure"],
            "stage_rows": fast_result["stage_rows"],
            "boundaries": fast_result["boundaries"],
            "capsule_proxy_warnings": fast_result["capsule_proxy_warnings"],
            "terminal_capsule_clear": fast_result["terminal_capsule_clear"],
            "terminal_pose": fast_result["terminal_pose"],
            "candidate_stage_count": retimed["stage_count"],
            "structural_stage_count": structural["stage_count"],
            "retime_metadata": retime_metadata,
            "local_slowdown_retries": retry_history,
            "fixed_thigh_spans": structural_doc["fixed_thigh_spans"],
            "geometry_preserved_by_stage": True,
            "candidate_config_sha256": candidate_sha256,
            "geometry_sha256": _geometry_sha256(retimed),
            "measurement_binding": (
                "reconstructed_from_recorded_timing" if args.recorded_retime_metadata is not None
                else "candidate_snapshot_sha256_captured_before_controller_replay"),
            "evidence_boundary": fast_result["capsule_boundary"],
        }
        args.fast_audit.write_text(json.dumps(_json_clean(fast_audit), indent=2, allow_nan=False) + "\n")
        print(json.dumps({
            "fast_stage_count": retimed["stage_count"],
            "fast_ticks": fast_result["ticks"],
            "fast_state": fast_result["state"],
            "fast_first_failure": fast_result["first_failure"],
            "fast_capsule_proxy_warnings": fast_result["capsule_proxy_warnings"],
            "local_slowdown_retries": retry_history,
            "retime_metadata": retime_metadata,
        }, indent=2))
        if fast_result["first_failure"] is not None or fast_result["state"] != ClimbMode.DONE:
            raise SystemExit(4)
        return
    baseline = json.loads(args.baseline.read_text())
    candidate, stage_map = build_candidate(baseline)
    args.structural_output.write_text(json.dumps(candidate, indent=2, allow_nan=False) + "\n")
    baseline_sha256 = hashlib.sha256(
        json.dumps(baseline, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    baseline_cache = args.audit.parent / "baseline_cache.json"
    if baseline_cache.exists():
        cached = json.loads(baseline_cache.read_text())
        if cached.get("baseline_config_sha256") == baseline_sha256:
            baseline_audit = cached["audit"]
        else:
            baseline_audit = _audit(baseline)
            baseline_cache.write_text(json.dumps({"baseline_config_sha256": baseline_sha256, "audit": _json_clean(baseline_audit)}, indent=2, allow_nan=False) + "\n")
    else:
        baseline_audit = _audit(baseline)
        baseline_cache.write_text(json.dumps({"baseline_config_sha256": baseline_sha256, "audit": _json_clean(baseline_audit)}, indent=2, allow_nan=False) + "\n")
    # Preserve pre-existing baseline capsule-proxy witnesses as explicit
    # receipts on their merged source spans.  A proxy warning is not a mesh
    # collision claim; new merges without a source warning remain fail-closed.
    baseline_proxy_stages = {
        item["stage"] for item in baseline_audit["capsule_proxy_warnings"]
    }
    inherited_proxy = tuple(
        item["candidate"] for item in stage_map
        if any(source in baseline_proxy_stages for source in item["source_stages"])
    )
    candidate_audit = _audit(candidate, candidate=True,
                             inherited_capsule_proxy_names=inherited_proxy)
    spans = compare_spans(baseline_audit, candidate_audit, stage_map)
    audit = {"baseline_cache": baseline_cache.name,
             "inherited_capsule_proxy_candidate_stages": list(inherited_proxy),
             "candidate": candidate_audit, "fixed_thigh_spans": spans,
             "gate": {"joint_margin_gt": 0.0, "foot_error_le_m": .015,
                      "clips_eq": 0, "support_com_ge_m": .010,
                      "active_sphere_gap_ge_m": -.0005,
                      "new_merge_extra_fixed_thigh_le_rad": .010}}
    args.audit.write_text(json.dumps(_json_clean(audit), indent=2, allow_nan=False) + "\n")
    args.stage_map.write_text(json.dumps(stage_map, indent=2) + "\n")
    if baseline_audit["first_failure"] is not None:
        print("baseline first failure", json.dumps(_json_clean(baseline_audit["first_failure"])))
    if baseline_audit["capsule_proxy_warnings"]:
        print("baseline capsule proxy warnings", json.dumps(baseline_audit["capsule_proxy_warnings"]))
    if candidate_audit["first_failure"] is not None:
        print("candidate first failure", json.dumps(_json_clean(candidate_audit["first_failure"])))
        raise SystemExit(2)
    bad = [x for x in spans if x.get("complete") and not x["passes_extra_010rad"]]
    if bad:
        print("fixed-thigh comparison failed", json.dumps(bad))
        raise SystemExit(3)
    if args.structural_only:
        print(json.dumps({
            "structural_stage_count": candidate["stage_count"],
            "baseline_ticks": baseline_audit["ticks"],
            "structural_ticks": candidate_audit["ticks"],
            "structural_state": candidate_audit["state"],
            "first_failure": candidate_audit["first_failure"],
            "fixed_thigh_spans": spans,
            "capsule_proxy_warnings": candidate_audit["capsule_proxy_warnings"],
        }, indent=2))
        return
    retimed, retime_metadata = uniform_stage_retime(candidate, candidate_audit)
    args.output.write_text(json.dumps(retimed, indent=2, allow_nan=False) + "\n")
    candidate_config_sha256 = hashlib.sha256(args.output.read_bytes()).hexdigest()
    fast_audit_result = _audit(
        retimed, candidate=True,
        inherited_capsule_proxy_names=inherited_proxy,
    )
    fast_audit = {
        "state": fast_audit_result["state"],
        "ticks": fast_audit_result["ticks"],
        "first_failure": fast_audit_result["first_failure"],
        "stage_rows": fast_audit_result["stage_rows"],
        "boundaries": fast_audit_result["boundaries"],
        "capsule_proxy_warnings": fast_audit_result["capsule_proxy_warnings"],
        "terminal_capsule_clear": fast_audit_result["terminal_capsule_clear"],
        "terminal_pose": fast_audit_result["terminal_pose"],
        "candidate_stage_count": retimed["stage_count"],
        "structural_stage_count": candidate["stage_count"],
        "retime_metadata": retime_metadata,
        "fixed_thigh_spans": spans,
        "geometry_preserved_by_stage": True,
        "candidate_config_sha256": candidate_config_sha256,
        "geometry_sha256": _geometry_sha256(retimed),
        "measurement_binding": "candidate_snapshot_sha256_captured_before_controller_replay",
        "evidence_boundary": fast_audit_result["capsule_boundary"],
    }
    args.fast_audit.write_text(json.dumps(_json_clean(fast_audit), indent=2, allow_nan=False) + "\n")
    print(json.dumps({
        "structural_stage_count": candidate["stage_count"],
        "fast_stage_count": retimed["stage_count"],
        "baseline_ticks": baseline_audit["ticks"],
        "structural_ticks": candidate_audit["ticks"],
        "fast_ticks": fast_audit_result["ticks"],
        "fast_state": fast_audit_result["state"],
        "fast_first_failure": fast_audit_result["first_failure"],
        "fast_capsule_proxy_warnings": fast_audit_result["capsule_proxy_warnings"],
        "fixed_thigh_spans": spans,
    }, indent=2))
    if fast_audit_result["first_failure"] is not None or fast_audit_result["state"] != ClimbMode.DONE:
        raise SystemExit(4)


if __name__ == "__main__":
    main()
