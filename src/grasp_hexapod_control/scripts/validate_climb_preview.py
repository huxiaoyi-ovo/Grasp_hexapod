#!/usr/bin/env python3
"""Fast structure and 30 Hz DLS checks for compact climb candidates.

The replay and geometry values are simulation/model diagnostics only; they do
not establish contact, load, stability, or real-robot authorization.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from climb_mode import ClimbMode
from control import GraspController
from utils import package_config_path
from utils.climb import gravity_projected_support, resolve_compact_stage_range
from utils.climb_retime import (
    assert_speed_report,
    segment_for_time,
    speed_report,
    update_speed_report,
)


DT = 1.0 / 30.0
ROOT = Path(__file__).resolve().parents[3]
ACTIVE_STAGE_NAMES = (
    "PREP", "RM", "BODY", "PAIR", "LB_LF_GROUND_SHIFT",
    "LM_GROUND_SHIFT", "BODY2", "RM_HIGH_C", "RB_RF_HIGH_C", "BODY3",
    "RB_RF_SHIFT1", "LM_GROUND_SHIFT1", "RM_PRE_ADVANCE",
    "LB_LF_BODY_ADVANCE_HIGH_STEP", "RB_RF_TOP_INWARD_PAIR", "LM_EDGE_STAGE",
    "BODY_A", "RM_RIGHT_SYMMETRY", "BODY_LEFT_TRANSFER_PREP",
    "LB_LOW_STEP", "BODY_RIGHT_BEFORE_LF", "LF_LOW_STEP",
    "BODY_PRELOAD_LM", "LM_LIFT",
    "BODY_ADVANCE_LM_AIR", "LM_LEFT_FINAL_LAND", "RB_RF_DIRECT_FINAL",
    "RM_DIRECT_FINAL", "BODY_REPOSITION", "LB_LF_DIRECT_FINAL",
    "BODY_DOCK_FINAL", "STAND_FINAL_HOLD",
)


def require(value, message):
    """Raise a concise assertion for a failed validator invariant."""

    if not value:
        raise AssertionError(message)


def load_config(path):
    """Read a compact candidate and run the runtime configuration gate."""

    with Path(path).open(encoding="utf-8") as file:
        compact = json.load(file)
    ClimbMode(None)._validate_config(compact)
    return compact


def finite_json(value):
    """Reject JSON output trees containing NaN or infinity."""

    if isinstance(value, dict):
        return all(finite_json(item) for item in value.values())
    if isinstance(value, list):
        return all(finite_json(item) for item in value)
    return not isinstance(value, float) or np.isfinite(value)


def base_relative_clearances(stage):
    """Return each active leg's configured base-link swing clearance."""

    if not stage["active_legs"]:
        return []
    if stage["anchor_curve"] == "relative_base_high_step":
        return [float(stage["relative_swing_height_m"])] * len(
            stage["active_legs"]
        )
    knots = np.asarray(stage["active_base_knots_m"], dtype=np.float64)
    endpoint = np.maximum(knots[0, :, 2], knots[-1, :, 2])
    return (np.max(knots[:, :, 2], axis=0) - endpoint).tolist()


def strict_contract(compact):
    """Lock the accepted active plan's identity-specific invariants."""

    if compact.get("stage_count") == 27:
        strict_concurrent27(compact)
        return

    stages = compact["stages"]
    require(compact["stage_count"] == len(ACTIVE_STAGE_NAMES) == len(stages),
            "32 active stages")
    require(tuple(stage["name"] for stage in stages) == ACTIVE_STAGE_NAMES,
            "active stage map")
    for stage in stages:
        if not stage["active_legs"]:
            continue
        require(stage["anchor_curve"] in (
            "piecewise_base_quintic", "relative_base_high_step"),
            "active feet must use base-relative trajectories: " + stage["name"])
    require(min(base_relative_clearances(stages[3])) >= .10,
            "C4 RB/RF first-platform clearance >= 100 mm")
    c7, c8 = stages[6:8]
    require(np.allclose(c7["pose_end"],
                        [.06, -.06769449763600001, .19, 0.0, -.56])
            and np.allclose(c8["pose_start"], c7["pose_end"])
            and np.allclose(c8["pose_end"],
                            [.06, -.06769449763600001, .19, 0.0, -.3]),
            "C7-C8 RM pitch-assisted transfer")
    require(c8["segment_durations_s"] == [.58, .76, .58],
            "C8 RM lift-transfer-touchdown timing")
    c8_durations = np.asarray(c8["segment_durations_s"], dtype=np.float64)
    c8_phase = np.r_[0.0, np.cumsum(c8_durations)] / np.sum(c8_durations)
    c8_weight = c8_phase ** 3 * (
        10.0 - 15.0 * c8_phase + 6.0 * c8_phase ** 2
    )
    c8_poses = (
        (1.0 - c8_weight[:, None]) * np.asarray(c8["pose_start"])
        + c8_weight[:, None] * np.asarray(c8["pose_end"])
    )
    c8_base = np.asarray(c8["active_base_knots_m"], dtype=np.float64)[:, 0]
    c8_world = np.stack([
        (ClimbMode._world_from_base(pose) @ np.r_[point, 1.0])[:3]
        for pose, point in zip(c8_poses, c8_base)
    ])
    c8_anchor_world = np.asarray(c8["anchor_knots"], dtype=np.float64)[:, 5]
    require(np.allclose(c8_world, c8_anchor_world, rtol=0.0, atol=1e-9),
            "C8 RM base/world path consistency")
    require(np.allclose(c8_world[1:3], [
                [.20449034338561017, -.06769450166012006,
                 .2044231715549064],
                [.2976919472902762, -.130924989575459,
                 .24031975690161706],
            ], rtol=0.0, atol=1e-12),
            "C8 RM airborne world path raised 30 mm")
    require(np.isclose(base_relative_clearances(stages[17])[0], .04),
            "C18 RM relative clearance = 40 mm")
    require(stages[10]["active_legs"] == [3], "C11 RB-only shift")
    require(stages[11]["active_legs"] == [2], "C12 LM shift")
    require(stages[12]["active_legs"] == [5], "C13 RM pre-advance")
    c14 = stages[13]
    require(c14["active_legs"] == [0, 1], "C14 LB/LF active")
    require(c14["anchor_curve"] == "relative_base_high_step", "C14 curve")
    require(np.isclose(c14["relative_swing_height_m"], 0.06), "C14 lift")
    require(len(c14["segment_durations_s"]) == 1, "C14 one critical segment")
    require(len(c14["anchor_knots"]) == 2, "C14 endpoint-only path")
    require(np.array_equal(np.asarray(c14["anchor_knots"])[0, 2:],
                           np.asarray(c14["anchor_knots"])[1, 2:]),
            "C14 fixed anchors")
    c19, c20, c21, c22, c23, c24, c25, c26 = stages[18:26]
    require(c19["active_legs"] == [], "C19 fixed-body prep")
    require(c20["active_legs"] == [0], "C20 LB-only step")
    require(c21["active_legs"] == [], "C21 fixed-body right shift")
    require(c22["active_legs"] == [1], "C22 LF-only step")
    require(c23["active_legs"] == [], "C23 LM preload")
    require(c20["pose_curve"] == "quintic_first_segment" and
            c20["segment_durations_s"] == [1.4, .5, 1.2] and
            np.allclose(c20["pose_start"], [.230, -.06769449763600001,
                                             .201, 0.0, -.2]) and
            np.allclose(c20["pose_end"], [.230, -.06769449763600001,
                                           .215, -np.pi / 12.0, -.2]),
            "C20 folded LB transfer")
    require(c21["segment_durations_s"] == [1.2] and
            np.allclose(c21["pose_start"], [.230, -.06769449763600001,
                                             .215, -np.pi / 12.0, -.2]) and
            np.allclose(c21["pose_end"], [.232, -.06769449763600001,
                                           .200, 0.0, -.2]),
            "C21 right shift and level return")
    require(c22["pose_curve"] == "quintic_first_segment" and
            c22["segment_durations_s"] == [1.4, .55, 1.0] and
            np.allclose(c22["pose_start"], [.232, -.06769449763600001,
                                             .200, 0.0, -.2]) and
            np.allclose(c22["pose_end"], [.232, -.06769449763600001,
                                           .226, .16, -.2]),
            "C22 folded LF transfer with rising roll")
    lf_landing = np.array([.21364857479269686, .12028708155300925,
                           .15592301975850517])
    require(np.allclose(np.asarray(c22["anchor_knots"])[-1, 1], lf_landing),
            "C22 LF landing is 10 mm centerward along the low plane")
    require(all(np.allclose(np.asarray(stage["anchor_knots"])[:, 1], lf_landing)
                for stage in stages[22:29]),
            "C23-C29 keep the shifted LF world anchor")
    require(np.allclose(np.asarray(stages[29]["anchor_knots"])[0, 1], lf_landing),
            "C30 begins from the shifted LF anchor")
    for stage_index in (19, 21):
        stage = stages[stage_index]
        reference = ClimbMode(None)
        reference.config = compact
        reference.stage_index = stage_index
        reference.phase_time = stage["segment_durations_s"][0]
        pose, _, _ = reference._stage_reference()
        require(np.allclose(pose, stage["pose_end"]),
                stage["name"] + " pose reaches its endpoint at lift end")
    require(c23["segment_durations_s"] == [1.0] and
            np.allclose(c23["pose_start"], [.232, -.06769449763600001,
                                             .226, .16, -.2]) and
            np.allclose(c23["pose_end"], [.239, -.06769449763600001,
                                           .201, 0.0, -.2]),
            "C23 descent from elevated left-transfer pose")
    require(c24["active_legs"] == [2] and c25["active_legs"] == [2]
            and c26["active_legs"] == [2], "C24-C26 LM transfer")
    require(stages[26]["active_legs"] == [3, 4], "C27 RB/RF direct final pair")
    require(not np.allclose(stages[26]["pose_start"], stages[26]["pose_end"]),
            "C27 pairs legs while moving to the raised RM-ready body pose")
    require(stages[27]["active_legs"] == [5], "C28 RM direct final")
    require(stages[28]["active_legs"] == [], "C29 body reposition")
    require(stages[29]["active_legs"] == [0, 1], "C30 LB/LF direct final pair")
    require(stages[30]["active_legs"] == [], "C31 final body dock")
    require(stages[31]["active_legs"] == [], "C32 final hold")
    for index in range(1, len(stages)):
        require(np.allclose(stages[index - 1]["pose_end"], stages[index]["pose_start"],
                            rtol=0.0, atol=1e-9), "pose boundary")
        require(np.allclose(stages[index - 1]["anchor_knots"][-1],
                            stages[index]["anchor_knots"][0], rtol=0.0, atol=1e-9),
                "anchor boundary")
    require(resolve_compact_stage_range(compact, "C1", "C32") == (0, 31),
            "active aliases")


def strict_concurrent27(compact):
    """Freeze the accepted named 68f-to-AB1C merge contract."""
    baseline_path = ROOT / "src/docs/evidence/climb_concurrency_20260908/baseline_config.json"
    require(baseline_path.is_file(), "concurrent frozen baseline evidence")
    require(hashlib.sha256(baseline_path.read_bytes()).hexdigest()
            == "68f60b97b167ca644f0cebe5fd4de1016b9705764a0376b0431459b0e818346c",
            "concurrent baseline SHA")
    baseline = load_config(baseline_path)
    stages = compact["stages"]
    require(len(stages) == compact.get("stage_count") == 27, "27 concurrent stages")
    sources = {"RB_BODY_ADVANCE": (9, 10), "LM_RM_PRE_ADVANCE": (11, 12),
               "LM_BODY_TRANSFER": (15, 16), "RM_BODY_REPOSITION": (27, 28),
               "LB_LF_DOCK_TRANSFER": (29, 30)}
    names = [stage["name"] for stage in stages]
    source_by_start = {start: name for name, (start, _) in sources.items()}
    consumed = {index for span in sources.values() for index in range(span[0], span[1] + 1)}
    expected_names = [source_by_start[index] if index in source_by_start else stage["name"]
                      for index, stage in enumerate(baseline["stages"]) if index not in consumed or index in source_by_start]
    require(names == expected_names, "concurrent stage map")
    original = {stage["name"]: stage for stage in baseline["stages"]}
    base_top, current_top = dict(baseline), dict(compact)
    base_top.pop("stages"); current_top.pop("stages")
    base_top.pop("stage_count"); current_top.pop("stage_count")
    require(base_top == current_top, "concurrent top-level identity")
    lm_level = next(stage for stage in stages if stage["name"] == "BODY_PRELOAD_LM")["segment_durations_s"] == [.5]
    lm_level_changed = {"BODY_PRELOAD_LM", "LM_LIFT", "BODY_ADVANCE_LM_AIR"} if lm_level else set()
    for index, source in enumerate(baseline["stages"]):
        if index not in consumed and source["name"] not in lm_level_changed:
            require(next(stage for stage in stages if stage["name"] == source["name"]) == source,
                    "unchanged frozen stage " + source["name"])
    for name, (start, end) in sources.items():
        stage = next(item for item in stages if item["name"] == name)
        first, last = baseline["stages"][start], baseline["stages"][end]
        expected_active = sorted({leg for source in baseline["stages"][start:end + 1]
                                  for leg in source["active_legs"]})
        require(stage["active_legs"] == expected_active, "merged active legs " + name)
        require(np.allclose(stage["pose_start"], first["pose_start"], rtol=0., atol=1e-12)
                and np.allclose(stage["pose_end"], last["pose_end"], rtol=0., atol=1e-12)
                and np.allclose(stage["anchor_knots"][0], first["anchor_knots"][0], rtol=0., atol=1e-12)
                and np.allclose(stage["anchor_knots"][-1], last["anchor_knots"][-1], rtol=0., atol=1e-12),
                "merged boundary " + name)
        fixed = [leg for leg in range(6) if leg not in stage["active_legs"]]
        require(all(np.allclose(np.asarray(knot)[fixed], np.asarray(first["anchor_knots"][0])[fixed], rtol=0., atol=1e-12)
                    for knot in stage["anchor_knots"]), "fixed anchor " + name)
        velocity = np.asarray(stage.get("active_base_velocities_m_s"), float)
        require(stage["anchor_curve"] == "piecewise_base_quintic"
                and stage["pose_curve"] == "quintic_full_stage"
                and velocity.shape == np.asarray(stage["active_base_knots_m"]).shape
                and np.allclose(velocity[[0, -1]], 0., rtol=0., atol=1e-12)
                and np.all(np.linalg.norm(velocity[1:-1], axis=2) > 0.), "Hermite curve " + name)
    left_names = ("BODY_LEFT_TRANSFER_PREP", "LB_LOW_STEP", "BODY_RIGHT_BEFORE_LF",
                  "LF_LOW_STEP", "BODY_PRELOAD_LM", "LM_LIFT",
                  "BODY_ADVANCE_LM_AIR", "LM_LEFT_FINAL_LAND")
    preload = next(stage for stage in stages if stage["name"] == "BODY_PRELOAD_LM")
    if lm_level:
        for name in left_names[:4] + left_names[-1:]:
            require(next(stage for stage in stages if stage["name"] == name) == original[name],
                    "frozen left stage " + name)
        lift = next(stage for stage in stages if stage["name"] == "LM_LIFT")
        air = next(stage for stage in stages if stage["name"] == "BODY_ADVANCE_LM_AIR")
        original_preload = original["BODY_PRELOAD_LM"]
        original_lift = original["LM_LIFT"]
        original_air = original["BODY_ADVANCE_LM_AIR"]
        require(np.allclose(preload["pose_start"], original_preload["pose_start"], rtol=0., atol=1e-12)
                and np.allclose(preload["pose_end"], [.239, -.06769449763600001, .226, .16, -.2], rtol=0., atol=1e-12)
                and preload["segment_durations_s"] == [.5]
                and preload["active_legs"] == []
                and preload["anchor_curve"] == original_preload["anchor_curve"]
                and np.array_equal(preload["anchor_knots"][0], preload["anchor_knots"][-1])
                and np.array_equal(preload["anchor_knots"][0], original_preload["anchor_knots"][0])
                and preload.get("settle_s") == original_preload.get("settle_s")
                and preload.get("settle_persistence_s") == original_preload.get("settle_persistence_s"),
                "LM-level preload .5s without dip or level")
        require(lift["active_legs"] == [2]
                and lift["anchor_curve"] == "piecewise_base_quintic"
                and lift["pose_curve"] == "quintic_first_segment"
                and lift["segment_durations_s"] == [1.2, .45]
                and np.allclose(lift["pose_start"], preload["pose_end"], rtol=0., atol=1e-12)
                and np.allclose(lift["pose_end"], [.239, -.06769449763600001, .226, 0., -.2], rtol=0., atol=1e-12)
                and np.allclose(np.asarray(lift["active_base_knots_m"])[1:], np.asarray(original_lift["active_base_knots_m"])[1:], rtol=0., atol=1e-12)
                and lift.get("settle_s") == original_lift.get("settle_s")
                and lift.get("settle_persistence_s") == original_lift.get("settle_persistence_s")
                and lift.get("continuous_air_transition") is True,
                "LM lift levels at unchanged height before transfer")
        require(air["active_legs"] == [2]
                and air["anchor_curve"] == original_air["anchor_curve"]
                and air["pose_curve"] == original_air["pose_curve"]
                and air["segment_durations_s"] == original_air["segment_durations_s"]
                and np.allclose(air["pose_start"], lift["pose_end"], rtol=0., atol=1e-12)
                and air["pose_end"] == original_air["pose_end"]
                and air["active_base_knots_m"] == original_air["active_base_knots_m"]
                and np.allclose(np.asarray(air["anchor_knots"])[-1], np.asarray(original_air["anchor_knots"])[-1], rtol=0., atol=1e-12)
                and air.get("settle_s") == original_air.get("settle_s")
                and air.get("settle_persistence_s") == original_air.get("settle_persistence_s")
                and air.get("continuous_air_transition") is True,
                "LM air reconnects frozen endpoint")
        lift_velocity = np.asarray(lift["active_base_velocities_m_s"], float)
        air_velocity = np.asarray(air["active_base_velocities_m_s"], float)
        require(np.allclose(lift_velocity[0], 0., rtol=0., atol=1e-12)
                and np.linalg.norm(lift_velocity[1, 0]) > 0.
                and np.allclose(air_velocity[-1], np.asarray(original_air["active_base_velocities_m_s"])[-1], rtol=0., atol=1e-12),
                "LM-level Hermite endpoint contract")
    else:
        for name in left_names:
            require(next(stage for stage in stages if stage["name"] == name) == original[name],
                    "critical frozen stage " + name)
    require(stages[-1]["pose_end"] == baseline["stages"][-1]["pose_end"]
            and stages[-1]["anchor_knots"][-1] == baseline["stages"][-1]["anchor_knots"][-1],
            "frozen terminal body and anchors")


def replay(compact, strict=False):
    """Replay a candidate through ClimbMode and GraspController at 30 Hz."""

    controller = GraspController(DT)
    q = np.asarray(compact["p0"]["q_rad"], dtype=np.float64)
    controller.enter_climb(q, compact)
    stages = compact["stages"]
    stage_report = [
        {"name": stage["name"], "max_active_foot_error_m": 0.0,
         "base_relative_clearance_m": base_relative_clearances(stage),
         "max_active_foot_error_source": (None if stage["active_legs"] else {
             "leg": None, "tick": None, "reason": "no active legs"}),
         "fixed_support_thigh_peak_rad": 0.0,
         "fixed_support_thigh_peak_source": None,
         "planned_base_raw_support_margin_m": None,
         "planned_base_raw_support_margin_source": None}
        for stage in stages
    ]
    min_margin, margin_source, peak_speed, peak_speed_source, ticks = np.inf, None, 0.0, None, 0
    stage_entry_q = [None] * len(stages)
    semantic_segments = [speed_report(index, stage)
                         for index, stage in enumerate(stages)]
    c14_q, c14_errors = [], []
    zeros = np.zeros(4)
    while controller.climb_mode.state == ClimbMode.RUNNING and ticks < 12000:
        stage_index = controller.climb_mode.stage_index
        stage_time = controller.climb_mode.phase_time
        before = q.copy()
        if stage_entry_q[stage_index] is None:
            stage_entry_q[stage_index] = before.copy()
        q = controller.update(q, zeros)
        require(np.all(np.isfinite(q)), "non-finite q")
        margins = controller.kinematic.joint_limit_margins(q)
        sample = float(np.min(margins))
        if sample < min_margin:
            leg, joint = np.unravel_index(np.argmin(margins), margins.shape)
            min_margin, margin_source = sample, {
                "stage": controller.climb_mode.phase, "tick": ticks,
                "leg": int(leg), "joint": int(joint),
            }
        speed = np.abs(q - before) / DT
        semantic = segment_for_time(
            stage_index, stages[stage_index], stage_time)
        if semantic is not None:
            peak = float(np.max(speed))
            leg, joint = np.unravel_index(np.argmax(speed), speed.shape)
            update_speed_report(
                semantic_segments[stage_index], semantic["segment_index"], peak,
                {"stage": stages[stage_index]["name"], "time_s": stage_time,
                 "tick": ticks, "leg": int(leg), "joint": int(joint),
                 "metric": "command_speed_rad_s", "actual": peak,
                 "threshold": semantic["hard_gate_rad_s"]},
            )
        if float(np.max(speed)) > peak_speed:
            leg, joint = np.unravel_index(np.argmax(speed), speed.shape)
            peak_speed, peak_speed_source = float(np.max(speed)), {
                "stage": controller.climb_mode.phase, "tick": ticks,
                "leg": int(leg), "joint": int(joint)}
        actual = controller.kinematic.forward_base(q)
        desired = controller.foot_desired_base
        active = stages[stage_index]["active_legs"]
        if active:
            error = np.linalg.norm(actual[active] - desired[active], axis=1)
            if (stage_report[stage_index]["max_active_foot_error_source"] is None
                    or float(np.max(error)) > stage_report[stage_index]["max_active_foot_error_m"]):
                local = int(np.argmax(error))
                stage_report[stage_index]["max_active_foot_error_m"] = float(error[local])
                stage_report[stage_index]["max_active_foot_error_source"] = {
                    "leg": int(active[local]), "tick": ticks}
        fixed = [leg for leg in range(6) if leg not in active]
        if fixed:
            thigh_delta = np.abs(q[fixed, 0] - stage_entry_q[stage_index][fixed, 0])
            if (stage_report[stage_index]["fixed_support_thigh_peak_source"] is None
                    or float(np.max(thigh_delta)) > stage_report[stage_index]["fixed_support_thigh_peak_rad"]):
                local = int(np.argmax(thigh_delta))
                stage_report[stage_index]["fixed_support_thigh_peak_rad"] = float(thigh_delta[local])
                stage_report[stage_index]["fixed_support_thigh_peak_source"] = {
                    "leg": int(fixed[local]), "tick": ticks}
        pose = np.asarray(controller.climb_mode.base_pose, dtype=np.float64)
        support = gravity_projected_support(
            pose[:3], controller.climb_mode.anchors_world[fixed], (0.0, 0.0, -1.0))
        if stage_report[stage_index]["planned_base_raw_support_margin_source"] is None:
            stage_report[stage_index]["planned_base_raw_support_margin_source"] = {
                "tick": ticks, "fixed_legs": fixed, "valid": bool(support.valid)}
        if support.valid:
            old = stage_report[stage_index]["planned_base_raw_support_margin_m"]
            value = float(support.raw_margin_m)
            if old is None or value < old:
                stage_report[stage_index]["planned_base_raw_support_margin_m"] = value
                stage_report[stage_index]["planned_base_raw_support_margin_source"] = {
                    "tick": ticks, "fixed_legs": fixed, "valid": True}
        if strict and stages[stage_index]["name"] == "LB_LF_BODY_ADVANCE_HIGH_STEP":
            c14_q.append(q.copy())
            c14_errors.append(np.linalg.norm(actual[:2] - desired[:2], axis=1))
        ticks += 1
    require(controller.climb_mode.state == ClimbMode.DONE, "preview did not finish")
    final_pose = np.asarray(stages[-1]["pose_end"], dtype=np.float64)
    final_inverse = np.linalg.inv(ClimbMode._world_from_base(final_pose))
    final_anchors = np.asarray(stages[-1]["anchor_knots"][-1], dtype=np.float64)
    final_desired = (
        np.column_stack((final_anchors, np.ones(6))) @ final_inverse.T
    )[:, :3]
    terminal_q = np.asarray(compact["terminal_q_rad"], dtype=np.float64)
    terminal_fk_error = float(np.max(np.linalg.norm(
        controller.kinematic.forward_base(terminal_q) - final_desired, axis=1)))
    if strict:
        require(terminal_fk_error <= 1e-5,
                "stored terminal_q_rad FK must match final body/anchors")
    for rows in semantic_segments[:len(stages) - 1]:
        assert_speed_report(rows, require)
    if strict:
        velocity = np.diff(np.asarray(c14_q), axis=0) / DT
        pair = velocity[:, :2]
        reversal = ((pair[1:] * pair[:-1] < 0.0) & (np.abs(pair[1:]) > .1)
                    & (np.abs(pair[:-1]) > .1))
        require(not np.any(reversal), "C14 LB/LF joint chatter")
        require(np.max(np.abs(pair)) < 3.0, "C14 LB/LF joint speed")
        require(np.max(np.asarray(c14_errors)) < .015, "C14 foot tracking")
        require(min_margin > .02, "active joint margin > .02")
    report = {
        "ticks": ticks, "state": controller.climb_mode.state,
        "min_joint_margin_rad": float(min_margin), "min_joint_margin_source": margin_source,
        "global_peak_command_speed_rad_s": float(peak_speed),
        "global_peak_command_speed_source": peak_speed_source,
        "semantic_segments": semantic_segments[:len(stages) - 1],
        "stages": stage_report,
        "model_diagnostic_only": "planned base/raw support margins are geometry diagnostics, not contact/load/stability proof",
        "final_foot_target_error_m": float(controller.climb_mode.last_foot_target_error_m),
        "terminal_q_fk_error_m": terminal_fk_error,
    }
    require(finite_json(report), "report contains non-finite JSON")
    return report


def main():
    """Run the active strict contract or generic compact candidate replay."""

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=package_config_path("climb_compact.json"))
    parser.add_argument("--generic", action="store_true", help="validate a candidate without active-stage assertions")
    parser.add_argument("--report", type=Path, help="write generic diagnostic JSON")
    args = parser.parse_args()
    compact = load_config(args.config)
    if not args.generic:
        strict_contract(compact)
    report = replay(compact, strict=not args.generic)
    if args.report:
        args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print("PREVIEW_VALID ticks={} min_margin_rad={:.6f} final_foot_target_error_m={:.3e}".format(
        report["ticks"], report["min_joint_margin_rad"], report["final_foot_target_error_m"]))


if __name__ == "__main__":
    main()
