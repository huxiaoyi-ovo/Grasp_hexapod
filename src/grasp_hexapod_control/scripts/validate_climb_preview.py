#!/usr/bin/env python3
"""Fast structure and 30 Hz DLS checks for compact climb candidates.

The replay and geometry values are simulation/model diagnostics only; they do
not establish contact, load, stability, or real-robot authorization.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from climb_mode import ClimbMode
from control import GraspController
from utils import package_config_path
from utils.climb import gravity_projected_support, resolve_compact_stage_range


DT = 1.0 / 30.0
ACTIVE_STAGE_NAMES = (
    "PREP", "RM", "BODY", "PAIR", "LB_LF_GROUND_SHIFT",
    "LM_GROUND_SHIFT", "BODY2", "RM_HIGH_C", "RB_RF_HIGH_C", "BODY3",
    "RB_RF_SHIFT1", "LM_GROUND_SHIFT1", "RM_PRE_ADVANCE",
    "LB_LF_BODY_ADVANCE_HIGH_STEP", "RB_RF_TOP_INWARD_PAIR", "LM_EDGE_STAGE",
    "BODY_A", "RM_RIGHT_SYMMETRY", "BODY_LEFT_TRANSFER_PREP",
    "LB_LOW_STEP", "LF_LOW_STEP", "BODY_PRELOAD_LM", "LM_LIFT",
    "BODY_ADVANCE_LM_AIR", "LM_LEFT_FINAL_LAND", "RB_DIRECT_FINAL",
    "RF_DIRECT_FINAL", "BODY_REPOSITION_1", "LM_DIRECT_FINAL",
    "RM_DIRECT_FINAL", "BODY_REPOSITION_2", "LB_DIRECT_FINAL",
    "LF_DIRECT_FINAL",
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


def strict_contract(compact):
    """Lock the accepted active plan's identity-specific invariants."""

    stages = compact["stages"]
    require(compact["stage_count"] == len(ACTIVE_STAGE_NAMES) == len(stages),
            "35 active stages")
    require(tuple(stage["name"] for stage in stages) == ACTIVE_STAGE_NAMES,
            "active stage map")
    require(stages[10]["active_legs"] == [3], "C11 RB-only shift")
    require(stages[11]["active_legs"] == [2], "C12 LM shift")
    require(stages[12]["active_legs"] == [5], "C13 RM pre-advance")
    c14 = stages[13]
    require(c14["active_legs"] == [0, 1], "C14 LB/LF active")
    require(c14["anchor_curve"] == "relative_base_high_step", "C14 curve")
    require(np.isclose(c14["relative_swing_height_m"], 0.06), "C14 lift")
    require(np.allclose(c14["segment_durations_s"], [1.8]), "C14 duration")
    require(len(c14["anchor_knots"]) == 2, "C14 endpoint-only path")
    require(np.array_equal(np.asarray(c14["anchor_knots"])[0, 2:],
                           np.asarray(c14["anchor_knots"])[1, 2:]),
            "C14 fixed anchors")
    c19, c20, c21, c22, c23, c24, c25 = stages[18:25]
    require(c19["active_legs"] == [], "C19 fixed-body prep")
    require(c20["active_legs"] == [0], "C20 LB-only step")
    require(c21["active_legs"] == [1], "C21 LF-only step")
    require(c22["active_legs"] == [], "C22 LM preload")
    require(c23["active_legs"] == [2] and c24["active_legs"] == [2]
            and c25["active_legs"] == [2], "C23-C25 LM transfer")
    require(np.allclose(c25["pose_end"], stages[25]["pose_start"]),
            "C25 restores RF entry pose")
    require(stages[25]["active_legs"] == [3], "C26 RB direct final")
    require(stages[26]["active_legs"] == [4], "C27 RF direct final")
    require(stages[27]["active_legs"] == [], "C28 body reposition 1")
    require(stages[28]["active_legs"] == [2], "C29 LM direct final")
    require(stages[29]["active_legs"] == [5], "C30 RM direct final")
    require(stages[30]["active_legs"] == [], "C31 body reposition 2")
    require(stages[31]["active_legs"] == [0], "C32 LB direct final")
    require(stages[32]["active_legs"] == [1], "C33 LF direct final")
    for index in range(1, len(stages)):
        require(np.allclose(stages[index - 1]["pose_end"], stages[index]["pose_start"],
                            rtol=0.0, atol=1e-9), "pose boundary")
        require(np.allclose(stages[index - 1]["anchor_knots"][-1],
                            stages[index]["anchor_knots"][0], rtol=0.0, atol=1e-9),
                "anchor boundary")
    require(resolve_compact_stage_range(compact, "C1", "C35") == (0, 34),
            "active aliases")


def replay(compact, strict=False):
    """Replay a candidate through ClimbMode and GraspController at 30 Hz."""

    controller = GraspController(DT)
    q = np.asarray(compact["p0"]["q_rad"], dtype=np.float64)
    controller.enter_climb(q, compact)
    stages = compact["stages"]
    stage_report = [
        {"name": stage["name"], "max_active_foot_error_m": 0.0,
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
    c14_q, c14_errors = [], []
    zeros = np.zeros(4)
    while controller.climb_mode.state == ClimbMode.RUNNING and ticks < 12000:
        stage_index = controller.climb_mode.stage_index
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
        if strict and stage_index == 13:
            c14_q.append(q.copy())
            c14_errors.append(np.linalg.norm(actual[:2] - desired[:2], axis=1))
        ticks += 1
    require(controller.climb_mode.state == ClimbMode.DONE, "preview did not finish")
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
        "stages": stage_report,
        "model_diagnostic_only": "planned base/raw support margins are geometry diagnostics, not contact/load/stability proof",
        "final_foot_target_error_m": float(controller.climb_mode.last_foot_target_error_m),
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
