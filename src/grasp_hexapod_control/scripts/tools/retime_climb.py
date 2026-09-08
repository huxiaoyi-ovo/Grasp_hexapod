#!/usr/bin/env python3
"""Offline continuous-seed retimer for all motion stages except final hold.

The default only prints the proposed plan.  Writes require ``--output`` or
``--in-place``.  Results are model kinematic diagnostics, not hardware proof.
"""

import argparse
import copy
import json
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / "src/grasp_hexapod_control/scripts"
sys.path.insert(0, str(SCRIPTS))

from climb_mode import ClimbMode
from control import GraspController
from utils import package_config_path
from utils.climb_retime import (
    FROZEN_LB_LOW_STEP_INDEX,
    FROZEN_LF_LOW_STEP_INDEX,
    FROZEN_PRELOAD_INDEX,
    LEFT_TRANSFER_BODY_MINIMUM_DURATION_S,
    segment_for_time,
    stage_specs,
)
from validate_final_transfer import reference, solve_exact


SAMPLES = 101
DT = 1.0 / 30.0
TRACKING_ERROR_LIMIT_M = 0.015
MAX_TRACKING_ITERATIONS = 8
COUPLED_PAIR_NAMES = frozenset(("RB_RF_DIRECT_FINAL", "LB_LF_DIRECT_FINAL"))


def round_up_centisecond(value):
    return float(np.ceil((value - 1e-12) * 100.0) / 100.0)


def require(value, detail):
    if not value:
        raise AssertionError(detail)


def allowed_difference(before, after):
    """Assert that only non-hold duration scalars are different."""

    require(before["stage_count"] == after["stage_count"] == len(before["stages"]),
            "retime inputs must share one active compact stage map")
    for stage_index, (old_stage, new_stage) in enumerate(
            zip(before["stages"], after["stages"])):
        old_copy, new_copy = copy.deepcopy(old_stage), copy.deepcopy(new_stage)
        old_durations = old_copy.pop("segment_durations_s")
        new_durations = new_copy.pop("segment_durations_s")
        require(old_copy == new_copy,
                "non-duration stage field changed: C{}".format(stage_index + 1))
        if stage_index == len(before["stages"]) - 1:
            require(old_durations == new_durations,
                    "final hold duration must remain identical")
        elif stage_index == FROZEN_LB_LOW_STEP_INDEX:
            require(old_durations == new_durations == [1.4, .5, 1.2],
                    "C20 verified duration contract")
        elif stage_index == 20:
            require(len(old_durations) == len(new_durations) == 1 and
                    new_durations[0] >= LEFT_TRANSFER_BODY_MINIMUM_DURATION_S,
                    "C21 body return must not shorten below 1.2 s")
        elif stage_index == FROZEN_PRELOAD_INDEX:
            require(old_durations == new_durations == [1.0],
                    "C23 BODY_PRELOAD_LM must remain 1.0 s")
        elif stage_index == FROZEN_LF_LOW_STEP_INDEX:
            require(old_durations == new_durations == [1.4, .55, 1.0],
                    "C22 verified duration contract")
        elif "active_base_velocities_m_s" in old_stage:
            require(old_durations == new_durations,
                    "continuous swing timing is frozen; rebuild and revalidate it")
        elif old_stage["name"] in COUPLED_PAIR_NAMES:
            require(old_durations == new_durations,
                    "coupled pair/body timing is frozen; rebuild and revalidate it")
    before_top, after_top = copy.deepcopy(before), copy.deepcopy(after)
    before_top.pop("stages")
    after_top.pop("stages")
    require(before_top == after_top, "non-stage compact field changed")


def dynamic_tracking_adjust(proposal, allow_adjustments):
    """Increase only the 30 Hz segment that exceeds the active-foot gate."""

    adjustments = []
    for iteration in range(MAX_TRACKING_ITERATIONS):
        controller = GraspController(DT)
        q = np.asarray(proposal["p0"]["q_rad"], dtype=np.float64).copy()
        controller.enter_climb(q, proposal)
        failures = {}
        while controller.climb_mode.state == ClimbMode.RUNNING:
            stage_index = controller.climb_mode.stage_index
            stage = proposal["stages"][stage_index]
            time_s = controller.climb_mode.phase_time
            before = q.copy()
            q = controller.update(q, np.zeros(4))
            if stage["name"] == "STAND_FINAL_HOLD":
                continue
            semantic = segment_for_time(stage_index, stage, time_s)
            speed = float(np.max(np.abs(q - before) / DT))
            require(speed <= semantic["hard_gate_rad_s"],
                    "30 Hz semantic hard speed C{} {}".format(
                        stage_index + 1, stage["name"]))
            if not stage["active_legs"]:
                continue
            active = stage["active_legs"]
            error = float(np.max(np.linalg.norm(
                controller.kinematic.forward_base(q)[active]
                - controller.foot_desired_base[active], axis=1)))
            key = (stage_index, semantic["segment_index"])
            if error > failures.get(key, {"error_m": 0.0})["error_m"]:
                failures[key] = {
                    "error_m": error,
                    "stage": stage["name"],
                    "semantic": semantic["semantic"],
                }
        failures = {key: value for key, value in failures.items()
                    if value["error_m"] > TRACKING_ERROR_LIMIT_M}
        if not failures:
            return adjustments
        require(allow_adjustments,
                "verified duration has 30 Hz active-foot tracking failure")
        for (stage_index, segment_index), item in sorted(failures.items()):
            require(stage_index not in (
                FROZEN_LB_LOW_STEP_INDEX,
                FROZEN_LF_LOW_STEP_INDEX,
                FROZEN_PRELOAD_INDEX,
            ), "frozen user trajectory exceeds 30 Hz tracking gate: " +
                    item["stage"])
            require(proposal["stages"][stage_index]["name"]
                    not in COUPLED_PAIR_NAMES,
                    "coupled pair/body tracking requires a rebuilt candidate: "
                    + item["stage"])
            require("active_base_velocities_m_s" not in proposal["stages"][stage_index],
                    "continuous swing timing requires a rebuilt candidate: "
                    + item["stage"])
            old_duration = proposal["stages"][stage_index]["segment_durations_s"][
                segment_index]
            scale = max(1.05, 1.02 * item["error_m"] / TRACKING_ERROR_LIMIT_M)
            new_duration = round_up_centisecond(old_duration * scale)
            proposal["stages"][stage_index]["segment_durations_s"][
                segment_index
            ] = new_duration
            adjustments.append({
                "iteration": iteration + 1,
                "stage": item["stage"],
                "segment_index": segment_index,
                "semantic": item["semantic"],
                "tracking_error_m": item["error_m"],
                "tracking_limit_m": TRACKING_ERROR_LIMIT_M,
                "scale": scale,
                "old_duration_s": old_duration,
                "new_duration_s": new_duration,
            })
    raise RuntimeError("30 Hz active-foot tracking retime did not converge")


def retime(compact, explore_durations=False):
    """Audit verified timings, or explicitly regenerate a duration proposal."""

    proposal = copy.deepcopy(compact)
    controller = GraspController(DT)
    q = np.asarray(compact["p0"]["q_rad"], dtype=np.float64).copy()
    mode = ClimbMode(None)
    mode.config = compact
    report = []
    for stage_index, stage in enumerate(compact["stages"]):
        if stage["name"] == "STAND_FINAL_HOLD":
            continue
        rows = []
        elapsed = 0.0
        for spec in stage_specs(stage_index, stage):
            previous_q = None
            previous_s = None
            normalized_peak = 0.0
            for normalized_s in np.linspace(0.0, 1.0, SAMPLES):
                _, _, _, desired = reference(
                    mode, stage_index,
                    elapsed + normalized_s * spec["duration_s"],
                )
                q, residual = solve_exact(controller.kinematic, q, desired)
                require(float(np.max(residual)) <= 1e-5,
                        "dense IK residual C{} {}".format(
                            stage_index + 1, stage["name"]))
                if previous_q is not None:
                    normalized_peak = max(
                        normalized_peak,
                        float(np.max(np.abs(q - previous_q) /
                                     (normalized_s - previous_s))),
                    )
                previous_q = q.copy()
                previous_s = normalized_s
            if (not explore_durations or stage_index in (
                    FROZEN_LB_LOW_STEP_INDEX,
                    FROZEN_LF_LOW_STEP_INDEX,
                    FROZEN_PRELOAD_INDEX)
                    or "active_base_velocities_m_s" in stage
                    or stage["name"] in COUPLED_PAIR_NAMES):
                new_duration = spec["duration_s"]
            else:
                new_duration = round_up_centisecond(max(
                    spec["minimum_duration_s"],
                    1.05 * normalized_peak / spec["target_rad_s"],
                ))
            proposal["stages"][stage_index]["segment_durations_s"][
                spec["segment_index"]
            ] = new_duration
            rows.append({
                **spec,
                "normalized_peak_rad": normalized_peak,
                "proposed_duration_s": new_duration,
                "predicted_peak_rad_s": normalized_peak / new_duration,
            })
            elapsed += spec["duration_s"]
        report.append({"stage": stage["name"], "segments": rows})
    adjustments = dynamic_tracking_adjust(proposal, explore_durations)
    allowed_difference(compact, proposal)
    return proposal, report, adjustments


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path,
                        default=package_config_path("climb_compact.json"))
    parser.add_argument("--explore-durations", action="store_true",
                        help="explicitly regenerate durations; default audits and preserves them")
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--output", type=Path)
    target.add_argument("--in-place", action="store_true")
    args = parser.parse_args()
    compact = json.loads(args.config.read_text(encoding="utf-8"))
    ClimbMode(None)._validate_config(compact)
    proposal, report, adjustments = retime(
        compact, explore_durations=args.explore_durations)
    destination = args.config if args.in_place else args.output
    if destination:
        destination.write_text(json.dumps(proposal, indent=2) + "\n",
                               encoding="utf-8")
    print(json.dumps({
        "written": None if destination is None else str(destination),
        "duration_mode": "explore" if args.explore_durations else "audit_preserve",
        "dense_base_stages": report,
        "dynamic_tracking_adjustments": adjustments,
        "evidence_boundary": (
            "Offline continuous-seed dense IK retiming only; not contact, load, "
            "friction, clearance, stability, GPU PhysX, or hardware authorization."
        ),
    }, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
