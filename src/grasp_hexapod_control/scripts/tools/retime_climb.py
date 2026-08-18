#!/usr/bin/env python3
"""Offline continuous-seed C1--C35 compact trajectory retimer.

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
    segment_for_time,
    stage_specs,
)
from validate_final_transfer import reference, solve_exact


SAMPLES = 101
DT = 1.0 / 30.0
TRACKING_ERROR_LIMIT_M = 0.015
MAX_TRACKING_ITERATIONS = 8


def round_up_centisecond(value):
    return float(np.ceil((value - 1e-12) * 100.0) / 100.0)


def require(value, detail):
    if not value:
        raise AssertionError(detail)


def allowed_difference(before, after):
    """Assert that only C1--C35 duration scalars are different."""

    require(before["stage_count"] == after["stage_count"] == 36,
            "expected active 36-stage compact")
    for stage_index, (old_stage, new_stage) in enumerate(
            zip(before["stages"], after["stages"])):
        old_copy, new_copy = copy.deepcopy(old_stage), copy.deepcopy(new_stage)
        old_durations = old_copy.pop("segment_durations_s")
        new_durations = new_copy.pop("segment_durations_s")
        require(old_copy == new_copy,
                "non-duration stage field changed: C{}".format(stage_index + 1))
        if stage_index >= 35:
            require(old_durations == new_durations,
                    "C36 duration must remain identical")
        elif stage_index == FROZEN_LB_LOW_STEP_INDEX:
            require(old_durations == new_durations == [1.4, 0.9, 0.8],
                    "C20 LB_LOW_STEP duration contract must remain identical")
        elif stage_index == FROZEN_PRELOAD_INDEX:
            require(old_durations == new_durations == [1.0],
                    "C23 BODY_PRELOAD_LM must remain 1.0 s")
        elif stage_index == FROZEN_LF_LOW_STEP_INDEX:
            require(old_durations == new_durations == [1.4, 0.9, 1.0],
                    "C22 LF_LOW_STEP duration contract must remain identical")
    before_top, after_top = copy.deepcopy(before), copy.deepcopy(after)
    before_top.pop("stages")
    after_top.pop("stages")
    require(before_top == after_top, "non-stage compact field changed")


def dynamic_tracking_adjust(proposal):
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
            if stage_index >= 35 or not stage["active_legs"]:
                continue
            semantic = segment_for_time(stage_index, stage, time_s)
            speed = float(np.max(np.abs(q - before) / DT))
            require(speed <= semantic["hard_gate_rad_s"],
                    "30 Hz semantic hard speed C{} {}".format(
                        stage_index + 1, stage["name"]))
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
        for (stage_index, segment_index), item in sorted(failures.items()):
            require(stage_index not in (
                FROZEN_LB_LOW_STEP_INDEX,
                FROZEN_LF_LOW_STEP_INDEX,
                FROZEN_PRELOAD_INDEX,
            ), "frozen user trajectory exceeds 30 Hz tracking gate: " +
                    item["stage"])
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


def retime(compact):
    """Sample all stages from inherited P0 state and return a copied proposal."""

    proposal = copy.deepcopy(compact)
    controller = GraspController(DT)
    q = np.asarray(compact["p0"]["q_rad"], dtype=np.float64).copy()
    mode = ClimbMode(None)
    mode.config = compact
    report = []
    for stage_index, stage in enumerate(compact["stages"][:35]):
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
            if stage_index in (FROZEN_LB_LOW_STEP_INDEX,
                               FROZEN_LF_LOW_STEP_INDEX,
                               FROZEN_PRELOAD_INDEX):
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
    adjustments = dynamic_tracking_adjust(proposal)
    allowed_difference(compact, proposal)
    return proposal, report, adjustments


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path,
                        default=package_config_path("climb_compact.json"))
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--output", type=Path)
    target.add_argument("--in-place", action="store_true")
    args = parser.parse_args()
    compact = json.loads(args.config.read_text(encoding="utf-8"))
    ClimbMode(None)._validate_config(compact)
    proposal, report, adjustments = retime(compact)
    destination = args.config if args.in_place else args.output
    if destination:
        destination.write_text(json.dumps(proposal, indent=2) + "\n",
                               encoding="utf-8")
    print(json.dumps({
        "written": None if destination is None else str(destination),
        "dense_base_stages": report,
        "dynamic_tracking_adjustments": adjustments,
        "evidence_boundary": (
            "Offline continuous-seed dense IK retiming only; not contact, load, "
            "friction, clearance, stability, GPU PhysX, or hardware authorization."
        ),
    }, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
