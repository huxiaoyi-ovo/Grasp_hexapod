#!/usr/bin/env python3
"""Fast playback integrity checks for the active Isaac-only compact plan."""

import json
import math

import numpy as np

from climb_mode import ClimbMode
from control import GraspController
from utils import package_config_path
from utils.climb import resolve_compact_stage_range


EXPECTED_SUFFIX = (
    ("BODY_RIGHT_SETTLE", []),
    ("LM_POSTURE_RESET", [2]),
    ("RB_RF_TOP_ENTRY", [3, 4]),
    ("TRIPOD_A_TOP_APPROACH", [0, 1, 5]),
    ("LM_FINAL_POSTURE_WITH_BODY", [2]),
    ("STAND_FINAL_HOLD", []),
)


def require(value, message):
    if not value:
        raise AssertionError(message)


def lm_tilt_deg(controller, q_rad, pose):
    rotation = ClimbMode._world_from_base(pose)[:3, :3]
    axis_world = controller.kinematic.terminal_axes_base(q_rad)[2] @ rotation.T
    return math.degrees(math.acos(np.clip(abs(axis_world[2]), -1.0, 1.0)))


def enter_range(controller, q_rad, compact, start_index, end_index):
    """以连续 CPU/DLS 回放生成中途入口快照，再进入指定闭区间。"""

    if start_index:
        warmup = GraspController(0.02)
        q_rad = warmup.replay_climb_prefix(
            q_rad, compact, start_index - 1, max_ticks=15000
        )
    controller.enter_climb(q_rad, compact, start_index, end_index)
    return q_rad


def run_range(compact, start_index, end_index):
    controller = GraspController(0.02)
    q_rad = np.asarray(compact["p0"]["q_rad"], dtype=np.float64)
    q_rad = enter_range(controller, q_rad, compact, start_index, end_index)
    require(controller.climb_mode.last_foot_target_error_m <=
            compact["settle_gate"]["max_foot_target_error_m"],
            "mid-stage entry foot target error")
    visited = [controller.climb_mode.phase]
    zeros = np.zeros(4, dtype=np.float64)
    for _ in range(15000):
        if controller.climb_mode.state != ClimbMode.RUNNING:
            break
        previous_index = controller.climb_mode.stage_index
        q_rad = controller.update(q_rad, zeros)
        if controller.climb_mode.stage_index != previous_index \
                and controller.climb_mode.state != ClimbMode.DONE:
            visited.append(controller.climb_mode.phase)
    require(controller.climb_mode.state == ClimbMode.DONE,
            "selected range did not finish")
    return visited


def main():
    with package_config_path("climb_compact.json").open() as file:
        compact = json.load(file)
    ClimbMode(None)._validate_config(compact)
    suffix = tuple((stage["name"], stage["active_legs"])
                   for stage in compact["stages"][-6:])
    require(compact["stage_count"] == 35, "expected 35 preview stages")
    require(suffix == EXPECTED_SUFFIX, "C30-C35 stage suffix")

    controller = GraspController(0.02)
    q_rad = np.asarray(compact["p0"]["q_rad"], dtype=np.float64)
    controller.enter_climb(q_rad, compact)
    stages = compact["stages"]
    visited = [controller.climb_mode.phase]
    completed_q = {}
    ticks = 0
    zeros = np.zeros(4, dtype=np.float64)
    while controller.climb_mode.state == ClimbMode.RUNNING and ticks < 15000:
        previous_index = controller.climb_mode.stage_index
        previous_name = controller.climb_mode.phase
        q_rad = controller.update(q_rad, zeros)
        require(np.all(np.isfinite(q_rad)), previous_name + " non-finite q")
        require(np.min(controller.kinematic.joint_limit_margins(q_rad)) >= 0.0,
                previous_name + " joint limit")
        ticks += 1
        if controller.climb_mode.stage_index != previous_index:
            completed_q[previous_name] = q_rad.copy()
            if controller.climb_mode.state != ClimbMode.DONE:
                visited.append(controller.climb_mode.phase)
    if controller.climb_mode.state == ClimbMode.DONE:
        completed_q[stages[-1]["name"]] = q_rad.copy()

    expected_names = [stage["name"] for stage in stages]
    require(controller.climb_mode.state == ClimbMode.DONE,
            "preview did not finish at tick {}: {}".format(
                ticks, controller.climb_mode.failure_reason))
    require(visited == expected_names, "visited stage order")

    c13_c15 = resolve_compact_stage_range(compact, "C13", "C15")
    runtime_c13_c15 = resolve_compact_stage_range(
        compact, stages[12]["name"], stages[14]["name"]
    )
    require(c13_c15 == (12, 14), "C13-C15 selector resolution")
    require(c13_c15 == runtime_c13_c15,
            "runtime stage selector resolution")
    require(run_range(compact, *c13_c15) ==
            [stage["name"] for stage in stages[12:15]],
            "C13-C15 visited stage order")
    for start, end in (("C0", "C1"), ("NO_SUCH_STAGE", "C1"),
                       ("C15", "C13")):
        try:
            resolve_compact_stage_range(compact, start, end)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid stage range was accepted")

    tilts = {}
    for stage in stages[-6:]:
        name = stage["name"]
        tilts[name] = lm_tilt_deg(controller, completed_q[name], stage["pose_end"])
    require(tilts["LM_FINAL_POSTURE_WITH_BODY"] <= 5.0,
            "C34 LM terminal axis tilt")
    require(tilts["STAND_FINAL_HOLD"] <= 5.0,
            "C35 LM terminal axis tilt")
    print(
        "PREVIEW_VALID ticks={} final_foot_target_error_m={:.3e} lm_tilt_deg={}".format(
            ticks,
            controller.climb_mode.last_foot_target_error_m,
            ",".join("{}:{:.2f}".format(name, tilt)
                     for name, tilt in tilts.items()),
        )
    )


if __name__ == "__main__":
    main()
