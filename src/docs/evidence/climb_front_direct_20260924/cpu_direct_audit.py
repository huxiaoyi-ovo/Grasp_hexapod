#!/usr/bin/env python3
"""Run one bounded 30 Hz CPU audit of the direct front candidate."""
import argparse
import base64
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / "src/grasp_hexapod_control/scripts"
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / "tools")]

from build_climb_front import (  # noqa: E402
    MESH, TRANSLATION, foot_sphere_signed_gap, load_triangles,
    support_margin, world_from_base,
)
from climb_mode import ClimbMode  # noqa: E402
from control import GraspController  # noqa: E402
from kinematics import JOINT_LOWER, JOINT_UPPER, Q_STAND  # noqa: E402

CONFIG = ROOT / "src/grasp_hexapod_control/config/climb_front_direct.json"
OUTPUT = Path(__file__).with_name("cpu_first.json")
DT = 1.0 / 30.0


def run(config_path, output_path, config_bytes_override=None,
        retry_metadata=None):
    source_bytes = config_path.read_bytes()
    source_sha = hashlib.sha256(source_bytes).hexdigest()
    config_bytes = source_bytes if config_bytes_override is None else config_bytes_override
    config_sha = hashlib.sha256(config_bytes).hexdigest()
    config = json.loads(config_bytes)
    ClimbMode(None)._validate_config(config)
    triangles = load_triangles(MESH) + TRANSLATION
    controller = GraspController(
        DT, enable_link_collision_check=False,
        climb_timeout_uses_wall_time=False,
    )
    controller.enter_climb(Q_STAND, config, 0, len(config["stages"]) - 1,
                           hardware_execution=False)
    mode = controller.climb_mode
    q = Q_STAND.copy()
    stages = {}
    ticks = []
    first_failure = None
    last_index = None
    tick = 0

    while mode.state == ClimbMode.RUNNING and tick < 20000:
        index = mode.stage_index
        stage = config["stages"][index]
        name = stage["name"]
        active = np.asarray(stage["active_legs"], dtype=int)
        fixed = np.asarray([leg for leg in range(6) if leg not in active], dtype=int)
        phase = float(mode.phase_time)
        _, anchors_ref, _ = mode._stage_reference()
        row = stages.get(name)
        if row is None:
            row = stages[name] = {
                "entry_q_rad": q.tolist(), "exit_q_rad": None,
                "max_fk_error_m": 0.0, "max_raw_joint_speed_rad_s": 0.0,
                "min_joint_margin_rad": float("inf"), "min_com_margin_m": None,
                "min_active_sphere_gap_m": None, "velocity_clips": 0,
                "capsule_proxy_first_warning": None,
            }
        old_q = q.copy()
        previous_ref = controller.foot_desired_base_prev.copy()
        q = controller.update(q, np.zeros(4))
        tick += 1
        actual_base_feet = controller.kinematic.hip_to_base(
            controller.kinematic.forward(q)
        )
        fk_error = float(np.max(np.linalg.norm(
            actual_base_feet - controller.foot_desired_base, axis=1
        )))
        margin = float(np.min(np.minimum(q - JOINT_LOWER, JOINT_UPPER - q)))
        actual_speed = np.abs(q - old_q) / DT
        desired_hip = controller.kinematic.base_to_hip(controller.foot_desired_base)
        previous_hip = controller.kinematic.base_to_hip(previous_ref)
        inverse = controller.kinematic.damped_inverse_jacobian(old_q)
        raw = (16.0 * np.einsum(
            "lij,lj->li", inverse,
            desired_hip - controller.kinematic.forward(old_q),
        ) + np.einsum(
            "lij,lj->li", inverse, (desired_hip - previous_hip) / DT,
        ))
        raw_peak = float(np.max(np.abs(raw)))
        com_base = controller.kinematic.center_of_mass_base(q)
        com_world = (world_from_base(mode.base_pose)
                     @ np.r_[com_base, 1.0])[:3]
        com_margin = support_margin(com_world[:2], anchors_ref[fixed]) if len(active) else None
        active_gap = None
        fk_active_gap = None
        if len(active):
            active_gap = min(
                foot_sphere_signed_gap(triangles, anchors_ref[leg])
                for leg in active
            )
            actual_root_fk_feet = (
                world_from_base(mode.base_pose)
                @ np.column_stack((actual_base_feet, np.ones(6))).T
            ).T[:, :3]
            fk_active_gap = min(
                foot_sphere_signed_gap(triangles, actual_root_fk_feet[leg])
                for leg in active
            )
        collision_points = controller.kinematic.collision_points_base(q)
        same_leg_mask = controller._same_leg_collision_free(collision_points)
        proxy_mask = controller._link_collision_free(q)
        clips = int(controller.last_update_velocity_limit_clip_count)

        row["max_fk_error_m"] = max(row["max_fk_error_m"], fk_error)
        row["max_raw_joint_speed_rad_s"] = max(row["max_raw_joint_speed_rad_s"], raw_peak)
        row["min_joint_margin_rad"] = min(row["min_joint_margin_rad"], margin)
        row["velocity_clips"] += clips
        if com_margin is not None:
            row["min_com_margin_m"] = (com_margin if row["min_com_margin_m"] is None
                                        else min(row["min_com_margin_m"], com_margin))
        if active_gap is not None:
            row["min_active_sphere_gap_m"] = (
                active_gap if row["min_active_sphere_gap_m"] is None
                else min(row["min_active_sphere_gap_m"], active_gap)
            )
        if not proxy_mask.all() and row["capsule_proxy_first_warning"] is None:
            row["capsule_proxy_first_warning"] = {
                "tick": tick, "stage_time_s": phase, "q_rad": q.tolist(),
                "base_pose": mode.base_pose.tolist(), "proxy_mask": proxy_mask.tolist(),
            }
        ticks.append({
            "tick": tick, "stage": name, "stage_index": index,
            "phase_before_s": phase, "phase_time_s": float(mode.phase_time),
            "q_rad": q.tolist(),
            "base_pose": mode.base_pose.tolist(), "active_legs": active.tolist(),
            "fk_error_m": fk_error, "raw_joint_speed_rad_s": raw_peak,
            "commanded_joint_speed_rad_s": float(np.max(actual_speed)),
            "joint_margin_rad": margin, "com_margin_m": com_margin,
            "active_sphere_gap_m": active_gap,
            "planned_root_fk_sphere_gap_m_diagnostic": fk_active_gap,
            "velocity_clips": clips, "same_leg_collision_free": same_leg_mask.tolist(),
        })

        reason = None
        if margin <= 0.0:
            reason = "joint_limit"
        elif fk_error > .015:
            reason = "fk_error"
        elif clips:
            reason = "velocity_clip"
        elif com_margin is not None and com_margin < .010:
            reason = "com_support_margin"
        elif active_gap is not None and active_gap < -.0005:
            reason = "actual_active_foot_sphere_gap"
        elif not same_leg_mask.all():
            reason = "same_leg_collision"
        if reason:
            first_failure = {
                "tick": tick, "stage": name, "stage_index": index,
                "phase_before_s": phase, "stage_time_s": float(mode.phase_time),
                "reason": reason,
                "q_rad": q.tolist(), "base_pose": mode.base_pose.tolist(),
                "active_legs": active.tolist(), "metrics": ticks[-1],
            }
            break
        row["exit_q_rad"] = q.tolist()
        if mode.stage_index != index:
            # The command at this tick is the actual state entering the next stage.
            row["exit_q_rad"] = q.tolist()
        last_index = index

    unchanged = hashlib.sha256(config_path.read_bytes()).hexdigest() == source_sha
    result = {
        "audit": "continuous GraspController CPU replay at 30 Hz",
        "state": mode.state, "failure_reason": mode.failure_reason,
        "ticks": tick, "first_failure": first_failure,
        "completed_stage_count": len(stages), "stage_rows": stages,
        "tick_samples": ticks, "terminal_q_rad": q.tolist(),
        "terminal_pose": mode.base_pose.tolist(),
        "config_path": str(config_path), "config_sha256_before": config_sha,
        "config_sha256_after": config_sha,
        "source_file_sha256_before": source_sha,
        "source_file_sha256_after": hashlib.sha256(config_path.read_bytes()).hexdigest(),
        "config_bytes_base64": base64.b64encode(config_bytes).decode("ascii"),
        "config_unchanged_during_audit": unchanged,
        "capsule_proxy_boundary": (
            "Advisory only. Controller link capsules are not exact STL self-collision; "
            "Isaac actor filtering is outside this CPU audit."
        ),
        "evidence_boundary": "CPU controller and model geometry diagnostics only.",
    }
    if retry_metadata is not None:
        result["velocity_retry"] = retry_metadata
    output_path.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"output": str(output_path), "state": mode.state,
                      "ticks": tick, "completed_stage_count": len(stages),
                      "first_failure": first_failure,
                      "config_sha256": config_sha}, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--retry-first-velocity-failure", action="store_true")
    args = parser.parse_args()
    if not args.retry_first_velocity_failure:
        run(args.config, args.output)
        return
    source_bytes = args.config.read_bytes()
    source_sha = hashlib.sha256(source_bytes).hexdigest()
    initial = json.loads(args.output.read_bytes())
    failure = initial.get("first_failure") or {}
    if (initial.get("config_sha256_before") != source_sha
            or failure.get("reason") != "velocity_clip"):
        raise ValueError("single retry requires this source SHA's first velocity-clip audit")
    config = json.loads(source_bytes)
    failed_stage = failure["stage"]
    stage = next(item for item in config["stages"] if item["name"] == failed_stage)
    original_durations = list(stage["segment_durations_s"])
    factor = 1.7
    stage["segment_durations_s"] = [float(value) * factor for value in original_durations]
    config["direct_cpu_velocity_retry"] = {
        "source_config_sha256": source_sha, "stage": failed_stage,
        "factor": factor, "original_durations_s": original_durations,
        "candidate_durations_s": stage["segment_durations_s"],
        "scope": "single failed stage; timing only; all geometry frozen",
    }
    candidate_bytes = (json.dumps(config, indent=2, allow_nan=False) + "\n").encode()
    retry_metadata = {
        "source_config_sha256": source_sha,
        "candidate_config_sha256": hashlib.sha256(candidate_bytes).hexdigest(),
        "candidate_config_bytes_base64": base64.b64encode(candidate_bytes).decode("ascii"),
        "factor": factor, "stage": failed_stage,
        "initial_audit": {
            "state": initial.get("state"), "ticks": initial.get("ticks"),
            "completed_stage_count": initial.get("completed_stage_count"),
            "config_sha256": initial.get("config_sha256_before"),
            "first_failure": failure,
        },
    }
    run(args.config, args.output, candidate_bytes, retry_metadata)


if __name__ == "__main__":
    main()
