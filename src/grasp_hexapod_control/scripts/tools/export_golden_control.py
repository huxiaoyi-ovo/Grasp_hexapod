#!/usr/bin/env python3
"""导出 C++ 控制器移植的金标数值回归数据。

用与 run_real.py 完全相同的 GraspController 控制链（纯 CPU，不依赖 ROS），
在确定性输入序列下逐帧录制 q_des / 足端目标，并导出运动学快照、
compact 左右镜像、dock_system 解析结果与接近几何，供
grasp_hexapod_control（C++ 部分）的 gtest 逐项对比。

序列设计（两侧实现必须完全一致）：
    reset_then_walk : 扰动关节角 -> B 回站 -> 恒定速度步态 -> 反向 -> 停止
    climb_left      : Q_STAND 入口，compact 左侧、hardware_execution=True、
                      理想舵服（q_cur = 上一帧 q_des），直到 DONE
伺服模型为"零延迟完美跟踪"，因此整个序列是确定性的。
"""

import json
import sys
from pathlib import Path

import numpy as np

SCRIPTS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS_DIR))

from climb_mode import ClimbMode  # noqa: E402
from control import GraspController  # noqa: E402
from dock_mode import load_dock_system  # noqa: E402
from kinematics import Q_STAND  # noqa: E402
from utils.climb import (  # noqa: E402
    derive_compact_approach_geometry,
    select_compact_climb_side,
)

DT = 1.0 / 30.0
CLIMB_FOOT_GATE_M = 0.05  # 与 run_real.launch 的 climb_foot_gate_m 默认一致
GOLDEN_PATH = (
    SCRIPTS_DIR.parents[1]
    / "grasp_hexapod_control"
    / "test"
    / "golden_control_trajectories.json"
)
DESCRIPTION_DIR = SCRIPTS_DIR.parents[1] / "grasp_hexapod_description"


def make_controller():
    return GraspController(
        DT,
        enable_link_collision_check=False,
        climb_timeout_uses_wall_time=False,
    )


def flatten_joints(q):
    return [float(value) for value in np.asarray(q, dtype=np.float64).reshape(18)]


def flatten_feet(feet):
    return [float(value) for value in np.asarray(feet, dtype=np.float64).reshape(18)]


def kinematics_samples():
    """固定关节角下的 FK/Jacobian/DLS/碰撞检查快照。"""
    rng = np.random.default_rng(20260909)
    samples = []
    q_values = [
        Q_STAND.copy(),
        Q_STAND + np.deg2rad(np.array([[6, -8, 5], [-4, 7, -6], [8, -5, 4],
                                       [-7, 6, -5], [5, -9, 7], [-6, 4, -8]])),
    ]
    for _ in range(3):
        q_values.append(
            np.clip(
                Q_STAND + rng.normal(0.0, np.deg2rad(10.0), size=(6, 3)),
                -1.2,
                1.2,
            )
        )
    controller = make_controller()
    for q in q_values:
        kinematic = controller.kinematic
        samples.append(
            {
                "q": flatten_joints(q),
                "forward_hip": flatten_feet(kinematic.forward(q)),
                "forward_base": flatten_feet(kinematic.forward_base(q)),
                "jacobian": [
                    float(value)
                    for value in np.asarray(kinematic.jacobian(q)).reshape(54)
                ],
                "damped_inverse": [
                    float(value)
                    for value in np.asarray(
                        kinematic.damped_inverse_jacobian(q)
                    ).reshape(54)
                ],
                "link_collision_free": [
                    int(value)
                    for value in controller._link_collision_free(q)
                ],
            }
        )
    return samples


def replay_reset_then_walk():
    controller = make_controller()
    q_start = Q_STAND + np.deg2rad(
        np.array([[10, -12, 8], [-9, 11, -7], [12, -10, 9],
                  [-11, 9, -8], [9, -13, 11], [-10, 8, -9]])
    )
    frames = []
    command = np.zeros(4, dtype=np.float64)

    # B 回站：reset_to_stand 后逐帧更新。
    controller.reset_to_stand(q_start)
    q_cur = q_start.copy()
    for _ in range(int(round(2.5 / DT))):
        q_des = controller.update(q_cur, command)
        frames.append(
            {
                "q_des": flatten_joints(q_des),
                "foot_desired": flatten_feet(controller.foot_desired_base),
            }
        )
        q_cur = q_des

    # 恒定速度步态（含升降与偏航）。
    command = np.array([0.05, 0.10, 0.0, 0.15], dtype=np.float64)
    for _ in range(300):
        q_des = controller.update(q_cur, command)
        frames.append(
            {
                "q_des": flatten_joints(q_des),
                "foot_desired": flatten_feet(controller.foot_desired_base),
            }
        )
        q_cur = q_des

    # 反向 + 机身上升。
    command = np.array([-0.08, -0.05, 0.008, -0.20], dtype=np.float64)
    for _ in range(300):
        q_des = controller.update(q_cur, command)
        frames.append(
            {
                "q_des": flatten_joints(q_des),
                "foot_desired": flatten_feet(controller.foot_desired_base),
            }
        )
        q_cur = q_des

    # 停止（摆动组落地、transfer、完全停止）。
    command = np.zeros(4, dtype=np.float64)
    for _ in range(150):
        q_des = controller.update(q_cur, command)
        frames.append(
            {
                "q_des": flatten_joints(q_des),
                "foot_desired": flatten_feet(controller.foot_desired_base),
            }
        )
        q_cur = q_des

    return frames, controller.mode


def replay_climb_left():
    controller = make_controller()
    config = controller.climb_mode._load_config()
    config = select_compact_climb_side(config, "left")
    config["settle_gate"]["max_foot_target_error_m"] = CLIMB_FOOT_GATE_M
    controller.enter_climb(Q_STAND.copy(), config=config, hardware_execution=True)

    command = np.zeros(4, dtype=np.float64)
    q_cur = Q_STAND.copy()
    frames = []
    for _ in range(20000):
        q_des = controller.update(q_cur, command)
        frames.append(
            {
                "q_des": flatten_joints(q_des),
                "foot_desired": flatten_feet(controller.foot_desired_base),
            }
        )
        q_cur = q_des
        if controller.climb_mode.state in (ClimbMode.DONE, ClimbMode.FAILED):
            break

    if controller.climb_mode.state != ClimbMode.DONE:
        raise RuntimeError(
            "golden climb replay did not finish: "
            + str(controller.climb_mode.failure_reason)
        )
    return frames, controller.mode


def climb_mirror_digest():
    """compact 右侧镜像的字段摘要（与 C++ selectCompactClimbSide 对比）。"""
    controller = make_controller()
    base_config = controller.climb_mode._load_config()
    mirrored = select_compact_climb_side(base_config, "right")
    stages = []
    for stage in mirrored["stages"]:
        stages.append(
            {
                "name": stage["name"],
                "pose_start": [float(v) for v in stage["pose_start"]],
                "pose_end": [float(v) for v in stage["pose_end"]],
                "active_legs": [int(v) for v in stage["active_legs"]],
                "anchor_knots": [
                    [float(v) for v in np.asarray(knot).reshape(18)]
                    for knot in stage["anchor_knots"]
                ],
            }
        )
    return {
        "xiaolan_translation": [float(v) for v in mirrored["xiaolan_translation"]],
        "p0_base": [float(v) for v in mirrored["p0"]["base"]],
        "p0_anchors": flatten_feet(mirrored["p0"]["anchors_world_m"]),
        "terminal_q": flatten_joints(mirrored["terminal_q_rad"]),
        "stage_names": [stage["name"] for stage in mirrored["stages"]],
        "stages": stages,
    }


def dock_system_digest():
    system = load_dock_system()
    return {
        "lock_from_camera": [float(v) for v in np.asarray(system["lock_from_camera"]).reshape(16)],
        "pin_from_tag": {
            str(tag_id): [float(v) for v in np.asarray(pose).reshape(16)]
            for tag_id, pose in system["pin_from_tag"].items()
        },
        "real_calibrated": bool(system["real_calibrated"]),
        "tag_size_m": float(system["tag_size_m"]),
    }


def approach_geometry_digest():
    controller = make_controller()
    config = controller.climb_mode._load_config()
    geometry = derive_compact_approach_geometry(config)
    return {
        "targets": {
            side: [float(v) for v in np.asarray(pose).reshape(16)]
            for side, pose in geometry["targets"].items()
        },
        "keepout": [float(v) for v in np.asarray(geometry["xiaolan_keepout_polygon_xy_m"]).reshape(-1)],
    }


def main():
    golden = {
        "dt": DT,
        "climb_foot_gate_m": CLIMB_FOOT_GATE_M,
        "kinematics": {"samples": kinematics_samples()},
    }

    walk_frames, walk_mode = replay_reset_then_walk()
    golden["sequences"] = [
        {"name": "reset_then_walk", "frames": walk_frames, "final_mode": str(walk_mode)}
    ]
    print(f"reset_then_walk: {len(walk_frames)} frames, mode={walk_mode}")

    climb_frames, climb_mode = replay_climb_left()
    golden["sequences"].append(
        {"name": "climb_left", "frames": climb_frames, "final_mode": str(climb_mode)}
    )
    print(f"climb_left: {len(climb_frames)} frames, mode={climb_mode}")

    golden["climb_mirror"] = climb_mirror_digest()
    golden["dock_system"] = dock_system_digest()
    golden["approach_geometry"] = approach_geometry_digest()

    GOLDEN_PATH.parent.mkdir(parents=True, exist_ok=True)
    with GOLDEN_PATH.open("w") as file:
        json.dump(golden, file, allow_nan=False)
    print(f"golden written: {GOLDEN_PATH}")


if __name__ == "__main__":
    main()
