#!/usr/bin/env python3
"""Isaac Gym仿真运行入口。

功能：
    配置CUDA/PhysX，加载六足、小蓝和地面，读取手柄与仿真关节状态，
    调用GraspController，并把18个关节位置目标发送给Isaac Gym；可选地
    自动执行固定行走序列并记录舵机目标和仿真反馈。加--ros时从ROS读取
    Joy/导航输入，但控制器仍在Isaac控制帧内同步运行，避免跨节点时序抖动。
输入：
    手柄归一化指令；Isaac Gym关节状态，内部转换为q_cur.shape=(6,3)，单位rad。
输出：
    Isaac顺序的18关节位置目标；viewer画面；可选的CSV时序轨迹。
结构：
    创建仿真与资产 -> 建立关节顺序映射 -> 主控制循环 -> 资源释放。
约定：
    base_link中+x向右、+y向前、+z向上；长度单位m，角度单位rad。
"""

import argparse
import csv
import json
from pathlib import Path
import struct
import sys
import time
from typing import Any

from isaacgym import gymapi as _gymapi
import numpy as np

# catkin转发脚本不与控制器模块同目录，ROS启动时需定位源码scripts。
scripts_dir = Path(__file__).resolve().parent
if not (scripts_dir / "control.py").exists():
    import rospkg

    scripts_dir = (
        Path(rospkg.RosPack().get_path("grasp_hexapod_control"))
        / "scripts"
    )
sys.path.insert(0, str(scripts_dir))

from control import GraspController
from climb_mode import ClimbMode
from kinematics import LEG_NAMES, JOINT_NAMES
from utils import (
    CONTROL_DOF_NAMES,
    NavigationState,
    build_dof_indices,
    control_to_external,
    external_to_control,
    package_config_path,
)
from utils.climb import resolve_compact_stage_range

# Isaac Gym的C扩展没有完整类型声明，编辑器无法静态识别其动态属性。
gymapi: Any = _gymapi


DEFAULT_TRACE_PATH = Path("logs/servo_walk_trace.csv")
TRACE_ACTION_DURATION = 5.0
MISSION_PV_BOUNDARY = np.array(
    [
        [-1.5, -1.5],
        [1.5, -1.5],
        [1.5, 1.5],
        [-1.5, 1.5],
    ],
    dtype=np.float64,
)


def _planar_transform(x, y, z, yaw):
    """生成用于模拟导航的世界位姿矩阵。"""

    cosine, sine = np.cos(yaw), np.sin(yaw)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = [
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ]
    transform[:3, 3] = (x, y, z)
    return transform


def _isaac_transform_matrix(transform):
    """把Isaac根位姿转换为4×4齐次矩阵。"""

    position = transform.p
    quaternion = np.array(
        [transform.r.x, transform.r.y, transform.r.z, transform.r.w],
        dtype=np.float64,
    )
    quaternion /= np.linalg.norm(quaternion)
    x, y, z, w = quaternion
    rotation = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    output = np.eye(4, dtype=np.float64)
    output[:3, :3] = rotation
    output[:3, 3] = (position.x, position.y, position.z)
    return output


class SimulatedRtkImu:
    """用仿真根位姿生成控制器可复用的RTK/IMU导航观测。"""

    def __init__(self, xiaolan_translation):
        self.pv_from_xiaolan = np.eye(4, dtype=np.float64)
        self.pv_from_xiaolan[:3, 3] = np.asarray(
            xiaolan_translation,
            dtype=np.float64,
        ).reshape(3)

    def snapshot(self, pv_from_base, stamp):
        """返回一帧理想且持续刷新的模拟导航数据。"""

        return NavigationState(
            stamp=float(stamp),
            valid=True,
            landing_confirmed=True,
            pv_from_base=np.asarray(
                pv_from_base,
                dtype=np.float64,
            ).reshape(4, 4).copy(),
            pv_from_xiaolan=self.pv_from_xiaolan.copy(),
            pv_boundary=MISSION_PV_BOUNDARY.copy(),
        )

    def xiaolan_from_base(self, pv_from_base):
        """返回小蓝坐标系中的六足相对位姿。"""

        return np.linalg.inv(self.pv_from_xiaolan) @ pv_from_base


def _random_mission_start(compact, base_height, requested_seed):
    """在compact入口外侧生成可复现的随机平地开局。"""

    seed = requested_seed
    if seed is None:
        seed = int(np.random.SeedSequence().generate_state(1)[0])
    random = np.random.default_rng(seed)
    target_xy = np.asarray(compact["p0"]["base"][:2], dtype=np.float64)
    xiaolan_xy = np.asarray(
        compact["xiaolan_translation"][:2],
        dtype=np.float64,
    )
    outward = target_xy - xiaolan_xy
    outward /= np.linalg.norm(outward)
    lateral = np.array([-outward[1], outward[0]], dtype=np.float64)
    start_xy = (
        target_xy
        + random.uniform(0.35, 0.55) * outward
        + random.uniform(-0.15, 0.15) * lateral
    )
    yaw = random.uniform(-np.pi, np.pi)
    return int(seed), np.array(
        [start_xy[0], start_xy[1], base_height, yaw],
        dtype=np.float64,
    )


def _drive_properties(base_properties, joint_speed):
    """按 Approach 共用基线生成 Isaac 关节驱动参数。"""

    properties = base_properties.copy()
    properties["driveMode"].fill(int(gymapi.DOF_MODE_POS))
    properties["stiffness"].fill(100.0)
    properties["damping"].fill(0.8)
    properties["velocity"] *= joint_speed
    return properties


class JoyStick:
    """仿真手柄输入；输出归一化的右移、前进、升降和偏航指令。"""

    DEADZONE = 0.20

    def __init__(self):
        # ROS仿真不需要pygame；只在原直接控制链路中加载它。
        import pygame

        self.pygame = pygame
        pygame.init()
        pygame.joystick.init()

        printed_waiting = False
        while pygame.joystick.get_count() == 0:
            pygame.event.pump()
            if not printed_waiting:
                print("Waiting for joystick to connect...")
                printed_waiting = True
            time.sleep(0.2)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"Joystick connected: {self.joystick.get_name()}")

    @staticmethod
    def _deadzone(value):
        magnitude = abs(value)
        if magnitude <= JoyStick.DEADZONE:
            return 0.0

        # 死区边缘映射到0，最大行程仍映射到1；
        # 因而速度随有效摇杆行程连续、线性变化。
        return np.sign(value) * (
            (magnitude - JoyStick.DEADZONE)
            / (1.0 - JoyStick.DEADZONE)
        )

    def get_commands(self):
        self.pygame.event.pump()
        axis_right = self._deadzone(self.joystick.get_axis(0))
        axis_forward = self._deadzone(-self.joystick.get_axis(1))
        axis_yaw = self._deadzone(-self.joystick.get_axis(3))
        return axis_right, axis_forward, axis_yaw


class RosSimTelemetry:
    """发布与实机相同的关节话题，但不让话题参与仿真控制闭环。"""

    def __init__(self):
        import rospy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray, Header

        self.rospy = rospy
        self.joint_state_type = JointState
        self.header_type = Header
        self.target_type = Float64MultiArray
        self.position_publishers = {
            leg: rospy.Publisher(
                f"/{leg}_pos",
                JointState,
                queue_size=1,
            )
            for leg in LEG_NAMES
        }
        self.target_publishers = {
            leg: rospy.Publisher(
                f"/{leg}_des",
                Float64MultiArray,
                queue_size=1,
            )
            for leg in LEG_NAMES
        }

    def publish_feedback(self, joint_position):
        joint_position = np.asarray(
            joint_position,
            dtype=np.float64,
        ).reshape(6, 3)
        for leg_index, leg_name in enumerate(LEG_NAMES):
            self.position_publishers[leg_name].publish(
                self.joint_state_type(
                    header=self.header_type(
                        stamp=self.rospy.Time.now()
                    ),
                    name=[
                        f"{leg_name}_thigh_joint",
                        f"{leg_name}_knee_joint",
                        f"{leg_name}_ankle_joint",
                    ],
                    position=joint_position[leg_index].tolist(),
                )
            )

    def publish_target(self, joint_target):
        joint_target = np.asarray(
            joint_target,
            dtype=np.float64,
        ).reshape(6, 3)
        for leg_index, leg_name in enumerate(LEG_NAMES):
            q_leg = joint_target[leg_index]
            self.target_publishers[leg_name].publish(
                self.target_type(
                    data=[
                        1.0,
                        float(q_leg[0]),
                        float(q_leg[1]),
                        float(q_leg[2]),
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                    ]
                )
            )


def parse_arguments():
    """读取仿真入口参数；不启用录制时保持原来的手柄控制。"""

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--record-servo-trace",
        nargs="?",
        const=DEFAULT_TRACE_PATH,
        type=Path,
        help="执行固定动作序列并写入 CSV 轨迹",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="无 viewer 运行轨迹记录或 ROS 仿真",
    )
    parser.add_argument(
        "--ros",
        action="store_true",
        help="使用 ROS Joy/导航输入及同步仿真控制",
    )
    parser.add_argument(
        "--control-rate",
        type=float,
        choices=(30.0, 60.0),
        default=30.0,
        help="控制器更新频率，默认与 30 Hz 执行器链路一致",
    )
    parser.add_argument(
        "--actuator-rate",
        type=float,
        choices=(30.0, 60.0),
        default=30.0,
        help="向 Isaac Gym 写入关节目标的频率，默认与实机舵机链路同为 30 Hz",
    )
    parser.add_argument(
        "--physics-rate",
        type=float,
        default=240.0,
        help="固定的 Isaac Gym 物理频率",
    )
    parser.add_argument(
        "--max-linear-speed",
        type=float,
        default=0.22,
        help="直接控制与轨迹的平面速度，单位 m/s",
    )
    parser.add_argument(
        "--joint-speed",
        type=float,
        default=1.2,
        help=(
            "普通仿真关节速度上限倍率，默认 1.2（约 4.8 rad/s，对齐 "
            "LX-15D 7.4V 官方空载上限）；大于 1.2 仅仿真诊断"
        ),
    )
    parser.add_argument(
        "--climb-start",
        action="store_true",
        help="加载 compact 攀爬场景并立即回放",
    )
    parser.add_argument(
        "--full-mission",
        action="store_true",
        help="随机开局并自动执行APPROACH、CLIMB到DOCK交接的仿真全流程",
    )
    parser.add_argument(
        "--mission-seed",
        type=int,
        help="全流程随机开局种子；省略时每次随机并在终端打印实际种子",
    )
    parser.add_argument(
        "--approach-timeout",
        type=float,
        default=60.0,
        help="自动接近超时，仅触发失败保持，不作为成功门槛，单位s",
    )
    parser.add_argument(
        "--climb-scene",
        action="store_true",
        help="加载 compact 所选起点与小蓝场景，等待 X 启动",
    )
    parser.add_argument(
        "--climb-config",
        type=Path,
        help=(
            "simulation-only compact 配置路径；仅可与 --climb-start、"
            "--climb-scene 或 --full-mission 一起使用"
        ),
    )
    parser.add_argument(
        "--climb-speed",
        type=float,
        default=1.0,
        help="compact 轨迹回放倍率（仅显式诊断），默认 1.0",
    )
    parser.add_argument(
        "--climb-joint-speed",
        type=float,
        default=1.2,
        help=(
            "compact 关节速度上限倍率，默认 1.2（LX-15D 官方空载上限候选，"
            "约 4.8 rad/s）；大于 1.2 仅仿真诊断，非实机对齐"
        ),
    )
    parser.add_argument(
        "--climb-from",
        help="compact 闭区间起点（C1..C35 或运行时阶段名）",
    )
    parser.add_argument(
        "--climb-to",
        help="compact 闭区间终点（C1..C35 或运行时阶段名）",
    )
    parser.add_argument(
        "--climb-metrics",
        type=Path,
        help="把 simulation-only compact 诊断指标写为 JSON",
    )
    parser.add_argument(
        "--max-vertical-speed",
        type=float,
        default=0.02,
        help="直接控制的机身升降速度，单位 m/s",
    )
    # roslaunch会附加__name:=和__log:=；普通命令行参数仍严格检查。
    argv = [
        argument
        for argument in sys.argv[1:]
        if not argument.startswith("__")
    ]
    return parser.parse_args(argv)


def _compact_root_quaternion(base):
    """返回与 ClimbMode._world_from_base 的 Ry(pitch) @ Rx(roll) 一致的四元数。"""

    _, _, _, roll, pitch = np.asarray(base, dtype=np.float64)
    half_roll, half_pitch = roll / 2.0, pitch / 2.0
    sx, cx = np.sin(half_roll), np.cos(half_roll)
    sy, cy = np.sin(half_pitch), np.cos(half_pitch)
    return gymapi.Quat(
        float(cy * sx), float(sy * cx), float(-sy * sx), float(cy * cx)
    )


def prepare_compact_stage_entry(compact, start_stage_index, end_stage_index, dt):
    """CPU 理想回放到区间入口，保留共同 DLS/碰撞保护产生的连续 IK 分支。"""

    if start_stage_index == 0:
        return (
            np.asarray(compact["p0"]["q_rad"], dtype=np.float64).copy(),
            np.array(
                (*compact["p0"]["base"][:3], 0.0, compact["p0"]["base"][3]),
                dtype=np.float64,
            ),
        )

    warmup = GraspController(dt=dt)
    q_snapshot = np.asarray(compact["p0"]["q_rad"], dtype=np.float64).copy()
    q_snapshot = warmup.replay_climb_prefix(
        q_snapshot,
        compact,
        start_stage_index - 1,
        max_ticks=200000,
    )
    stage = compact["stages"][start_stage_index]
    pose = np.asarray(stage["pose_start"], dtype=np.float64).copy()
    # 独立入口检查让 selected pose/anchors 和快照 q 走同一 ClimbMode 门限。
    entry_check = GraspController(dt=dt)
    entry_check.enter_climb(
        q_snapshot,
        compact,
        start_stage_index,
        end_stage_index,
    )
    return q_snapshot, pose


def _new_climb_metric(index, name):
    return {
        "stage_index": index,
        "alias": f"C{index + 1}",
        "runtime_name": name,
        "simulated_duration_s": 0.0,
        "max_joint_target_tracking_error_rad": 0.0,
        "max_kinematic_foot_target_error_m": 0.0,
        "worst_joint_leg": None,
        "worst_joint_name": None,
        "worst_joint_time_s": None,
        "worst_joint_actual_rad": None,
        "worst_joint_target_rad": None,
        "worst_foot_leg": None,
        "worst_foot_time_s": None,
        "worst_foot_actual_base_xyz_m": None,
        "worst_foot_target_base_xyz_m": None,
        "end_joint_target_tracking_error_rad": 0.0,
        "end_foot_target_error_m": 0.0,
        "max_root_position_error_m": 0.0,
        "worst_root_time_s": None,
        "worst_root_actual_xyz_m": None,
        "worst_root_target_xyz_m": None,
        "end_root_position_error_m": 0.0,
        "max_world_foot_anchor_error_m": 0.0,
        "worst_world_foot_anchor_leg": None,
        "worst_world_foot_anchor_time_s": None,
        "worst_world_foot_actual_xyz_m": None,
        "worst_world_foot_anchor_xyz_m": None,
        "end_world_foot_anchor_error_m": 0.0,
        "stage_start_anchor_world_xyz_m": None,
        "stage_start_actual_world_foot_xyz_m": None,
        "max_support_foot_world_drift_m": 0.0,
        "worst_support_foot_drift_leg": None,
        "worst_support_foot_drift_time_s": None,
        "worst_support_foot_start_world_xyz_m": None,
        "worst_support_foot_current_world_xyz_m": None,
        "end_support_foot_world_drift_m": 0.0,
        "min_joint_limit_margin_rad": None,
    }


def _world_foot_positions(root_position, root_quaternion_xyzw, feet_base):
    """用只读的实际根位姿把 FK 足端转换到世界坐标。"""

    root_position = np.asarray(root_position, dtype=np.float64).reshape(3)
    quaternion = np.asarray(root_quaternion_xyzw, dtype=np.float64).reshape(4)
    feet_base = np.asarray(feet_base, dtype=np.float64).reshape(6, 3)
    norm = np.linalg.norm(quaternion)
    if (
        not np.isfinite(root_position).all()
        or not np.isfinite(quaternion).all()
        or not np.isfinite(feet_base).all()
        or norm == 0.0
    ):
        raise ValueError("actual root pose and FK feet must be finite")
    x, y, z, w = quaternion / norm
    rotation = np.array(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=np.float64,
    )
    return feet_base @ rotation.T + root_position


def _update_climb_metric(
    metric, controller, q_current, root_position, root_quaternion_xyzw, dt
):
    """更新不反馈到控制器的关节、根部和足端漂移诊断。"""

    metric["simulated_duration_s"] += dt
    joint_errors = np.abs(q_current - controller.q_des)
    joint_index = np.unravel_index(np.argmax(joint_errors), joint_errors.shape)
    joint_error = float(joint_errors[joint_index])
    metric["end_joint_target_tracking_error_rad"] = joint_error
    if joint_error > metric["max_joint_target_tracking_error_rad"]:
        metric["max_joint_target_tracking_error_rad"] = joint_error
        metric["worst_joint_leg"] = LEG_NAMES[joint_index[0]]
        metric["worst_joint_name"] = JOINT_NAMES[joint_index[1]]
        metric["worst_joint_time_s"] = metric["simulated_duration_s"]
        metric["worst_joint_actual_rad"] = float(q_current[joint_index])
        metric["worst_joint_target_rad"] = float(controller.q_des[joint_index])

    actual_base = controller.kinematic.forward_base(q_current)
    foot_errors = np.linalg.norm(
        actual_base - controller.foot_desired_base,
        axis=1,
    )
    foot_index = int(np.argmax(foot_errors))
    foot_error = float(foot_errors[foot_index])
    metric["end_foot_target_error_m"] = foot_error
    if foot_error > metric["max_kinematic_foot_target_error_m"]:
        metric["max_kinematic_foot_target_error_m"] = foot_error
        metric["worst_foot_leg"] = LEG_NAMES[foot_index]
        metric["worst_foot_time_s"] = metric["simulated_duration_s"]
        metric["worst_foot_actual_base_xyz_m"] = actual_base[foot_index].tolist()
        metric["worst_foot_target_base_xyz_m"] = (
            controller.foot_desired_base[foot_index].tolist()
        )

    actual_root = np.asarray(root_position, dtype=np.float64).reshape(3)
    target_root = np.asarray(controller.climb_mode.base_pose[:3], dtype=np.float64)
    root_error = float(np.linalg.norm(actual_root - target_root))
    metric["end_root_position_error_m"] = root_error
    if root_error > metric["max_root_position_error_m"]:
        metric["max_root_position_error_m"] = root_error
        metric["worst_root_time_s"] = metric["simulated_duration_s"]
        metric["worst_root_actual_xyz_m"] = actual_root.tolist()
        metric["worst_root_target_xyz_m"] = target_root.tolist()

    actual_world = _world_foot_positions(
        actual_root, root_quaternion_xyzw, actual_base
    )
    anchors_world = np.asarray(
        controller.climb_mode.anchors_world, dtype=np.float64
    ).reshape(6, 3)
    anchor_errors = np.linalg.norm(actual_world - anchors_world, axis=1)
    anchor_index = int(np.argmax(anchor_errors))
    anchor_error = float(anchor_errors[anchor_index])
    metric["end_world_foot_anchor_error_m"] = anchor_error
    if anchor_error > metric["max_world_foot_anchor_error_m"]:
        metric["max_world_foot_anchor_error_m"] = anchor_error
        metric["worst_world_foot_anchor_leg"] = LEG_NAMES[anchor_index]
        metric["worst_world_foot_anchor_time_s"] = metric["simulated_duration_s"]
        metric["worst_world_foot_actual_xyz_m"] = actual_world[anchor_index].tolist()
        metric["worst_world_foot_anchor_xyz_m"] = anchors_world[anchor_index].tolist()

    if metric["stage_start_anchor_world_xyz_m"] is None:
        metric["stage_start_anchor_world_xyz_m"] = anchors_world.tolist()
        metric["stage_start_actual_world_foot_xyz_m"] = actual_world.tolist()
    start_anchors = np.asarray(metric["stage_start_anchor_world_xyz_m"])
    start_actual_world = np.asarray(metric["stage_start_actual_world_foot_xyz_m"])
    stationary = np.linalg.norm(anchors_world - start_anchors, axis=1) <= 1e-12
    if stationary.any():
        drifts = np.linalg.norm(actual_world - start_actual_world, axis=1)
        drifts[~stationary] = -np.inf
        drift_index = int(np.argmax(drifts))
        drift = float(drifts[drift_index])
        metric["end_support_foot_world_drift_m"] = drift
        if drift > metric["max_support_foot_world_drift_m"]:
            metric["max_support_foot_world_drift_m"] = drift
            metric["worst_support_foot_drift_leg"] = LEG_NAMES[drift_index]
            metric["worst_support_foot_drift_time_s"] = metric["simulated_duration_s"]
            metric["worst_support_foot_start_world_xyz_m"] = (
                start_actual_world[drift_index].tolist()
            )
            metric["worst_support_foot_current_world_xyz_m"] = (
                actual_world[drift_index].tolist()
            )

    margin = float(np.min(controller.kinematic.joint_limit_margins(q_current)))
    previous_margin = metric["min_joint_limit_margin_rad"]
    metric["min_joint_limit_margin_rad"] = (
        margin if previous_margin is None else min(previous_margin, margin)
    )


def _write_climb_metrics(
    path,
    compact,
    start_index,
    end_index,
    climb_speed,
    joint_speed,
    climb_mode,
    metrics_by_stage,
    mission=None,
):
    """写出不参与控制或阶段门限的 simulation-only 诊断。"""

    result = {
        "schema": "SIMULATION_ONLY_CLIMB_PREVIEW_METRICS_V1",
        "simulation_only": True,
        "diagnostics_only_not_contact_or_stability_proof": True,
        "resolved_range": {
            "from": {
                "alias": f"C{start_index + 1}",
                "runtime_name": compact["stages"][start_index]["name"],
            },
            "to": {
                "alias": f"C{end_index + 1}",
                "runtime_name": compact["stages"][end_index]["name"],
            },
        },
        "climb_speed": climb_speed,
        "climb_joint_speed": joint_speed,
        "final_state": climb_mode.state,
        "final_reason": climb_mode.failure_reason or "none",
        "per_stage": [
            metrics_by_stage[index]
            for index in sorted(metrics_by_stage)
        ],
    }
    if mission is not None:
        finite_or_none = lambda value: (
            float(value) if np.isfinite(value) else None
        )
        result["full_mission"] = {
            "state": mission.state,
            "reason": mission.reason or "none",
            "dock_handoff_reached": mission.state == mission.DOCK,
            "entry_position_error_m": finite_or_none(
                mission.last_prepare_position_error_m
            ),
            "entry_yaw_error_deg": finite_or_none(
                np.degrees(mission.last_prepare_yaw_error_rad)
            ),
            "entry_joint_error_rad": finite_or_none(
                mission.last_prepare_joint_error_rad
            ),
            "entry_world_foot_error_m": finite_or_none(
                mission.last_prepare_world_foot_error_m
            ),
            "entry_linear_speed_m_s": finite_or_none(
                mission.last_prepare_linear_speed_m_s
            ),
            "entry_angular_speed_deg_s": finite_or_none(
                np.degrees(mission.last_prepare_angular_speed_rad_s)
            ),
            "terminal_position_error_m": finite_or_none(
                mission.final_position_error_m
            ),
            "terminal_orientation_error_deg": finite_or_none(
                np.degrees(mission.final_orientation_error_rad)
            ),
            "terminal_world_foot_error_m": finite_or_none(
                mission.final_world_foot_error_m
            ),
            "geometry_gate_only_not_contact_load_or_docking_proof": True,
        }
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        json.dump(result, file, indent=2, allow_nan=False)
    print(f"[climb] diagnostics-only metrics: {path}")


def build_servo_trace_script(controller, linear_speed, yaw_rate):
    """生成初始化、前进、右移和旋转的确定性测试时间轴。"""

    dt = controller.dt
    half_cycle = (
        controller.approach_mode.phase_duration
        + controller.approach_mode.transfer_duration
    )
    full_cycle = 2.0 * half_cycle
    action_cycles = max(
        1,
        int(round(TRACE_ACTION_DURATION / full_cycle)),
    )
    action_duration = action_cycles * full_cycle
    print(
        f"Trace action: {action_cycles} full gait cycles, "
        f"{action_duration:.2f} s"
    )
    stages = (
        ("initialize", 1.0, [0.0, 0.0, 0.0, 0.0]),
        ("forward", action_duration, [0.0, linear_speed, 0.0, 0.0]),
        ("settle", 2.0 * half_cycle, [0.0, 0.0, 0.0, 0.0]),
        ("right", action_duration, [linear_speed, 0.0, 0.0, 0.0]),
        ("settle", 2.0 * half_cycle, [0.0, 0.0, 0.0, 0.0]),
        ("rotate", action_duration, [0.0, 0.0, 0.0, yaw_rate]),
        ("settle", 2.0 * half_cycle, [0.0, 0.0, 0.0, 0.0]),
    )

    script = []
    for name, duration, command in stages:
        frame_count = int(round(duration / dt))
        script.extend(
            (name, np.asarray(command, dtype=np.float64))
            for _ in range(frame_count)
        )
    return script


def write_servo_trace(path, rows):
    """保存控制器顺序的目标角和同帧Isaac Gym反馈角。"""

    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        "frame",
        "time_s",
        "stage",
        "vx",
        "vy",
        "vz",
        "wz",
    ]
    header.extend(f"target_{name}" for name in CONTROL_DOF_NAMES)
    header.extend(f"actual_{name}" for name in CONTROL_DOF_NAMES)

    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(rows)

    print(f"Servo trace saved: {path}")
    duration = 0.0
    if len(rows) >= 2:
        duration = rows[-1][1] + rows[1][1] - rows[0][1]
    print(f"Recorded frames: {len(rows)}, duration: {duration:.2f} s")


def add_static_stl_triangle_mesh(gym, sim, mesh_path, position):
    """把二进制STL原始三角面作为静态PhysX碰撞面加入仿真。"""
    data = mesh_path.read_bytes()
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    record_dtype = np.dtype(
        [
            ("normal", "<f4", (3,)),
            ("vertices", "<f4", (3, 3)),
            ("attribute", "<u2"),
        ]
    )
    expected_size = 84 + triangle_count * record_dtype.itemsize
    if len(data) != expected_size:
        raise ValueError(f"Only binary STL is supported: {mesh_path}")

    records = np.frombuffer(
        data,
        dtype=record_dtype,
        count=triangle_count,
        offset=84,
    )
    vertices = np.ascontiguousarray(
        records["vertices"].reshape(-1, 3)
    )
    triangles = np.arange(
        triangle_count * 3,
        dtype=np.uint32,
    ).reshape(-1, 3)

    mesh_params = gymapi.TriangleMeshParams()
    mesh_params.nb_vertices = vertices.shape[0]
    mesh_params.nb_triangles = triangles.shape[0]
    mesh_params.transform.p = position
    mesh_params.static_friction = 1.0
    mesh_params.dynamic_friction = 0.8

    gym.add_triangle_mesh(
        sim,
        vertices.ravel(),
        triangles.ravel(),
        mesh_params,
    )
    print(
        f"Xiaolan loaded: {triangle_count} exact static collision triangles"
    )


def print_model_info(gym, env, actor) :
    # 后续控制器要靠名称建立索引，不能默认Isaac Gym的数组顺序
    rigid_body_names = gym.get_actor_rigid_body_names(env, actor)
    dof_names = gym.get_actor_dof_names(env, actor)
    print(f"Robot loaded: {len(rigid_body_names)} rigid bodies, {len(dof_names)} DOFs")

    return rigid_body_names, dof_names



def main() -> None:
    args = parse_arguments()
    climb_scene = args.climb_start or args.climb_scene or args.full_mission
    if args.full_mission and (args.climb_start or args.climb_scene):
        raise ValueError(
            "--full-mission cannot be combined with --climb-start or "
            "--climb-scene"
        )
    if args.full_mission and (
        args.climb_from is not None or args.climb_to is not None
    ):
        raise ValueError("--full-mission always runs the complete compact climb")
    if args.mission_seed is not None and not args.full_mission:
        raise ValueError("--mission-seed requires --full-mission")
    if args.climb_config is not None and not climb_scene:
        raise ValueError(
            "--climb-config requires --climb-start, --climb-scene or "
            "--full-mission"
        )
    if (args.climb_from is not None or args.climb_to is not None
            or args.climb_metrics is not None) and not climb_scene:
        raise ValueError(
            "--climb-from, --climb-to and --climb-metrics require "
            "--climb-start or --climb-scene"
        )
    if args.ros and args.record_servo_trace is not None:
        raise ValueError("--ros cannot be combined with trace recording")
    if args.ros and (args.climb_start or args.full_mission):
        raise ValueError(
            "--ros compact climbing requires --climb-scene and X start"
        )
    if args.ros and args.climb_scene and (
        args.climb_config is not None
        or args.climb_from is not None
        or args.climb_to is not None
        or args.climb_speed != 1.0
    ):
        raise ValueError(
            "--ros --climb-scene only supports active default compact C1-C35 "
            "at --climb-speed 1.0"
        )
    if climb_scene and args.record_servo_trace is not None:
        raise ValueError("compact climb cannot be combined with --record-servo-trace")
    if args.headless and not args.ros and args.record_servo_trace is None \
            and not args.climb_start and not args.full_mission:
        raise ValueError(
            "--headless is only used with --ros, --record-servo-trace "
            "or an automatic climb/full mission"
        )
    if args.ros:
        import rospy

        rospy.init_node("grasp_hexapod_sim")

    if not np.isfinite(args.climb_speed) or not 0.25 <= args.climb_speed <= 4.0:
        raise ValueError("--climb-speed must be between 0.25 and 4.0")
    if (
        not np.isfinite(args.climb_joint_speed)
        or not 0.5 <= args.climb_joint_speed <= 5.0
    ):
        raise ValueError("--climb-joint-speed must be between 0.5 and 5.0")
    if (
        not np.isfinite(args.joint_speed)
        or not 0.5 <= args.joint_speed <= 5.0
    ):
        raise ValueError("--joint-speed must be between 0.5 and 5.0")
    if (
        not np.isfinite(args.approach_timeout)
        or args.approach_timeout <= 0.0
    ):
        raise ValueError("--approach-timeout must be positive")

    rate_ratio = args.physics_rate / args.control_rate
    if abs(rate_ratio - round(rate_ratio)) > 1e-9:
        raise ValueError("physics rate must be an integer multiple of control rate")
    control_interval = int(round(rate_ratio))
    actuator_ratio = args.control_rate / args.actuator_rate
    if abs(actuator_ratio - round(actuator_ratio)) > 1e-9:
        raise ValueError(
            "control rate must be an integer multiple of actuator rate"
        )
    actuator_interval = int(round(actuator_ratio))
    render_interval = max(1, int(round(args.physics_rate / 60.0)))

    # 仿真从源码树直接运行，不依赖ROS_PACKAGE_PATH或source devel/setup.bash。
    description_root = (
        Path(__file__).resolve().parents[2]
        / "grasp_hexapod_description"
    )
    if not description_root.is_dir():
        import rospkg

        description_root = Path(
            rospkg.RosPack().get_path("grasp_hexapod_description")
        )
    gym = gymapi.acquire_gym()
    # 创建仿真器参数。
    sim_params = gymapi.SimParams()

    sim_params.dt = 1.0 / args.physics_rate
    sim_params.substeps = 2  # 每个物理步的子步数。
    # PhysX明确使用cuda:0计算；控制器仍使用NumPy，因此保留CPU数据管线。
    sim_params.use_gpu_pipeline = False
    sim_params.up_axis = gymapi.UP_AXIS_Z
    sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
    # TGS 求解器：0 为 PGS，1 为 TGS，2 为带热启动的 TGS。
    sim_params.physx.use_gpu = True
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 8
    sim_params.physx.num_velocity_iterations = 2
    sim_params.physx.rest_offset = 0.0
    sim_params.physx.contact_offset = 0.02
    sim_params.physx.friction_offset_threshold = 0.001
    sim_params.physx.friction_correlation_distance = 0.0005
    print(
        "PhysX contact scales: rest_offset=0m contact_offset=20mm "
        "friction_offset_threshold=1mm friction_correlation_distance=0.5mm "
        "robot_thickness=1mm"
    )

    # 创建仿真。
    compute_device_id = 0
    graphics_device_id = -1 if args.headless else 0
    sim = gym.create_sim(
        compute_device_id,
        graphics_device_id,
        gymapi.SIM_PHYSX,
        sim_params,
    )
    if sim is None:
        raise RuntimeError("Failed to create Isaac Gym simulation.")
    #创建地面和viewer
    plane = gymapi.PlaneParams()
    plane.normal = gymapi.Vec3(0.0, 0.0, 1.0)
    plane.static_friction = 1.5
    plane.dynamic_friction = 1.2

    gym.add_ground(sim, plane)

    #加载urdf
    asset_root = str(description_root)
    asset_file = "urdf/hexapod_isaacgym_view.urdf"
    asset_options = gymapi.AssetOptions()
    asset_options.fix_base_link = False
    asset_options.collapse_fixed_joints = False
    asset_options.use_mesh_materials = True
    asset_options.thickness = 0.001

    robot_asset = gym.load_asset(sim, asset_root, asset_file, asset_options)
    if robot_asset is None:
        raise RuntimeError("Failed to load robot asset.")

    compact = None
    climb_start_index = 0
    climb_end_index = None
    if climb_scene:
        compact_path = (
            package_config_path("climb_compact.json")
            if args.climb_config is None else args.climb_config
        )
        with compact_path.open() as compact_file:
            compact = json.load(compact_file)
        if (
            compact.get("schema") != "SIMULATION_ONLY_CLIMB_COMPACT_V2"
            or compact.get("simulation_only") is not True
            or compact.get("simulation_candidate_only") is not True
            or compact.get("stage_count") != len(compact.get("stages", ()))
        ):
            raise ValueError("invalid compact climb scene config")
        ClimbMode(None)._validate_config(compact)
        print("Compact climb config selected: {}".format(compact_path))
        climb_start_index, climb_end_index = resolve_compact_stage_range(
            compact, args.climb_from, args.climb_to
        )
        for stage in compact["stages"]:
            stage["segment_durations_s"] = [
                duration / args.climb_speed
                for duration in stage["segment_durations_s"]
            ]
            stage["settle_s"] /= args.climb_speed
        selected_stages = compact["stages"][climb_start_index:climb_end_index + 1]
        playback_duration = sum(
            sum(stage["segment_durations_s"]) + stage["settle_s"]
            for stage in selected_stages
        )
        print(
            "Compact climb scene loaded (C{}:{} -> C{}:{}, {:.2g}x, {:.1f}s)".format(
                climb_start_index + 1, compact["stages"][climb_start_index]["name"],
                climb_end_index + 1, compact["stages"][climb_end_index]["name"],
                args.climb_speed, playback_duration
            )
        )

    # 仅显式攀爬参数会改变普通仿真场景。
    if compact is not None:
        xiaolan_position = gymapi.Vec3(*compact["xiaolan_translation"])
    else:
        xiaolan_position = gymapi.Vec3(0.0, 0.8, 0.0)
    # 直接使用原始三角面而不是凸包近似。
    # PhysX只允许静态物体使用这种凹三角网格，正适合当前固定的对接目标。
    add_static_stl_triangle_mesh(
        gym,
        sim,
        description_root
        / "meshes"
        / "xiaolan"
        / "base_link_xiaolan.STL",
        xiaolan_position,
    )

    lower = gymapi.Vec3(-1.0, -1.0, 0.0)
    upper = gymapi.Vec3(1.0, 1.0, 1.0)
    num_per_row = 1

    # ROS仿真也在Isaac控制帧内同步运行控制器；ROS只缓存Joy/导航输入。
    ros_controller = None
    ros_telemetry = None
    if args.ros:
        from run_real import RosControlNode

        ros_controller = RosControlNode(
            local_execution=True,
            controller_rate_hz=args.control_rate,
        )
        ros_telemetry = RosSimTelemetry()
        controller = ros_controller.controller
    else:
        controller = GraspController(dt=1.0 / args.control_rate)

    print(
        f"Physics: {args.physics_rate:.0f} Hz, controller: "
        f"{args.control_rate:.0f} Hz, actuator: "
        f"{args.actuator_rate:.0f} Hz"
    )
    if args.ros and args.climb_scene:
        print(
            "ROS Isaac compact alignment: physics q/control sampling {:.0f}Hz, "
            "/pos telemetry {:.0f}Hz, target write {:.0f}Hz, physics {:.0f}Hz, "
            "~{:.3g}ms target hold, same controller and feedback-paced gates. "
            "Not simulated: serial per-servo I/O/retry, inter-board skew, "
            "power/load/backlash/torque, or real contact proof.".format(
                args.control_rate,
                args.actuator_rate,
                args.actuator_rate,
                args.physics_rate,
                1000.0 / args.actuator_rate,
            )
        )
    #创建环境和actor
    env = gym.create_env(sim, lower, upper, num_per_row)

    mission_seed = None
    mission_start_pose = None
    if compact is not None and not args.full_mission:
        q_init_control, compact_entry_pose = prepare_compact_stage_entry(
            compact, climb_start_index, climb_end_index, 1.0 / args.control_rate
        )
    elif args.full_mission:
        mission_seed, mission_start_pose = _random_mission_start(
            compact,
            controller.base_height_at_stand,
            args.mission_seed,
        )

    pose = gymapi.Transform()
    if args.full_mission:
        pose.p = gymapi.Vec3(*mission_start_pose[:3])
        half_yaw = 0.5 * mission_start_pose[3]
        pose.r = gymapi.Quat(
            0.0,
            0.0,
            float(np.sin(half_yaw)),
            float(np.cos(half_yaw)),
        )
    elif compact is not None:
        pose.p = gymapi.Vec3(*compact_entry_pose[:3])
        pose.r = _compact_root_quaternion(compact_entry_pose)
    else:
        pose.p = gymapi.Vec3(
            0.0,
            0.0,
            # 直接按标准站姿的足端球半径落在地面上，不再额外悬空25 mm。
            float(controller.base_height_at_stand),
        )

    actor = gym.create_actor(env, robot_asset, pose, "grasp_hexapod", 0, 1)

    rigid_body_names, dof_names = print_model_info(gym, env, actor)
    dof_indices = build_dof_indices(dof_names)
    print(f"Control DOF mapping ready: {len(dof_indices)} joints")
    if "base_link" not in rigid_body_names:
        raise ValueError("Isaac actor has no base_link rigid body for diagnostics")
    base_body_index = rigid_body_names.index("base_link")

    base_dof_properties = gym.get_actor_dof_properties(env, actor)
    climb_drive_active = compact is not None and not args.full_mission
    joint_speed = (
        args.climb_joint_speed if climb_drive_active else args.joint_speed
    )
    dof_properties = _drive_properties(
        base_dof_properties,
        joint_speed,
    )
    if climb_drive_active:
        print(
            "Compact climb joint velocity limit: {:.2g}x "
            "(same Isaac drive baseline as Approach: stiffness=100, "
            "damping=0.8, URDF effort unchanged; 1.2x is the LX-15D "
            "official no-load "
            "upper-bound candidate, >1.2x is diagnostics-only and not "
            "hardware-aligned; Isaac gains are not calibrated servo gains)".format(
                joint_speed
            )
        )
    else:
        print(
            "Normal joint velocity limit: {:.2g}x (about {:.2f} rad/s; "
            "stiffness, damping and effort unchanged)".format(
                joint_speed,
                float(np.min(dof_properties["velocity"])),
            )
        )

    gym.set_actor_dof_properties(
        env,
        actor,
        dof_properties,
    )
    #做一系列控制器和isaac的顺序转换
    lower_control = external_to_control(
        dof_properties["lower"],
        dof_indices,
    )
    upper_control = external_to_control(
        dof_properties["upper"],
        dof_indices,
    )
    if compact is not None and not args.full_mission:
        print(
            "Compact climb: initialized at C{}:{} entry snapshot".format(
                climb_start_index + 1, compact["stages"][climb_start_index]["name"]
            )
        )
    else:
        q_init_control = controller.q_init
    q_init_isaac = control_to_external(
        q_init_control,
        dof_indices
    )
    dof_states = gym.get_actor_dof_states(env, actor, gymapi.STATE_ALL)
    dof_states["pos"][:] = q_init_isaac
    dof_states["vel"][:] = 0.0

    gym.set_actor_dof_states(
        env,
        actor,
        dof_states,
        gymapi.STATE_ALL,
    )
    # 位置驱动目标必须与Q_STAND同步，否则仿真开始第一帧会发生关节跳变。
    gym.set_actor_dof_position_targets(
        env,
        actor,
        q_init_isaac,
    )
    if ros_controller is not None and args.climb_scene:
        ros_controller.arm_local_climb(q_init_control)

    # 所有资产与场景加载完成后再创建 viewer, 避免加载期窗口黑屏无响应。
    viewer = None
    if not args.headless:
        viewer = gym.create_viewer(sim, gymapi.CameraProperties())
        if viewer is None:
            raise RuntimeError("Failed to create Isaac Gym viewer.")

    # 镜头中心放在两个机器人之间，初始画面可以同时观察六足和小蓝。
    if viewer is not None:
        # 攀爬模式相机对准机器人与小蓝之间, 默认视角常看不到机器人
        if args.full_mission:
            midpoint = 0.5 * (
                mission_start_pose[:2]
                + np.asarray(compact["xiaolan_translation"][:2])
            )
            cam_eye = gymapi.Vec3(
                float(midpoint[0] + 0.65),
                float(midpoint[1] - 1.05),
                0.65,
            )
            cam_target = gymapi.Vec3(
                float(midpoint[0]),
                float(midpoint[1]),
                0.10,
            )
        elif compact is not None:
            cam_eye = gymapi.Vec3(0.45, -1.2, 0.5)
            cam_target = gymapi.Vec3(0.25, 0.0, 0.12)
        else:
            cam_eye = gymapi.Vec3(0.75, -0.75, 0.5)
            cam_target = gymapi.Vec3(0.0, 0.38, 0.10)
        gym.viewer_camera_look_at(viewer, None, cam_eye, cam_target)

    # 直接仿真、轨迹录制和ROS实机默认使用同一速度上限。
    max_linear_speed = args.max_linear_speed
    max_vertical_speed = args.max_vertical_speed
    # 令标准足端的旋转切向速度等于平移速度上限。
    nominal_foot_radius = np.mean(
        np.linalg.norm(controller.foot_init_base[:, :2], axis=1)
    )
    max_yaw_rate = max_linear_speed / nominal_foot_radius

    sim_navigation = None
    navigation_state = None
    mission_target_pose = None
    if args.full_mission:
        sim_navigation = SimulatedRtkImu(
            compact["xiaolan_translation"]
        )
        mission_target_pose = _planar_transform(
            compact["p0"]["base"][0],
            compact["p0"]["base"][1],
            compact["p0"]["base"][2],
            0.0,
        )
        xiaolan_from_target = (
            np.linalg.inv(sim_navigation.pv_from_xiaolan)
            @ mission_target_pose
        )
        # full-mission直接从当前compact P0导出同一固定左侧相对基准。
        controller.approach_mode.configure_fixed_approach(
            xiaolan_from_target,
            target_side="left",
            linear_speed=min(0.12, max_linear_speed),
            yaw_rate=max_yaw_rate,
        )
        initial_pose = _planar_transform(*mission_start_pose)
        navigation_state = sim_navigation.snapshot(initial_pose, 0.0)
        relative = sim_navigation.xiaolan_from_base(initial_pose)
        result = controller.start_full_mission(
            navigation_state,
            compact,
            start_stage_index=climb_start_index,
            end_stage_index=climb_end_index,
            approach_timeout_s=args.approach_timeout,
            approach_position_tolerance_m=0.002,
            approach_yaw_tolerance_rad=np.deg2rad(0.2),
            climb_hardware_execution=True,
        )
        if result.failed:
            raise RuntimeError(
                "full mission approach planning failed: " + result.reason
            )
        relative_distance = np.linalg.norm(relative[:2, 3])
        initial_yaw = np.arctan2(initial_pose[1, 0], initial_pose[0, 0])
        print(
            "[mission] seed={} random base=({:.3f}, {:.3f}, yaw={:.1f}deg) "
            "distance_to_xiaolan={:.3f}m".format(
                mission_seed,
                mission_start_pose[0],
                mission_start_pose[1],
                np.degrees(initial_yaw),
                relative_distance,
            )
        )
        print(
            "[sim-rtk/imu] base_in_xiaolan=({:.3f}, {:.3f}, {:.3f})m; "
            "ideal simulation observation, not a contact/load signal".format(
                relative[0, 3],
                relative[1, 3],
                relative[2, 3],
            )
        )
        print("[mission] APPROACH started; target is compact C1 entry")

    trace_script = None
    trace_rows = []
    script_frame = 0
    if args.record_servo_trace is not None:
        trace_script = build_servo_trace_script(
            controller,
            max_linear_speed,
            max_yaw_rate,
        )
        print(
            f"Recording {args.control_rate:.0f} Hz servo trace: "
            "initialize -> forward -> right -> rotate"
        )
    else:
        if (
            ros_controller is None
            and not args.climb_start
            and not args.full_mission
        ):
            joystick = JoyStick()

    command = np.zeros(
        4,
        dtype=np.float64,
    )
    motion_state = "WAIT_B"
    if args.climb_start or args.full_mission:
        motion_state = "RUNNING"
    elif trace_script is not None:
        motion_state = "HOLD"
    if args.climb_start:
        controller.enter_climb(
            q_init_control,
            compact,
            climb_start_index,
            climb_end_index,
            hardware_execution=True,
        )
        print("Compact climb started (--climb-start, real feedback gates)")
    button_a_was_down = False
    button_b_was_down = False
    button_x_was_down = False
    climb_report_stage = 0
    climb_report_t0 = 0.0
    climb_report_q_ref = None
    climb_phase_reported = None
    climb_entered = bool(args.climb_start)
    climb_metrics_by_stage = {}
    climb_metrics_written = False
    mission_state_reported = (
        controller.mission.state if args.full_mission else None
    )
    approach_state_reported = (
        controller.approach_mode.approach_plan.state
        if args.full_mission
        else None
    )
    mission_terminal_stage = 0
    mission_terminal_t0 = 0.0
    mission_terminal_q_ref = None
    q_des_control = q_init_control.copy()
    if (
        trace_script is None
        and ros_controller is None
        and not args.full_mission
    ):
        print("A: enable/pause motion | B: reset to stand | X: compact climb")

    physics_frame = 0
    t_render_prev = time.time()
    control_frame = 0

    # PhysX固定高频运行，控制器只在自己的更新帧读取状态并生成新目标。
    while (
        (
            viewer is None
            or not gym.query_viewer_has_closed(viewer)
        )
        and (
            ros_controller is None
            or not rospy.is_shutdown()
        )
    ):
        if trace_script is not None and script_frame >= len(trace_script):
            break

        control_tick = physics_frame % control_interval == 0
        if control_tick:
            actuator_tick = control_frame % actuator_interval == 0
            dof_states = gym.get_actor_dof_states(
                env,
                actor,
                gymapi.STATE_POS,
            )
            # Isaac一维顺序 → 控制器(6,3)顺序
            q_control = external_to_control(
                dof_states["pos"],
                dof_indices,
            )
            root_transform = None
            if args.full_mission:
                root_states = gym.get_actor_rigid_body_states(
                    env,
                    actor,
                    gymapi.STATE_POS,
                )
                root_transform = gymapi.Transform.from_buffer(
                    root_states["pose"][base_body_index]
                )
                navigation_state = sim_navigation.snapshot(
                    _isaac_transform_matrix(root_transform),
                    physics_frame * sim_params.dt,
                )

            if ros_controller is not None:
                if actuator_tick:
                    ros_telemetry.publish_feedback(q_control)
                synchronous_target = ros_controller.update_from_feedback(
                    q_control
                )
                if synchronous_target is not None:
                    q_des_control = synchronous_target
                    ros_telemetry.publish_target(q_des_control)
                if (
                    args.climb_scene
                    and not climb_entered
                    and controller.climb_mode.state == ClimbMode.RUNNING
                ):
                    climb_entered = True
                    print("ROS Isaac compact climb started by X (hardware gates)")
            elif trace_script is not None:
                stage, scripted_command = trace_script[script_frame]
                command[:] = scripted_command
            elif args.climb_start or args.full_mission:
                # 自动攀爬/全流程由控制器状态机自驱，无手柄输入。
                command[:] = 0.0
            else:
                (
                    axis_right,
                    axis_forward,
                    axis_yaw,
                ) = joystick.get_commands()

                # 北通BTP-KP20的axis 4/5是两个扳机，静止值均为-1。
                left_trigger = 0.5 * (
                    joystick.joystick.get_axis(4) + 1.0
                )
                right_trigger = 0.5 * (
                    joystick.joystick.get_axis(5) + 1.0
                )
                axis_body = right_trigger - left_trigger
                if abs(axis_body) < 0.05:
                    axis_body = 0.0

                button_a_down = bool(joystick.joystick.get_button(0))
                button_b_down = bool(joystick.joystick.get_button(1))
                button_x_down = bool(joystick.joystick.get_button(2))
                button_a_pressed = (
                    button_a_down and not button_a_was_down
                )
                button_b_pressed = (
                    button_b_down and not button_b_was_down
                )
                button_x_pressed = (
                    button_x_down and not button_x_was_down
                )
                button_a_was_down = button_a_down
                button_b_was_down = button_b_down
                button_x_was_down = button_x_down

                if button_b_pressed:
                    if controller.mode == controller.CLIMB:
                        controller.hold_climb()
                        motion_state = "HOLD"
                        print("Compact climb: HOLD")
                    else:
                        motion_state = "RESETTING"
                        controller.reset_to_stand(q_control)
                        print("Controller returning to stand")
                elif button_x_pressed:
                    if compact is not None and not climb_entered:
                        climb_entered = True
                        controller.enter_climb(
                            q_control,
                            compact,
                            climb_start_index,
                            climb_end_index,
                            hardware_execution=True,
                        )
                        motion_state = "RUNNING"
                        print("Compact climb started (X, real feedback gates)")
                    elif compact is None:
                        print("X rejected: start with --climb-scene")
                    else:
                        print("Compact climb already active")
                elif button_a_pressed:
                    if (
                        controller.mode == controller.CLIMB
                        and (
                            controller.climb_mode.state
                            == controller.climb_mode.HOLD
                        )
                    ):
                        controller.resume_climb()
                        motion_state = "RUNNING"
                        print("Compact climb: RESUME")
                    elif controller.mode == controller.CLIMB:
                        print("A ignored: compact climb is already running")
                    elif motion_state == "HOLD":
                        motion_state = "RUNNING"
                        print("Motion control: ENABLED")
                    elif motion_state == "RUNNING":
                        motion_state = "HOLD"
                        print("Motion control: PAUSED")
                    else:
                        print("A ignored: press B and wait for stand first")

                if motion_state == "RUNNING":
                    translation_axes = np.array(
                        [axis_right, axis_forward],
                        dtype=np.float64,
                    )
                    translation_norm = np.linalg.norm(translation_axes)
                    if translation_norm > 1.0:
                        translation_axes /= translation_norm

                    command[:] = [
                        max_linear_speed * translation_axes[0],
                        max_linear_speed * translation_axes[1],
                        max_vertical_speed * axis_body,
                        max_yaw_rate * axis_yaw,
                    ]
                else:
                    # 暂停时让当前摆动腿先落地再停止。
                    command[:] = 0.0

            metric_stage_index = None
            metric_stage_name = None
            if controller.mode == controller.CLIMB:
                # update() 内可能切换阶段；本 tick 的 diagnostics
                # 用切换前的实际根位姿和足端参考，仍属于切换前阶段。
                metric_stage_index = controller.climb_mode.stage_index
                metric_stage_name = controller.climb_mode.phase

            if metric_stage_index is not None:
                if root_transform is None:
                    root_states = gym.get_actor_rigid_body_states(
                        env, actor, gymapi.STATE_POS
                    )
                    root_transform = gymapi.Transform.from_buffer(
                        root_states["pose"][base_body_index]
                    )
                root_position = np.array(
                    (root_transform.p.x, root_transform.p.y, root_transform.p.z),
                    dtype=np.float64,
                )
                root_quaternion_xyzw = np.array(
                    (
                        root_transform.r.x,
                        root_transform.r.y,
                        root_transform.r.z,
                        root_transform.r.w,
                    ),
                    dtype=np.float64,
                )
                metric = climb_metrics_by_stage.setdefault(
                    metric_stage_index,
                    _new_climb_metric(metric_stage_index, metric_stage_name),
                )
                _update_climb_metric(
                    metric,
                    controller,
                    q_control,
                    root_position,
                    root_quaternion_xyzw,
                    1.0 / args.control_rate,
                )

            if (
                ros_controller is None
                and compact is not None
                and not climb_entered
                and not args.full_mission
            ):
                # --climb-scene 的入口已是 selected snapshot；等待 X 时不能让
                # APPROACH 的足端参考把它重新拉向 Q_STAND。
                q_des_control = q_init_control.copy()
            elif ros_controller is None:
                # 直接模式仍由本文件读取手柄、规划足端并执行DLS。
                q_des_control = controller.update(
                    q_control,
                    command,
                    navigation_state if args.full_mission else None,
                )
                if (
                    trace_script is None
                    and motion_state == "RESETTING"
                    and not controller.reset_active
                ):
                    motion_state = "HOLD"
                    print("Stand initialization complete; press A to move")

            if args.full_mission:
                approach_state = (
                    controller.approach_mode.approach_plan.state
                )
                if (
                    controller.mission.state == controller.mission.APPROACH
                    and approach_state != approach_state_reported
                ):
                    approach_state_reported = approach_state
                    print("[mission] APPROACH state=" + approach_state)

                mission_state = controller.mission.state
                if mission_state != mission_state_reported:
                    mission_state_reported = mission_state
                    if mission_state == controller.mission.CLIMB:
                        climb_entered = True
                        current_pose = navigation_state.pv_from_base
                        position_error = np.linalg.norm(
                            current_pose[:2, 3]
                            - mission_target_pose[:2, 3]
                        )
                        current_yaw = np.arctan2(
                            current_pose[1, 0],
                            current_pose[0, 0],
                        )
                        print(
                            "[mission] CLIMB entered after stopped-pose "
                            "recheck: position_error={:.3f}m "
                            "yaw_error={:.2f}deg joint_error={:.4f}rad "
                            "world_foot_error={:.4f}m speed={:.4f}m/s "
                            "angular_speed={:.2f}deg/s".format(
                                position_error,
                                abs(np.degrees(current_yaw)),
                                controller.mission.last_prepare_joint_error_rad,
                                controller.mission.last_prepare_world_foot_error_m,
                                controller.mission.last_prepare_linear_speed_m_s,
                                np.degrees(
                                    controller.mission.last_prepare_angular_speed_rad_s
                                ),
                            )
                        )
                        print(
                            "Compact climb joint velocity limit: {:.2g}x "
                            "(same Isaac drive baseline as Approach; "
                            "simulation-only)".format(args.climb_joint_speed)
                        )
                    elif (
                        mission_state
                        == controller.mission.PREPARE_CLIMB
                    ):
                        climb_properties = _drive_properties(
                            base_dof_properties,
                            args.climb_joint_speed,
                        )
                        gym.set_actor_dof_properties(
                            env,
                            actor,
                            climb_properties,
                        )
                        print(
                            "[mission] PREPARE_CLIMB entered: smooth return "
                            "to Q_STAND with the shared Approach drive "
                            "baseline, then persistent "
                            "joint-feedback gate"
                        )
                    elif mission_state == controller.mission.APPROACH:
                        approach_state_reported = (
                            controller.approach_mode.approach_plan.state
                        )
                        print(
                            "[mission] APPROACH correction restarted after "
                            "posture-preparation recheck: position_error="
                            "{:.3f}m yaw_error={:.2f}deg world_foot_error="
                            "{:.4f}m".format(
                                controller.mission.last_prepare_position_error_m,
                                np.degrees(
                                    controller.mission.last_prepare_yaw_error_rad
                                ),
                                controller.mission.last_prepare_world_foot_error_m,
                            )
                        )
                    elif mission_state == controller.mission.DOCK:
                        print(
                            "[mission] DOCK entered after terminal gate: "
                            "position_error={:.4f}m orientation_error="
                            "{:.2f}deg world_foot_error={:.4f}m; holding "
                            "for future DockMode".format(
                                controller.mission.final_position_error_m,
                                np.degrees(
                                    controller.mission.final_orientation_error_rad
                                ),
                                controller.mission.final_world_foot_error_m,
                            )
                        )
                    elif mission_state == controller.mission.FAILED:
                        print(
                            "[mission] FAILED: "
                            + (controller.mission.reason or "unknown")
                        )

            if (
                climb_entered
                and controller.climb_mode.stage_index is not None
            ):
                phase_key = (
                    controller.climb_mode.state,
                    controller.climb_mode.phase,
                    controller.climb_mode.stage_index,
                )
                if phase_key != climb_phase_reported:
                    if (
                        climb_phase_reported is not None
                        and metric_stage_index is not None
                        and (
                            controller.climb_mode.stage_index != metric_stage_index
                            or controller.climb_mode.state in (
                                controller.climb_mode.DONE,
                                controller.climb_mode.FAILED,
                            )
                        )
                    ):
                        finished = climb_metrics_by_stage[metric_stage_index]
                        worst_joint = "-"
                        if finished["worst_joint_time_s"] is not None:
                            worst_joint = "{}/{}@{:.2f}s".format(
                                finished["worst_joint_leg"],
                                finished["worst_joint_name"],
                                finished["worst_joint_time_s"],
                            )
                        worst_foot = "-"
                        if finished["worst_foot_time_s"] is not None:
                            worst_foot = "{}@{:.2f}s".format(
                                finished["worst_foot_leg"],
                                finished["worst_foot_time_s"],
                            )
                        worst_root = "-"
                        if finished["worst_root_time_s"] is not None:
                            worst_root = "@{:.2f}s".format(
                                finished["worst_root_time_s"]
                            )
                        worst_world_foot = "-"
                        if finished["worst_world_foot_anchor_time_s"] is not None:
                            worst_world_foot = "{}@{:.2f}s".format(
                                finished["worst_world_foot_anchor_leg"],
                                finished["worst_world_foot_anchor_time_s"],
                            )
                        print(
                            "[climb] C{} {} metrics: t={:.2f}s track={:.4g}rad "
                            "foot={:.4g}m limit_margin={:.4g}rad "
                            "worst_joint={} worst_foot={} "
                            "root={:.4g}m{} world_foot={:.4g}m{} "
                            "support_drift={:.4g}m "
                            "end_track={:.4g}rad end_foot={:.4g}m "
                            "end_root={:.4g}m end_world_foot={:.4g}m".format(
                                metric_stage_index + 1, metric_stage_name,
                                finished["simulated_duration_s"],
                                finished["max_joint_target_tracking_error_rad"],
                                finished["max_kinematic_foot_target_error_m"],
                                finished["min_joint_limit_margin_rad"],
                                worst_joint,
                                worst_foot,
                                finished["max_root_position_error_m"],
                                worst_root,
                                finished["max_world_foot_anchor_error_m"],
                                worst_world_foot,
                                finished["max_support_foot_world_drift_m"],
                                finished["end_joint_target_tracking_error_rad"],
                                finished["end_foot_target_error_m"],
                                finished["end_root_position_error_m"],
                                finished["end_world_foot_anchor_error_m"],
                            )
                        )
                    climb_phase_reported = phase_key
                    stage_number = (
                        controller.climb_mode.stage_index + 1
                        if controller.climb_mode.stage_index is not None
                        else 0
                    )
                    print(
                        "[climb] state={} stage={}/{} phase={} "
                        "prior_tracking_error={:.6g}rad "
                        "prior_foot_target_error={:.6g}m".format(
                            controller.climb_mode.state,
                            stage_number,
                            len(controller.climb_mode.stage_names),
                            controller.climb_mode.phase,
                            controller.climb_mode.last_tracking_error_rad,
                            controller.climb_mode.last_foot_target_error_m,
                        )
                    )

            if (
                args.climb_metrics is not None
                and climb_entered
                and not climb_metrics_written
                and controller.climb_mode.state in (
                    controller.climb_mode.DONE,
                    controller.climb_mode.FAILED,
                )
            ):
                _write_climb_metrics(
                    args.climb_metrics,
                    compact,
                    climb_start_index,
                    climb_end_index,
                    args.climb_speed,
                    args.climb_joint_speed,
                    controller.climb_mode,
                    climb_metrics_by_stage,
                    controller.mission if args.full_mission else None,
                )
                climb_metrics_written = True

            if args.full_mission and controller.mission.state in (
                controller.mission.DOCK,
                controller.mission.FAILED,
            ):
                if mission_terminal_stage == 0:
                    mission_terminal_stage = 1
                    mission_terminal_t0 = physics_frame * sim_params.dt
                    mission_terminal_q_ref = q_control.copy()
                    print(
                        "[mission] terminal state={} at t={:.1f}s; "
                        "holding for 2s".format(
                            controller.mission.state,
                            mission_terminal_t0,
                        )
                    )
                elif (
                    mission_terminal_stage == 1
                    and physics_frame * sim_params.dt
                    - mission_terminal_t0 >= 2.0
                ):
                    mission_terminal_stage = 2
                    q_drift = (
                        np.abs(q_control - mission_terminal_q_ref).max()
                        * 1000.0
                    )
                    print(
                        "[mission] terminal hold joint drift={:.1f}mrad; "
                        "result={}".format(
                            q_drift,
                            (
                                "DOCK_HANDOFF_REACHED"
                                if controller.mission.state
                                == controller.mission.DOCK
                                else "FAILED"
                            ),
                        )
                    )
                    break

            # headless 验证: 攀爬完成后保持 2 秒, 检查姿态维持稳定性。
            if args.climb_start and controller.climb_mode.state in (
                    controller.climb_mode.DONE,
                    controller.climb_mode.FAILED):
                if climb_report_stage == 0:
                    climb_report_stage = 1
                    climb_report_t0 = physics_frame * sim_params.dt
                    climb_report_q_ref = q_control.copy()
                    failure_reason = (
                        controller.climb_mode.failure_reason or "none"
                    )
                    print(
                        f"[climb] {controller.climb_mode.state} at t="
                        f"{physics_frame * sim_params.dt:.1f}s, "
                        "tracking_error="
                        f"{controller.climb_mode.last_tracking_error_rad:.6g}rad, "
                        "foot_target_error="
                        f"{controller.climb_mode.last_foot_target_error_m:.6g}m, "
                        f"reason={failure_reason}, "
                        "holding final pose for 2s"
                    )
                elif climb_report_stage == 1 and \
                        physics_frame * sim_params.dt - climb_report_t0 >= 2.0:
                    climb_report_stage = 2
                    q_drift = np.abs(q_control - climb_report_q_ref).max() * 1000
                    track_err = np.abs(q_des_control - q_control).max() * 1000
                    final_metric = climb_metrics_by_stage.get(climb_end_index)
                    selected_rows = [
                        climb_metrics_by_stage[index]
                        for index in range(climb_start_index, climb_end_index + 1)
                        if index in climb_metrics_by_stage
                    ]
                    end_foot = (None if final_metric is None else
                                final_metric["end_foot_target_error_m"])
                    end_root = (None if final_metric is None else
                                final_metric["end_root_position_error_m"])
                    root_values = [
                        row["max_root_position_error_m"]
                        for row in selected_rows
                        if row["max_root_position_error_m"] is not None
                    ]
                    margin_values = [
                        row["min_joint_limit_margin_rad"]
                        for row in selected_rows
                        if row["min_joint_limit_margin_rad"] is not None
                    ]
                    global_root = max(root_values) if root_values else None
                    global_min_margin = min(margin_values) if margin_values else None
                    checks = (
                        ("hold_q_drift_lt_20mrad", q_drift, 20.0,
                         q_drift < 20.0),
                        ("end_foot_target_error_le_0.015m", end_foot, 0.015,
                         end_foot is not None and end_foot <= 0.015),
                        ("end_root_position_error_le_0.05m", end_root, 0.05,
                         end_root is not None and end_root <= 0.05),
                        ("global_max_root_position_error_le_0.20m", global_root,
                         0.20, global_root is not None and global_root <= 0.20),
                        ("global_min_joint_limit_margin_ge_0rad",
                         global_min_margin, 0.0,
                         global_min_margin is not None and global_min_margin >= 0.0),
                    )
                    print(
                        f"[climb] hold check: joint drift over 2s "
                        f"{q_drift:.1f}mrad, "
                        f"joint tracking error {track_err:.1f}mrad"
                    )
                    for label, value, threshold, passed in checks:
                        shown = "missing" if value is None else "{:.6g}".format(value)
                        print(
                            "[climb] preview gate {}: value={} threshold={} {}".format(
                                label, shown, threshold,
                                "PASS" if passed else "FAIL",
                            )
                        )
                    print(
                        "[climb] PREVIEW VERDICT: " +
                        ("SUCCESS" if all(check[-1] for check in checks) else "FAILED")
                        + " (simulation preview gates only; not contact, load, "
                        "stability, or hardware proof)"
                    )
                    break

            q_target_control = np.clip(
                q_des_control,
                lower_control,
                upper_control,
            )

            # 只有机械限位实际改变目标时才重新检查改变后的姿态。
            if (
                not np.array_equal(q_target_control, q_des_control)
            ):
                q_target_control = controller.collision_guard(
                    q_target_control,
                    q_control,
                )

            q_target_isaac = control_to_external(
                q_target_control,
                dof_indices,
            )
            if actuator_tick:
                gym.set_actor_dof_position_targets(
                    env,
                    actor,
                    q_target_isaac,
                )

            if trace_script is not None:
                trace_rows.append(
                    [
                        script_frame,
                        script_frame * controller.dt,
                        stage,
                        *command,
                        *q_target_control.reshape(18),
                        *q_control.reshape(18),
                    ]
                )
                script_frame += 1
            control_frame += 1

        t_render = time.time()
        if physics_frame % 240 == 0:
            print(f"[render] frame={physics_frame} fps={240/(time.time()-t_render_prev+1e-9):.1f}",
                  flush=True)
            t_render_prev = time.time()
        gym.simulate(sim)
        gym.fetch_results(sim, True)
        physics_frame += 1

        if viewer is not None:
            if physics_frame % render_interval == 0:
                gym.step_graphics(sim)
                gym.draw_viewer(viewer, sim, True)
            gym.sync_frame_time(sim)
        elif ros_controller is not None:
            # ROS输入仿真按真实时间运行，避免无Viewer时无限加速。
            gym.sync_frame_time(sim)

    if args.record_servo_trace is not None:
        write_servo_trace(args.record_servo_trace, trace_rows)
    if (
        args.climb_metrics is not None
        and climb_entered
        and not climb_metrics_written
    ):
        _write_climb_metrics(
            args.climb_metrics,
            compact,
            climb_start_index,
            climb_end_index,
            args.climb_speed,
            args.climb_joint_speed,
            controller.climb_mode,
            climb_metrics_by_stage,
            controller.mission if args.full_mission else None,
        )
    if viewer is not None:
        gym.destroy_viewer(viewer)
    gym.destroy_sim(sim)


if __name__ == "__main__":
    main()
