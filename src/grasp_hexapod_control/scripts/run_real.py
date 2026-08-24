#!/usr/bin/env python3
"""抓取六足ROS高层控制入口。

功能：
    订阅手柄和18个关节反馈，执行B回正、A启停、导航/手柄仲裁和
    GraspController，再发布6×3关节目标。实机和ROS仿真共用本节点。
输入：
    /joy；六个/<leg>_pos。
输出：
    六个/<leg>_des，格式与grasp_hexapod_servo约定一致。
边界：
    本文件只处理ROS、整机启停和控制循环；步态与运动学仍由
    GraspController负责，舵机协议仍由grasp_hexapod_servo负责。
"""

import json
from pathlib import Path
import sys
from threading import Lock

import numpy as np
import rospy
from geometry_msgs.msg import PolygonStamped, PoseStamped
from sensor_msgs.msg import Imu, JointState, Joy
from std_msgs.msg import Bool, Float64MultiArray

# 源码直启时使用当前目录；rosrun/roslaunch时从ROS包路径找到scripts。
scripts_dir = Path(__file__).resolve().parent
if not (scripts_dir / "control.py").exists():
    import rospkg

    scripts_dir = (
        Path(rospkg.RosPack().get_path("grasp_hexapod_control"))
        / "scripts"
    )
sys.path.insert(0, str(scripts_dir))

from climb_mode import ClimbMode
from control import GraspController
from kinematics import LEG_NAMES
from utils import NavigationState, package_config_path, pose_to_transform


def load_fixed_approach_config():
    """加载固定左侧P0接近基准；其仅是仿真基线。"""

    with package_config_path("approach_fixed.json").open() as file:
        config = json.load(file)
    if config.get("target_side") != "left":
        raise ValueError("approach_fixed.json target_side must be left")
    if config.get("simulation_baseline_only") is not True:
        raise ValueError("approach_fixed.json must remain simulation-baseline-only")
    matrix = np.asarray(
        config.get("xiaolan_from_base"),
        dtype=np.float64,
    )
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("approach_fixed.json xiaolan_from_base must be 4x4")
    return config["target_side"], matrix


class NavigationInput:
    """把ROS导航话题缓存为控制器每帧读取的NavigationState。"""

    FRAME = "pv_map"

    def __init__(self):
        self.max_age = float(rospy.get_param("~max_pose_age", 0.5))
        self.lock = Lock()
        self.base_stamp = 0.0
        self.xiaolan_stamp = 0.0
        self.pv_from_base = None
        self.pv_from_xiaolan = None
        self.pv_boundary = np.empty((0, 2), dtype=np.float64)
        self.landing_confirmed = False

        self.subscribers = [
            rospy.Subscriber(
                rospy.get_param(
                    "~base_pose_topic",
                    "/grasp_hexapod/navigation/base_pose",
                ),
                PoseStamped,
                self._base_callback,
                queue_size=1,
            ),
            rospy.Subscriber(
                rospy.get_param(
                    "~xiaolan_pose_topic",
                    "/grasp_hexapod/navigation/xiaolan_pose",
                ),
                PoseStamped,
                self._xiaolan_callback,
                queue_size=1,
            ),
            rospy.Subscriber(
                rospy.get_param(
                    "~pv_boundary_topic",
                    "/grasp_hexapod/navigation/pv_boundary",
                ),
                PolygonStamped,
                self._boundary_callback,
                queue_size=1,
            ),
            rospy.Subscriber(
                rospy.get_param(
                    "~landing_topic",
                    "/grasp_hexapod/landing_confirmed",
                ),
                Bool,
                self._landing_callback,
                queue_size=1,
            ),
        ]

    @classmethod
    def _valid_frame(cls, message):
        if message.header.frame_id.lstrip("/") == cls.FRAME:
            return True
        rospy.logwarn_throttle(2.0, "Navigation frame must be pv_map")
        return False

    def _base_callback(self, message):
        if not self._valid_frame(message):
            return
        transform = pose_to_transform(message.pose)
        if transform is not None:
            with self.lock:
                self.pv_from_base = transform
                self.base_stamp = message.header.stamp.to_sec()

    def _xiaolan_callback(self, message):
        if not self._valid_frame(message):
            return
        transform = pose_to_transform(message.pose)
        if transform is not None:
            with self.lock:
                self.pv_from_xiaolan = transform
                self.xiaolan_stamp = message.header.stamp.to_sec()

    def _boundary_callback(self, message):
        if not self._valid_frame(message):
            return
        boundary = np.array(
            [[point.x, point.y] for point in message.polygon.points],
            dtype=np.float64,
        ).reshape(-1, 2)
        if np.isfinite(boundary).all():
            with self.lock:
                self.pv_boundary = boundary

    def _landing_callback(self, message):
        with self.lock:
            self.landing_confirmed = bool(message.data)

    def snapshot(self):
        """返回同一时刻的导航快照；位姿过期时valid=False。"""

        now = rospy.Time.now().to_sec()
        with self.lock:
            stamp = min(self.base_stamp, self.xiaolan_stamp)
            valid = (
                self.pv_from_base is not None
                and self.pv_from_xiaolan is not None
                and len(self.pv_boundary) >= 3
                and stamp > 0.0
                and 0.0 <= now - stamp <= self.max_age
            )
            return NavigationState(
                stamp=stamp,
                valid=valid,
                landing_confirmed=self.landing_confirmed,
                pv_from_base=(
                    np.eye(4)
                    if self.pv_from_base is None
                    else self.pv_from_base.copy()
                ),
                pv_from_xiaolan=(
                    np.eye(4)
                    if self.pv_from_xiaolan is None
                    else self.pv_from_xiaolan.copy()
                ),
                pv_boundary=self.pv_boundary.copy(),
            )

    def motion_snapshot(self):
        """返回实机攀爬用的RTK/LoRa相对位姿快照。"""

        now = rospy.Time.now().to_sec()
        with self.lock:
            stamp = min(self.base_stamp, self.xiaolan_stamp)
            valid = (
                self.pv_from_base is not None
                and self.pv_from_xiaolan is not None
                and stamp > 0.0
                and 0.0 <= now - stamp <= self.max_age
            )
            return NavigationState(
                stamp=stamp,
                valid=valid,
                landing_confirmed=self.landing_confirmed,
                pv_from_base=(
                    np.eye(4) if self.pv_from_base is None
                    else self.pv_from_base.copy()
                ),
                pv_from_xiaolan=(
                    np.eye(4) if self.pv_from_xiaolan is None
                    else self.pv_from_xiaolan.copy()
                ),
                pv_boundary=self.pv_boundary.copy(),
            )


class ImuInput:
    """缓存实机IMU姿态和角速度，仅为攀爬安全观察提供输入。"""

    def __init__(self):
        self.max_age = float(rospy.get_param("~real_climb_max_imu_age", 0.2))
        self.lock = Lock()
        self.stamp = 0.0
        self.rotation = None
        self.angular_velocity = np.full(3, np.nan, dtype=np.float64)
        self.subscriber = rospy.Subscriber(
            rospy.get_param("~imu_topic", "/grasp_hexapod/imu"),
            Imu,
            self._callback,
            queue_size=1,
        )

    def _callback(self, message):
        """保存有效四元数和角速度。"""

        q = np.array(
            [
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            ],
            dtype=np.float64,
        )
        norm = np.linalg.norm(q)
        angular_velocity = np.array(
            [
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ],
            dtype=np.float64,
        )
        stamp = message.header.stamp.to_sec()
        if norm <= 0.0 or stamp <= 0.0 or not np.isfinite(angular_velocity).all():
            return
        x, y, z, w = q / norm
        rotation = np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )
        if not np.isfinite(rotation).all():
            return
        with self.lock:
            self.rotation = rotation
            self.angular_velocity = angular_velocity
            self.stamp = stamp

    def snapshot(self):
        """返回新鲜IMU快照。"""

        now = rospy.Time.now().to_sec()
        with self.lock:
            valid = (
                self.rotation is not None
                and self.stamp > 0.0
                and 0.0 <= now - self.stamp <= self.max_age
            )
            return {
                "valid": valid,
                "rotation": np.eye(3) if self.rotation is None else self.rotation.copy(),
                "angular_velocity": self.angular_velocity.copy(),
            }


class BoolInput:
    """缓存可选锁紧确认输入。"""

    def __init__(self, topic):
        self.lock = Lock()
        self.value = None
        self.received_at = 0.0
        self.subscriber = rospy.Subscriber(topic, Bool, self._callback, queue_size=1)

    def _callback(self, message):
        with self.lock:
            self.value = bool(message.data)
            self.received_at = rospy.Time.now().to_sec()

    def snapshot(self):
        with self.lock:
            return self.value

    def snapshot_with_time(self):
        with self.lock:
            return self.value, self.received_at


class RosControlNode:
    """处理ROS输入和整机状态机；实机发布目标，仿真可同步调用。"""

    WAIT_B = "WAIT_B"
    RESETTING = "RESETTING"
    HOLD = "HOLD"
    RUNNING = "RUNNING"
    SERVO_BOARD_LEGS = (
        ("left", ("lf", "lm", "lb")),
        ("right", ("rf", "rm", "rb")),
    )
    LEG_INDEX = {name: index for index, name in enumerate(LEG_NAMES)}

    @staticmethod
    def _climb_foot_gate_m(value):
        """验证仅实机入口可覆盖的 FK 足端任务门。"""

        value = float(value)
        if not np.isfinite(value) or not 0.0 < value <= 0.10:
            raise ValueError("~climb_foot_gate_m must be finite and in (0, 0.10]")
        return value

    def __init__(self, local_execution=False, controller_rate_hz=None):
        self.local_execution = bool(local_execution)
        if controller_rate_hz is None:
            controller_rate_hz = rospy.get_param(
                "~controller_rate_hz", 30.0
            )
        self.rate_hz = float(controller_rate_hz)
        # 控制器仍只在一帧完整的新六腿反馈上按30 Hz数学步长推进。
        # 提高轮询频率仅缩短两块独立Servo板反馈错相时的等待，不会重复
        # 消费同一帧反馈。
        self.poll_rate_hz = self.rate_hz * 4.0
        self.enable_link_collision_check = bool(
            rospy.get_param("~enable_link_collision_check", True)
        )
        self.max_feedback_age = float(
            rospy.get_param("~max_feedback_age", 0.30)
        )
        self.max_feedback_skew = float(
            rospy.get_param("~max_feedback_skew", 0.20)
        )
        if (
            self.max_feedback_skew <= 0.0
            or self.max_feedback_skew > self.max_feedback_age
        ):
            raise ValueError(
                "~max_feedback_skew must be positive and no greater than "
                "~max_feedback_age"
            )
        self.max_joy_age = float(
            rospy.get_param("~max_joy_age", 0.2)
        )
        self.enable_real_climb = bool(
            rospy.get_param("~enable_real_climb", False)
        )
        self.climb_foot_gate_m = self._climb_foot_gate_m(
            rospy.get_param("~climb_foot_gate_m", 0.02)
        )
        self.enable_real_dock = bool(
            rospy.get_param("~enable_real_dock", True)
        )
        self.dock_system_config = str(rospy.get_param(
            "~dock_system_config", str(package_config_path("dock_system.yaml"))
        ))
        self.dock_require_real_calibrated = bool(
            rospy.get_param("~dock_require_real_calibrated", True)
        )
        self.dock_allow_uncalibrated = bool(
            rospy.get_param("~dock_allow_uncalibrated", False)
        )
        self.dock_lock_confirmation_max_age = float(rospy.get_param(
            "~dock_lock_confirmation_max_age_s", 0.5
        ))
        if self.dock_lock_confirmation_max_age <= 0.0:
            raise ValueError("~dock_lock_confirmation_max_age_s must be positive")
        self.dock_session_started_at = 0.0
        self.control_source = str(
            rospy.get_param("~control_source", "teleop")
        ).strip().lower()
        if self.control_source not in ("teleop", "navigation"):
            raise ValueError("~control_source must be teleop or navigation")

        self.controller = GraspController(
            1.0 / self.rate_hz,
            enable_link_collision_check=self.enable_link_collision_check,
            climb_timeout_uses_wall_time=not self.local_execution,
        )
        self.lock = Lock()
        if not self.enable_link_collision_check:
            rospy.logwarn(
                "LINK COLLISION CHECK DISABLED: joint limits, workspace "
                "projection, and inter-foot clearance checks remain enabled"
            )

        # q_cur的行顺序与GraspController一致：lb、lf、lm、rb、rf、rm。
        self.q_cur = np.full((6, 3), np.nan, dtype=np.float64)
        self.feedback_stamp = np.zeros(6, dtype=np.float64)
        # 两块板分别顺序读取三条腿；仅当六条腿都产生新反馈后
        # 推进一次控制。
        self.last_control_feedback_stamp = np.zeros(6, dtype=np.float64)
        self.axes = np.empty(0, dtype=np.float64)
        self.buttons = np.empty(0, dtype=np.int32)
        self.button_press_latch = np.empty(0, dtype=np.int32)
        self.joy_stamp = 0.0

        self.button_a = int(rospy.get_param("~button_a", 0))
        self.button_b = int(rospy.get_param("~button_b", 1))
        self.button_x = int(rospy.get_param("~button_x", 2))
        self.button_y = int(rospy.get_param("~button_y", 3))
        self.axis_right = int(rospy.get_param("~axis_right", 0))
        self.axis_forward = int(rospy.get_param("~axis_forward", 1))
        self.axis_yaw = int(rospy.get_param("~axis_yaw", 3))
        self.axis_body_down = int(
            rospy.get_param("~axis_body_down", 4)
        )
        self.axis_body_up = int(
            rospy.get_param("~axis_body_up", 5)
        )
        self.axis_right_scale = float(
            rospy.get_param("~axis_right_scale", -1.0)
        )
        self.axis_forward_scale = float(
            rospy.get_param("~axis_forward_scale", 1.0)
        )
        self.axis_yaw_scale = float(
            rospy.get_param("~axis_yaw_scale", 1.0)
        )
        self.axis_body_scale = float(
            rospy.get_param("~axis_body_scale", -1.0)
        )

        self.max_linear_speed = float(
            rospy.get_param("~max_linear_speed", 0.20)
        )
        self.max_vertical_speed = float(
            rospy.get_param("~max_vertical_speed", 0.02)
        )
        foot_radius = np.mean(
            np.linalg.norm(self.controller.foot_init_base[:, :2], axis=1)
        )
        self.max_yaw_rate = float(
            rospy.get_param(
                "~max_yaw_rate",
                self.max_linear_speed / foot_radius,
            )
        )

        # 单一状态表达完整运动顺序，避免多个布尔量出现非法组合。
        self.state = self.WAIT_B
        self.local_climb_armed = False
        self.local_climb_entry_q = None
        self.manual_override = False
        self.command = np.zeros(4, dtype=np.float64)
        self.climb_start_navigation = None
        self.climb_start_imu_rotation = None
        self.climb_start_planned_pose = None
        self.real_climb_monitor_active = False
        self.real_climb_speed_diagnostic = None
        self.climb_bad_frames = 0
        self.climb_good_frames = 0
        self.real_climb_persistence_frames = int(
            rospy.get_param("~real_climb_persistence_frames", 3)
        )
        self.real_climb_max_position_error = float(
            rospy.get_param("~real_climb_max_position_error_m", 0.05)
        )
        self.real_climb_max_orientation_error = np.deg2rad(float(
            rospy.get_param("~real_climb_max_orientation_error_deg", 10.0)
        ))
        self.real_climb_max_angular_speed = np.deg2rad(float(
            rospy.get_param("~real_climb_max_angular_speed_deg_s", 30.0)
        ))

        self.navigation = NavigationInput()
        self.imu = ImuInput()
        self.lock_confirmation = BoolInput(
            rospy.get_param(
                "~lock_confirmed_topic",
                "/grasp_hexapod/dock/lock_confirmed",
            )
        )
        if self.control_source == "navigation":
            target_side, xiaolan_from_base = load_fixed_approach_config()
            self.controller.approach_mode.configure_fixed_approach(
                xiaolan_from_base,
                target_side=target_side,
                linear_speed=float(
                    rospy.get_param(
                        "~navigation_linear_speed",
                        self.max_linear_speed,
                    )
                ),
                yaw_rate=float(
                    rospy.get_param(
                        "~navigation_yaw_rate",
                        self.max_yaw_rate,
                    )
                ),
            )

        self.publishers = {}
        if not self.local_execution:
            self.publishers = {
                leg: rospy.Publisher(
                    f"/{leg}_des",
                    Float64MultiArray,
                    queue_size=1,
                )
                for leg in LEG_NAMES
            }

        # Subscriber的回调只保存最新消息，控制计算统一放在step()中。
        self.subscribers = [
            rospy.Subscriber(
                "/joy",
                Joy,
                self._joy_callback,
                # 摇杆是连续控制量，只消费最新值；按键按下沿由回调锁存。
                queue_size=1,
                tcp_nodelay=True,
            )
        ]
        if not self.local_execution:
            for leg_index, leg_name in enumerate(LEG_NAMES):
                self.subscribers.append(
                    rospy.Subscriber(
                        f"/{leg_name}_pos",
                        JointState,
                        self._make_feedback_callback(leg_index, leg_name),
                        queue_size=1,
                        tcp_nodelay=True,
                    )
                )

        if self.local_execution:
            rospy.loginfo(
                "Control source=%s; synchronous simulator control; "
                "climb source=isaac_sim_feedback foot_gate_m=%.3f; press B before A",
                self.control_source,
                self.climb_foot_gate_m,
            )
        else:
            rospy.loginfo(
                "Control source=%s; climb source=hardware_feedback foot_gate_m=%.3f; "
                "persistence/timeout come from compact config; waiting for feedback; "
                "press B before A",
                self.control_source,
                self.climb_foot_gate_m,
            )

    def _joy_callback(self, message):
        """保存最新摇杆，并锁存按钮按下沿直到控制帧消费。"""
        axes = np.asarray(message.axes, dtype=np.float64)
        buttons = np.asarray(message.buttons, dtype=np.int32)
        with self.lock:
            previous = np.zeros_like(buttons)
            common_size = min(previous.size, self.buttons.size)
            previous[:common_size] = self.buttons[:common_size]
            if self.button_press_latch.shape != buttons.shape:
                self.button_press_latch = np.zeros_like(buttons)
            self.button_press_latch[
                (buttons != 0) & (previous == 0)
            ] = 1
            self.axes = axes.copy()
            self.buttons = buttons.copy()
            self.joy_stamp = rospy.Time.now().to_sec()

    def _make_feedback_callback(self, leg_index, leg_name):
        """把一条腿的thigh、knee、ankle反馈写入q_cur对应行。"""

        def callback(message):
            position = np.asarray(message.position, dtype=np.float64)
            if position.shape != (3,):
                rospy.logwarn_throttle(
                    1.0,
                    f"/{leg_name}_pos must contain 3 joint positions",
                )
                return
            stamp = message.header.stamp.to_sec()
            if stamp <= 0.0:
                rospy.logwarn_throttle(
                    1.0,
                    f"/{leg_name}_pos must contain a valid timestamp",
                )
                return

            with self.lock:
                self.q_cur[leg_index] = position
                self.feedback_stamp[leg_index] = stamp

        return callback

    @staticmethod
    def _feedback_frame_state(
        q_cur,
        feedback_stamp,
        last_control_feedback_stamp,
        now,
        max_feedback_age,
        max_feedback_skew=0.20,
    ):
        """区分“反馈有效”和“两块板六条腿均已更新”。"""

        q_cur = np.asarray(q_cur, dtype=np.float64).reshape(6, 3)
        feedback_stamp = np.asarray(
            feedback_stamp, dtype=np.float64
        ).reshape(6)
        last_control_feedback_stamp = np.asarray(
            last_control_feedback_stamp, dtype=np.float64
        ).reshape(6)
        age = float(now) - feedback_stamp
        snapshot_skew = float(
            np.max(feedback_stamp) - np.min(feedback_stamp)
        )
        feedback_ready = bool(
            np.isfinite(q_cur).all()
            and (feedback_stamp > 0.0).all()
            and (age >= 0.0).all()
            and (age <= float(max_feedback_age)).all()
            and snapshot_skew <= float(max_feedback_skew)
        )
        complete_new_frame = bool(
            feedback_ready
            and (feedback_stamp > last_control_feedback_stamp).all()
        )
        return feedback_ready, complete_new_frame

    @classmethod
    def _feedback_issue(cls, q_cur, feedback_stamp, now, max_feedback_age):
        """生成按双板分组的反馈故障摘要，便于定位掉线腿。"""

        q_cur = np.asarray(q_cur, dtype=np.float64).reshape(6, 3)
        feedback_stamp = np.asarray(
            feedback_stamp, dtype=np.float64
        ).reshape(6)
        board_issues = []
        for board_name, leg_names in cls.SERVO_BOARD_LEGS:
            leg_issues = []
            for leg_name in leg_names:
                index = cls.LEG_INDEX[leg_name]
                stamp = feedback_stamp[index]
                if not np.isfinite(q_cur[index]).all() or stamp <= 0.0:
                    leg_issues.append(f"{leg_name}=missing")
                    continue
                age = float(now) - stamp
                if age < 0.0:
                    leg_issues.append(f"{leg_name}=future({-age:.3f}s)")
                elif age > float(max_feedback_age):
                    leg_issues.append(f"{leg_name}=stale({age:.3f}s)")
            if leg_issues:
                board_issues.append(
                    f"{board_name}[{', '.join(leg_issues)}]"
                )
        valid_stamp = feedback_stamp[feedback_stamp > 0.0]
        skew = (
            float(np.max(valid_stamp) - np.min(valid_stamp))
            if valid_stamp.size >= 2
            else float("nan")
        )
        return "; ".join(board_issues) + f"; snapshot_skew={skew:.3f}s"

    @staticmethod
    def _read(values, index):
        """读取Joy数组中的一个元素；不存在的索引按0处理。"""
        if index < 0 or index >= values.size:
            return 0.0
        return float(values[index])

    def _make_command(self, axes):
        """把归一化摇杆值转换成[vx_right, vy_forward, vz, yaw_rate]。"""
        # joy_node的轴符号由手柄驱动决定；scale参数把它统一到右/前语义。
        planar = np.array(
            [
                self.axis_right_scale
                * self._read(axes, self.axis_right),
                self.axis_forward_scale
                * self._read(axes, self.axis_forward),
            ],
            dtype=np.float64,
        )
        planar_norm = np.linalg.norm(planar)
        if planar_norm > 1.0:
            planar /= planar_norm

        return np.array(
            [
                self.max_linear_speed * planar[0],
                self.max_linear_speed * planar[1],
                self.max_vertical_speed * self._body_axis(axes),
                self.max_yaw_rate
                * self.axis_yaw_scale
                * self._read(axes, self.axis_yaw),
            ],
            dtype=np.float64,
        )

    def _body_axis(self, axes):
        """把RT/LT两个同基准扳机合成为[-1,1]升降指令。"""

        return 0.5 * self.axis_body_scale * (
            self._read(axes, self.axis_body_up)
            - self._read(axes, self.axis_body_down)
        )

    def _manual_command_active(self, axes):
        """判断摇杆是否请求接管导航；升降、平移或转向任一有效即接管。"""

        return max(
            abs(self._read(axes, self.axis_right)),
            abs(self._read(axes, self.axis_forward)),
            abs(self._body_axis(axes)),
            abs(self._read(axes, self.axis_yaw)),
        ) > 0.1

    def _publish_targets(self, q_des):
        """把控制器6×3关节目标拆成六个固定10元素消息。"""
        q_des = np.asarray(q_des, dtype=np.float64).reshape(6, 3)
        if not np.isfinite(q_des).all():
            raise ValueError("Controller produced non-finite q_des")

        for leg_index, leg_name in enumerate(LEG_NAMES):
            q_leg = q_des[leg_index]
            self.publishers[leg_name].publish(
                Float64MultiArray(
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

    def _ensure_dock_mode(self):
        """按显式实机请求创建DockMode，不给普通控制器增加ROS资源。"""

        if self.controller.dock_mode is not None:
            return self.controller.dock_mode
        from dock_mode import DockMode, DockPerception

        perception = DockPerception(
            max_age=float(rospy.get_param("~dock_max_perception_age", 0.35)),
            lock_frame=rospy.get_param("~dock_lock_frame", "dock_lock_center"),
            pin_frame_prefix=rospy.get_param(
                "~dock_pin_frame_prefix", "dock_pin_from_tag_"
            ),
            dock_system_path=self.dock_system_config,
        )
        dock_mode = DockMode(
            self.controller,
            perception=perception,
            linear_speed_m_s=float(
                rospy.get_param("~dock_linear_speed_m_s", 0.05)
            ),
            update_rate_hz=float(rospy.get_param("~dock_update_rate_hz", 10.0)),
            perception_rate_hz=float(
                rospy.get_param("~dock_perception_rate_hz", 10.0)
            ),
            leg_lift_speed_m_s=float(
                rospy.get_param("~dock_leg_lift_speed_m_s", 0.05)
            ),
            sit_settle_duration_s=float(
                rospy.get_param("~dock_sit_settle_duration_s", 0.5)
            ),
            require_lock_confirmation=bool(
                rospy.get_param("~dock_require_lock_confirmation", False)
            ),
        )
        self.controller.attach_dock_mode(dock_mode)
        return dock_mode

    @staticmethod
    def _rotation_angle(rotation):
        """返回正交旋转矩阵的最小夹角。"""

        cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
        return float(np.arccos(cosine))

    def _real_climb_observation(self):
        """比较部署传感器相对运动与当前计划机身运动。"""

        navigation = self.navigation.motion_snapshot()
        imu = self.imu.snapshot()
        if not navigation.valid:
            return False, "RTK/LoRa navigation pose is stale or invalid"
        if not imu["valid"]:
            return False, "IMU is stale or invalid"
        if (
            self.climb_start_navigation is None
            or self.climb_start_imu_rotation is None
            or self.climb_start_planned_pose is None
            or self.controller.climb_mode.base_pose is None
        ):
            return False, "climb safety reference is missing"
        start_xiaolan_from_base = (
            np.linalg.inv(self.climb_start_navigation.pv_from_xiaolan)
            @ self.climb_start_navigation.pv_from_base
        )
        current_xiaolan_from_base = (
            np.linalg.inv(navigation.pv_from_xiaolan)
            @ navigation.pv_from_base
        )
        actual_relative = (
            np.linalg.inv(start_xiaolan_from_base)
            @ current_xiaolan_from_base
        )
        planned_now = ClimbMode._world_from_base(self.controller.climb_mode.base_pose)
        planned_relative = (
            np.linalg.inv(self.climb_start_planned_pose) @ planned_now
        )
        position_error = float(np.linalg.norm(
            actual_relative[:3, 3] - planned_relative[:3, 3]
        ))
        imu_relative = self.climb_start_imu_rotation.T @ imu["rotation"]
        orientation_error = self._rotation_angle(
            planned_relative[:3, :3].T @ imu_relative
        )
        angular_speed = float(np.linalg.norm(imu["angular_velocity"]))
        if position_error > self.real_climb_max_position_error:
            return False, "relative RTK position error {:.3f} m".format(position_error)
        if orientation_error > self.real_climb_max_orientation_error:
            return False, "IMU attitude error {:.1f} deg".format(
                np.degrees(orientation_error)
            )
        if angular_speed > self.real_climb_max_angular_speed:
            return False, "IMU angular speed {:.1f} deg/s".format(
                np.degrees(angular_speed)
            )
        return True, ""

    def _monitor_real_climb(self):
        """持续观察已启用的可选相对运动监控。"""

        if (
            not self.real_climb_monitor_active
            or self.controller.mode != self.controller.CLIMB
        ):
            return
        okay, reason = self._real_climb_observation()
        if okay:
            self.climb_good_frames += 1
            self.climb_bad_frames = 0
            return
        self.climb_good_frames = 0
        if "stale" in reason or "invalid" in reason or "missing" in reason:
            self.climb_bad_frames = self.real_climb_persistence_frames
        else:
            self.climb_bad_frames += 1
        if (
            self.controller.climb_mode.state == ClimbMode.RUNNING
            and self.climb_bad_frames >= self.real_climb_persistence_frames
        ):
            self.controller.hold_climb()
            self._flush_real_climb_speed_diagnostic("safety hold")
            self.state = self.HOLD
            self.command[:] = 0.0
            rospy.logwarn(
                "Diagnostic replay CLIMB HOLD: %s; joint-feedback gates "
                "are not contact/load evidence",
                reason,
            )

    def _reset_real_climb_speed_diagnostic(self):
        """Start a hardware-only, observation-only stage diagnostic session."""

        self.real_climb_speed_diagnostic = None

    def _warn_hardware_climb_phase_hold(self):
        """报告实机反馈门冻结，保留完整足端与关节定位。"""

        if self.controller.mode != self.controller.CLIMB:
            return
        climb = self.controller.climb_mode
        if (
            climb.state != ClimbMode.RUNNING
            or not climb.hardware_execution
            or not climb.last_phase_hold
        ):
            return
        rospy.logwarn_throttle(
            0.5,
            "CLIMB PHASE HOLD: source=%s stage=%s %s collision_guard_hold=%s",
            "isaac_sim_feedback"
            if getattr(self, "local_execution", False)
            else "hardware_feedback",
            climb.phase,
            climb.tracking_diagnostic_summary(),
            "true" if climb.last_collision_guard_hold else "false",
        )

    def _info_hardware_climb_active_trace(self):
        """周期输出活动腿的 base_link 规划/反馈对照，不推断因果。"""

        if self.controller.mode != self.controller.CLIMB:
            return
        climb = self.controller.climb_mode
        if climb.state != ClimbMode.RUNNING or not climb.hardware_execution:
            return
        rospy.loginfo_throttle(
            1.0,
            "CLIMB ACTIVE LEG TRACE: source=%s stage=%s %s; diagnostic only: "
            "desired_z/planned lift low suggests planning, while desired-correct "
            "actual lag with same-leg joint error suggests load/execution/feedback; "
            "not causal proof",
            "isaac_sim_feedback"
            if getattr(self, "local_execution", False)
            else "hardware_feedback",
            climb.last_diagnostic_stage_name,
            climb.active_leg_diagnostic_summary(),
        )

    def _flush_real_climb_speed_diagnostic(self, reason):
        """Log and discard the completed stage aggregate without affecting motion."""

        item = self.real_climb_speed_diagnostic
        if item is None:
            return
        peak_command = item["peak_command_speed_rad_s"]
        peak_measured = item["peak_measured_speed_rad_s"]
        peak_ratio = (peak_measured / peak_command
                      if peak_command > 0.0 else float("nan"))
        mean_ratio = (item["ratio_sum"] / item["ratio_count"]
                      if item["ratio_count"] else float("nan"))
        rospy.loginfo(
            "CLIMB speed diagnostic-only stage=%s reason=%s peak_cmd=%.3f "
            "peak_meas=%.3f R_v=%.3f mean_meas_cmd=%.3f samples=%d "
            "peak_tracking=%.4f clip_joints=%d guard_holds=%d; mean ratio "
            "uses per-joint |cmd_speed| >= 0.05 rad/s and is not contact/load evidence",
            item["stage"], reason, peak_command, peak_measured, peak_ratio,
            mean_ratio, item["ratio_count"], item["peak_tracking_error_rad"],
            item["velocity_limit_clip_count"], item["collision_guard_hold_count"],
        )
        self.real_climb_speed_diagnostic = None

    def _record_real_climb_speed_diagnostic(
        self,
        stage,
        q_cur,
        q_des,
        feedback_stamp,
        sample_time,
    ):
        """Aggregate actual-hardware CLIMB feedback; this never gates a stage."""

        if self.local_execution or self.controller.mode != self.controller.CLIMB:
            return
        q_cur = np.asarray(q_cur, dtype=np.float64).reshape(6, 3)
        q_des = np.asarray(q_des, dtype=np.float64).reshape(6, 3)
        feedback_stamp = np.asarray(
            feedback_stamp, dtype=np.float64
        ).reshape(6)
        sample_time = float(sample_time)
        item = self.real_climb_speed_diagnostic
        if item is None or item["stage"] != stage:
            self._flush_real_climb_speed_diagnostic("stage transition")
            item = {
                "stage": stage,
                "previous_q_cur": q_cur.copy(),
                "previous_q_des": q_des.copy(),
                "previous_feedback_stamp": feedback_stamp.copy(),
                "previous_sample_time": sample_time,
                "peak_command_speed_rad_s": 0.0,
                "peak_measured_speed_rad_s": 0.0,
                "peak_tracking_error_rad": float(np.max(np.abs(q_des - q_cur))),
                "ratio_sum": 0.0,
                "ratio_count": 0,
                "velocity_limit_clip_count": int(
                    self.controller.last_update_velocity_limit_clip_count),
                "collision_guard_hold_count": int(
                    self.controller.last_update_collision_guard_hold_count),
            }
            self.real_climb_speed_diagnostic = item
            return
        command_dt = sample_time - item["previous_sample_time"]
        feedback_dt = feedback_stamp - item["previous_feedback_stamp"]
        if command_dt <= 0.0 or (feedback_dt <= 0.0).any():
            item["previous_q_cur"] = q_cur.copy()
            item["previous_q_des"] = q_des.copy()
            item["previous_feedback_stamp"] = feedback_stamp.copy()
            item["previous_sample_time"] = sample_time
            return
        command_speed = np.abs(q_des - item["previous_q_des"]) / command_dt
        measured_speed = (
            np.abs(q_cur - item["previous_q_cur"])
            / feedback_dt[:, np.newaxis]
        )
        item["peak_command_speed_rad_s"] = max(
            item["peak_command_speed_rad_s"], float(np.max(command_speed)))
        item["peak_measured_speed_rad_s"] = max(
            item["peak_measured_speed_rad_s"], float(np.max(measured_speed)))
        item["peak_tracking_error_rad"] = max(
            item["peak_tracking_error_rad"], float(np.max(np.abs(q_des - q_cur))))
        significant = command_speed >= 0.05
        if np.any(significant):
            item["ratio_sum"] += float(np.sum(
                measured_speed[significant] / command_speed[significant]))
            item["ratio_count"] += int(np.count_nonzero(significant))
        item["velocity_limit_clip_count"] += int(
            self.controller.last_update_velocity_limit_clip_count)
        item["collision_guard_hold_count"] += int(
            self.controller.last_update_collision_guard_hold_count)
        item["previous_q_cur"] = q_cur.copy()
        item["previous_q_des"] = q_des.copy()
        item["previous_feedback_stamp"] = feedback_stamp.copy()
        item["previous_sample_time"] = sample_time

    def _start_real_climb(self, q_cur, controls_ready):
        """在回站和关节反馈门限通过后进入诊断回放C1。"""

        if not self.enable_real_climb:
            rospy.logwarn_throttle(2.0, "X ignored: enable_real_climb is false")
            return
        if not controls_ready or self.state != self.HOLD or self.controller.mode != self.controller.APPROACH:
            rospy.logwarn_throttle(2.0, "X ignored: reset must finish and controls must be fresh")
            return
        navigation = self.navigation.motion_snapshot()
        imu = self.imu.snapshot()
        try:
            config = self.controller.climb_mode._load_config()
            gate_m = getattr(self, "climb_foot_gate_m", 0.02)
            config["settle_gate"]["max_foot_target_error_m"] = gate_m
            self.controller.enter_climb(
                q_cur, config=config, hardware_execution=True
            )
        except ValueError as error:
            rospy.logwarn("X ignored: compact entry gate failed: %s", error)
            return
        self.real_climb_monitor_active = navigation.valid and imu["valid"]
        if self.real_climb_monitor_active:
            self.climb_start_navigation = navigation
            self.climb_start_imu_rotation = imu["rotation"]
            self.climb_start_planned_pose = ClimbMode._world_from_base(
                self.controller.climb_mode.base_pose
            )
            rospy.loginfo("Diagnostic replay: optional IMU/RTK relative-motion monitoring enabled")
        else:
            self.climb_start_navigation = None
            self.climb_start_imu_rotation = None
            self.climb_start_planned_pose = None
            rospy.logwarn(
                "Diagnostic replay: IMU or RTK/LoRa unavailable at start; "
                "optional relative-motion monitoring disabled for this replay"
            )
        self.climb_bad_frames = 0
        self.climb_good_frames = 0
        self._reset_real_climb_speed_diagnostic()
        self.command[:] = 0.0
        self.local_climb_armed = False
        self.local_climb_entry_q = None
        self.state = self.RUNNING
        rospy.loginfo(
            "X accepted: diagnostic replay C1-C36 started with source=%s "
            "foot_gate_m=%.3f; joint-feedback gates are not contact/load evidence",
            "isaac_sim_feedback" if self.local_execution else "hardware_feedback",
            gate_m,
        )

    def _start_real_dock(self, q_cur, controls_ready):
        """在实机输入新鲜时进入DockMode并恢复缓存的攀爬末姿态。"""

        if not self.enable_real_dock:
            rospy.logwarn_throttle(2.0, "Y ignored: enable_real_dock is false")
            return
        if not controls_ready or self.state != self.HOLD:
            rospy.logwarn_throttle(2.0, "Y ignored: reset must finish and controls must be fresh")
            return
        try:
            if (
                getattr(self, "dock_require_real_calibrated", False)
                and not getattr(self, "dock_allow_uncalibrated", False)
            ):
                from dock_mode import load_dock_system
                dock_system = load_dock_system(self.dock_system_config)
                if not dock_system["real_calibrated"]:
                    rospy.logwarn_throttle(
                        2.0, "Y ignored: dock_system.yaml is not real-calibrated"
                    )
                    return
            self._ensure_dock_mode()
            self.controller.enter_dock(q_cur)
        except (ImportError, RuntimeError, ValueError) as error:
            rospy.logwarn("Y ignored: DockMode entry failed: %s", error)
            return
        self.command[:] = 0.0
        self.dock_session_started_at = rospy.Time.now().to_sec()
        self.state = self.RUNNING
        rospy.loginfo(
            "Y accepted: entering climb terminal posture, then starting visual docking"
        )

    def arm_local_climb(self, q_entry):
        """把同步 Isaac 场景锁在 C1 入口，等待 X 启动同一实机门控链。"""

        if not self.local_execution:
            raise RuntimeError("local climb arm is only valid in Isaac")
        self.local_climb_entry_q = np.asarray(
            q_entry, dtype=np.float64
        ).reshape(6, 3).copy()
        if not np.isfinite(self.local_climb_entry_q).all():
            raise ValueError("local climb entry q must be finite")
        self.local_climb_armed = True
        self.controller.q_des = self.local_climb_entry_q.copy()
        self.controller.reset_active = False
        self.state = self.HOLD
        self.command[:] = 0.0
        rospy.loginfo(
            "Isaac compact C1 armed: HOLD entry target; press X to start"
        )

    def _dock_lock_confirmed(self):
        """只接受本次Y后、未过期的锁紧确认。"""

        lock_confirmed, received_at = self.lock_confirmation.snapshot_with_time()
        now = rospy.Time.now().to_sec()
        age = now - received_at
        if (
            received_at < self.dock_session_started_at
            or age < 0.0
            or age > self.dock_lock_confirmation_max_age
        ):
            return None
        return lock_confirmed

    def _hold_motion(self, reason, log=True):
        """停止推进步态并保持上力；再次按A才能恢复。"""
        if self.state == self.RUNNING:
            if self.controller.mode == self.controller.DOCK:
                if self.controller.dock_mode is not None:
                    self.controller.dock_mode.fail_execution(
                        "DockMode stopped: " + reason
                    )
                self.state = self.HOLD
                self.command[:] = 0.0
                if log:
                    rospy.logwarn("DockMode failed into HOLD: %s", reason)
                return
            self.controller.approach_mode.cancel_autonomous_approach(reason)
            self.controller.hold_climb()
            self._flush_real_climb_speed_diagnostic("hold: " + reason)
            self.state = self.HOLD
            self.command[:] = 0.0
            if log:
                rospy.loginfo("Motion paused: %s", reason)

    def _process_buttons(self, button_presses, controls_ready, q_cur):
        """处理一次按钮事件；B不依赖Joy或关节反馈是否有效。"""
        a_pressed = bool(self._read(button_presses, self.button_a))
        b_pressed = bool(self._read(button_presses, self.button_b))
        x_pressed = bool(self._read(button_presses, self.button_x))
        y_pressed = bool(self._read(button_presses, self.button_y))
        if b_pressed:
            if self.local_execution:
                self.local_climb_armed = False
                self.local_climb_entry_q = None
            self._flush_real_climb_speed_diagnostic("reset")
            self.state = self.RESETTING
            self.controller.reset_active = False
            self.controller.mission.cancel("reset requested by B")
            self.controller.abort_climb()
            if self.controller.dock_mode is not None and self.controller.dock_mode.active:
                self.controller.dock_mode.exit()
            self.manual_override = False
            self.command[:] = 0.0
            rospy.loginfo("B pressed: returning to stand")
            return

        if x_pressed and y_pressed:
            rospy.logwarn_throttle(2.0, "X/Y ignored: climb and dock requests conflict")
            return
        if x_pressed:
            self._start_real_climb(q_cur, controls_ready)
            return
        if y_pressed:
            self._start_real_dock(q_cur, controls_ready)
            return

        if not a_pressed:
            return

        if getattr(self, "local_climb_armed", False):
            rospy.logwarn_throttle(
                2.0, "A ignored: Isaac compact C1 is armed; press X or B"
            )
            return

        if self.controller.mode == self.controller.DOCK:
            rospy.logwarn_throttle(2.0, "A ignored while DockMode is active")
            return
        if self.controller.mode == self.controller.CLIMB:
            if self.state == self.HOLD:
                if not self.real_climb_monitor_active:
                    self.controller.resume_climb()
                    self.state = self.RUNNING
                    rospy.loginfo("Diagnostic replay CLIMB resumed without optional motion monitoring")
                else:
                    okay, reason = self._real_climb_observation()
                    if okay and self.climb_good_frames >= self.real_climb_persistence_frames:
                        self.controller.resume_climb()
                        self.state = self.RUNNING
                        rospy.loginfo("Diagnostic replay CLIMB resumed after IMU/RTK persistence")
                    else:
                        rospy.logwarn("A ignored: CLIMB HOLD persists: %s", reason)
            elif self.state == self.RUNNING:
                self.controller.hold_climb()
                self._flush_real_climb_speed_diagnostic("paused by A")
                self.state = self.HOLD
                self.command[:] = 0.0
                rospy.loginfo("CLIMB paused by A")
            return

        if not controls_ready:
            rospy.logwarn("A ignored: waiting for valid control inputs")
        elif self.state == self.HOLD:
            self.state = self.RUNNING
            if (
                self.control_source == "navigation"
                and not self.manual_override
            ):
                result = self.controller.start_autonomous_approach(
                    self.navigation.snapshot()
                )
                if result.failed:
                    rospy.logwarn(
                        "Navigation holding: %s; move joystick to take over",
                        result.reason,
                    )
                else:
                    rospy.loginfo(
                        "Navigation started: side=%s",
                        result.target_side,
                    )
            else:
                rospy.loginfo("Motion enabled")
        elif self.state == self.RUNNING:
            self._hold_motion("paused by A")
        else:
            rospy.logwarn("A ignored: press B and wait for stand first")

    def _update_control(
        self,
        q_cur,
        axes,
        button_presses,
        joy_stamp,
        now,
        feedback_ready=True,
        feedback_stamp=None,
    ):
        """用一帧完整反馈处理B/A和运动指令，返回18关节目标。"""
        joy_fresh = (
            joy_stamp > 0.0
            and 0.0 <= now - joy_stamp <= self.max_joy_age
        )
        self._process_buttons(
            button_presses,
            joy_fresh and feedback_ready,
            q_cur,
        )

        if not joy_fresh:
            self._hold_motion("joystick lost")
            axes = np.empty(0, dtype=np.float64)

        # 第一次B以前不发布目标，Servo保持卸力并只读反馈。
        if self.state == self.WAIT_B:
            return None

        if getattr(self, "local_climb_armed", False) and self.state == self.HOLD:
            return self.local_climb_entry_q.copy()

        if self.state == self.RESETTING:
            if not self.controller.reset_active:
                self.controller.reset_to_stand(q_cur)
            q_des = self.controller.update(q_cur, self.command)
            if not self.controller.reset_active:
                self.state = self.HOLD
                rospy.loginfo(
                    "Stand initialization complete; press A to move"
                )
            return q_des

        navigation_state = None
        dock_robot_state = None
        climb_stage = (
            self.controller.climb_mode.phase
            if (not self.local_execution
                and self.controller.mode == self.controller.CLIMB
                and self.controller.climb_mode.state == ClimbMode.RUNNING)
            else None
        )
        if self.state == self.RUNNING:
            if self.controller.mode == self.controller.DOCK:
                self.command[:] = 0.0
                dock_robot_state = {
                    "joints": q_cur,
                    "lock_confirmed": self._dock_lock_confirmed(),
                }
            elif (
                self.control_source == "navigation"
                and not self.manual_override
                and self._manual_command_active(axes)
            ):
                self.manual_override = True
                self.controller.approach_mode.cancel_autonomous_approach(
                    "joystick takeover"
                )
                rospy.loginfo("Joystick took over navigation")

            if self.manual_override or self.control_source == "teleop":
                self.command[:] = self._make_command(axes)
            else:
                self.command[:] = 0.0
                navigation_state = self.navigation.snapshot()
        else:
            # 暂停时让当前摆动腿先落地再停止。
            self.command[:] = 0.0

        # update()内部完成足端规划、工作空间检查和DLS逆运动学；
        # reset_active期间则输出五次曲线回站立轨迹。
        q_des = self.controller.update(
            q_cur,
            self.command,
            navigation_state,
            dock_robot_state,
        )
        self._warn_hardware_climb_phase_hold()
        self._info_hardware_climb_active_trace()
        if climb_stage is not None and feedback_stamp is not None:
            self._record_real_climb_speed_diagnostic(
                climb_stage,
                q_cur,
                q_des,
                feedback_stamp,
                now,
            )
        if (
            self.controller.mode == self.controller.DOCK
            and self.controller.dock_mode is not None
            and self.controller.dock_mode.state
            in self.controller.dock_mode.TERMINAL_STATES
            and self.state == self.RUNNING
        ):
            self.state = self.HOLD
            rospy.loginfo("DockMode terminal HOLD: %s", self.controller.dock_mode.reason)
        if self.controller.mode == self.controller.CLIMB:
            if self.controller.climb_mode.state in (
                ClimbMode.DONE,
                ClimbMode.FAILED,
            ) and self.state == self.RUNNING:
                self._flush_real_climb_speed_diagnostic("terminal")
                self.state = self.HOLD
                self.command[:] = 0.0
                if self.controller.climb_mode.state == ClimbMode.DONE:
                    rospy.loginfo("CLIMB DONE: HOLD; press Y to request docking")
                else:
                    rospy.logwarn(
                        "CLIMB FAILED: HOLD: %s",
                        self.controller.climb_mode.failure_reason,
                    )
            if self.real_climb_monitor_active:
                self._monitor_real_climb()
        return q_des

    def update_from_feedback(self, q_cur):
        """Isaac控制帧同步调用，避免ROS双循环造成目标重复和卡顿。"""
        with self.lock:
            axes = self.axes.copy()
            button_presses = self.button_press_latch.copy()
            self.button_press_latch[:] = 0
            joy_stamp = self.joy_stamp
        now = rospy.Time.now().to_sec()

        return self._update_control(
            np.asarray(q_cur, dtype=np.float64).reshape(6, 3),
            axes,
            button_presses,
            joy_stamp,
            now,
            feedback_ready=True,
        )

    def step(self):
        """实机循环：读取完整反馈，计算一次目标并发布给两块Servo板。"""
        with self.lock:
            q_cur = self.q_cur.copy()
            feedback_stamp = self.feedback_stamp.copy()
            axes = self.axes.copy()
            joy_stamp = self.joy_stamp
            # 在反馈快照之后取时钟，避免回调刚写入的新时间戳
            # 落到now之后。
            now = rospy.Time.now().to_sec()
            feedback_ready, complete_new_frame = self._feedback_frame_state(
                q_cur,
                feedback_stamp,
                self.last_control_feedback_stamp,
                now,
                self.max_feedback_age,
                self.max_feedback_skew,
            )
            b_pending = bool(self._read(
                self.button_press_latch, self.button_b
            ))
            if complete_new_frame or not feedback_ready or b_pending:
                button_presses = self.button_press_latch.copy()
                self.button_press_latch[:] = 0
            else:
                # A/X/Y最多等待另一个板完成本轮反馈；B仍在上方立即消费。
                button_presses = np.zeros_like(self.button_press_latch)
            if complete_new_frame:
                self.last_control_feedback_stamp = feedback_stamp.copy()

        if not feedback_ready:
            # B已经被转换为RESETTING状态，即使反馈暂时不可用也不会丢失。
            self._process_buttons(
                button_presses,
                controls_ready=False,
                q_cur=q_cur,
            )
            self._hold_motion("joint feedback lost")
            rospy.logwarn_throttle(
                1.0,
                "Waiting for valid 18-DOF feedback: %s",
                self._feedback_issue(
                    q_cur,
                    feedback_stamp,
                    now,
                    self.max_feedback_age,
                ),
            )
            return

        if not complete_new_frame:
            # 两块板独立计时，等待较慢板的三条腿；
            # 不重复推进控制器状态。
            if b_pending:
                self._process_buttons(
                    button_presses,
                    controls_ready=False,
                    q_cur=q_cur,
                )
            return

        q_des = self._update_control(
            q_cur,
            axes,
            button_presses,
            joy_stamp,
            now,
            feedback_ready=feedback_ready,
            feedback_stamp=feedback_stamp,
        )
        if q_des is not None:
            # 初始化、暂停和行走都保持舵机上力；暂停不等于卸力。
            self._publish_targets(q_des)


def main():
    rospy.init_node("grasp_hexapod_control")
    node = RosControlNode()
    rate = rospy.Rate(node.poll_rate_hz)

    while not rospy.is_shutdown():
        node.step()
        rate.sleep()


if __name__ == "__main__":
    main()
