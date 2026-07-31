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

from pathlib import Path
import sys
from threading import Lock

import numpy as np
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

# 源码直启时使用当前目录；rosrun/roslaunch时从ROS包路径找到scripts。
scripts_dir = Path(__file__).resolve().parent
if not (scripts_dir / "control.py").exists():
    import rospkg

    scripts_dir = (
        Path(rospkg.RosPack().get_path("grasp_hexapod_control"))
        / "scripts"
    )
sys.path.insert(0, str(scripts_dir))

from control import GraspController
from kinematics import LEG_NAMES


class RosControlNode:
    """缓存ROS输入，在固定周期内完成一次高层控制闭环。"""

    def __init__(self):
        self.rate_hz = float(rospy.get_param("~control_rate_hz", 30.0))
        self.max_feedback_age = float(
            rospy.get_param("~max_feedback_age", 0.5)
        )
        self.max_joy_age = float(rospy.get_param("~max_joy_age", 0.5))
        self.control_source = str(
            rospy.get_param("~control_source", "teleop")
        ).strip().lower()
        if self.control_source not in ("teleop", "navigation"):
            raise ValueError("~control_source must be teleop or navigation")

        self.controller = GraspController(1.0 / self.rate_hz)
        self.lock = Lock()

        # q_cur的行顺序与GraspController一致：lb、lf、lm、rb、rf、rm。
        self.q_cur = np.full((6, 3), np.nan, dtype=np.float64)
        self.feedback_stamp = np.zeros(6, dtype=np.float64)
        self.axes = np.empty(0, dtype=np.float64)
        self.buttons = np.empty(0, dtype=np.int32)
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

        self.max_linear_speed = float(
            rospy.get_param("~max_linear_speed", 0.02)
        )
        self.max_vertical_speed = float(
            rospy.get_param("~max_vertical_speed", 0.005)
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

        # 三个布尔量直接表达安全顺序：必须B完成后，A才能使能。
        self.reset_started = False
        self.initialized = False
        self.enabled = False
        self.last_a_down = False
        self.last_b_down = False
        self.last_x_down = False
        self.last_y_down = False
        self.manual_override = False
        self.command = np.zeros(4, dtype=np.float64)

        self.navigation = None
        if self.control_source == "navigation":
            from navigation_ros import NavigationRosInput

            self.navigation = NavigationRosInput()
            left_pose = rospy.get_param(
                "~xiaolan_from_left_base",
                [],
            )
            right_pose = rospy.get_param(
                "~xiaolan_from_right_base",
                [],
            )
            left_pose = np.asarray(left_pose, dtype=np.float64)
            right_pose = np.asarray(right_pose, dtype=np.float64)
            if left_pose.size == 16 and right_pose.size == 16:
                self.controller.approach_mode.configure_autonomous_approach(
                    left_pose.reshape(4, 4),
                    right_pose.reshape(4, 4),
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
                queue_size=1,
            )
        ]
        for leg_index, leg_name in enumerate(LEG_NAMES):
            self.subscribers.append(
                rospy.Subscriber(
                    f"/{leg_name}_pos",
                    Float64MultiArray,
                    self._make_feedback_callback(leg_index, leg_name),
                    queue_size=1,
                )
            )

        rospy.loginfo(
            "Control source=%s; waiting for feedback; press B before A",
            self.control_source,
        )

    def _joy_callback(self, message):
        """保存最新手柄数组；使用接收时间判断手柄是否掉线。"""
        with self.lock:
            self.axes = np.asarray(message.axes, dtype=np.float64).copy()
            self.buttons = np.asarray(message.buttons, dtype=np.int32).copy()
            self.joy_stamp = rospy.Time.now().to_sec()

    def _make_feedback_callback(self, leg_index, leg_name):
        """把一条腿的thigh、knee、ankle反馈写入q_cur对应行。"""

        def callback(message):
            position = np.asarray(message.data, dtype=np.float64)
            if position.shape != (3,):
                rospy.logwarn_throttle(
                    1.0,
                    f"/{leg_name}_pos must contain 3 values",
                )
                return

            with self.lock:
                self.q_cur[leg_index] = position
                self.feedback_stamp[leg_index] = rospy.Time.now().to_sec()

        return callback

    @staticmethod
    def _read(values, index):
        """读取Joy数组中的一个元素；不存在的索引按0处理。"""
        if index < 0 or index >= values.size:
            return 0.0
        return float(values[index])

    def _make_command(self, axes):
        """把归一化摇杆值转换成[vx_right, vy_forward, vz, yaw_rate]。"""
        planar = np.array(
            [
                self._read(axes, self.axis_right),
                -self._read(axes, self.axis_forward),
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
                -self.max_yaw_rate * self._read(axes, self.axis_yaw),
            ],
            dtype=np.float64,
        )

    def _body_axis(self, axes):
        """把RT/LT两个同基准扳机合成为[-1,1]升降指令。"""

        return 0.5 * (
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

    def step(self):
        """读取同一时刻的输入，处理B/A，再调用控制器并发布目标。"""
        now = rospy.Time.now().to_sec()
        with self.lock:
            q_cur = self.q_cur.copy()
            feedback_stamp = self.feedback_stamp.copy()
            axes = self.axes.copy()
            buttons = self.buttons.copy()
            joy_stamp = self.joy_stamp

        feedback_ready = (
            np.isfinite(q_cur).all()
            and (feedback_stamp > 0.0).all()
            and ((now - feedback_stamp) <= self.max_feedback_age).all()
        )
        if not feedback_ready:
            rospy.logwarn_throttle(1.0, "Waiting for valid 18-DOF feedback")
            self.reset_started = False
            self.initialized = False
            self.enabled = False
            return

        joy_fresh = (
            joy_stamp > 0.0
            and 0.0 <= now - joy_stamp <= self.max_joy_age
        )
        if not joy_fresh:
            if self.enabled:
                rospy.logwarn("Joystick lost; motion paused")
                self.controller.approach_mode.cancel_autonomous_approach(
                    "joystick lost"
                )
            self.enabled = False
            axes = np.empty(0, dtype=np.float64)
            buttons = np.empty(0, dtype=np.int32)

        a_down = bool(self._read(buttons, self.button_a))
        b_down = bool(self._read(buttons, self.button_b))
        x_down = bool(self._read(buttons, self.button_x))
        y_down = bool(self._read(buttons, self.button_y))
        a_pressed = a_down and not self.last_a_down
        b_pressed = b_down and not self.last_b_down
        x_pressed = x_down and not self.last_x_down
        y_pressed = y_down and not self.last_y_down
        self.last_a_down = a_down
        self.last_b_down = b_down
        self.last_x_down = x_down
        self.last_y_down = y_down

        # B优先级最高：停止当前行为，并从真实q_cur平滑返回标准站姿。
        if b_pressed:
            self.enabled = False
            self.initialized = False
            self.reset_started = True
            self.manual_override = False
            self.command[:] = 0.0
            self.controller.reset_to_stand(q_cur)
            rospy.loginfo("B pressed: returning to stand")
        elif x_pressed or y_pressed:
            # 接口先固定，模式实现完成前绝不进入会抛异常的占位状态。
            self.enabled = False
            self.manual_override = True
            self.controller.approach_mode.cancel_autonomous_approach(
                "reserved mode button pressed"
            )
            mode = "CLIMB" if x_pressed else "DOCK"
            rospy.logwarn("%s is reserved but not implemented", mode)
        elif a_pressed:
            if self.initialized:
                self.enabled = not self.enabled
                if not self.enabled:
                    self.controller.approach_mode.cancel_autonomous_approach(
                        "paused by A"
                    )
                    rospy.loginfo("Motion paused")
                elif (
                    self.control_source == "navigation"
                    and not self.manual_override
                ):
                    result = self.controller.start_autonomous_approach(
                        self.navigation.snapshot()
                    )
                    if result.failed:
                        rospy.logwarn(
                            "Navigation holding: %s; "
                            "move joystick to take over",
                            result.reason,
                        )
                    else:
                        rospy.loginfo(
                            "Navigation started: side=%s",
                            result.target_side,
                        )
                else:
                    rospy.loginfo("Motion enabled")
            else:
                rospy.logwarn("A ignored: press B and wait for stand first")

        # 第一次B以前不发布目标，Servo保持卸力并只读反馈。
        if not self.reset_started:
            return

        navigation_state = None
        if self.enabled and joy_fresh:
            if (
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
        )

        if not self.initialized and not self.controller.reset_active:
            self.initialized = True
            rospy.loginfo("Stand initialization complete; press A to move")

        # 初始化、暂停和行走都保持舵机上力；暂停不等于卸力。
        self._publish_targets(q_des)

def main():
    rospy.init_node("grasp_hexapod_control")
    node = RosControlNode()
    rate = rospy.Rate(node.rate_hz)

    while not rospy.is_shutdown():
        node.step()
        rate.sleep()


if __name__ == "__main__":
    main()
