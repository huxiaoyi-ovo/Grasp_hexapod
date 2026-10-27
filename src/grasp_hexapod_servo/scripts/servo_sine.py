#!/usr/bin/env python3
# encoding: utf-8
"""lf腿单舵机正弦摆动节点（默认ID 3 = lf ankle）。

通过 /lf_des 驱动lf腿中的一个舵机做正弦摆动：
    目标角 = 中心角 + 幅值*sin(w*t)
    目标速度 = 幅值*w*cos(w*t)   (rad/s，解析导数)

目标关节默认以舵机中位（0deg）为中心、100deg幅值摆动，即
-100deg ~ +100deg全范围；其余关节保持启动时的反馈位置。

lf腿ID与关节对应：1=thigh，2=knee，3=ankle。

注意：
    1. 驱动板要求三条腿都收到目标且都请求加载才整板上电。
       ~power_whole_board为true时本节点同时发布/lm_des、/lb_des
       保持另外两条腿；若还有其他节点在发布这两个话题，应设为false。
    2. 当前servo.py驱动通过command_duration_ms控制运动，不消费
       速度字段；本节点仍按解析导数填入data[4:7]，供上层或后续驱动使用。

用法：
    rosrun grasp_hexapod_servo servo_sine.py
    rosrun grasp_hexapod_servo servo_sine.py _amplitude_deg:=30 _frequency_hz:=0.2
"""

import math
import time
from threading import Lock

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class ServoSineNode:
    """以/lf_des为目标接口驱动lf腿单个舵机做正弦摆动。"""

    # 左板lf腿：ID 1/2/3依次对应thigh/knee/ankle。
    LF_SERVO_IDS = (1, 2, 3)
    JOINT_NAMES = ("thigh", "knee", "ankle")

    # LX-15D行程0~240度，中位500脉冲，即偏离中位最多120度。
    MAX_TRAVEL_DEG = 120.0

    def __init__(self):
        self.servo_id = int(rospy.get_param("~servo_id", 3))
        if self.servo_id not in self.LF_SERVO_IDS:
            raise ValueError(
                "~servo_id must be one of %s (lf leg)"
                % (self.LF_SERVO_IDS,)
            )
        self.joint_index = self.LF_SERVO_IDS.index(self.servo_id)

        self.amplitude_rad = math.radians(
            float(rospy.get_param("~amplitude_deg", 100.0))
        )
        self.center_rad = math.radians(
            float(rospy.get_param("~center_deg", 0.0))
        )
        self.frequency_hz = float(
            rospy.get_param("~frequency_hz", 1)
        )
        self.publish_rate_hz = float(
            rospy.get_param("~publish_rate_hz", 30.0)
        )
        self.power_whole_board = bool(
            rospy.get_param("~power_whole_board", True)
        )
        self.feedback_timeout_s = float(
            rospy.get_param("~feedback_timeout_s", 5.0)
        )

        if self.amplitude_rad < 0.0:
            raise ValueError("~amplitude_deg must be >= 0")
        if self.frequency_hz <= 0.0:
            raise ValueError("~frequency_hz must be > 0")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("~publish_rate_hz must be > 0")
        if self.feedback_timeout_s < 0.0:
            raise ValueError("~feedback_timeout_s must be >= 0")

        # lm/lb只在需要补足整板上电条件时才相关。
        self.hold_legs = ["lf"]
        if self.power_whole_board:
            self.hold_legs.append("lm")
            self.hold_legs.append("lb")

        self.lock = Lock()
        self.base_pos = {
            leg: [0.0, 0.0, 0.0]
            for leg in self.hold_legs
        }
        self.have_feedback = {
            leg: False
            for leg in self.hold_legs
        }

        self.pos_subs = [
            rospy.Subscriber(
                f"/{leg}_pos",
                JointState,
                self._make_pos_callback(leg),
                queue_size=1,
            )
            for leg in self.hold_legs
        ]

        self.des_pubs = {
            leg: rospy.Publisher(
                f"/{leg}_des",
                Float64MultiArray,
                queue_size=1,
            )
            for leg in self.hold_legs
        }

        self._capture_base_pose()
        self._check_travel()

        # 正弦相位从发布第一帧目标开始。
        self.t0 = time.monotonic()
        self.rate = rospy.Rate(self.publish_rate_hz)

        rospy.loginfo(
            "Servo sine node ready: servo_id=%d joint=%s "
            "center=%.1fdeg amplitude=%.1fdeg frequency=%.3fHz "
            "rate=%.1fHz hold_legs=%s",
            self.servo_id,
            self.JOINT_NAMES[self.joint_index],
            math.degrees(self.center_rad),
            math.degrees(self.amplitude_rad),
            self.frequency_hz,
            self.publish_rate_hz,
            self.hold_legs,
        )

    def _make_pos_callback(self, leg):
        """为一条腿生成反馈回调，仅用于启动时捕获基准角。"""

        def callback(message):
            if len(message.position) < 3:
                return
            with self.lock:
                self.base_pos[leg] = list(message.position[:3])
                self.have_feedback[leg] = True

        return callback

    def _capture_base_pose(self):
        """等待各腿反馈作为保持基准；超时的腿退回零位。"""

        deadline = time.monotonic() + self.feedback_timeout_s
        while (
            not rospy.is_shutdown()
            and time.monotonic() < deadline
            and not all(self.have_feedback.values())
        ):
            time.sleep(0.05)

        for sub in self.pos_subs:
            sub.unregister()

        with self.lock:
            missing = [
                leg
                for leg in self.hold_legs
                if not self.have_feedback[leg]
            ]

        if missing:
            rospy.logwarn(
                "No feedback on %s within %.1fs, "
                "holding zero for missing legs",
                ",".join(f"/{leg}_pos" for leg in missing),
                self.feedback_timeout_s,
            )

    def _check_travel(self):
        """中心角叠加幅值超出LX-15D行程时提示会被限位。"""

        center_deg = math.degrees(self.center_rad)
        amplitude_deg = math.degrees(self.amplitude_rad)
        if abs(center_deg) + amplitude_deg > self.MAX_TRAVEL_DEG:
            rospy.logwarn(
                "base %.1fdeg + amplitude %.1fdeg exceeds LX-15D "
                "+/-%.0fdeg travel, command clamps at limit",
                center_deg,
                amplitude_deg,
                self.MAX_TRAVEL_DEG,
            )

    def _make_des_message(self, leg, offset_rad, velocity_rad_s):
        """拼装10元素目标：[power, pos*3, vel*3, 0, 0, 0]。"""

        with self.lock:
            targets = list(self.base_pos[leg])
        velocities = [0.0, 0.0, 0.0]

        # 目标关节以center_deg为中心摆动，其余关节保持启动时的位置。
        if leg == "lf":
            targets[self.joint_index] = self.center_rad + offset_rad
            velocities[self.joint_index] = velocity_rad_s

        message = Float64MultiArray()
        message.data = (
            [1.0] + targets + velocities + [0.0, 0.0, 0.0]
        )
        return message

    def spin(self):
        """按设定频率发布正弦目标，直到节点被关闭。"""

        angular_freq = 1.2 * math.pi * self.frequency_hz

        while not rospy.is_shutdown():
            phase = angular_freq * (time.monotonic() - self.t0)
            offset_rad = self.amplitude_rad * math.sin(phase)
            velocity_rad_s = (
                self.amplitude_rad * angular_freq * math.cos(phase)
            )

            for leg in self.hold_legs:
                self.des_pubs[leg].publish(
                    self._make_des_message(
                        leg,
                        offset_rad,
                        velocity_rad_s,
                    )
                )

            rospy.loginfo_throttle(
                10.0,
                "Sine t=%.1fs target=%.3frad vel=%.3frad/s",
                time.monotonic() - self.t0,
                self.center_rad + offset_rad,
                velocity_rad_s,
            )

            self.rate.sleep()

    def unload(self):
        """退出时对已发布话题下发卸力（power=0会卸载整块板）。"""

        for leg in self.hold_legs:
            message = Float64MultiArray()
            message.data = [0.0] * 10
            self.des_pubs[leg].publish(message)


if __name__ == "__main__":
    rospy.init_node("servo_sine_node")
    node = ServoSineNode()
    rospy.on_shutdown(node.unload)
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass
