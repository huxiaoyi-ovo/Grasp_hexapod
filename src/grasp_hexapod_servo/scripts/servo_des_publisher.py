#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LX-15D话题连通性测试入口。

输入：ROS参数中的幅值、频率和偏置；输出：六条腿的正弦目标话题。
仅用于架空机器人测试通信，不进入正常实机控制流程。
"""

import math

import rospy
from std_msgs.msg import Float64MultiArray


class ServoDesPublisher:
    """发布 6 条腿的期望位置/速度到 *_des 话题。"""

    def __init__(self):
        self.publish_rate_hz = float(rospy.get_param('~publish_rate_hz', 100.0))
        self.wave_frequency_hz = float(rospy.get_param('~wave_frequency_hz', 0.1))
        self.amplitude_deg = float(rospy.get_param('~amplitude_deg', 120.0))
        self.offset_deg = float(rospy.get_param('~offset_deg', 0.0))
        self.use_same_for_all_joints = bool(rospy.get_param('~use_same_for_all_joints', True))

        self.topics = rospy.get_param(
            '~topics',
            ['/lf_des', '/lm_des', '/lb_des', '/rf_des', '/rm_des', '/rb_des'],
        )
        self.pubs = [rospy.Publisher(topic, Float64MultiArray, queue_size=10) for topic in self.topics]

        self._start_time = rospy.get_time()

    def _build_des_data(self, pos_rad, vel_rad_s):
        if self.use_same_for_all_joints:
            pos = [pos_rad, pos_rad, pos_rad]
            vel = [vel_rad_s, vel_rad_s, vel_rad_s]
        else:
            pos = [pos_rad, 0.0, 0.0]
            vel = [vel_rad_s, 0.0, 0.0]

        # 协议: pos pos pos vel vel vel 0 0 0
        return [1.0]+pos + vel + [0.0, 0.0, 0.0]

    def spin(self):
        rate = rospy.Rate(max(1.0, self.publish_rate_hz))
        omega = 2.0 * math.pi * abs(self.wave_frequency_hz)
        amplitude_rad = math.radians(self.amplitude_deg)
        offset_rad = math.radians(self.offset_deg)

        while not rospy.is_shutdown():
            t = max(0.0, rospy.get_time() - self._start_time)
            pos_rad = offset_rad + amplitude_rad * math.sin(omega * t)
            vel_rad_s = abs(amplitude_rad * omega * math.cos(omega * t))

            data = self._build_des_data(pos_rad, vel_rad_s)
            msg = Float64MultiArray(data=data)
            for pub in self.pubs:
                pub.publish(msg)

            rospy.loginfo_throttle(
                2.0,
                '发布_des: amp=%.1fdeg freq=%.3fHz pos=%.2fdeg vel=%.2fdeg/s',
                self.amplitude_deg,
                abs(self.wave_frequency_hz),
                math.degrees(pos_rad),
                math.degrees(vel_rad_s),
            )
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('servo_des_publisher')
    ServoDesPublisher().spin()
