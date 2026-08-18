#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import rospy
import math
import tf
from sensor_msgs.msg import Imu
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# 数据存储
time_data = []
roll_data = []
pitch_data = []
yaw_data = []

start_time = None

# 标签
ROLL_LABEL = u"Roll (\u00B0)"
PITCH_LABEL = u"Pitch (\u00B0)"
YAW_LABEL = u"Yaw (\u00B0)"

def imu_callback(msg):
    """订阅 IMU 数据并转为欧拉角"""
    global start_time
    if start_time is None:
        start_time = time.time()

    # 四元数
    q = msg.orientation
    quaternion = (q.x, q.y, q.z, q.w)

    # 转欧拉角
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

    # 弧度转度
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)

    t = time.time() - start_time

    time_data.append(t)
    roll_data.append(roll_deg)
    pitch_data.append(pitch_deg)
    yaw_data.append(yaw_deg)

def animate(i):
    """动画刷新绘图"""
    # 对齐数据长度
    min_len = min(len(time_data), len(roll_data), len(pitch_data), len(yaw_data))
    if min_len == 0:
        return

    t = time_data[:min_len]
    r = roll_data[:min_len]
    p = pitch_data[:min_len]
    y = yaw_data[:min_len]

    # Roll
    ax1.clear()
    ax1.plot(t, r, color='#1f77b4', linewidth=1.2)
    ax1.set_ylabel(ROLL_LABEL)
    ax1.grid(True, linestyle='--', alpha=0.7)

    # Pitch
    ax2.clear()
    ax2.plot(t, p, color='#ff7f0e', linewidth=1.2)
    ax2.set_ylabel(PITCH_LABEL)
    ax2.grid(True, linestyle='--', alpha=0.7)

    # Yaw
    ax3.clear()
    ax3.plot(t, y, color='#2ca02c', linewidth=1.2)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel(YAW_LABEL)
    ax3.grid(True, linestyle='--', alpha=0.7)

def main():
    global ax1, ax2, ax3
    rospy.init_node('imu_plot_rpy_node', anonymous=True)
    rospy.Subscriber('/handsfree/imu', Imu, imu_callback)

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    ani = animation.FuncAnimation(fig, animate, interval=200)

    plt.tight_layout()
    plt.show()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
