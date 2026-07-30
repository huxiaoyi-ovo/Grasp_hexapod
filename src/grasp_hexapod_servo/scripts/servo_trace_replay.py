#!/usr/bin/env python3
"""把Isaac Gym导出的关节目标轨迹按原时间轴发布给LX-15D ROS节点。

输入：run_sim.py生成的CSV，只读取target_*列。
输出：/lb_des、/lf_des、/lm_des、/rb_des、/rf_des、/rm_des。
消息：[使能, thigh, knee, ankle, 三个目标速度, 0, 0, 0]，角度单位rad。

CSV只规定控制器关节语义，不规定实机舵机ID、raw零位和旋转方向。实机驱动
必须先完成joint name到物理舵机的映射，不能按CSV列号直接解释为舵机ID。
"""

import csv
from pathlib import Path

import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray


LEG_NAMES = ("lb", "lf", "lm", "rb", "rf", "rm")
JOINT_NAMES = ("thigh", "knee", "ankle")


def load_trace(path):
    """读取时间戳和18关节目标，保持控制器规定的关节顺序。"""

    times = []
    targets = []
    with Path(path).expanduser().open(
        newline="",
        encoding="utf-8",
    ) as file:
        for row in csv.DictReader(file):
            times.append(float(row["time_s"]))
            targets.append(
                [
                    float(row[f"target_{leg}_{joint}_joint"])
                    for leg in LEG_NAMES
                    for joint in JOINT_NAMES
                ]
            )

    times = np.asarray(times, dtype=np.float64)
    targets = np.asarray(targets, dtype=np.float64).reshape(-1, 6, 3)
    if len(times) < 2 or np.any(np.diff(times) <= 0.0):
        raise ValueError("Trace must contain increasing time_s values")
    return times, targets


def main():
    rospy.init_node("servo_trace_replay")
    trace_path = rospy.get_param(
        "~trace",
        "logs/servo_walk_trace.csv",
    )
    start_delay = float(rospy.get_param("~start_delay", 2.0))
    times, targets = load_trace(trace_path)

    publishers = {
        leg: rospy.Publisher(
            f"/{leg}_des",
            Float64MultiArray,
            queue_size=1,
        )
        for leg in LEG_NAMES
    }

    # 现有舵机消息保留速度字段；驱动目前只执行位置目标。
    velocities = np.zeros_like(targets)
    velocities[1:] = (
        np.diff(targets, axis=0)
        / np.diff(times)[:, np.newaxis, np.newaxis]
    )

    rospy.loginfo(
        "Servo trace ready: %d frames, %.2f s; starts in %.1f s",
        len(times),
        times[-1] - times[0],
        start_delay,
    )
    rospy.logwarn(
        "Trace order is lb,lf,lm,rb,rf,rm and each leg is "
        "thigh,knee,ankle. Verify physical IDs, zero offsets and "
        "directions before enabling servos."
    )
    start_time = rospy.Time.now().to_sec() + start_delay

    for frame_index, relative_time in enumerate(times - times[0]):
        if rospy.is_shutdown():
            break

        sleep_time = start_time + relative_time - rospy.Time.now().to_sec()
        if sleep_time > 0.0:
            rospy.sleep(sleep_time)

        for leg_index, leg in enumerate(LEG_NAMES):
            data = [
                1.0,
                *targets[frame_index, leg_index],
                *velocities[frame_index, leg_index],
                0.0,
                0.0,
                0.0,
            ]
            publishers[leg].publish(Float64MultiArray(data=data))

    rospy.loginfo("Servo trace replay finished; holding the final target")


if __name__ == "__main__":
    main()
