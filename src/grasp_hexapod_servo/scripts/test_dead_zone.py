#!/usr/bin/env python3
# encoding: utf-8
"""实测LX-15D舵机"目标位置接近当前位置时不转动"的死区范围。

方法：
    对指定舵机加载力矩，从一个参考位置出发，向正负两个方向
    以1脉冲递增的目标差值发送 MOVE_TIME_WRITE，每次发完读回
    实际位置，判断舵机是否真的发生了移动。

输出：
    每个方向"命令了但实际没动"的最大脉冲差（即死区），
    以及"最小可动脉冲差"。

用法：
    python3 test_dead_zone.py [--id 6] [--port /dev/ttyTHS0] [--trials 3]
        [--angles 0,15,-15]

注意：
    --angles 按 servo.py 的映射转为脉冲（pulse = 角度*1000/240 + 500，
    0° = 500脉冲）。不指定时以当前位置为参考。
    测试会移动舵机，请确认机器处于安全放置状态。
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from hiwonder_servo_controller import HiwonderServoController


# 递增的测试差值（脉冲）。0~1000脉冲对应0~240度，1脉冲=0.24度。
DELTAS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 15, 20, 30, 50]
SETTLE_S = 0.7     # 回参考点后等待稳定
MOVE_DURATION_MS = 800  # 目标运动时间，慢速便于观察
OBSERVE_S = 1.1    # 发完目标后等待到位

RESOLUTION = 1000.0 / 240.0  # 与 servo.py 一致：脉冲/度


def angle_to_pulse(angle_deg):
    """角度(度)转脉冲，方向为+1，与 servo.py 的 rad_to_servo 一致。"""
    return int(round(angle_deg * RESOLUTION + 500.0))


def read_stable(control, servo_id, n=3):
    """连续读n次位置，丢弃None，取中位数。"""
    values = []
    for _ in range(n * 2):
        value = control.get_servo_position(servo_id)
        if value is not None:
            values.append(value)
        if len(values) >= n:
            break
    if not values:
        return None
    return sorted(values)[len(values) // 2]


def run_sweep(control, servo_id, reference, direction, deltas):
    """从reference出发，向direction方向递增测试，返回 (dead_zone, min_move)。"""
    dead_zone = 0       # 发了但没动的最大差值
    min_move = None     # 能真正移动的最小差值
    for delta in deltas:
        # 回到参考点（若上一轮没动，回参考点也在死区内，无副作用）
        control.set_servo_position(servo_id, reference, MOVE_DURATION_MS)
        time.sleep(SETTLE_S)

        pos_before = read_stable(control, servo_id)
        if pos_before is None:
            print("  [warn] 读取失败，跳过 delta=%d" % delta)
            continue

        target = pos_before + direction * delta
        if target < 0 or target > 1000:
            print("  [warn] 目标越界(%d)，跳过 delta=%d" % (target, delta))
            continue

        control.set_servo_position(servo_id, target, MOVE_DURATION_MS)
        time.sleep(OBSERVE_S)

        pos_after = read_stable(control, servo_id)
        if pos_after is None:
            print("  [warn] 读取失败，跳过 delta=%d" % delta)
            continue

        moved = pos_after - pos_before
        print("  delta=%-3d target=%-4d pos %d -> %d  moved=%+d%s"
              % (delta, target, pos_before, pos_after, moved,
                 "" if moved else "   <- 未转动"))
        if moved != 0:
            min_move = delta
            break
        dead_zone = delta

    return dead_zone, min_move


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=int, default=6)
    parser.add_argument("--port", default="/dev/ttyTHS0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--angles", default=None,
                        help="逗号分隔的测试角度(度)，如 '0,15,-15'；"
                             "不指定则以当前位置为参考")
    args = parser.parse_args()

    control = HiwonderServoController(args.port, args.baudrate)

    servo_id = args.id

    voltage = control.get_servo_voltage(servo_id)
    print("servo #%d 电压: %s" % (servo_id,
          "%d mV" % voltage if voltage else "N/A"))

    load_before = control.get_servo_load_state(servo_id)
    print("初始加载状态: %s" % load_before)
    # 测试需要力矩：加载舵机
    control.unload_servo(servo_id, 1)
    time.sleep(0.5)
    print("已加载力矩，准备测试（舵机%d会移动）..." % servo_id)

    # 参考位置：默认当前位置，或由 --angles 转换
    if args.angles:
        references = [
            angle_to_pulse(float(angle))
            for angle in args.angles.split(",")
            if angle.strip() != ""
        ]
        print("测试角度: %s -> 参考脉冲 %s"
              % (args.angles, references))
    else:
        current = read_stable(control, servo_id)
        if current is None:
            print("无法读取舵机%d当前位置，退出" % servo_id)
            sys.exit(1)
        references = [current]
        print("参考位置(当前位置): %d" % current)

    for reference in references:
        # 先移动到参考位置并等稳定
        control.set_servo_position(servo_id, reference, MOVE_DURATION_MS)
        time.sleep(SETTLE_S)
        actual = read_stable(control, servo_id)
        angle = (reference - 500.0) / RESOLUTION
        print("\n========== 参考角度 %+.1f° (脉冲 %d, 实测 %s) =========="
              % (angle, reference,
                 actual if actual is not None else "N/A"))

        print("\n=== 正向 (+) 测试 ===")
        for trial in range(1, args.trials + 1):
            print("--- 第 %d 次 ---" % trial)
            dz, mm = run_sweep(control, servo_id, reference, +1, DELTAS)
            print("正向第%d次: 死区=%d脉冲, 最小可动=%s脉冲" % (trial, dz, mm))

        print("\n=== 反向 (-) 测试 ===")
        for trial in range(1, args.trials + 1):
            print("--- 第 %d 次 ---" % trial)
            dz, mm = run_sweep(control, servo_id, reference, -1, DELTAS)
            print("反向第%d次: 死区=%d脉冲, 最小可动=%s脉冲" % (trial, dz, mm))

    # 回到最后一个参考点，恢复初始加载状态
    control.set_servo_position(servo_id, references[-1], MOVE_DURATION_MS)
    time.sleep(SETTLE_S)
    if load_before is not None:
        control.unload_servo(servo_id, load_before)
    print("\n已回到参考位置并恢复初始加载状态")


if __name__ == "__main__":
    main()
