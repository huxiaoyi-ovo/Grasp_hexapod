#!/usr/bin/env python3
# encoding: utf-8
"""实测LX-15D舵机移动后位置反馈多久恢复正常（稳定到目标值附近）。

方法：
    对舵机下发 MOVE_TIME_WRITE（运动时间固定），从下发时刻开始
    每50ms读一次位置，记录读数随时间的轨迹，找出"最后一次读数
    偏离最终稳定值超过 ±3 脉冲"的时刻，即为收敛时间。

用法：
    python3 test_feedback_settle.py [--id 6] [--port /dev/ttyTHS0]

注意：
    测试会移动舵机，请确认机器处于安全放置状态。
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from hiwonder_servo_controller import HiwonderServoController


MOVE_DURATION_MS = 800      # 运动时间，与死区测试一致
SAMPLE_INTERVAL_S = 0.05    # 采样间隔
TOTAL_S = 5.0               # 每次移动后采样总时长
BAND = 3                    # 收敛判定带宽（脉冲），与稳定后读数噪声一致

# (起始位置, 目标位置)：小/中/大行程各测一次
MOVES = [(500, 520), (520, 600), (600, 500)]


def median(values):
    values = sorted(values)
    return values[len(values) // 2]


def sample_after_move(control, servo_id, target, duration_ms, total_s,
                      interval_s):
    """下发移动并从此刻开始高频采样，返回 [(t, pos), ...]（t相对下发时刻）。"""
    t0 = time.time()
    control.set_servo_position(servo_id, target, duration_ms)
    samples = []
    while True:
        now = time.time()
        if now - t0 > total_s:
            break
        pos = control.get_servo_position(servo_id)
        if pos is not None:
            samples.append((now - t0, pos))
        time.sleep(interval_s)  # 读位置本身有耗时，实际间隔会略大于设定值
    return samples


def run_move(control, servo_id, start, target, interval_s):
    print("\n--- 移动 %d -> %d (%dms) ---" % (start, target, MOVE_DURATION_MS))
    samples = sample_after_move(control, servo_id, target, MOVE_DURATION_MS,
                                TOTAL_S, interval_s)
    if not samples:
        print("  [warn] 无有效读数")
        return

    finals = [pos for _, pos in samples[-20:]]
    final = median(finals)
    print("  最终稳定值: %d (最后20个读数中位数, 范围 %d~%d)"
          % (final, min(finals), max(finals)))

    # 收敛时间 = 最后一次偏离 final 超过 BAND 的时刻
    last_out = None
    max_dev_motion = 0.0
    for t, pos in samples:
        dev = abs(pos - final)
        if t < MOVE_DURATION_MS / 1000.0:
            max_dev_motion = max(max_dev_motion, dev)
        if dev > BAND:
            last_out = t
    if last_out is None:
        conv = samples[0][0]
    else:
        conv = last_out + interval_s
    print("  收敛时间: %.2fs (从下发时刻起，此后读数保持在 %d±%d 内)"
          % (conv, final, BAND))
    print("  运动阶段(前%.1fs)最大偏差: %d 脉冲" % (MOVE_DURATION_MS / 1000.0,
                                             max_dev_motion))

    # 时间线：每100ms打一行
    print("  时间线(每100ms):")
    last_tick = -1
    for t, pos in samples:
        tick = int(round(t * 10))
        if tick != last_tick:
            last_tick = tick
            marker = "  <= 收敛点" if t >= conv and t - interval_s < conv else ""
            print("    t=%5.2fs  pos=%3d  dev=%+d%s"
                  % (t, pos, pos - final, marker))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=int, default=6)
    parser.add_argument("--port", default="/dev/ttyTHS0")
    parser.add_argument("--baudrate", type=int, default=115200)
    args = parser.parse_args()

    control = HiwonderServoController(args.port, args.baudrate)
    servo_id = args.id

    voltage = control.get_servo_voltage(servo_id)
    print("servo #%d 电压: %s" % (servo_id,
          "%d mV" % voltage if voltage else "N/A"))

    load_before = control.get_servo_load_state(servo_id)
    print("初始加载状态: %s" % load_before)
    control.unload_servo(servo_id, 1)  # 测试需要力矩
    time.sleep(0.5)
    print("已加载力矩（舵机%d会移动）..." % servo_id)

    # 先到起点
    control.set_servo_position(servo_id, MOVES[0][0], MOVE_DURATION_MS)
    time.sleep(2.0)

    for start, target in MOVES:
        run_move(control, servo_id, start, target, SAMPLE_INTERVAL_S)
        time.sleep(1.0)

    if load_before is not None:
        control.unload_servo(servo_id, load_before)
    print("\n完成，已恢复初始加载状态")


if __name__ == "__main__":
    main()
