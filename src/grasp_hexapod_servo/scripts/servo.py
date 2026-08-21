#!/usr/bin/env python3
"""单块九舵机驱动板的ROS节点。

每个节点管理一块串口板和三条腿：
    left  -> lf、lm、lb
    right -> rf、rm、rb

订阅：
    /<leg>_des
    [power, thigh_pos, knee_pos, ankle_pos,
     thigh_vel, knee_vel, ankle_vel, 0, 0, 0]

发布：
    /<leg>_pos
    sensor_msgs/JointState，包含时间戳和三个关节位置。

诊断：
    ~enable_diagnostics为true时输出时序和供电电压。

ROS侧角度单位统一为rad。
"""

import ast
import math
import os
import sys
import time
from threading import Lock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Header

import hiwonder_servo_controller


class ServoSideNode:
    """管理一块驱动板上的九个LX-15D舵机。"""

    JOINT_NAMES = ("thigh", "knee", "ankle")

    SIDE_CONFIG = {
        "left": {
            "port": "/dev/ttyTHS0",
            "legs": ("lf", "lm", "lb"),
            "directions": (1, 1, 1, 1, 1, 1, 1, 1, 1),
            "id_map": {
                "lf": (1, 2, 3),
                "lm": (4, 5, 6),
                "lb": (7, 8, 9),
            },
        },
        "right": {
            "port": "/dev/ttyACM0",
            "legs": ("rf", "rm", "rb"),
            "directions": (1, -1, -1, 1, -1, -1, 1, -1, -1),
            "id_map": {
                "rf": (10, 11, 12),
                "rm": (13, 14, 15),
                "rb": (16, 17, 18),
            },
        },
    }

    def __init__(self):
        self.side = str(
            rospy.get_param("~side", "left")
        ).strip().lower()

        if self.side not in self.SIDE_CONFIG:
            raise ValueError(
                "~side must be one of: left, right"
            )

        config = self.SIDE_CONFIG[self.side]
        self.legs = config["legs"]
        self.id_map = config["id_map"]
        self.default_directions = config["directions"]

        self.port = rospy.get_param(
            "~port",
            config["port"],
        )
        self.baudrate = int(
            rospy.get_param("~baudrate", 115200)
        )
        self.servo_rate_hz = float(
            rospy.get_param("~servo_rate_hz", 30.0)
        )
        self.command_duration_ms = int(
            rospy.get_param("~command_duration_ms", 33)
        )
        self.enable_diagnostics = bool(
            rospy.get_param("~enable_diagnostics", True)
        )
        self.voltage_report_interval_s = float(
            rospy.get_param("~voltage_report_interval_s", 2.0)
        )
        if self.voltage_report_interval_s <= 0.0:
            raise ValueError("~voltage_report_interval_s must be positive")

        # 一块板固定管理九个舵机。
        self.servo_ids = tuple(
            servo_id
            for leg in self.legs
            for servo_id in self.id_map[leg]
        )

        self.directions = self._load_directions()

        self.control = (
            hiwonder_servo_controller.HiwonderServoController(
                self.port,
                self.baudrate,
            )
        )

        # LX-15D：0~1000脉冲对应0~240度。
        self.resolution = 1000.0 / 240.0

        # 回调和定时控制循环共享目标数据。
        self.lock = Lock()

        # power_on表示整块板的九个舵机是否加载。
        self.power_on = False

        # 每条腿是否收到过完整目标。
        self.received_des = {
            leg: False
            for leg in self.legs
        }

        # 每条腿请求的板级加载状态。
        # Control会给六条腿发送相同的power值。
        self.power_request = {
            leg: False
            for leg in self.legs
        }

        self.des_pos = {
            leg: [0.0, 0.0, 0.0]
            for leg in self.legs
        }

        # 时序汇总本身不增加串口读写或改变定时器节拍。
        self._timing_window_started = None
        self._timing_callbacks = 0
        self._timing_max_loop_s = 0.0
        self._timing_overruns = 0
        self._timing_read_retries = {
            servo_id: 0 for servo_id in self.servo_ids
        }
        self._timing_read_failures = {
            servo_id: 0 for servo_id in self.servo_ids
        }

        # 每个控制周期最多读一个电压，避免一次连读九个占用总线。
        self._voltage_labels = {
            servo_id: f"{leg}_{joint_name}"
            for leg in self.legs
            for joint_name, servo_id in zip(
                self.JOINT_NAMES,
                self.id_map[leg],
            )
        }
        self._voltage_pending_ids = []
        self._voltage_samples_mv = {}
        self._voltage_next_report_at = (
            time.monotonic() + self.voltage_report_interval_s
        )

        # 启动时整块板全部卸力。
        # 此时仍然可以读取舵机位置。
        self._set_board_power(False)

        self.des_subs = {}
        self.pos_pubs = {}

        for leg in self.legs:
            self.des_subs[leg] = rospy.Subscriber(
                f"/{leg}_des",
                Float64MultiArray,
                self._make_des_callback(leg),
                queue_size=1,
            )

            self.pos_pubs[leg] = rospy.Publisher(
                f"/{leg}_pos",
                JointState,
                queue_size=1,
            )

        # 统计窗口从定时器启用前开始，不把启动卸力和pub/sub创建计入。
        self._timing_window_started = time.monotonic()
        self.timer = rospy.Timer(
            rospy.Duration(
                1.0 / self.servo_rate_hz
            ),
            self.control_loop,
        )

        rospy.loginfo(
            "Servo board ready: side=%s port=%s legs=%s "
            "ids=%s rate=%.1fHz",
            self.side,
            self.port,
            ",".join(self.legs),
            self.servo_ids,
            self.servo_rate_hz,
        )

    def _load_directions(self):
        """读取本板九个舵机的安装方向。"""

        direction_param = rospy.get_param(
            "~directions",
            None,
        )

        if direction_param is None:
            directions = list(self.default_directions)
        elif isinstance(direction_param, str):
            directions = list(
                ast.literal_eval(direction_param)
            )
        else:
            directions = list(direction_param)

        if len(directions) != len(self.servo_ids):
            raise ValueError(
                "~directions length %d does not match "
                "servo count %d"
                % (
                    len(directions),
                    len(self.servo_ids),
                )
            )
        if any(int(direction) not in (-1, 1) for direction in directions):
            raise ValueError("~directions values must be 1 or -1")

        return {
            servo_id: int(direction)
            for servo_id, direction in zip(
                self.servo_ids,
                directions,
            )
        }

    def _set_board_power(self, enabled):
        """统一加载或卸载本板的全部九个舵机。"""

        status = 1 if enabled else 0

        for servo_id in self.servo_ids:
            self.control.unload_servo(
                servo_id,
                status,
            )

        self.power_on = bool(enabled)

        rospy.loginfo(
            "Servo board %s power: %s",
            self.side,
            "ON" if enabled else "OFF",
        )

    def _make_des_callback(self, leg):
        """为一条腿生成目标消息回调。"""

        def callback(message):
            data = list(message.data)

            # 接口固定为10个元素，避免上下游对数组含义理解不同。
            if len(data) != 10:
                rospy.logwarn_throttle(
                    1.0,
                    f"/{leg}_des must contain 10 values, "
                    f"got {len(data)}",
                )
                return

            power_value = float(data[0])
            target_position = [
                float(data[1]),
                float(data[2]),
                float(data[3]),
            ]

            if power_value not in (0.0, 1.0):
                rospy.logwarn_throttle(
                    1.0,
                    f"/{leg}_des power must be 0 or 1",
                )
                return

            if not all(
                math.isfinite(value)
                for value in target_position
            ):
                rospy.logwarn_throttle(
                    1.0,
                    f"/{leg}_des contains non-finite position",
                )
                return

            with self.lock:
                first_message = not self.received_des[leg]

                self.des_pos[leg] = target_position
                self.power_request[leg] = bool(
                    power_value
                )
                self.received_des[leg] = True

            if first_message:
                rospy.loginfo(
                    "Received first target for %s",
                    leg,
                )

            # data[4:7]是预留的目标速度。
            # LX-15D当前通过command_duration_ms控制运动时间，
            # 第一版实机控制不使用速度字段。

        return callback

    def rad_to_servo(self, angle_rad, direction):
        """ROS关节角rad转换为LX-15D脉冲。"""

        servo_position = (
            direction
            * math.degrees(angle_rad)
            * self.resolution
            + 500.0
        )

        return int(
            max(
                0,
                min(1000, round(servo_position)),
            )
        )

    def servo_to_rad(self, servo_position, direction):
        """LX-15D脉冲转换为ROS关节角rad。"""

        return direction * math.radians(
            (servo_position - 500.0)
            / self.resolution
        )

    def _read_position(self, servo_id):
        """读取一次位置；失败时只追加一次即时重试。"""
        position = self.control.get_servo_position(servo_id)
        if position is None:
            self._timing_read_retries[servo_id] += 1
            position = self.control.get_servo_position(servo_id)
            if position is None:
                self._timing_read_failures[servo_id] += 1
        return position

    @staticmethod
    def _timing_counts_text(counts):
        """稳定格式化本窗口内有事件的舵机计数。"""

        return ",".join(
            "%d=%d" % (servo_id, count)
            for servo_id, count in sorted(counts.items())
            if count
        ) or "none"

    def _record_timing_diagnostics(self, started_at):
        """每秒输出一次本板实际循环与读回异常汇总。"""

        elapsed = max(0.0, time.monotonic() - started_at)
        self._timing_callbacks += 1
        self._timing_max_loop_s = max(self._timing_max_loop_s, elapsed)
        if elapsed > 1.0 / self.servo_rate_hz:
            self._timing_overruns += 1

        window_elapsed = time.monotonic() - self._timing_window_started
        if window_elapsed < 1.0:
            return

        rospy.loginfo(
            "Servo timing: side=%s actual_hz=%.2f max_loop_ms=%.3f "
            "overruns=%d retries=%s failures=%s",
            self.side,
            self._timing_callbacks / window_elapsed,
            self._timing_max_loop_s * 1000.0,
            self._timing_overruns,
            self._timing_counts_text(self._timing_read_retries),
            self._timing_counts_text(self._timing_read_failures),
        )
        self._timing_window_started = time.monotonic()
        self._timing_callbacks = 0
        self._timing_max_loop_s = 0.0
        self._timing_overruns = 0
        self._timing_read_retries = {
            servo_id: 0 for servo_id in self.servo_ids
        }
        self._timing_read_failures = {
            servo_id: 0 for servo_id in self.servo_ids
        }

    def _update_voltage_diagnostics(self):
        """分散读取本板电压，完成九路后统一输出。"""

        now = time.monotonic()
        if not self._voltage_pending_ids:
            if now < self._voltage_next_report_at:
                return
            self._voltage_pending_ids = list(self.servo_ids)
            self._voltage_samples_mv = {}
            self._voltage_next_report_at = (
                now + self.voltage_report_interval_s
            )

        servo_id = self._voltage_pending_ids.pop(0)
        self._voltage_samples_mv[servo_id] = (
            self.control.get_servo_voltage(servo_id)
        )

        if self._voltage_pending_ids:
            return

        values = []
        for sample_id in self.servo_ids:
            voltage_mv = self._voltage_samples_mv.get(sample_id)
            voltage_text = (
                "N/A"
                if voltage_mv is None
                else f"{voltage_mv / 1000.0:.2f}V"
            )
            values.append(
                "%s(ID%d)=%s"
                % (
                    self._voltage_labels[sample_id],
                    sample_id,
                    voltage_text,
                )
            )

        rospy.loginfo(
            "Servo voltage: side=%s %s",
            self.side,
            " ".join(values),
        )

    def control_loop(self, _event):
        """执行最新目标，再读取并发布本周期有效反馈。"""

        started_at = time.monotonic()

        # 回调只更新缓存；串口操作期间不持有缓存锁。
        with self.lock:
            targets_ready = all(
                self.received_des.values()
            )
            requested_on = (
                targets_ready
                and all(self.power_request.values())
            )
            target_snapshot = {
                leg: tuple(self.des_pos[leg])
                for leg in self.legs
            }

        if requested_on != self.power_on:
            self._set_board_power(requested_on)

        # 一、优先写入最新目标，避免反馈读取占用本周期命令延迟。
        if self.power_on:
            for leg in self.legs:
                for joint_index, servo_id in enumerate(
                    self.id_map[leg]
                ):
                    servo_position = self.rad_to_servo(
                        target_snapshot[leg][joint_index],
                        self.directions[servo_id],
                    )
                    self.control.set_servo_position(
                        servo_id,
                        servo_position,
                        self.command_duration_ms,
                    )

        # 二、一条腿三个位置都有效时才发布一帧带时间戳的反馈。
        for leg in self.legs:
            read_position = []
            for servo_id in self.id_map[leg]:
                raw_position = self._read_position(servo_id)
                if raw_position is None:
                    read_position = []
                    break
                read_position.append(
                    self.servo_to_rad(
                        raw_position,
                        self.directions[servo_id],
                    )
                )

            if not read_position:
                rospy.logwarn_throttle(
                    1.0,
                    "Servo feedback unavailable: %s",
                    leg,
                )
                continue

            self.pos_pubs[leg].publish(
                JointState(
                    header=Header(stamp=rospy.Time.now()),
                    name=[
                        f"{leg}_thigh_joint",
                        f"{leg}_knee_joint",
                        f"{leg}_ankle_joint",
                    ],
                    position=read_position,
                )
            )

        if self.enable_diagnostics:
            self._update_voltage_diagnostics()
            self._record_timing_diagnostics(started_at)


if __name__ == "__main__":
    rospy.init_node("servo_side_node")
    ServoSideNode()
    rospy.spin()
