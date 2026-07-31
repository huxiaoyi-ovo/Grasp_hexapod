#!/usr/bin/env python3
"""单块六舵机驱动板的ROS节点。

每个节点管理一块串口板和两条腿：
    left  -> lf、lm
    right -> rf、rm
    mid   -> lb、rb

订阅：
    /<leg>_des
    [power, thigh_pos, knee_pos, ankle_pos,
     thigh_vel, knee_vel, ankle_vel, 0, 0, 0]

发布：
    /<leg>_pos
    [thigh_pos, knee_pos, ankle_pos]

ROS侧角度单位统一为rad。
"""

import ast
import math
import os
import sys
from threading import Lock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from std_msgs.msg import Float64MultiArray

import hiwonder_servo_controller


class ServoSideNode:
    """管理一块驱动板上的六个LX-15D舵机。"""

    SIDE_CONFIG = {
        "left": {
            "port": "/dev/ttyUSB0",
            "legs": ("lf", "lm"),
            "directions": (1, 1, 1, 1, 1, 1),
            "id_map": {
                "lf": (1, 2, 3),
                "lm": (4, 5, 6),
            },
        },
        "right": {
            "port": "/dev/ttyUSB1",
            "legs": ("rf", "rm"),
            "directions": (1, -1, -1, 1, -1, -1),
            "id_map": {
                "rf": (10, 11, 12),
                "rm": (13, 14, 15),
            },
        },
        "mid": {
            "port": "/dev/ttyUSB2",
            "legs": ("lb", "rb"),
            "directions": (1, 1, 1, 1, -1, -1),
            "id_map": {
                "lb": (7, 8, 9),
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
                "~side must be one of: left, right, mid"
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
            rospy.get_param("~baudrate", 115200) # pyright: ignore[reportArgumentType]
        )
        self.control_rate_hz = float(
            rospy.get_param("~control_rate_hz", 30.0) # type: ignore
        )
        self.command_duration_ms = int(
            rospy.get_param("~command_duration_ms", 33) # pyright: ignore[reportArgumentType]
        )

        # 一块板固定管理六个舵机。
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

        # power_on表示整块板的六个舵机是否加载。
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
                Float64MultiArray,
                queue_size=1,
            )

        self.timer = rospy.Timer(
            rospy.Duration(
                1.0 / self.control_rate_hz
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
            self.control_rate_hz,
        )

    def _load_directions(self):
        """读取本板六个舵机的安装方向。"""

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
        """统一加载或卸载本板的全部六个舵机。"""

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

    def control_loop(self, _event):
        """顺序完成反馈读取、板级加载切换和目标写入。"""

        if not self.lock.acquire(False):
            return

        try:
            # 一、始终读取并发布六个舵机的位置。
            for leg in self.legs:
                read_position = []

                for servo_id in self.id_map[leg]:
                    raw_position = (
                        self.control.get_servo_position(
                            servo_id
                        )
                    )

                    if raw_position is None:
                        read_position.append(
                            float("nan")
                        )
                    else:
                        read_position.append(
                            self.servo_to_rad(
                                raw_position,
                                self.directions[
                                    servo_id
                                ],
                            )
                        )

                self.pos_pubs[leg].publish(
                    Float64MultiArray(
                        data=read_position
                    )
                )

            # 二、两条腿都收到目标并且都请求加载时，
            # 才统一加载这块板的六个舵机。
            targets_ready = all(
                self.received_des.values()
            )
            requested_on = (
                targets_ready
                and all(self.power_request.values())
            )

            if requested_on != self.power_on:
                self._set_board_power(requested_on)

            # 三、只有整块板加载后才写入目标。
            if not self.power_on:
                return

            for leg in self.legs:
                for joint_index, servo_id in enumerate(
                    self.id_map[leg]
                ):
                    servo_position = self.rad_to_servo(
                        self.des_pos[leg][joint_index],
                        self.directions[servo_id],
                    )

                    self.control.set_servo_position(
                        servo_id,
                        servo_position,
                        self.command_duration_ms,
                    )

        finally:
            self.lock.release()


if __name__ == "__main__":
    rospy.init_node("servo_side_node")
    ServoSideNode()
    rospy.spin()
