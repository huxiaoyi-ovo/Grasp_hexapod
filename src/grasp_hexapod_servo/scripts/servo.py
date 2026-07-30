#!/usr/bin/env python3
import ast
import math
import os
import sys
from threading import Lock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
import hiwonder_servo_controller
from std_msgs.msg import Float64MultiArray


class ServoSideNode:
    """
    单块驱动板舵机控制节点（left / right / mid 三块板，各接一个串口）。
    - left:  lf(1,2,3) lm(4,5,6)           -> ID 1~6
    - right: rf(10,11,12) rm(13,14,15)     -> ID 10~15
    - mid:   lb(7,8,9) rb(16,17,18)        -> ID 7,8,9,16,17,18

    订阅: /<leg>_des, 数据格式: [pos pos pos vel vel vel 0 0 0]
    发布: /<leg>_pos, 数据格式: [pos pos pos]

    参数：
      ~side:       驱动板标识 left/right/mid，决定该串口控制的舵机 ID
      ~port:       串口设备，三块板各不相同，默认按 side 取 /dev/ttyUSB0/1/2
      ~directions: 各舵机方向列表（1 正 / -1 反），按舵机 ID 顺序排列，默认全 1

    功能：程序启动时仅读取发布当前位置，接收到话题数据后才开始写入舵机
    """

    # 三块驱动板的固定配置：串口默认值、腿列表、腿->舵机ID映射
    SIDE_CONFIG = {
        'left': {
            'port': '/dev/ttyUSB0',
            'legs': ['lf', 'lm'],
            'id_map': {'lf': [1, 2, 3], 'lm': [4, 5, 6]},
        },
        'right': {
            'port': '/dev/ttyUSB1',
            'legs': ['rf', 'rm'],
            'id_map': {'rf': [10, 11, 12], 'rm': [13, 14, 15]},
        },
        'mid': {
            'port': '/dev/ttyUSB2',
            'legs': ['lb', 'rb'],
            'id_map': {'lb': [7, 8, 9], 'rb': [16, 17, 18]},
        },
    }

    def __init__(self):
        self.side = str(rospy.get_param('~side', 'left')).strip().lower()
        if self.side not in self.SIDE_CONFIG:
            raise ValueError("~side must be one of 'left', 'right', 'mid'")
        cfg = self.SIDE_CONFIG[self.side]
        self.legs = cfg['legs']
        self.id_map = cfg['id_map']

        self.port = rospy.get_param('~port', cfg['port'])
        self.baudrate = int(rospy.get_param('~baudrate', 115200))
        self.control_rate_hz = float(rospy.get_param('~control_rate_hz', 30.0))
        self.command_duration_ms = int(rospy.get_param('~command_duration_ms', 33))

        # 各舵机方向（1 正转 / -1 反转），列表顺序与本板舵机 ID 顺序一致
        self.servo_ids = [sid for leg in self.legs for sid in self.id_map[leg]]
        dir_param = rospy.get_param('~directions', None)
        if dir_param is None:
            directions = [1] * len(self.servo_ids)
        else:
            if isinstance(dir_param, str):
                directions = list(ast.literal_eval(dir_param))
            else:
                directions = list(dir_param)
        if len(directions) != len(self.servo_ids):
            raise ValueError(
                '~directions length %d != servo count %d (ids=%s)'
                % (len(directions), len(self.servo_ids), self.servo_ids)
            )
        self.directions = {
            sid: (1 if int(d) >= 0 else -1)
            for sid, d in zip(self.servo_ids, directions)
        }

        self.control = hiwonder_servo_controller.HiwonderServoController(self.port, self.baudrate)
        self.resolution = 1000.0 / 240.0  # pulse per degree
        self._lock = Lock()
        self.power_on = False
        # 标记每条腿是否接收到过目标话题数据，初始为未接收
        self.received_des = {leg: False for leg in ['lf', 'lm', 'lb', 'rf', 'rm', 'rb']}

        # 按腿保存目标角度（rad）与速度（rad/s）
        self.des_pos = {leg: [0.0, 0.0, 0.0] for leg in self.legs}
        self.des_vel = {leg: [0.0, 0.0, 0.0] for leg in self.legs}

        self.des_subs = {}
        self.pos_pubs = {}
        for leg in self.legs:
            self.des_subs[leg] = rospy.Subscriber(
                '/{}_des'.format(leg),
                Float64MultiArray,
                self._make_des_callback(leg),
                queue_size=10,
            )
            self.pos_pubs[leg] = rospy.Publisher(
                '/{}_pos'.format(leg),
                Float64MultiArray,
                queue_size=10,
            )

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(1.0, self.control_rate_hz)),
            self.control_loop,
        )

        for leg in self.legs:
                ids = self.id_map[leg]
                for id in ids:
                    self.control.unload_servo(id, 0)
        rospy.loginfo(
            'servo_side_node started: side=%s, legs=%s, port=%s, hz=%.1f, ids=%s, directions=%s\n'
            '等待话题数据，启动后仅读取舵机位置，不执行写入',
            self.side,
            ','.join(self.legs),
            self.port,
            self.control_rate_hz,
            self.servo_ids,
            [self.directions[sid] for sid in self.servo_ids],
        )

    def _make_des_callback(self, leg):
        def _cb(msg):
            data = list(msg.data)
            if len(data) < 6:
                rospy.logwarn_throttle(1.0, '/%s_des length < 6, got %d', leg, len(data))
                return
            # 约定: [status pos pos pos vel vel vel 0 0 0]
            if not self.power_on and data[0] == 1:
                if not self.control.load_status(self.id_map[leg][0]):
                    self.control.unload_servo(self.id_map[leg][0], 1)
                    self.power_on = True
                else:
                    self.power_on = True
            elif self.power_on and data[0] == 0:
                if self.control.load_status(self.id_map[leg][0]):
                    self.control.unload_servo(self.id_map[leg][0], 0)
                    self.power_on = False
                else:
                    self.power_on = False

            self.des_pos[leg] = [float(data[1]), float(data[2]), float(data[3])]
            self.des_vel[leg] = [float(data[4]), float(data[5]), float(data[6])]
            # 标记该腿已接收到目标数据，允许写入
            self.received_des[leg] = True
            rospy.loginfo(f'已接收到 {leg} 腿的目标数据，开始写入舵机')

        return _cb

    def rad_to_servo(self, angle_rad, direction=1):
        pos = direction * math.degrees(angle_rad) * self.resolution + 500.0
        return int(max(0, min(1000, round(pos))))

    def servo_to_rad(self, servo_pos, direction=1):
        return direction * math.radians((servo_pos - 500.0) / self.resolution)

    def control_loop(self, _event):
        if not self._lock.acquire(False):
            return

        try:
            for leg in self.legs:
                ids = self.id_map[leg]
                # 读取舵机当前位置（始终执行，不受话题数据影响），按方向换算
                read_pos = []
                for i in range(3):
                    servo_id = ids[i]
                    raw = self.control.get_servo_position(servo_id)
                    if raw is None:
                        read_pos.append(float('nan'))
                    else:
                        read_pos.append(self.servo_to_rad(raw, self.directions[servo_id]))
                # 发布当前位置（始终执行）
                self.pos_pubs[leg].publish(Float64MultiArray(data=read_pos))

                # 关键：仅当接收到该腿的目标数据后，才执行写入操作
                if self.received_des[leg] and self.power_on:
                    target_pos = self.des_pos[leg]
                    for i in range(3):
                        servo_id = ids[i]
                        cmd = self.rad_to_servo(target_pos[i], self.directions[servo_id])
                        self.control.set_servo_position(servo_id, cmd, self.command_duration_ms)

        finally:
            self._lock.release()


if __name__ == '__main__':
    rospy.init_node('servo_side_node')
    ServoSideNode()
    rospy.spin()
