#!/usr/bin/env python3
"""单侧LX-15D舵机ROS节点。

输入：三条腿的Float64MultiArray目标；输出：对应关节位置反馈。
结构：ROS话题适配 -> rad/raw转换 -> 总线读写；不负责步态和运动学。
"""

import math
from pathlib import Path
import sys
from threading import Lock

import rospy
from std_msgs.msg import Float64MultiArray

# catkin生成的启动转发脚本不自动加入本包scripts目录。
sys.path.insert(0, str(Path(__file__).resolve().parent))

import hiwonder_servo_controller


class ServoSideNode:
    """
    单侧舵机控制节点（左侧或右侧）。
    - left:  lf(1,2,3) lm(4,5,6) lb(7,8,9)
    - right: rf(10,11,12) rm(13,14,15) rb(16,17,18)

    订阅: /<leg>_des, 数据格式: [status pos pos pos vel vel vel 0 0 0]
    发布: /<leg>_pos, 数据格式: [pos pos pos]

    功能修改：程序启动时仅读取发布当前位置，接收到话题数据后才开始写入舵机
    """

    def __init__(self):
        self.side = str(rospy.get_param('~side', 'left')).strip().lower()
        default_port = '/dev/ttyUSB0' if self.side == 'left' else '/dev/tty0'
        self.port = rospy.get_param('~port', default_port)
        self.baudrate = int(rospy.get_param('~baudrate', 115200))
        self.control_rate_hz = float(rospy.get_param('~control_rate_hz', 50.0))
        self.command_duration_ms = int(rospy.get_param('~command_duration_ms', 20))

        self.control = hiwonder_servo_controller.HiwonderServoController(self.port, self.baudrate)
        self.resolution = 1000.0 / 240.0  # pulse per degree
        self._lock = Lock()
        self.power_on = False
        # 新增：标记每条腿是否接收到过目标话题数据，初始为未接收
        self.received_des = {leg: False for leg in ['lf', 'lm', 'lb', 'rf', 'rm', 'rb']}

        if self.side == 'left':
            self.legs = ['lf', 'lm', 'lb']
            self.id_map = {
                'lf': [1, 2, 3],
                'lm': [4, 5, 6],
                'lb': [7, 8, 9],
            }
        elif self.side == 'right':
            self.legs = ['rf', 'rm', 'rb']
            self.id_map = {
                'rf': [10, 11, 12],
                'rm': [13, 14, 15],
                'rb': [16, 17, 18],
            }
        else:
            raise ValueError("~side must be 'left' or 'right'")

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
            'servo_side_node started: side=%s, legs=%s, port=%s, hz=%.1f\n'
            '等待话题数据，启动后仅读取舵机位置，不执行写入',
            self.side,
            ','.join(self.legs),
            self.port,
            self.control_rate_hz,
        )

    def _make_des_callback(self, leg):
        def _cb(msg):
            data = list(msg.data)
            if len(data) < 7:
                rospy.logwarn_throttle(1.0, '/%s_des length < 7, got %d', leg, len(data))
                return
            # 约定: [pos pos pos vel vel vel 0 0 0]
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

    def rad_to_servo(self, angle_rad):
        pos = math.degrees(angle_rad) * self.resolution + 500.0
        return int(max(0, min(1000, round(pos))))

    def servo_to_rad(self, servo_pos):
        return math.radians((servo_pos - 500.0) / self.resolution)

    def control_loop(self, _event):
        if not self._lock.acquire(False):
            return

        try:
            for leg in self.legs:
                ids = self.id_map[leg]
                # 读取舵机当前位置（始终执行，不受话题数据影响）
                read_pos = []
                for i in range(3):
                    servo_id = ids[2-i]
                    raw = self.control.get_servo_position(servo_id)
                    if raw is None:
                        read_pos.append(float('nan'))
                    else:
                        read_pos.append(self.servo_to_rad(raw))
                # 发布当前位置（始终执行）
                self.pos_pubs[leg].publish(Float64MultiArray(data=read_pos))

                # 关键：仅当接收到该腿的目标数据后，才执行写入操作
                if self.received_des[leg] and self.power_on:
                    target_pos = self.des_pos[leg]
                    for i in range(3):
                        servo_id = ids[2-i]
                        cmd = self.rad_to_servo(target_pos[i])
                        self.control.set_servo_position(servo_id, cmd, self.command_duration_ms)

        finally:
            self._lock.release()


if __name__ == '__main__':
    rospy.init_node('servo_side_node')
    ServoSideNode()
    rospy.spin()
