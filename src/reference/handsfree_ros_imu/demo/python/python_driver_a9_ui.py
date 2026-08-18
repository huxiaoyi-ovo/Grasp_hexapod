#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
File: python_driver_a9_ui.py
Title: IMU Serial Reader & Parser (Python 2/3)
Deps: pyserial  (pip install pyserial)

Usage (Linux):
  python python_driver_a9_ui.py --port /dev/ttyUSB0 --baud 921600

Usage (Windows):
  python python_driver_a9_ui.py --port COM5 --baud 921600

Args:
  --port <device>   default: /dev/ttyUSB0
  --baud <rate>     default: 921600
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Circle
import matplotlib.animation as animation
import serial
import serial.tools.list_ports
import struct
import platform
import argparse

# 全局方向调整因子
roll_direction = 1
pitch_direction = -1  
yaw_direction = -1   
yaw_offset = 0

# 串口相关全局变量
key = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]
pub_flag = [True, True]
python_version = platform.python_version()[0]

class IMUVisualizer:
    def __init__(self):
        # 创建图形界面 - 分左右两部分
        self.fig = plt.figure(figsize=(14, 7))
        self.fig.suptitle("IMU 3D Visualization", fontsize=16)
        
        # 左侧：3D视图
        self.ax1 = self.fig.add_subplot(121, projection='3d')
        self.ax1.set_title("3D Platform View")
        
        # 右侧：2D指示器视图
        self.ax2 = self.fig.add_subplot(122)
        self.ax2.set_aspect('equal')
        self.ax2.set_title("Attitude Indicators")
        self.ax2.axis('off')
        
        # 设置3D视图范围
        self.ax1.set_xlim(-1.5, 1.5)
        self.ax1.set_ylim(-1.5, 1.5)
        self.ax1.set_zlim(-1.5, 1.5)
        self.ax1.set_xlabel('X (forward)')
        self.ax1.set_ylabel('Y (left)')
        self.ax1.set_zlabel('Z (up)')
        self.ax1.grid(True)
        
        # 设置2D视图范围
        self.ax2.set_xlim(-1, 1)
        self.ax2.set_ylim(-1, 1)
        
        # 初始化固定坐标系
        self.init_fixed_coordinate_system()
        
        # 初始化平台
        self.init_platform()
        
        # 初始化2D指示器
        self.init_indicators()
        
        # 数据存储
        self.roll_raw = 0
        self.pitch_raw = 0
        self.yaw_raw = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.lin_acc = [0, 0, 0]
        self.ang_vel = [0, 0, 0]
        self.data_updated = False
        
        # 初始化串口
        self.init_serial()
        
        # 动画控制
        self.ani = animation.FuncAnimation(
            self.fig, self.update_animation, interval=50, blit=False
        )
        
        # 键盘事件
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        plt.tight_layout()
        plt.subplots_adjust(top=0.9)
        
        print("IMU 3D Visualization started")
        print("Press 'A' to align yaw axis")
        print("Press 'R' to reverse roll direction")
        print("Press 'P' to reverse pitch direction")
        print("Press 'Y' to reverse yaw direction")
        print("Press 'C' to reset 3D view")
        print("Default: Pitch and Yaw reversed for proper orientation")

    def init_serial(self):
        """初始化串口连接"""
        # 查找可用串口
        posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device or 'ACM' in port.device]
        print('Available USB ports: {}'.format(posts))
        
        if not posts:
            print("No USB ports found!")
            exit(1)
        parser = argparse.ArgumentParser(description="IMU Serial Port Param Reading Program")
        parser.add_argument("--port", type=str, default="/dev/ttyUSB0",
                            help="Specify the IMU serial port number, for example /dev/ttyUSB0")
        parser.add_argument("--baud", type=int, default=921600,
                            help="Specify the serial baud rate, e.g. 921600 / 115200")
        args = parser.parse_args()
        
        port = args.port
        baudrate = args.baud
        
        try:
            self.imu_serial = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
            if self.imu_serial.isOpen():
                print("Serial port opened successfully")
            else:
                self.imu_serial.open()
                print("Serial port opened successfully")
        except Exception as e:
            print(e)
            print("Failed to open serial port")
            exit(1)

    def crc_check(self, list_data, check_data):
        """CRC校验"""
        data = bytearray(list_data)
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for i in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return hex(((crc & 0xff) << 8) + (crc >> 8)) == hex(check_data[0] << 8 | check_data[1])

    def hex_to_ieee(self, raw_data):
        """16进制转IEEE浮点数"""
        ieee_data = []
        raw_data.reverse()
        for i in range(0, len(raw_data), 4):
            data2str = hex(raw_data[i] | 0xff00)[4:6] + hex(raw_data[i + 1] | 0xff00)[4:6] + \
                       hex(raw_data[i + 2] | 0xff00)[4:6] + hex(raw_data[i + 3] | 0xff00)[4:6]
            if python_version == '2':
                ieee_data.append(struct.unpack('>f', data2str.decode('hex'))[0])
            if python_version == '3':
                ieee_data.append(struct.unpack('>f', bytes.fromhex(data2str))[0])
        ieee_data.reverse()
        return ieee_data

    def handle_serial_data(self, raw_data):
        global key, buff, angularVelocity, magnetometer, pub_flag
        
        if python_version == '2':
            buff[key] = ord(raw_data)
        if python_version == '3':
            buff[key] = raw_data

        key += 1
        if buff[0] != 0xaa:
            key = 0
            return
        if key < 3:
            return
        if buff[1] != 0x55:
            key = 0
            return
        if key < buff[2] + 5:  # 根据数据长度位的判断, 来获取对应长度数据
            return

        else:
            data_buff = list(buff.values())  # 获取字典所有value

            if buff[2] == 0x2c and pub_flag[0]:
                if self.crc_check(data_buff[2:47], data_buff[47:49]):
                    data = self.hex_to_ieee(data_buff[7:47])
                    angularVelocity = data[1:4]
                    
                    # 获取原始加速度数据
                    raw_acc = data[4:7]
                    
                    # 计算加速度向量的模
                    acc_k = math.sqrt(raw_acc[0]**2 + raw_acc[1]**2 + raw_acc[2]**2)
                    
                    # 应用重力补偿处理
                    if acc_k > 0:
                        self.acceleration = [
                            raw_acc[0] * 9.8 / acc_k,
                            raw_acc[1] * 9.8 / acc_k,
                            raw_acc[2] * 9.8 / acc_k
                        ]
                    else:
                        self.acceleration = raw_acc
                    
                    magnetometer = data[7:10]
                else:
                    print('CRC check failed')
                pub_flag[0] = False
            elif buff[2] == 0x14 and pub_flag[1]:
                if self.crc_check(data_buff[2:23], data_buff[23:25]):
                    data = self.hex_to_ieee(data_buff[7:23])
                    angle_degree = data[1:4]
                    
                    # 保存原始值（未经方向调整和偏移）
                    self.roll_raw = math.radians(angle_degree[0])
                    self.pitch_raw = math.radians(angle_degree[1])
                    self.yaw_raw = math.radians(angle_degree[2])
                    
                    # 应用方向调整和偏航偏移
                    self.roll = self.roll_raw * roll_direction
                    self.pitch = self.pitch_raw * pitch_direction
                    self.yaw = self.yaw_raw * yaw_direction + yaw_offset
                    
                    self.ang_vel = angularVelocity
                    self.data_updated = True
                else:
                    print('CRC check failed')
                pub_flag[1] = False
            else:
                print("Unhandled data type: " + str(buff[2]))
                print("or data error")
                buff = {}
                key = 0
                return

            buff = {}
            key = 0
            if pub_flag[0] == True or pub_flag[1] == True:
                return
            pub_flag[0] = pub_flag[1] = True

    def init_fixed_coordinate_system(self):
        """初始化固定坐标系"""
        origin = (0, 0, 0)
        
        # 绘制固定坐标系
        # X轴 (红色) - 向前
        self.ax1.quiver(
            origin[0], origin[1], origin[2],
            1.5, 0, 0, 
            color='red', arrow_length_ratio=0.1, alpha=0.6
        )
        # Y轴 (绿色) - 向左
        self.ax1.quiver(
            origin[0], origin[1], origin[2],
            0, 1.5, 0, 
            color='green', arrow_length_ratio=0.1, alpha=0.6
        )
        # Z轴 (蓝色) - 向上
        self.ax1.quiver(
            origin[0], origin[1], origin[2],
            0, 0, 1.5, 
            color='blue', arrow_length_ratio=0.1, alpha=0.6
        )
        
        # 添加坐标轴标签
        self.ax1.text(1.6, 0, 0, "X", color='red', fontsize=12)
        self.ax1.text(0, -1.6, 0, "Y", color='green', fontsize=12)
        self.ax1.text(0, 0, 1.6, "Z", color='blue', fontsize=12)

    def init_platform(self):
        """初始化平台"""
        # 顶点定义
        self.verts = np.array([
            [-0.6, 0.4, -0.2],  # 前左下
            [0.6, 0.4, -0.2],   # 前右下
            [0.6, -0.4, -0.2],  # 后右下
            [-0.6, -0.4, -0.2], # 后左下
            
            [-0.6, 0.4, 0.2],   # 前左上
            [0.6, 0.4, 0.2],    # 前右上
            [0.6, -0.4, 0.2],   # 后右上
            [-0.6, -0.4, 0.2]   # 后左上
        ])
        
        # 面定义
        self.faces = [
            [0,1,2,3],  # 底面
            [4,5,6,7],  # 顶面
            [0,1,5,4],  # 前面
            [1,2,6,5],  # 右面
            [2,3,7,6],  # 后面
            [3,0,4,7]   # yaw_direction左面
        ]
        
        # 创建平台
        self.platform_poly = Poly3DCollection(
            [self.verts[face] for face in self.faces], 
            facecolors='#FF7F7F',  # 淡红色
            edgecolors='k', 
            alpha=0.8
        )
        self.ax1.add_collection3d(self.platform_poly)
        
        # 平台中心
        self.platform_center = np.mean(self.verts, axis=0)
        
        # 平台坐标系箭头 - 增加长度到1.0使更明显
        # X轴 (红色)
        self.imu_x_arrow = self.ax1.quiver(
            self.platform_center[0], self.platform_center[1], self.platform_center[2],
            1.0, 0, 0, 
            color='red', arrow_length_ratio=0.15, linewidth=3
        )
        # Y轴 (绿色)
        self.imu_y_arrow = self.ax1.quiver(
            self.platform_center[0], self.platform_center[1], self.platform_center[2],
            0, 1.0, 0, 
            color='green', arrow_length_ratio=0.15, linewidth=3
        )
        # Z轴 (蓝色)
        self.imu_z_arrow = self.ax1.quiver(
            self.platform_center[0], self.platform_center[1], self.platform_center[2],
            0, 0, 1.0, 
            color='blue', arrow_length_ratio=0.15, linewidth=3
        )
        
        # 添加标签
        self.ax1.text(1.1, 0, 0, "X", color='red', fontsize=12, weight='bold')
        self.ax1.text(0, -1.1, 0, "Y", color='green', fontsize=12, weight='bold')
        self.ax1.text(0, 0, 1.1, "Z", color='blue', fontsize=12, weight='bold')

    def init_indicators(self):
        """初始化2D指示器"""
        # 滚转指示器
        self.roll_circle = Circle((-0.5, 0.3), 0.2, fill=False, edgecolor='red', linewidth=2)
        self.roll_line, = self.ax2.plot([-0.5, -0.5], [0.3, 0.5], 'r-', linewidth=2)
        self.ax2.add_patch(self.roll_circle)
        
        # 俯仰指示器 
        self.pitch_arrow = self.ax2.quiver(
            0.5, 0.3, 0, 0.2, color='g', scale=1, scale_units='xy', linewidth=2
        )
        
        # 偏航指示器
        self.yaw_circle = Circle((0, -0.4), 0.2, fill=False, edgecolor='blue', linewidth=2)
        self.yaw_arrow, = self.ax2.plot([0, 0], [-0.4, -0.2], 'b-', linewidth=2)
        self.ax2.add_patch(self.yaw_circle)
        
        # 标签和文本
        self.ax2.text(-0.5, 0.6, "Roll", ha='center', fontsize=12, weight='bold')
        self.ax2.text(0.5, 0.6, "Pitch", ha='center', fontsize=12, weight='bold')
        self.ax2.text(0, 0.0, "Yaw", ha='center', fontsize=12, weight='bold')
        
        self.roll_text = self.ax2.text(-0.5, 0.52, "-", ha='center', fontsize=10)
        self.pitch_text = self.ax2.text(0.5, 0.52, "-", ha='center', fontsize=10)
        self.yaw_text = self.ax2.text(0, -0.08, "-", ha='center', fontsize=10)
        
        self.ax2.text(0, 0.9, "Linear Acceleration (m/s^2)", ha='center', fontsize=10)
        self.ax2.text(0, -0.9, "Angular Velocity (rad/s)", ha='center', fontsize=10)
        self.lin_acc_text = self.ax2.text(0, 0.82, "X: 0.0   Y: 0.0   Z: 0.0", ha='center', fontsize=9)
        self.ang_vel_text = self.ax2.text(0, -0.98, "X: 0.0   Y: 0.0   Z: 0.0", ha='center', fontsize=9)
        
        # 方向标记
        directions = ['N', 'S', 'E', 'W', 'NE', 'NW', 'SE', 'SW']
        positions = [(0, -0.2), (0, -0.6), (0.24, -0.4), (-0.24, -0.4),
                     (0.18, -0.22), (-0.18, -0.22), (0.18, -0.58), (-0.18, -0.58)]
        for pos, text in zip(positions, directions):
            self.ax2.text(pos[0], pos[1], text, ha='center', color='purple', fontsize=9)
            
        # 方向反转控制提示
        self.ax2.text(-0.9, 1, "Controls:", fontsize=10)
        self.ax2.text(-0.9, 0.95, "A: Align Yaw", fontsize=9)
        self.ax2.text(-0.9, 0.9, "R: Reverse Roll", fontsize=9)
        self.ax2.text(-0.9, 0.85, "P: Reverse Pitch", fontsize=9)
        self.ax2.text(-0.9, 0.8, "Y: Reverse Yaw", fontsize=9)

    def update_animation(self, frame):
        """更新动画帧"""
        # 读取并处理串口数据
        try:
            buff_count = self.imu_serial.in_waiting
            if buff_count > 0:
                buff_data = self.imu_serial.read(buff_count)
                for i in range(0, buff_count):
                    self.handle_serial_data(buff_data[i])
        except Exception as e:
            print("Serial error: " + str(e))
            print("IMU disconnected")
            exit(0)
        
        # 只有在有新数据时才更新可视化
        if not self.data_updated:
            return []
        
        # 更新3D平台
        self.update_3d_platform(self.roll, self.pitch, self.yaw)
        
        # 更新2D指示器
        self.update_indicators(self.roll, self.pitch, self.yaw)
        
        # 重置数据更新标志
        self.data_updated = False
        
        return []

    def update_3d_platform(self, roll, pitch, yaw):
        """更新3D平台姿态"""
        # 计算旋转矩阵
        R_roll = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])
        
        R_pitch = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])
        
        R_yaw = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # 组合旋转
        R = np.dot(np.dot(R_yaw, R_pitch), R_roll)
        
        # 应用旋转到顶点
        rotated_verts = np.dot(R, (self.verts - self.platform_center).T).T + self.platform_center
        
        # 更新平台
        self.platform_poly.set_verts([rotated_verts[face] for face in self.faces])
        
        # 计算新的平台中心
        center = np.mean(rotated_verts, axis=0)
        
        # 计算箭头位置（长度增加到1.2）
        x_vec = np.dot(R, np.array([1.5, 0, 0]))
        y_vec = np.dot(R, np.array([0, 1.2, 0]))
        z_vec = np.dot(R, np.array([0, 0, 1.2]))
        
        # 更新箭头
        if hasattr(self, 'imu_x_arrow'):
            self.imu_x_arrow.remove()
        self.imu_x_arrow = self.ax1.quiver(
            center[0], center[1], center[2],
            x_vec[0], x_vec[1], x_vec[2],
            color='red', arrow_length_ratio=0.15, linewidth=3
        )
        
        if hasattr(self, 'imu_y_arrow'):
            self.imu_y_arrow.remove()
        self.imu_y_arrow = self.ax1.quiver(
            center[0], center[1], center[2],
            y_vec[0], y_vec[1], y_vec[2],
            color='green', arrow_length_ratio=0.15, linewidth=3
        )
        
        if hasattr(self, 'imu_z_arrow'):
            self.imu_z_arrow.remove()
        self.imu_z_arrow = self.ax1.quiver(
            center[0], center[1], center[2],
            z_vec[0], z_vec[1], z_vec[2],
            color='blue', arrow_length_ratio=0.15, linewidth=3
        )
        
        # 更新标签位置
        self.ax1.texts = []  # 清除旧标签
        self.ax1.text(
            center[0] + x_vec[0] * 1.1, 
            center[1] + x_vec[1] * 1.1, 
            center[2] + x_vec[2] * 1.1, 
            "X", color='red', fontsize=12, weight='bold'
        )
        self.ax1.text(
            center[0] + y_vec[0] * 1.1, 
            center[1] + y_vec[1] * 1.1, 
            center[2] + y_vec[2] * 1.1, 
            "Y", color='green', fontsize=12, weight='bold'
        )
        self.ax1.text(
            center[0] + z_vec[0] * 1.1, 
            center[1] + z_vec[1] * 1.1, 
            center[2] + z_vec[2] * 1.1, 
            "Z", color='blue', fontsize=12, weight='bold'
        )

    def update_indicators(self, roll, pitch, yaw):

        # 滚转指示器
        roll_line_x = -0.5 + 0.15 * math.sin(roll)
        roll_line_y = 0.3 + 0.15 * math.cos(roll)
        self.roll_line.set_data([-0.5, roll_line_x], [0.3, roll_line_y])
        
        # 俯仰指示器
        pitch_dx = 0.15 * math.cos(pitch)
        pitch_dy = -0.15 * math.sin(pitch)  # 正数，指向上方
        self.pitch_arrow.set_UVC(pitch_dx, pitch_dy)
        
        # 偏航指示器 - 反转Yaw方向以匹配视觉方向
        corrected_yaw = -yaw
        
        # 1. 角度归一化：将角度转换到[-π, π]区间
        normalized_yaw = math.atan2(math.sin(corrected_yaw), math.cos(corrected_yaw))
        
        # 2. 将弧度转换为角度并归一化到[-180°, 180°]
        angle_deg = math.degrees(normalized_yaw)
        
        # 3. 确保角度在[-180, 180]范围内
        if angle_deg > 180:
            angle_deg -= 360
        elif angle_deg < -180:
            angle_deg += 360
        
        # 4. 在±180°附近显示为180°而不是-180°（可选）
        if abs(angle_deg + 180) < 1:  # 接近-180°
            display_yaw = 180  # 显示为180°而不是-180°
        else:
            display_yaw = angle_deg
        
        # 绘制偏航指示器箭头
        yaw_x = 0.15 * math.sin(normalized_yaw)
        yaw_y = 0.15 * math.cos(normalized_yaw)
        self.yaw_arrow.set_data([0, yaw_x], [-0.4, -0.4 + yaw_y])
        
        # 更新文本
        rad2deg = 180.0 / math.pi
        
        # 滚转角度显示
        roll_deg = roll * rad2deg
        self.roll_text.set_text(
            "{:.2f} deg / {:.2f} rad".format(roll_deg, roll)
        )
        
        # 俯仰角度显示
        pitch_deg = pitch * rad2deg
        self.pitch_text.set_text(
            "{:.2f} deg / {:.2f} rad".format(pitch_deg, pitch)
        )
        
        # 偏航角度显示（使用归一化后的角度）
        self.yaw_text.set_text(
            "{:.2f} deg / {:.2f} rad".format(-display_yaw, -normalized_yaw)
        )
        
        # 显示处理后的加速度值
        self.lin_acc_text.set_text(
            "X: {:.2f}   Y: {:.2f}   Z: {:.2f}".format(
                self.acceleration[0], self.acceleration[1], self.acceleration[2]
            )
        )
        
        # 显示角速度值
        self.ang_vel_text.set_text(
            "X: {:.2f}   Y: {:.2f}   Z: {:.2f}".format(
                self.ang_vel[0], self.ang_vel[1], self.ang_vel[2]
            )
        )
        
        # 更新方向标记（可选）
        # 清除旧的方向标记（如果有的话）
        for text in self.ax2.texts:
            if text.get_text() in ['N', 'E', 'S', 'W', 'NE', 'NW', 'SE', 'SW']:
                text.remove()
        
        # 重新添加方向标记
        directions = ['N', 'S', 'E', 'W', 'NE', 'NW', 'SE', 'SW']
        positions = [(0, -0.2), (0, -0.6), (0.24, -0.4), (-0.24, -0.4),
                    (0.18, -0.22), (-0.18, -0.22), (0.18, -0.58), (-0.18, -0.58)]
        for pos, text in zip(positions, directions):
            self.ax2.text(pos[0], pos[1], text, ha='center', color='purple', fontsize=9)

    def on_key_press(self, event):
        """处理键盘事件"""
        global yaw_offset, roll_direction, pitch_direction, yaw_direction
        
        # Yaw对齐功能
        if event.key == 'a' or event.key == 'A':
            # 修复：使用当前原始偏航值计算偏移量
            yaw_offset = -self.yaw_raw * yaw_direction
            print("Yaw aligned! New offset: %.2f rad" % yaw_offset)
        
        # 方向反转控制
        if event.key == 'r' or event.key == 'R':
            roll_direction *= -1
            print("Roll direction reversed: %d" % roll_direction)
            
        if event.key == 'p' or event.key == 'P':
            pitch_direction *= -1
            print("Pitch direction reversed: %d" % pitch_direction)
            
        if event.key == 'y' or event.key == 'Y':
            yaw_direction *= -1
            print("Yaw direction reversed: %d" % yaw_direction)
            
        # 重置视图
        if event.key == 'c' or event.key == 'C':
            self.ax1.view_init(elev=30, azim=45)
            print("3D view reset")
            
        # 更新显示值以反映方向变化
        self.roll = self.roll_raw * roll_direction
        self.pitch = self.pitch_raw * pitch_direction
        self.yaw = self.yaw_raw * yaw_direction + yaw_offset

if __name__ == "__main__":
    visualizer = IMUVisualizer()
    plt.show()