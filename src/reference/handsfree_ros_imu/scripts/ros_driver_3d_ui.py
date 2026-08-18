#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Circle
import matplotlib.animation as animation
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

# 全局方向调整因子
roll_direction = 1
pitch_direction = 1  
yaw_direction = 1   
yaw_offset = 0

class IMUVisualizer:
    def __init__(self):
        rospy.init_node("display_3D_visualization_node")
        
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
        
        # 订阅IMU话题
        self.sub = rospy.Subscriber('/handsfree/imu', Imu, self.imu_callback)
        
        # 数据存储
        self.roll = self.pitch = self.yaw = 0
        self.lin_acc = [0, 0, 0]
        self.ang_vel = [0, 0, 0]
        self.data_updated = False
        
        # 动画控制
        self.ani = animation.FuncAnimation(
            self.fig, self.update_animation, interval=50, blit=False
        )
        
        # 键盘事件
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        plt.tight_layout()
        plt.subplots_adjust(top=0.9)
        plt.show()
        
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
        self.ax1.text(0, 1.6, 0, "Y", color='green', fontsize=12)
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
            [3,0,4,7]   # 左面
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

    def imu_callback(self, imu_msg):
        """IMU数据回调函数"""
        global roll_direction, pitch_direction, yaw_direction
        
        try:
            quaternion = (
                imu_msg.orientation.x,
                imu_msg.orientation.y,
                imu_msg.orientation.z,
                imu_msg.orientation.w
            )
            
            # 提取欧拉角
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            
            # 应用方向调整 - yaw和pitch默认反转
            self.roll = roll * roll_direction
            self.pitch = pitch * pitch_direction  # 默认反转pitch方向
            self.yaw = yaw * yaw_direction  # 默认反转yaw方向
            
            # 保存加速度和角速度
            self.lin_acc = [
                imu_msg.linear_acceleration.x,
                imu_msg.linear_acceleration.y,
                imu_msg.linear_acceleration.z
            ]
            self.ang_vel = [
                imu_msg.angular_velocity.x,
                imu_msg.angular_velocity.y,
                imu_msg.angular_velocity.z
            ]
            
            # 设置数据更新标志
            self.data_updated = True
            
        except Exception as e:
            rospy.logerr("Error processing IMU data: %s", str(e))

    def update_animation(self, frame):
        """更新动画帧"""
        global yaw_offset
        
        # 只有在有新数据时才更新可视化
        if not self.data_updated:
            return []
        
        # 应用偏航偏移
        yaw_display = self.yaw + yaw_offset
        
        # 更新3D平台
        self.update_3d_platform(self.roll, self.pitch, yaw_display)
        
        # 更新2D指示器
        self.update_indicators(self.roll, self.pitch, yaw_display)
        
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
        
        # 计算箭头位置（长度增加到1.0）
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
        """更新2D指示器 - 修复方向问题"""
        # 滚转指示器
        roll_line_x = -0.5 + 0.15 * math.sin(roll)
        roll_line_y = 0.3 + 0.15 * math.cos(roll)
        self.roll_line.set_data([-0.5, roll_line_x], [0.3, roll_line_y])
        
        # 俯仰指示器 - 修复方向问题
        # 使用正数dy使箭头指向上方
        pitch_dx = 0.15 * math.cos(pitch)
        pitch_dy = -0.15 * math.sin(pitch)  # 正数，指向上方
        self.pitch_arrow.set_UVC(pitch_dx, pitch_dy)
        
        # 偏航指示器
        yaw_x = 0.15 * math.sin(-yaw)
        yaw_y = 0.15 * math.cos(-yaw)
        self.yaw_arrow.set_data([0, yaw_x], [-0.4, -0.4 + yaw_y])
        
        # 更新文本
        rad2deg = 180.0 / math.pi
        self.roll_text.set_text(
            "{:.2f} deg / {:.2f} rad".format(roll * rad2deg, roll)
        )
        self.pitch_text.set_text(
            "{:.2f} deg / {:.2f} rad".format(pitch * rad2deg, pitch)
        )
        self.yaw_text.set_text(
            "{:.2f} deg / {:.2f} rad".format(yaw * rad2deg, yaw)
        )
        
        self.lin_acc_text.set_text(
            "X: {:.2f}   Y: {:.2f}   Z: {:.2f}".format(
                self.lin_acc[0], self.lin_acc[1], self.lin_acc[2]
            )
        )
        
        self.ang_vel_text.set_text(
            "X: {:.2f}   Y: {:.2f}   Z: {:.2f}".format(
                self.ang_vel[0], self.ang_vel[1], self.ang_vel[2]
            )
        )

    def on_key_press(self, event):
        """处理键盘事件"""
        global yaw_offset, roll_direction, pitch_direction, yaw_direction
        
        # Yaw对齐功能
        if event.key == 'a' or event.key == 'A':
            yaw_offset = -self.yaw
            rospy.loginfo("Yaw aligned! New offset: %.2f rad", yaw_offset)
        
        # 方向反转控制
        if event.key == 'r' or event.key == 'R':
            roll_direction *= -1
            rospy.loginfo("Roll direction reversed: %d", roll_direction)
            
        if event.key == 'p' or event.key == 'P':
            pitch_direction *= -1
            rospy.loginfo("Pitch direction reversed: %d", pitch_direction)
            
        if event.key == 'y' or event.key == 'Y':
            yaw_direction *= -1
            rospy.loginfo("Yaw direction reversed: %d", yaw_direction)
            
        # 重置视图
        if event.key == 'c' or event.key == 'C':
            self.ax1.view_init(elev=30, azim=45)
            rospy.loginfo("3D view reset")

if __name__ == "__main__":
    try:
        rospy.loginfo("Starting IMU 3D Visualization...")
        rospy.loginfo("Press 'A' to align yaw axis")
        rospy.loginfo("Press 'R' to reverse roll direction")
        rospy.loginfo("Press 'P' to reverse pitch direction")
        rospy.loginfo("Press 'Y' to reverse yaw direction")
        rospy.loginfo("Press 'C' to reset 3D view")
        rospy.loginfo("Default: Pitch and Yaw reversed for proper orientation")
        
        visualizer = IMUVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down IMU visualization")