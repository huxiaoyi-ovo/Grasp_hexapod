#!/usr/bin/env python
# -*- coding:utf-8 -*-
import serial
import struct
import rospy
import serial.tools.list_ports
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

# 查找 ttyUSB* 设备
def find_ttyUSB():
    print('The default serial port for IMU is /dev/ttyUSB0. If multiple serial devices are recognized, please modify the serial port in the launch file')
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device or 'ACM' in port.device]
    print("Currently connected {} serial devices: {} found — {}".format('USB', len(posts), posts))

# CRC16校验 (Modbus)
def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF  # 保证 16 位

# 处理完整数据包 (73字节)
def process_packet(packet):
    global imu_msg, mag_msg
    
    # 头与长度检查
    if packet[0] != 0xAA or packet[1] != 0x55:
        rospy.logwarn("Invalid packet header")
        return False
    pkt_len = packet[2]
    if pkt_len != 0x44:
        rospy.logwarn("Invalid packet length: %d", pkt_len)
        return False

    # CRC校验，设备发送低字节在前、高字节在后
    calc_crc = crc16_modbus(packet[2:71])
    recv_crc_lo = packet[71]
    recv_crc_hi = packet[72]
    recv_crc = (recv_crc_hi << 8) | recv_crc_lo

    if recv_crc != calc_crc:
        rospy.logwarn(
            "CRC mismatch (recv=%04X, calc=%04X) head=%02X%02X len=%02X tail=%02X%02X",
            recv_crc, calc_crc, packet[0], packet[1], packet[2], packet[71], packet[72]
        )
        return False
    
    # 解析数据包内容 (68字节)
    data = packet[7:71]
    
    # 1. 解析时间戳 (4字节)
    timestamp = struct.unpack('<I', bytes(data[0:4]))[0]  # uint32 小端
    
    # 2. 解析系统状态 (2字节)
    sys_status = struct.unpack('<H', bytes(data[4:6]))[0] & 0x07  # 只取低3位
    
    # 3. 解析欧拉角 (滚转、俯仰、航向)
    roll = struct.unpack('<h', bytes(data[6:8]))[0] * 0.01   # int16 0.01°
    pitch = struct.unpack('<h', bytes(data[8:10]))[0] * 0.01  # int16 0.01°
    yaw = struct.unpack('<H', bytes(data[10:12]))[0] * 0.01   # uint16 0.01°
    # print(roll,pitch,yaw)
    
    # 4. 解析四元数 (Q1-Q4)
    q4 = struct.unpack('<i', bytes(data[12:16]))[0] * 1e-7  # int32 1e-7
    q1 = struct.unpack('<i', bytes(data[16:20]))[0] * 1e-7
    q2 = struct.unpack('<i', bytes(data[20:24]))[0] * 1e-7
    q3 = struct.unpack('<i', bytes(data[24:28]))[0] * 1e-7
    # print(q1,q2,q3,q4)
    
    # 5. 解析陀螺仪数据 (rad/s)
    gyro_x = struct.unpack('<i', bytes(data[28:32]))[0] * 0.00001  # int32 0.00001 rad/s
    gyro_y = struct.unpack('<i', bytes(data[32:36]))[0] * 0.00001
    gyro_z = struct.unpack('<i', bytes(data[36:40]))[0] * 0.00001
    # print(gyro_x,gyro_y,gyro_z)
    
    # 6. 解析加速度计数据 (g)
    accel_x = struct.unpack('<i', bytes(data[40:44]))[0] * 0.00001  # int32 0.00001g
    accel_y = struct.unpack('<i', bytes(data[44:48]))[0] * 0.00001
    accel_z = struct.unpack('<i', bytes(data[48:52]))[0] * 0.00001
    # print(accel_x,accel_y,accel_z)
    
    # 7. 解析磁力计数据
    mag_x = struct.unpack('<h', bytes(data[52:54]))[0] * 0.001  # int16 0.001
    mag_y = struct.unpack('<h', bytes(data[54:56]))[0] * 0.001
    mag_z = struct.unpack('<h', bytes(data[56:58]))[0] * 0.001
    # print(mag_x,mag_y,mag_z)
    
    # 8. 解析温度和更新频率
    temperature = struct.unpack('b', bytes(data[58:59]))[0]  # int8 ℃
    update_rate = struct.unpack('B', bytes(data[59:60]))[0] * 10  # uint8 10Hz
    # print(temperature,update_rate)
    
    # 填充ROS消息
    stamp = rospy.get_rostime()
    
    # IMU消息
    imu_msg.header.stamp = stamp
    imu_msg.header.frame_id = "imu_link"
    
    # 设置四元数
    imu_msg.orientation.x = q1
    imu_msg.orientation.y = q2
    imu_msg.orientation.z = q3
    imu_msg.orientation.w = q4
    
    # 设置角速度 (rad/s)
    imu_msg.angular_velocity.x = gyro_x
    imu_msg.angular_velocity.y = gyro_y
    imu_msg.angular_velocity.z = gyro_z
    
    # 设置线加速度 (m/s²)
    imu_msg.linear_acceleration.x = accel_x * 9.8
    imu_msg.linear_acceleration.y = accel_y * 9.8
    imu_msg.linear_acceleration.z = accel_z * 9.8
    
    # 磁力计消息
    mag_msg.header.stamp = stamp
    mag_msg.header.frame_id = "imu_link"
    mag_msg.magnetic_field.x = mag_x
    mag_msg.magnetic_field.y = mag_y
    mag_msg.magnetic_field.z = mag_z
    
    return True

# 全局变量
imu_msg = Imu()
mag_msg = MagneticField()
packet_buffer = bytearray()
packet_size = 73  # 完整数据包大小

if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node("imu_driver")
    
    # 获取参数
    port = rospy.get_param("~port", "/dev/ttyACM0")
    baudrate = rospy.get_param("~baudrate", 921600)
    
    # 查找可用串口
    find_ttyUSB()
    
    # 创建发布者
    imu_pub = rospy.Publisher("handsfree/imu", Imu, queue_size=10)
    mag_pub = rospy.Publisher("handsfree/mag", MagneticField, queue_size=10)
    
    # 初始化串口对象
    imu_serial = None
    
    while not rospy.is_shutdown():
        if imu_serial is None:
            try:
                # 打开串口
                imu_serial = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
                if imu_serial.isOpen():
                    rospy.loginfo("\033[32mSerial port successfully opened: %s\033[0m", port)
                else:
                    imu_serial.open()
                    rospy.loginfo("\033[32mSerial port successfully opened: %s\033[0m", port)
            except Exception as e:
                rospy.logerr("Serial port opening failed:  %s, reason: %s", port, str(e))
                if imu_serial is not None:
                    imu_serial.close()
                imu_serial = None
                rospy.sleep(1.0)  # 等一秒再重试
                continue
        
        try:
            # 读取串口数据
            data = imu_serial.read(imu_serial.inWaiting())
            # print(type(data)) 
            # print("Method 1:", binascii.hexlify(data))
            if data:
                # 将数据添加到缓冲区
                packet_buffer.extend(data)
                
                # 处理完整数据包
                while len(packet_buffer) >= packet_size:
                    # 查找包头 (0xAA 0x55)
                    start_idx = -1
                    for i in range(len(packet_buffer) - 1):
                        if packet_buffer[i] == 0xAA and packet_buffer[i+1] == 0x55:
                            start_idx = i
                            break
                    
                    if start_idx == -1:
                        # 没有找到包头，清空缓冲区
                        packet_buffer = bytearray()
                        break
                    
                    if start_idx > 0:
                        # 移除包头前的无效数据
                        packet_buffer = packet_buffer[start_idx:]
                    
                    if len(packet_buffer) >= packet_size:
                        # 提取完整数据包
                        packet = packet_buffer[:packet_size]
                        packet_buffer = packet_buffer[packet_size:]
                        
                        # 处理数据包
                        if process_packet(packet):
                            # 发布消息
                            imu_pub.publish(imu_msg)
                            mag_pub.publish(mag_msg)
            else:
                rospy.sleep(0.0001) 
            
        except (serial.SerialException, IOError) as e:
            rospy.logerr("Serial port error: %s, Preparing to reconnect", str(e))
            if imu_serial is not None:
                imu_serial.close()
            imu_serial = None
            rospy.sleep(1.0)
            continue
        except rospy.ROSInterruptException:
            # 关闭串口
            if imu_serial is not None:
                imu_serial.close()
            rospy.loginfo("IMU driver turned off")
    
    # 关闭串口
    if imu_serial is not None:
        imu_serial.close()
    rospy.loginfo("IMU driver turned off")
    