#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
File: python_driver_tb100.py
Title: IMU Serial Reader & Parser (Python 2/3)
Deps: pyserial  (pip install pyserial)

Usage (Linux):
  python python_driver_tb100.py --port /dev/ttyACM0 --baud 921600

Usage (Windows):
  python python_driver_tb100.py --port COM5 --baud 921600

Args:
  --port <device>   default: /dev/ttyACM0
  --baud <rate>     default: 921600
"""

import serial
import struct
import platform
import serial.tools.list_ports
import time
import argparse

# 查找 ttyUSB* 设备
def find_ttyUSB():
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device or 'ACM' in port.device]
    print('There are currently {} serial port devices connected to the computer, totaling {} in number: {}'.format('USB', len(posts), posts))

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
    
    # 头与长度检查
    if packet[0] != 0xAA or packet[1] != 0x55:
        print("\033[31mInvalid packet header\033[0m")
        return False
    pkt_len = packet[2]
    if pkt_len != 0x44:
        print("\033[31mInvalid packet length: {}\033[0m".format(pkt_len))
        return False

    # CRC校验，设备发送低字节在前、高字节在后
    calc_crc = crc16_modbus(packet[2:71])
    recv_crc_lo = packet[71]
    recv_crc_hi = packet[72]
    recv_crc = (recv_crc_hi << 8) | recv_crc_lo

    if recv_crc != calc_crc:
        print(
            "\033[31mCRC mismatch → recv={:04X}, calc={:04X}, "
            "head={:02X}{:02X}, len={:02X}, tail={:02X}{:02X}\033[0m".format(
                recv_crc, calc_crc, packet[0], packet[1], packet[2], packet[71], packet[72]
            )
        )
        return False
    # 解析数据包内容 (68字节)
    data = packet[7:71]

    # 1. 解析时间戳 (4字节)。毫秒
    timestamp = struct.unpack('<I', bytes(data[0:4]))[0]  # uint32 小端
    timestamp_s = timestamp / 1e6

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

    print('''
Timestamp : %f s
System Status: %s | Temperature: %.2f℃ | Update Frequency: %.2f Hz

Euler angle(deg):
    x-axis : %.2f
    y-axis : %.2f
    z-axis : %.2f

Quaternion:
    qx : %.2f
    qy : %.2f
    qz : %.2f
    qw : %.2f

Angular velocity (rad/s):
    x-axis : %.2f
    y-axis : %.2f
    z-axis : %.2f

Acceleration (m/s^2):
    x-axis : %.2f
    y-axis : %.2f
    z-axis : %.2f

Magnetic field (uT):
    x-axis : %.2f
    y-axis : %.2f
    z-axis : %.2f
------
        ''' % (timestamp_s,
            sys_status, temperature, update_rate,
            roll, pitch, yaw,
            q1, q2, q3, q4,
            gyro_x, gyro_y, gyro_z,
            accel_x * 9.8, accel_y * 9.8, accel_z * 9.8,
            mag_x, mag_y, mag_z
        ))

packet_buffer = bytearray()
packet_size = 73  # 完整数据包大小

def _list_candidate_ports():
    """返回当前可用的 ACM/USB 端口列表（字符串数组）。"""
    return [p.device for p in serial.tools.list_ports.comports() if ('ACM' in p.device or 'USB' in p.device)]

def _choose_port(preferred):
    ports = _list_candidate_ports()
    return preferred if (preferred and preferred in ports) else None

def _open_serial_with_retry(preferred_port, baudrate, retry_delay=1.0):
    while True:
        find_ttyUSB()
        port_to_try = _choose_port(preferred_port)
        if port_to_try is None:
            print("\033[33mWait for the specified port to appear:% s (retry after % s s))\033[0m" % (preferred_port, retry_delay))
            time.sleep(retry_delay)
            continue

        try:
            ser = serial.Serial(port=port_to_try, baudrate=baudrate, timeout=0.5)
            if not ser.is_open:
                ser.open()
            print("\033[32mSerial port successfully opened\033[0m (%s @ %d)" % (port_to_try, baudrate))
            return ser, port_to_try
        except Exception as e:
            print("\033[31mFailed to open serial port (%s): %s\033[0m" % (port_to_try, str(e)))
            time.sleep(retry_delay)

def _reconnect(imu_serial, preferred_port, baudrate, retry_delay=1.0):
    """关闭旧串口并进入重连逻辑；返回 (new_ser, real_port)。"""
    try:
        if imu_serial is not None:
            imu_serial.close()
    except Exception as e:
        print(e)
    print("\033[33mSerial port disconnected or abnormal, automatic reconnection begins\033[0m")
    return _open_serial_with_retry(preferred_port, baudrate, retry_delay)

if __name__ == "__main__":
    python_version = platform.python_version()[0]

    parser = argparse.ArgumentParser(description="IMU Serial Port Param Reading Program")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0",
                        help="Specify the IMU serial port number, for example /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=921600,
                    help="Specify the serial baud rate, e.g. 921600 / 115200")
    args = parser.parse_args()

    find_ttyUSB()
    port = args.port
    baudrate = args.baud

    imu_serial, current_port = _open_serial_with_retry(port, baudrate, retry_delay=1.0)

    try:
        while True:
            try:
                # 读取串口数据
                n_wait = imu_serial.in_waiting if hasattr(imu_serial, "in_waiting") else imu_serial.inWaiting()
                data = imu_serial.read(n_wait)

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
                            process_packet(packet)
                else:
                    # 空读，稍作休眠以降低 CPU
                    time.sleep(0.0001)

            except (serial.SerialException, OSError, IOError) as e:
                print("\033[31mSerial port error: %s\033[0m" % str(e))
                # === Hotplug additions: 进入自动重连 ===
                imu_serial, current_port = _reconnect(imu_serial, port, baudrate, retry_delay=1.0)
                # 进入下一轮 while True，继续读新串口

            except KeyboardInterrupt:
                print("\nUser interrupted, ready to exit")
                break

    finally:
        # 关闭串口
        try:
            if imu_serial is not None:
                imu_serial.close()
        except Exception as e:
            print(e)
        imu_serial.loginfo("IMU driver turned off")
