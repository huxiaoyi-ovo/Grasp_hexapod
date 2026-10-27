#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
File: python_driver_a9.py
Title: IMU Serial Reader & Parser (Python 2/3)
Deps: pyserial  (pip install pyserial)

Usage (Linux):
  python python_driver_a9.py --port /dev/ttyUSB0 --baud 921600

Usage (Windows):
  python python_driver_a9.py --port COM5 --baud 921600

Args:
  --port <device>   default: /dev/ttyUSB0
  --baud <rate>     default: 921600
"""

import serial
import struct
import platform
import serial.tools.list_ports
import math
import time
import argparse

# 查找 ttyUSB* 设备
def find_ttyUSB():
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device or 'ACM' in port.device]
    print('There are currently {} serial port devices connected to the computer, totaling {} in number: {}'.format('USB', len(posts), posts))

# crc 校验
def checkSum(list_data, check_data):
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

# 16 进制转 ieee 浮点数
def hex_to_ieee(raw_data):
    ieee_data = []
    raw_data.reverse()
    for i in range(0, len(raw_data), 4):
        data2str = (hex(raw_data[i] | 0xff00)[4:6] +
                    hex(raw_data[i + 1] | 0xff00)[4:6] +
                    hex(raw_data[i + 2] | 0xff00)[4:6] +
                    hex(raw_data[i + 3] | 0xff00)[4:6])
        if python_version == '2':
            ieee_data.append(struct.unpack('>f', data2str.decode('hex'))[0])
        if python_version == '3':
            ieee_data.append(struct.unpack('>f', bytes.fromhex(data2str))[0])
    ieee_data.reverse()
    return ieee_data

# 处理串口数据
def handleSerialData(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag, timestamp
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
        data_buff = list(buff.values())  # 获取字典所以 value

        if buff[2] == 0x2c and pub_flag[0]:
            if checkSum(data_buff[2:47], data_buff[47:49]):
                ts_us = ((data_buff[10] & 0xFF) << 24) | ((data_buff[9] & 0xFF) << 16) | ((data_buff[8] & 0xFF) << 8) | (data_buff[7] & 0xFF)
                timestamp = ts_us / 1e6  # -> 秒(float)
                # timestamp = (data_buff[6] & 0xFF) << 24 | (data_buff[5] & 0xFF) << 16 | (data_buff[4] & 0xFF) << 8 | (data_buff[3] & 0xFF) / 1e6
                data = hex_to_ieee(data_buff[7:47])
                angularVelocity = data[1:4]
                acceleration = data[4:7]
                magnetometer = data[7:10]
            else:
                print('0x2c Verification failed')
            pub_flag[0] = False
        elif buff[2] == 0x14 and pub_flag[1]:
            if checkSum(data_buff[2:23], data_buff[23:25]):
                data = hex_to_ieee(data_buff[7:23])
                angle_degree = data[1:4]
            else:
                print('0x14 Verification failed')
            pub_flag[1] = False
        else:
            print("The data processing class does not provide a parsing for the " + str(buff[2]) + "or data error.")
            buff = {}
            key = 0

        buff = {}
        key = 0
        if pub_flag[0] == True or pub_flag[1] == True:
            return
        pub_flag[0] = pub_flag[1] = True

        acc_k = math.sqrt(acceleration[0] ** 2 + acceleration[1] ** 2 + acceleration[2] ** 2)

        print('''
Timestamp : %f s

Acceleration(m/s²):
    x-axis : %.2f
    y-axis : %.2f
    z-axis : %.2f

Angular velocity(rad/s):
    x-axis : %.2f
    y-axis : %.2f
    z-axis : %.2f

Euler angle(deg):
    x-axis : %.2f
    y-axis : %.2f
    z-axis : %.2f

Magnetic field:
    x-axis : %.2f
    y-axis : %.2f
    z-axis : %.2f
------
''' % (timestamp, 
        acceleration[0] * 9.8 / acc_k, acceleration[1] * 9.8 / acc_k, acceleration[2] * 9.8 / acc_k,
       angularVelocity[0], angularVelocity[1], angularVelocity[2],
       angle_degree[0], angle_degree[1], angle_degree[2],
       magnetometer[0], magnetometer[1], magnetometer[2]
      ))


key = 0
flag = 0
buff = {}
timestamp = 0
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]
pub_flag = [True, True]

def _list_candidate_ports():
    """列出现有 ACM/USB 端口"""
    return [p.device for p in serial.tools.list_ports.comports()
            if ('ACM' in p.device or 'USB' in p.device)]

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
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0",
                        help="Specify the IMU serial port number, for example /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=921600,
                        help="Specify the serial baud rate, e.g. 921600 / 115200")
    args = parser.parse_args()

    find_ttyUSB()
    port = args.port
    baudrate = args.baud

    imu_serial, current_port = _open_serial_with_retry(port, baudrate, retry_delay=1.0)

    while True:
        try:
 
            try:
                buff_count = imu_serial.in_waiting
            except AttributeError:
                buff_count = imu_serial.inWaiting()

            if buff_count > 0:
                buff_data = imu_serial.read(buff_count)
                for i in range(0, buff_count):
                    # 在 Python3 中，bytes 索引是 int；handleSerialData 已适配
                    handleSerialData(buff_data[i])
            else:
                time.sleep(0.0001)

        except (serial.SerialException, OSError, IOError) as e:
            # 典型场景：设备拔出、端口消失、I/O 错误等
            print("exception:" + str(e))
            print("IMU loses connection, poor contact, or disconnection")
            imu_serial, current_port = _reconnect(imu_serial, port, baudrate, retry_delay=1.0)
            # 重连成功后继续 while 循环

        except KeyboardInterrupt:
            try:
                if imu_serial is not None:
                    imu_serial.close()
            except Exception:
                pass
            break
