#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
File: python_driver_b6.py
Title: IMU Serial Reader & Parser (Python 2/3)
Deps: pyserial  (pip install pyserial)

Usage (Linux):
  python python_driver_b6.py --port /dev/ttyUSB0 --baud 921600

Usage (Windows):
  python python_driver_b6.py --port COM5 --baud 921600

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

# 校验
def checkSum(list_data, check_data):
    return sum(list_data) & 0xff == check_data

# 16 进制转 ieee 浮点数
def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))

# 处理串口数据
def handleSerialData(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag, temperature
    if python_version == '2':
        buff[key] = ord(raw_data)
    if python_version == '3':
        buff[key] = raw_data

    key += 1
    if buff[0] != 0x55:
        key = 0
        return
    if key < 11:  # 根据数据长度位的判断, 来获取对应长度数据
        return
    else:
        data_buff = list(buff.values())  # 获取字典所有 value

        if buff[1] == 0x51 and pub_flag[0]:
            if checkSum(data_buff[0:10], data_buff[10]):
                # acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * -9.8 for i in range(0, 3)]
                ax_ay_az_temp = struct.unpack('<hhhh', bytearray(data_buff[2:10]))
                acceleration = [ax_ay_az_temp[i] / 32768.0 * 16 * -9.8 for i in range(3)]
                temperature = ax_ay_az_temp[3] / 100.0  # ℃
            else:
                print('0x51 Verification failed')
            pub_flag[0] = False

        elif buff[1] == 0x52 and pub_flag[1]:
            if checkSum(data_buff[0:10], data_buff[10]):
                angularVelocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(0, 3)]
            else:
                print('0x52 Verification failed')
            pub_flag[1] = False

        elif buff[1] == 0x53 and pub_flag[2]:
            if checkSum(data_buff[0:10], data_buff[10]):
                angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)]
            else:
                print('0x53 Verification failed')
            pub_flag[2] = False

        else:
            print("The data processing class does not provide a parsing for the " + str(buff[1]) + "or data error.")
            buff = {}
            key = 0

        buff = {}
        key = 0
        if pub_flag[0] == True or pub_flag[1] == True or pub_flag[2] == True:
            return
        pub_flag[0] = pub_flag[1] = pub_flag[2] = True

        print('''
System Status: Temperature: %.2f℃

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
------
''' % (temperature, 
            acceleration[0], acceleration[1], acceleration[2],
            angularVelocity[0], angularVelocity[1], angularVelocity[2],
            angle_degree[0], angle_degree[1], angle_degree[2]))

key = 0
flag = 0
buff = {}
temperature = 0
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]
pub_flag = [True, True, True]


def _list_candidate_ports():
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
            # 兼容不同 pyserial 版本的 in_waiting
            try:
                buff_count = imu_serial.in_waiting
            except AttributeError:
                buff_count = imu_serial.inWaiting()

            if buff_count > 0:
                buff_data = imu_serial.read(buff_count)
                for i in range(0, buff_count):
                    handleSerialData(buff_data[i])
            else:
                time.sleep(0.0001)

        except (serial.SerialException, OSError, IOError) as e:
            print("exception:" + str(e))
            print("IMU loses connection, poor contact, or disconnection")
            imu_serial, current_port = _reconnect(imu_serial, port, baudrate, retry_delay=1.0)
            # 重连成功后继续 while True 循环

        except KeyboardInterrupt:
            try:
                if imu_serial is not None:
                    imu_serial.close()
            except Exception:
                pass
            break
