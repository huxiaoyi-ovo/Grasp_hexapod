#!/usr/bin/env python3
# encoding: utf-8
import time
import serial
from threading import Lock
# import Jetson.GPIO as GPIO
try:
    from .hiwonder_servo_cmd import *
except ImportError:
    from hiwonder_servo_cmd import *

exception = None
# rx_pin = 17
# tx_pin = 27

# def port_as_write():
#     GPIO.output(tx_pin, 1)  # 拉高TX_CON 即 GPIO27
#     GPIO.output(rx_pin, 0)  # 拉低RX_CON 即 GPIO17

# def port_as_read():
#     GPIO.output(rx_pin, 1)  # 拉高RX_CON 即 GPIO17
#     GPIO.output(tx_pin, 0)  # 拉低TX_CON 即 GPIO27

# def port_init():
#     GPIO.setwarnings(False)
#     mode = GPIO.getmode()
#     if mode == 1 or mode is None:
#         GPIO.setmode(GPIO.BCM)
#     GPIO.setup(rx_pin, GPIO.OUT)  # 配置RX_CON 即 GPIO17 为输出
#     GPIO.output(rx_pin, 0)
#     GPIO.setup(tx_pin, GPIO.OUT)  # 配置TX_CON 即 GPIO27 为输出
#     GPIO.output(tx_pin, 1)

# port_init()
# port_as_write()

class servo_state:
    """保存单个舵机的估计状态。"""

    def __init__(self):
        """初始化默认状态。"""

        self.start_timestamp = time.time()
        self.end_timestamp = time.time()
        self.speed = 200
        self.goal = 500
        self.estimated_pos = 500

class HiwonderServoController:
    """通过串口读写 LX-15D 舵机。"""

    def __init__(self, port, baudrate):
        """打开串口。

        参数:
            port: 串口设备路径。
            baudrate: 串口波特率。
        """
        try:
            self.serial_mutex = Lock()
            self.ser = None
            self.timeout = 10
            self.ser = serial.Serial(port, baudrate, timeout=0.01)
            self.port_name = port
        except SerialOpenError:
            raise SerialOpenError(port, baudrate)

    def __del__(self):
        self.close()

    def close(self):
        """关闭当前串口。"""
        if self.ser:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.close()

    def __write_serial(self, data):
        self.ser.flushInput()
        self.ser.write(data)
        time.sleep(0.00034)

    def __read_response(self, servo_id):
        data = []
        try:
            data.extend(self.ser.read(4))
            if not data[0:2] == [0x55, 0x55]:
                raise Exception('Wrong packet prefix' + str(data[0:2]))
            data.extend(self.ser.read(data[3] - 1))
        except Exception as e:
            raise DroppedPacketError('Invalid response received from servo ' + str(servo_id) + ' ' + str(e))
        checksum = 255 - (sum(data[2: -1]) % 256)
        if not checksum == data[-1]:
            raise ChecksumError(servo_id, data, checksum)
        return data

    def read(self, servo_id, cmd):
        # 标准报头（0xFF, 0xFF, id, length）之后的字节数。
        length = 3  # instruction, address, size, checksum

        ##计算校验和
        checksum = 255 - ((servo_id + length + cmd) % 256)
        # 数据包：0x55  0x55  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packet = [0x55, 0x55, servo_id, length, cmd, checksum]
        
        with self.serial_mutex:
            try:
                self.__write_serial(packet)
                data = self.__read_response(servo_id)
            except (DroppedPacketError, ChecksumError):
                return []

        data.append(time.time())
        return data

    def write(self, servo_id, cmd, params):
        """向舵机写入一条指令。

        参数:
            servo_id: 舵机 ID。
            cmd: 指令字节。
            params: 指令参数字节。
        """
        # 标准报头（0xFF, 0xFF, id）之后的字节数。
        length = 3 + len(params)  # length, cmd, params, checksum
        # 校验和 = ~ ((ID + LENGTH + COMMAND + PARAM_1 + ... + PARAM_N) & 0xFF)
        checksum = 255 - ((servo_id + length + cmd + sum(params)) % 256)
        # 数据包：FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packet = [0x55, 0x55, servo_id, length, cmd]
        packet.extend(params)
        packet.append(checksum)
        with self.serial_mutex:
            self.__write_serial(packet)

    def get_servo_position(self, servo_id):
        response = self.read(servo_id, HIWONDER_SERVO_POS_READ)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching present position')
            return response[5] + (response[6] << 8)

    def get_servo_voltage(self, servo_id):
        response = self.read(servo_id, HIWONDER_SERVO_VIN_READ)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching supplied voltage')
            return response[5] + (response[6] << 8)

    def set_timeout(self, t=10):
        self.timeout = t

    def set_servo_id(self, oldid, newid):
        """修改舵机 ID。

        参数:
            oldid: 当前 ID。
            newid: 新 ID。
        """
        self.write(oldid, HIWONDER_SERVO_ID_WRITE, (newid, ))
    
    def get_servo_id(self, servo_id=None):
        """读取舵机 ID。

        参数:
            servo_id: 要查询的 ID；为空时广播查询。

        返回:
            舵机 ID；超时返回 None。
        """
        count = 0
        while True:
            count += 1
            response = None
            if servo_id is None:  # 总线上只能有一个舵机
                response = self.read(0xfe, HIWONDER_SERVO_ID_READ)
            else:
                response = self.read(servo_id, HIWONDER_SERVO_ID_READ)
            if response:
                count = 0
                self.exception_on_error(response[4], servo_id, 'fetching present position')
                return self.parse_result(response)
            if count > self.timeout:
                count = 0
                return None

    def set_servo_position(self, servo_id, position, duration=None):
        """让舵机转到指定脉冲位置。

        参数:
            servo_id: 舵机 ID。
            position: 目标脉冲。
            duration: 运动时间，单位 ms。
        """
        # print("id:{}, pos:{}, duration:{}".format(servo_id, position, duration))

        current_timestamp = time.time()
        if duration is None:
            duration = 20
        duration = 0 if duration < 0 else 30000 if duration > 30000 else duration
        position = 0 if position < 0 else 1000 if position > 1000 else position
        duration = int(duration)
        position = int(position)
        loVal = int(position & 0xFF)
        hiVal = int(position >> 8)
        loTime = int(duration & 0xFF)
        hiTime = int(duration >> 8)
        self.write(servo_id, HIWONDER_SERVO_MOVE_TIME_WRITE, (loVal, hiVal, loTime, hiTime))

    def stop(self, servo_id):
        """停止指定舵机。

        参数:
            servo_id: 舵机 ID。
        """
        self.write(servo_id, HIWONDER_SERVO_MOVE_STOP, ())

    def set_servo_deviation(self, servo_id, dev=0):
        """设置舵机偏差。

        参数:
            servo_id: 舵机 ID。
            dev: 偏差值。
        """
        self.write(servo_id, HIWONDER_SERVO_ANGLE_OFFSET_ADJUST, (dev, ))

    def save_servo_deviation(self, servo_id):
        """保存舵机偏差。

        参数:
            servo_id: 舵机 ID。
        """
        self.write(servo_id, HIWONDER_SERVO_ANGLE_OFFSET_WRITE, ())
        
    def get_servo_deviation(self, servo_id):
        """读取舵机偏差。

        参数:
            servo_id: 舵机 ID。

        返回:
            偏差值；超时返回 None。
        """
        # 发送读取偏差指令
        count = 0
        while True:
            count += 1
            response = self.read(servo_id, HIWONDER_SERVO_ANGLE_OFFSET_READ)
            if response:
                count = 0
                self.exception_on_error(response[4], servo_id, 'fetching present position')
                return self.parse_result(response)
            if count > self.timeout:
                count = 0
                return None

    def set_servo_range(self, servo_id, low, high):
        """设置舵机脉冲范围。

        参数:
            servo_id: 舵机 ID。
            low: 最小脉冲。
            high: 最大脉冲。
        """
        low = int(low)
        high = int(high)
        loLow = int(low & 0xFF)
        hiLow = int(low >> 8)
        loHigh = int(high & 0xFF)
        hiHigh = int(high >> 8)
        self.write(servo_id, HIWONDER_SERVO_ANGLE_LIMIT_WRITE, (loLow, hiLow, loHigh, hiHigh))

    def parse_result(self, data):
        data_len = data[3]
        if data_len == 4:
            return data[5]
        elif data_len == 5:
            return data[5] + (data[6] << 8)
        elif data_len == 7:
            return data[5] + (data[6] << 8), data[7] + (data[8] << 8)
        else:
            return None

    def get_servo_range(self, servo_id):
        """读取舵机脉冲范围。

        参数:
            servo_id: 舵机 ID。

        返回:
            最小和最大脉冲；超时返回 None。
        """
        count = 0
        while True:
            count += 1
            response = self.read(servo_id, HIWONDER_SERVO_ANGLE_LIMIT_READ)
            if response:
                count = 0
                self.exception_on_error(response[4], servo_id, 'fetching present position')
                return self.parse_result(response)
            if count > self.timeout:
                count = 0
                return None

    def set_servo_vin_range(self, servo_id, low, high):
        """设置舵机电压范围。

        参数:
            servo_id: 舵机 ID。
            low: 最低电压阈值。
            high: 最高电压阈值。
        """
        low = int(low)
        high = int(high)
        loLow = int(low & 0xFF)
        hiLow = int(low >> 8)
        loHigh = int(high & 0xFF)
        hiHigh = int(high >> 8)
        self.write(servo_id, HIWONDER_SERVO_VIN_LIMIT_WRITE, (loLow, hiLow, loHigh, hiHigh))

    def get_servo_vin_range(self, servo_id):
        """读取舵机电压范围。

        参数:
            servo_id: 舵机 ID。

        返回:
            最低和最高电压阈值；超时返回 None。
        """
        count = 0
        while True:
            response = self.read(servo_id, HIWONDER_SERVO_VIN_LIMIT_READ)
            if response:
                count = 0
                self.exception_on_error(response[4], servo_id, 'fetching present position')
                return self.parse_result(response)
            if count > self.timeout:
                count = 0
                return None

    def set_servo_temp_range(self, servo_id, m_temp):
        """设置最高温度阈值。

        参数:
            servo_id: 舵机 ID。
            m_temp: 最高温度阈值。
        """
        self.write(servo_id, HIWONDER_SERVO_TEMP_MAX_LIMIT_WRITE, (m_temp, ))

    def get_servo_temp_range(self, servo_id):
        """读取最高温度阈值。

        参数:
            servo_id: 舵机 ID。

        返回:
            最高温度阈值；超时返回 None。
        """
        count = 0
        while True:
            count += 1
            response = self.read(servo_id, HIWONDER_SERVO_TEMP_MAX_LIMIT_READ)
            if response:
                count = 0
                self.exception_on_error(response[4], servo_id, 'fetching present position')
                return self.parse_result(response)
            if count > self.timeout:
                count = 0
                return None

    def get_servo_temp(self, servo_id):
        """读取舵机温度。

        参数:
            servo_id: 舵机 ID。

        返回:
            温度值；超时返回 None。
        """
        count = 0
        while True:
            count += 1
            response = self.read(servo_id, HIWONDER_SERVO_TEMP_READ)
            if response:
                count = 0
                self.exception_on_error(response[4], servo_id, 'fetching present position')
                return self.parse_result(response)
            if count > self.timeout:
                count = 0
                return None

    def get_servo_vin(self, servo_id):
        """读取舵机电压。

        参数:
            servo_id: 舵机 ID。

        返回:
            电压值；超时返回 None。
        """
        count = 0
        while True:
            count += 1
            response = self.read(servo_id, HIWONDER_SERVO_VIN_READ)
            if response:
                count = 0
                self.exception_on_error(response[4], servo_id, 'fetching present position')
                return self.parse_result(response)
            if count > self.timeout:
                count = 0
                return None

    def reset_servo(self, servo_id):
        # 舵机清零偏差和P值中位（500）
        self.set_deviation(servo_id, 0)    # 清零偏差
        time.sleep(0.1)
        self.write(servo_id, HIWONDER_SERVO_MOVE_TIME_WRITE, 500, 100)    # 中位

    def unload_servo(self, servo_id, status):
        self.write(servo_id, HIWONDER_SERVO_LOAD_OR_UNLOAD_WRITE, (status, ))

    def get_servo_load_state(self, servo_id):
        count = 0
        while True:
            count += 1
            response = self.read(servo_id, HIWONDER_SERVO_LOAD_OR_UNLOAD_READ)
            if response:
                count = 0
                self.exception_on_error(response[4], servo_id, 'fetching present position')
                return self.parse_result(response)
            if count > self.timeout:
                count = 0
                return None
    def load_status(self, servo_id):
        data = self.get_servo_load_state(servo_id)
        if data is None:
            return False
        if data == 0x01:
            return True
        else:
            return False
    def exception_on_error(self, error_code, servo_id, command_failed):
        global exception
        exception = None

        if not isinstance(error_code, int):
            ex_message = '[servo #%d on %s@%sbps]: %s failed' % (servo_id, self.ser.port, self.ser.baudrate, command_failed)
            msg = 'Communcation Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, 0)
            return

class SerialOpenError(Exception):
    """无法打开串口时抛出的异常。"""

    def __init__(self, port, baud):
        Exception.__init__(self)
        self.message = "Cannot open port '%s' at %d bps" % (port, baud)
        self.port = port
        self.baud = baud

    def __str__(self):
        return self.message

class ChecksumError(Exception):
    """收到的数据包校验和错误时抛出的异常。"""

    def __init__(self, servo_id, response, checksum):
        Exception.__init__(self)
        self.message = 'Checksum received from motor %d does not match the expected one (%d != %d)' \
                       % (servo_id, response[-1], checksum)
        self.response_data = response
        self.expected_checksum = checksum

    def __str__(self):
        return self.message

class FatalErrorCodeError(Exception):
    """表示舵机返回的致命错误。"""

    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const

    def __str__(self):
        return self.message

class NonfatalErrorCodeError(Exception):
    """表示舵机返回的非致命错误。"""

    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const

    def __str__(self):
        return self.message

class ErrorCodeError(Exception):
    """保存舵机错误码。"""

    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const

    def __str__(self):
        return self.message

class DroppedPacketError(Exception):
    """收到不完整或无效数据包时抛出的异常。"""

    def __init__(self, message):
        Exception.__init__(self)
        self.message = message

    def __str__(self):
        return self.message

class UnsupportedFeatureError(Exception):
    """请求了舵机不支持的功能时抛出的异常。"""

    def __init__(self, model_id, feature_id):
        Exception.__init__(self)
        model = 'Unknown'
        self.message = "Feature %d not supported by model %d (%s)" % (feature_id, model_id, model)

    def __str__(self):
        return self.message
