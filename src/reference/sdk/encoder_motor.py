#!/usr/bin/env python3
# coding=utf8
import os
import smbus2
import threading
import struct

ENCODER_MOTOR_MODULE_ADDRESS = 0x34

class EncoderMotorController:
    def __init__(self, i2c_port, motor_type=3):
        self.i2c_port = i2c_port
        with smbus2.SMBus(self.i2c_port) as bus:
            bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 20, [motor_type, ])

    def set_speed(self, speed, motor_id=None, offset=0):
        with smbus2.SMBus(self.i2c_port) as bus:
            # motor speed  control register address is 51 + motor_id - 1
            # [motor1, mottor2, motor3, motor4]对应板子上的1-4, 1和4方向相同，2和3方向相同，1和2方向相反, 正对轮外侧1大于0时逆时针旋转
            if motor_id is None:
                for i in range(len(speed)):
                    if speed[i] > 100:
                        speed[i] = 100
                    elif speed[i] < -100:
                        speed[i] = -100
                    try:
                        bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 51 + i, [speed[i], ])
                    except BaseException as e:
                        print(e)
                        bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 51 + i, [speed[i], ])
            else:
                if 0 < motor_id <= 4:
                    speed = 100 if speed > 100 else speed
                    speed = -100 if speed < -100 else speed
                    try:
                        bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 50 + motor_id, [speed, ])
                    except BaseException as e:
                        print(e)
                        bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 50 + motor_id, [speed, ])
                else:
                    raise ValueError("Invalid motor id")

    def clear_encoder(self, motor_id=None):
        with smbus2.SMBus(self.i2c_port) as bus:
            if motor_id is None:
                bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 60, [0]*16)
            else:
                if 0 <= motor_id <= 4:
                    bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 60 + motor_id*4, [0]*4)
                else:
                    raise ValueError("Invalid motor id")

    def read_encoder(self, motor_id=None):
        with smbus2.SMBus(self.i2c_port) as bus:
            if motor_id is None:
                return list(struct.unpack('<iiii',bytes(bus.read_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 60, 16))))
            else:
                if 0 <= motor_id <= 4:
                    return struct.unpack('<i',bytes(bus.read_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 60 + motor_id*4, 4)))[0]
                else:
                    raise ValueError("Invalid motor id")
