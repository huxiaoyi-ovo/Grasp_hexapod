// 幻尔 LX-15D 总线协议的命令常量。
// 与 Python 版 hiwonder_servo_cmd.py 一一对应，只实现官方协议，不解释机器人关节和 ROS 话题。
#ifndef GRASP_HEXAPOD_SERVO_CPP_HIWONDER_SERVO_CMD_H_
#define GRASP_HEXAPOD_SERVO_CPP_HIWONDER_SERVO_CMD_H_

#include <cstdint>

namespace grasp_hexapod_servo_cpp {

constexpr uint8_t HIWONDER_SERVO_FRAME_HEADER = 0x55;
constexpr uint8_t HIWONDER_SERVO_MOVE_TIME_WRITE = 1;
constexpr uint8_t HIWONDER_SERVO_MOVE_TIME_READ = 2;
constexpr uint8_t HIWONDER_SERVO_MOVE_TIME_WAIT_WRITE = 7;
constexpr uint8_t HIWONDER_SERVO_MOVE_TIME_WAIT_READ = 8;
constexpr uint8_t HIWONDER_SERVO_MOVE_START = 11;
constexpr uint8_t HIWONDER_SERVO_MOVE_STOP = 12;
constexpr uint8_t HIWONDER_SERVO_ID_WRITE = 13;
constexpr uint8_t HIWONDER_SERVO_ID_READ = 14;
constexpr uint8_t HIWONDER_SERVO_ANGLE_OFFSET_ADJUST = 17;
constexpr uint8_t HIWONDER_SERVO_ANGLE_OFFSET_WRITE = 18;
constexpr uint8_t HIWONDER_SERVO_ANGLE_OFFSET_READ = 19;
constexpr uint8_t HIWONDER_SERVO_ANGLE_LIMIT_WRITE = 20;
constexpr uint8_t HIWONDER_SERVO_ANGLE_LIMIT_READ = 21;
constexpr uint8_t HIWONDER_SERVO_VIN_LIMIT_WRITE = 22;
constexpr uint8_t HIWONDER_SERVO_VIN_LIMIT_READ = 23;
constexpr uint8_t HIWONDER_SERVO_TEMP_MAX_LIMIT_WRITE = 24;
constexpr uint8_t HIWONDER_SERVO_TEMP_MAX_LIMIT_READ = 25;
constexpr uint8_t HIWONDER_SERVO_TEMP_READ = 26;
constexpr uint8_t HIWONDER_SERVO_VIN_READ = 27;
constexpr uint8_t HIWONDER_SERVO_POS_READ = 28;
constexpr uint8_t HIWONDER_SERVO_OR_MOTOR_MODE_WRITE = 29;
constexpr uint8_t HIWONDER_SERVO_OR_MOTOR_MODE_READ = 30;
constexpr uint8_t HIWONDER_SERVO_LOAD_OR_UNLOAD_WRITE = 31;
constexpr uint8_t HIWONDER_SERVO_LOAD_OR_UNLOAD_READ = 32;
constexpr uint8_t HIWONDER_SERVO_LED_CTRL_WRITE = 33;
constexpr uint8_t HIWONDER_SERVO_LED_CTRL_READ = 34;
constexpr uint8_t HIWONDER_SERVO_LED_ERROR_WRITE = 35;
constexpr uint8_t HIWONDER_SERVO_LED_ERROR_READ = 36;

}  // namespace grasp_hexapod_servo_cpp

#endif  // GRASP_HEXAPOD_SERVO_CPP_HIWONDER_SERVO_CMD_H_
