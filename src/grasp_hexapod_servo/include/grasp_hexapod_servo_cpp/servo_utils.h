// 与 ROS 无关的纯函数：角度换算、方向参数解析、板级上电决策。
// 全部 header-inline，供 servo_side_node.cpp 与单元测试直接复用。
#ifndef GRASP_HEXAPOD_SERVO_CPP_SERVO_UTILS_H_
#define GRASP_HEXAPOD_SERVO_CPP_SERVO_UTILS_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace grasp_hexapod_servo_cpp {

// LX-15D：0~1000 脉冲对应 0°~240°，中位 500。
constexpr double kServoResolution = 1000.0 / 240.0;  // 脉冲/度
constexpr double kServoMidpoint = 500.0;

// 使用 nearbyint（round-half-to-even，即 Python round 的银行家舍入），
// 保证与 Python 版 rad_to_servo 的取整行为一致。
inline int roundHalfToEven(double value) {
  return static_cast<int>(std::nearbyint(value));
}

// ROS 关节角 rad -> LX-15D 脉冲。先舍入再钳位到 0~1000，与 Python 版一致。
inline int radToServo(double angle_rad, int direction) {
  const double pulse =
      direction * (angle_rad * 180.0 / M_PI) * kServoResolution + kServoMidpoint;
  const int rounded = roundHalfToEven(pulse);
  return std::max(0, std::min(1000, rounded));
}

// LX-15D 脉冲 -> ROS 关节角 rad。
inline double servoToRad(int servo_pulse, int direction) {
  return direction * ((servo_pulse - kServoMidpoint) / kServoResolution) *
         (M_PI / 180.0);
}

// 板级上电决策：三条腿都收到过完整目标 且 都请求加载，才返回 true。
// 对应 Python 版 control_loop 中的 targets_ready && all(power_request)。
inline bool computeRequestedOn(const std::vector<bool>& received,
                               const std::vector<bool>& power_request) {
  if (received.size() != power_request.size()) {
    return false;
  }
  for (size_t i = 0; i < received.size(); ++i) {
    if (!received[i] || !power_request[i]) {
      return false;
    }
  }
  return true;
}

// 方向参数校验：长度必须等于舵机数，且每个值只能是 1 或 -1。
inline bool validateDirections(const std::vector<int>& directions,
                               size_t servo_count) {
  if (directions.size() != servo_count) {
    return false;
  }
  for (int direction : directions) {
    if (direction != 1 && direction != -1) {
      return false;
    }
  }
  return true;
}

// 解析字符串形式的方向参数，如 "[1, -1, 1, ...]" 或 "1,-1,1"。
// 解析失败返回 nullopt；成功返回整数列表。
inline std::optional<std::vector<int>> parseDirectionsString(
    const std::string& text) {
  std::string cleaned = text;
  // 去掉首尾空白。
  size_t first = cleaned.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return std::nullopt;
  }
  size_t last = cleaned.find_last_not_of(" \t\r\n");
  cleaned = cleaned.substr(first, last - first + 1);
  // 去掉可选的一对方括号。
  if (cleaned.size() >= 2 && cleaned.front() == '[' &&
      cleaned.back() == ']') {
    cleaned = cleaned.substr(1, cleaned.size() - 2);
  }
  if (cleaned.empty()) {
    return std::nullopt;
  }

  std::vector<int> result;
  size_t start = 0;
  while (start <= cleaned.size()) {
    size_t comma = cleaned.find(',', start);
    if (comma == std::string::npos) {
      comma = cleaned.size();
    }
    std::string token = cleaned.substr(start, comma - start);
    size_t token_first = token.find_first_not_of(" \t\r\n");
    size_t token_last = token.find_last_not_of(" \t\r\n");
    if (token_first == std::string::npos) {
      return std::nullopt;  // 空 token（如连续逗号、结尾逗号）。
    }
    token = token.substr(token_first, token_last - token_first + 1);
    try {
      result.push_back(std::stoi(token));
    } catch (const std::exception&) {
      return std::nullopt;
    }
    if (comma == cleaned.size()) {
      break;
    }
    start = comma + 1;
  }
  return result;
}

}  // namespace grasp_hexapod_servo_cpp

#endif  // GRASP_HEXAPOD_SERVO_CPP_SERVO_UTILS_H_
