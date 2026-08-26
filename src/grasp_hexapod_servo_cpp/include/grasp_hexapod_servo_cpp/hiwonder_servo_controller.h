// 幻尔 LX-15D 串口总线舵机控制器。
// C++ 版对应 Python 的 hiwonder_servo_controller.py，串口时序与封包逻辑逐条对齐：
//   - 写前 flushInput，写后延时 340us；
//   - 响应帧：先读 4 字节头（0x55 0x55 ID LEN），校验前缀，再读 LEN-1 字节；
//   - 校验和 = 255 - (sum(data[2:-1]) % 256)，与帧末字节比较。
#ifndef GRASP_HEXAPOD_SERVO_CPP_HIWONDER_SERVO_CONTROLLER_H_
#define GRASP_HEXAPOD_SERVO_CPP_HIWONDER_SERVO_CONTROLLER_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <serial/serial.h>

namespace grasp_hexapod_servo_cpp {

// 无法打开串口时抛出的异常（对应 Python 的 SerialOpenError）。
class SerialOpenException : public std::runtime_error {
 public:
  SerialOpenException(const std::string& port, int baud)
      : std::runtime_error("Cannot open port '" + port + "' at " +
                           std::to_string(baud) + " bps") {}
};

class HiwonderServoController {
 public:
  // 打开串口；失败抛 SerialOpenException。
  explicit HiwonderServoController(const std::string& port, int baudrate);
  ~HiwonderServoController();
  void close();

  // ---- 低层收发（与 Python read/write 对齐） ----
  // 读命令：发读包、收响应；丢包/校验失败返回空。
  std::vector<uint8_t> read(uint8_t servo_id, uint8_t cmd);
  // 写命令：发写包，不等待响应。
  void write(uint8_t servo_id, uint8_t cmd,
             const std::vector<uint8_t>& params);

  // ---- 高层 API（与 Python 版方法名一一对应） ----
  std::optional<uint16_t> getServoPosition(uint8_t servo_id);
  std::optional<uint16_t> getServoVoltage(uint8_t servo_id);  // VIN_READ
  void setServoPosition(uint8_t servo_id, int position, int duration_ms = 20);
  void unloadServo(uint8_t servo_id, uint8_t status);  // LOAD_OR_UNLOAD_WRITE
  void stop(uint8_t servo_id);
  void setServoId(uint8_t old_id, uint8_t new_id);
  std::optional<uint8_t> getServoId(std::optional<uint8_t> servo_id = std::nullopt);
  void setServoDeviation(uint8_t servo_id, uint8_t dev = 0);
  void saveServoDeviation(uint8_t servo_id);
  std::optional<int> getServoDeviation(uint8_t servo_id);
  void setServoRange(uint8_t servo_id, int low, int high);
  std::optional<std::pair<int, int>> getServoRange(uint8_t servo_id);
  void setServoVinRange(uint8_t servo_id, int low, int high);
  std::optional<std::pair<int, int>> getServoVinRange(uint8_t servo_id);
  void setServoTempRange(uint8_t servo_id, uint8_t max_temp);
  std::optional<int> getServoTempRange(uint8_t servo_id);
  std::optional<int> getServoTemp(uint8_t servo_id);
  std::optional<int> getServoVin(uint8_t servo_id);
  void resetServo(uint8_t servo_id);
  std::optional<uint8_t> getServoLoadState(uint8_t servo_id);
  bool loadStatus(uint8_t servo_id);

  // ---- 静态纯函数（不依赖串口，供单元测试） ----
  // 帧校验和：255 - ((id + len + cmd + sum(params)) % 256)。
  static uint8_t computeFrameChecksum(uint8_t servo_id, uint8_t length,
                                      uint8_t cmd,
                                      const std::vector<uint8_t>& params);
  // 组写包：[0x55, 0x55, id, len, cmd, params..., checksum]。
  static std::vector<uint8_t> buildWritePacket(
      uint8_t servo_id, uint8_t cmd, const std::vector<uint8_t>& params);
  // 组读包：[0x55, 0x55, id, 3, cmd, checksum]。
  static std::vector<uint8_t> buildReadPacket(uint8_t servo_id, uint8_t cmd);
  // 校验响应帧：前缀 0x55 0x55 + 校验和正确。
  static bool verifyResponse(const std::vector<uint8_t>& data);
  // 解析双字节结果（如位置、电压）：data[5] + (data[6] << 8)。
  static std::optional<uint16_t> parseTwoByteResult(
      const std::vector<uint8_t>& data);
  // 按响应 LEN 解析单/双字节结果（对应 Python parse_result 的 4/5 分支）。
  static std::optional<int> parseResult(const std::vector<uint8_t>& data);
  // 解析双字对结果（LEN=7，用于 range 类读取）。
  static std::optional<std::pair<int, int>> parsePairResult(
      const std::vector<uint8_t>& data);

 private:
  void writeSerial(const std::vector<uint8_t>& data);
  std::vector<uint8_t> readResponse(uint8_t servo_id);
  // 带重试循环的读（对应 Python 中 count > self.timeout 的重试方法）。
  std::optional<int> readWithRetry(uint8_t servo_id, uint8_t cmd);

  std::string port_name_;
  std::unique_ptr<serial::Serial> ser_;
  std::mutex mutex_;  // 串口收发互斥（对应 Python serial_mutex）。
  int timeout_ = 10;  // 重试上限（对应 Python self.timeout）。
};

}  // namespace grasp_hexapod_servo_cpp

#endif  // GRASP_HEXAPOD_SERVO_CPP_HIWONDER_SERVO_CONTROLLER_H_
