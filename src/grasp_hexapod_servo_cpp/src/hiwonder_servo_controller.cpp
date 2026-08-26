// 幻尔 LX-15D 串口总线舵机控制器实现。
#include "grasp_hexapod_servo_cpp/hiwonder_servo_controller.h"

#include <array>
#include <chrono>
#include <thread>

#include "grasp_hexapod_servo_cpp/hiwonder_servo_cmd.h"

namespace grasp_hexapod_servo_cpp {

namespace {

constexpr uint8_t kFrameHeader = HIWONDER_SERVO_FRAME_HEADER;
constexpr uint8_t kBroadcastId = 0xFE;

}  // namespace

HiwonderServoController::HiwonderServoController(const std::string& port,
                                                 int baudrate)
    : port_name_(port) {
  try {
    // 10ms 读超时，对应 Python serial.Serial(port, baudrate, timeout=0.01)。
    ser_.reset(new serial::Serial(port, static_cast<uint32_t>(baudrate),
                                  serial::Timeout::simpleTimeout(10)));
  } catch (const serial::SerialException&) {
    throw SerialOpenException(port, baudrate);
  } catch (const serial::IOException&) {
    throw SerialOpenException(port, baudrate);
  }
}

HiwonderServoController::~HiwonderServoController() { close(); }

void HiwonderServoController::close() {
  if (ser_ && ser_->isOpen()) {
    ser_->flushInput();
    ser_->flushOutput();
    ser_->close();
  }
}

void HiwonderServoController::writeSerial(const std::vector<uint8_t>& data) {
  ser_->flushInput();
  ser_->write(data.data(), data.size());
  // 写后延时 340us，对应 Python __write_serial 的 time.sleep(0.00034)。
  std::this_thread::sleep_for(std::chrono::microseconds(340));
}

std::vector<uint8_t> HiwonderServoController::readResponse(uint8_t servo_id) {
  std::vector<uint8_t> data;
  std::array<uint8_t, 4> header{};
  const size_t header_got = ser_->read(header.data(), 4);
  if (header_got < 4) {
    return data;  // 读头不完整，视为丢包。
  }
  data.assign(header.begin(), header.end());
  if (data[0] != kFrameHeader || data[1] != kFrameHeader) {
    return data;  // 前缀错误（Python 抛 DroppedPacketError 后被吞掉）。
  }
  const size_t remaining = static_cast<size_t>(data[3]) - 1;
  std::vector<uint8_t> rest(remaining);
  const size_t rest_got = ser_->read(rest.data(), remaining);
  data.insert(data.end(), rest.begin(), rest.begin() + rest_got);
  if (!verifyResponse(data)) {
    return {};  // 校验和错误（Python 抛 ChecksumError 后被吞掉）。
  }
  return data;
}

std::vector<uint8_t> HiwonderServoController::read(uint8_t servo_id,
                                                   uint8_t cmd) {
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    writeSerial(buildReadPacket(servo_id, cmd));
    return readResponse(servo_id);
  } catch (const serial::SerialException&) {
    return {};
  } catch (const serial::IOException&) {
    return {};
  }
}

void HiwonderServoController::write(uint8_t servo_id, uint8_t cmd,
                                    const std::vector<uint8_t>& params) {
  std::lock_guard<std::mutex> lock(mutex_);
  // 与 Python 一致：写失败异常向上抛出，由调用方（节点）统一兜底。
  writeSerial(buildWritePacket(servo_id, cmd, params));
}

std::optional<int> HiwonderServoController::readWithRetry(uint8_t servo_id,
                                                          uint8_t cmd) {
  for (int attempt = 0; attempt < timeout_; ++attempt) {
    std::vector<uint8_t> response = read(servo_id, cmd);
    if (!response.empty()) {
      return parseResult(response);
    }
  }
  return std::nullopt;
}

std::optional<uint16_t> HiwonderServoController::getServoPosition(
    uint8_t servo_id) {
  std::vector<uint8_t> response =
      read(servo_id, HIWONDER_SERVO_POS_READ);
  if (response.empty()) {
    return std::nullopt;
  }
  return parseTwoByteResult(response);
}

std::optional<uint16_t> HiwonderServoController::getServoVoltage(
    uint8_t servo_id) {
  std::vector<uint8_t> response =
      read(servo_id, HIWONDER_SERVO_VIN_READ);
  if (response.empty()) {
    return std::nullopt;
  }
  return parseTwoByteResult(response);
}

void HiwonderServoController::setServoPosition(uint8_t servo_id,
                                               int position,
                                               int duration_ms) {
  if (duration_ms < 0) {
    duration_ms = 0;
  } else if (duration_ms > 30000) {
    duration_ms = 30000;
  }
  if (position < 0) {
    position = 0;
  } else if (position > 1000) {
    position = 1000;
  }
  const uint8_t lo_pos = static_cast<uint8_t>(position & 0xFF);
  const uint8_t hi_pos = static_cast<uint8_t>((position >> 8) & 0xFF);
  const uint8_t lo_time = static_cast<uint8_t>(duration_ms & 0xFF);
  const uint8_t hi_time = static_cast<uint8_t>((duration_ms >> 8) & 0xFF);
  write(servo_id, HIWONDER_SERVO_MOVE_TIME_WRITE,
        {lo_pos, hi_pos, lo_time, hi_time});
}

void HiwonderServoController::unloadServo(uint8_t servo_id, uint8_t status) {
  write(servo_id, HIWONDER_SERVO_LOAD_OR_UNLOAD_WRITE, {status});
}

void HiwonderServoController::stop(uint8_t servo_id) {
  write(servo_id, HIWONDER_SERVO_MOVE_STOP, {});
}

void HiwonderServoController::setServoId(uint8_t old_id, uint8_t new_id) {
  write(old_id, HIWONDER_SERVO_ID_WRITE, {new_id});
}

std::optional<uint8_t> HiwonderServoController::getServoId(
    std::optional<uint8_t> servo_id) {
  const uint8_t query_id = servo_id.value_or(kBroadcastId);
  for (int attempt = 0; attempt < timeout_; ++attempt) {
    std::vector<uint8_t> response =
        read(query_id, HIWONDER_SERVO_ID_READ);
    if (!response.empty()) {
      std::optional<int> result = parseResult(response);
      if (result) {
        return static_cast<uint8_t>(*result);
      }
      return std::nullopt;
    }
  }
  return std::nullopt;
}

void HiwonderServoController::setServoDeviation(uint8_t servo_id,
                                                uint8_t dev) {
  write(servo_id, HIWONDER_SERVO_ANGLE_OFFSET_ADJUST, {dev});
}

void HiwonderServoController::saveServoDeviation(uint8_t servo_id) {
  write(servo_id, HIWONDER_SERVO_ANGLE_OFFSET_WRITE, {});
}

std::optional<int> HiwonderServoController::getServoDeviation(
    uint8_t servo_id) {
  return readWithRetry(servo_id, HIWONDER_SERVO_ANGLE_OFFSET_READ);
}

void HiwonderServoController::setServoRange(uint8_t servo_id, int low,
                                            int high) {
  const uint8_t lo_low = static_cast<uint8_t>(low & 0xFF);
  const uint8_t hi_low = static_cast<uint8_t>((low >> 8) & 0xFF);
  const uint8_t lo_high = static_cast<uint8_t>(high & 0xFF);
  const uint8_t hi_high = static_cast<uint8_t>((high >> 8) & 0xFF);
  write(servo_id, HIWONDER_SERVO_ANGLE_LIMIT_WRITE,
        {lo_low, hi_low, lo_high, hi_high});
}

std::optional<std::pair<int, int>> HiwonderServoController::getServoRange(
    uint8_t servo_id) {
  for (int attempt = 0; attempt < timeout_; ++attempt) {
    std::vector<uint8_t> response =
        read(servo_id, HIWONDER_SERVO_ANGLE_LIMIT_READ);
    if (!response.empty()) {
      return parsePairResult(response);
    }
  }
  return std::nullopt;
}

void HiwonderServoController::setServoVinRange(uint8_t servo_id, int low,
                                               int high) {
  const uint8_t lo_low = static_cast<uint8_t>(low & 0xFF);
  const uint8_t hi_low = static_cast<uint8_t>((low >> 8) & 0xFF);
  const uint8_t lo_high = static_cast<uint8_t>(high & 0xFF);
  const uint8_t hi_high = static_cast<uint8_t>((high >> 8) & 0xFF);
  write(servo_id, HIWONDER_SERVO_VIN_LIMIT_WRITE,
        {lo_low, hi_low, lo_high, hi_high});
}

std::optional<std::pair<int, int>> HiwonderServoController::getServoVinRange(
    uint8_t servo_id) {
  for (int attempt = 0; attempt < timeout_; ++attempt) {
    std::vector<uint8_t> response =
        read(servo_id, HIWONDER_SERVO_VIN_LIMIT_READ);
    if (!response.empty()) {
      return parsePairResult(response);
    }
  }
  return std::nullopt;
}

void HiwonderServoController::setServoTempRange(uint8_t servo_id,
                                                uint8_t max_temp) {
  write(servo_id, HIWONDER_SERVO_TEMP_MAX_LIMIT_WRITE, {max_temp});
}

std::optional<int> HiwonderServoController::getServoTempRange(
    uint8_t servo_id) {
  return readWithRetry(servo_id, HIWONDER_SERVO_TEMP_MAX_LIMIT_READ);
}

std::optional<int> HiwonderServoController::getServoTemp(uint8_t servo_id) {
  return readWithRetry(servo_id, HIWONDER_SERVO_TEMP_READ);
}

std::optional<int> HiwonderServoController::getServoVin(uint8_t servo_id) {
  return readWithRetry(servo_id, HIWONDER_SERVO_VIN_READ);
}

void HiwonderServoController::resetServo(uint8_t servo_id) {
  // Python 版 reset_servo 调用了不存在的方法（set_deviation）且 write 参数错误，
  // 属于坏代码；这里按协议意图实现：清零偏差 -> 回到中位。
  setServoDeviation(servo_id, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  setServoPosition(servo_id, 500, 100);
}

std::optional<uint8_t> HiwonderServoController::getServoLoadState(
    uint8_t servo_id) {
  std::optional<int> result = readWithRetry(servo_id, HIWONDER_SERVO_LOAD_OR_UNLOAD_READ);
  if (!result) {
    return std::nullopt;
  }
  return static_cast<uint8_t>(*result);
}

bool HiwonderServoController::loadStatus(uint8_t servo_id) {
  std::optional<uint8_t> data = getServoLoadState(servo_id);
  return data && *data == 0x01;
}

uint8_t HiwonderServoController::computeFrameChecksum(
    uint8_t servo_id, uint8_t length, uint8_t cmd,
    const std::vector<uint8_t>& params) {
  uint8_t sum = static_cast<uint8_t>(servo_id + length + cmd);
  for (uint8_t param : params) {
    sum = static_cast<uint8_t>(sum + param);
  }
  return static_cast<uint8_t>(255 - sum);
}

std::vector<uint8_t> HiwonderServoController::buildWritePacket(
    uint8_t servo_id, uint8_t cmd, const std::vector<uint8_t>& params) {
  const uint8_t length = static_cast<uint8_t>(3 + params.size());
  std::vector<uint8_t> packet = {kFrameHeader, kFrameHeader, servo_id,
                                 length,       cmd};
  packet.insert(packet.end(), params.begin(), params.end());
  packet.push_back(computeFrameChecksum(servo_id, length, cmd, params));
  return packet;
}

std::vector<uint8_t> HiwonderServoController::buildReadPacket(uint8_t servo_id,
                                                              uint8_t cmd) {
  constexpr uint8_t kReadLength = 3;  // instruction + checksum
  const std::vector<uint8_t> no_params;
  return {kFrameHeader, kFrameHeader, servo_id, kReadLength, cmd,
          computeFrameChecksum(servo_id, kReadLength, cmd, no_params)};
}

bool HiwonderServoController::verifyResponse(
    const std::vector<uint8_t>& data) {
  if (data.size() < 5) {
    return false;
  }
  if (data[0] != kFrameHeader || data[1] != kFrameHeader) {
    return false;
  }
  uint8_t sum = 0;
  for (size_t i = 2; i + 1 < data.size(); ++i) {
    sum = static_cast<uint8_t>(sum + data[i]);
  }
  return static_cast<uint8_t>(255 - sum) == data.back();
}

std::optional<uint16_t> HiwonderServoController::parseTwoByteResult(
    const std::vector<uint8_t>& data) {
  if (data.size() < 7) {
    return std::nullopt;
  }
  return static_cast<uint16_t>(data[5] +
                               (static_cast<uint16_t>(data[6]) << 8));
}

std::optional<int> HiwonderServoController::parseResult(
    const std::vector<uint8_t>& data) {
  if (data.size() < 4) {
    return std::nullopt;
  }
  switch (data[3]) {  // 响应帧 LEN 字段。
    case 4:  // 单字节结果。
      if (data.size() < 6) {
        return std::nullopt;
      }
      return data[5];
    case 5: {  // 双字节结果。
      if (data.size() < 7) {
        return std::nullopt;
      }
      return data[5] + (static_cast<int>(data[6]) << 8);
    }
    default:
      return std::nullopt;
  }
}

std::optional<std::pair<int, int>> HiwonderServoController::parsePairResult(
    const std::vector<uint8_t>& data) {
  if (data.size() < 10 || data[3] != 7) {
    return std::nullopt;
  }
  return std::make_pair(data[5] + (static_cast<int>(data[6]) << 8),
                        data[7] + (static_cast<int>(data[8]) << 8));
}

}  // namespace grasp_hexapod_servo_cpp
