// 编码器串口协议实现。
#include "encoder_driver/encoder_frame.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace encoder_driver {

uint16_t ModbusCrc16(const std::vector<uint8_t>& data) {
  uint16_t crc = 0xFFFF;
  for (uint8_t byte : data) {
    crc ^= byte;
    for (int i = 0; i < 8; ++i) {
      if (crc & 0x0001) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

std::vector<uint8_t> AppendCrc(const std::vector<uint8_t>& frame) {
  uint16_t crc = ModbusCrc16(frame);
  std::vector<uint8_t> out = frame;
  out.push_back(static_cast<uint8_t>(crc & 0xFF));
  out.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  return out;
}

bool CheckCrc(const std::vector<uint8_t>& frame) {
  if (frame.size() < 2) {
    return false;
  }
  std::vector<uint8_t> body(frame.begin(), frame.end() - 2);
  uint16_t payload_crc = static_cast<uint16_t>(frame[frame.size() - 2] |
                                               (frame[frame.size() - 1] << 8));
  return ModbusCrc16(body) == payload_crc;
}

size_t FrameTotalLen(uint8_t len_byte) {
  return 3u + len_byte + 2u;
}

uint32_t BytesToUint(const std::vector<uint8_t>& data) {
  uint32_t value = 0;
  for (uint8_t byte : data) {
    value = (value << 8) | byte;
  }
  return value;
}

Response ParseResponse(const std::vector<uint8_t>& frame) {
  if (frame.size() < 3) {
    throw std::invalid_argument("响应帧过短: " + std::to_string(frame.size()) +
                                " 字节");
  }
  Response resp;
  resp.slave_id = frame[0];
  resp.cmd = frame[1];
  resp.length = frame[2];
  resp.data.assign(frame.begin() + 3, frame.begin() + 3 + resp.length);
  resp.crc.assign(frame.begin() + 3 + resp.length, frame.end());
  resp.raw = BytesToUint(resp.data);
  return resp;
}

double RawToAngle(uint32_t raw, double resolution, double full_circle) {
  return static_cast<double>(raw) / resolution * full_circle;
}

std::vector<uint8_t> BuildReadQuery(uint8_t slave_id, uint16_t start_reg,
                                    uint16_t reg_count) {
  std::vector<uint8_t> frame = {
      slave_id,
      kCmdRead,
      static_cast<uint8_t>((start_reg >> 8) & 0xFF),
      static_cast<uint8_t>(start_reg & 0xFF),
      static_cast<uint8_t>((reg_count >> 8) & 0xFF),
      static_cast<uint8_t>(reg_count & 0xFF),
  };
  return AppendCrc(frame);
}

std::string FormatHex(const std::vector<uint8_t>& data) {
  std::string out;
  char buf[4];
  for (size_t i = 0; i < data.size(); ++i) {
    if (i > 0) {
      out.push_back(' ');
    }
    std::snprintf(buf, sizeof(buf), "%02X", data[i]);
    out += buf;
  }
  return out;
}

ResponseParser::ResponseParser(uint8_t slave_id, uint8_t cmd)
    : slave_id_(slave_id), cmd_(cmd) {}

std::vector<std::vector<uint8_t>> ResponseParser::Feed(
    const std::vector<uint8_t>& chunk) {
  buf_.insert(buf_.end(), chunk.begin(), chunk.end());
  return Extract();
}

int ResponseParser::FindHeader() const {
  for (size_t i = 0; i + 1 < buf_.size(); ++i) {
    if (buf_[i] == slave_id_ && buf_[i + 1] == cmd_) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

std::vector<std::vector<uint8_t>> ResponseParser::Extract() {
  std::vector<std::vector<uint8_t>> frames;
  while (true) {
    int header = FindHeader();
    if (header < 0) {
      // 未找到帧头：保留最多 1 字节，避免帧头跨读取分段。
      if (!buf_.empty()) {
        std::vector<uint8_t> tail(buf_.end() - 1, buf_.end());
        buf_ = tail;
      }
      break;
    }
    if (header > 0) {
      buf_.erase(buf_.begin(), buf_.begin() + header);
    }
    if (buf_.size() < 3) {
      break;
    }
    size_t total = FrameTotalLen(buf_[2]);
    if (buf_.size() < total) {
      break;
    }
    std::vector<uint8_t> candidate(buf_.begin(), buf_.begin() + total);
    buf_.erase(buf_.begin(), buf_.begin() + total);
    if (CheckCrc(candidate)) {
      frames.push_back(candidate);
    }
    // CRC 不合法则丢弃该候选，继续从剩余字节中寻找下一个帧头。
  }
  return frames;
}

bool SelfTest() {
  // 示例 1：00 03 04 00 01 5F CF C2 97 -> 原始 90063 -> 123.682708 度。
  const std::vector<uint8_t> frame1 = {0x00, 0x03, 0x04, 0x00, 0x01,
                                       0x5F, 0xCF, 0xC2, 0x97};
  if (!CheckCrc(frame1)) {
    return false;
  }
  Response resp1 = ParseResponse(frame1);
  if (resp1.raw != 0x00015FCF || resp1.raw != 90063) {
    return false;
  }
  double angle1 = RawToAngle(resp1.raw);
  if (std::abs(angle1 - 123.68270874023438) > 1e-6) {
    return false;
  }

  // 示例 2：00 03 02 00 00 85 84。
  const std::vector<uint8_t> frame2 = {0x00, 0x03, 0x02, 0x00, 0x00, 0x85, 0x84};
  if (!CheckCrc(frame2)) {
    return false;
  }
  if (ParseResponse(frame2).raw != 0) {
    return false;
  }

  // 解析器应能从夹带/分段字节流中正确提取两帧。
  ResponseParser parser(0x00);
  std::vector<uint8_t> stream = {0x00, 0x00};
  stream.insert(stream.end(), frame1.begin(), frame1.end());
  stream.push_back(0xFF);
  stream.insert(stream.end(), frame2.begin(), frame2.end());
  std::vector<std::vector<uint8_t>> got = parser.Feed(stream);
  if (got.size() != 2) {
    return false;
  }
  if (ParseResponse(got[0]).raw != 90063) {
    return false;
  }
  if (ParseResponse(got[1]).raw != 0) {
    return false;
  }

  // 读请求帧自身应能通过 CRC 校验。
  std::vector<uint8_t> query = BuildReadQuery(0x00, 0, 2);
  if (!CheckCrc(query)) {
    return false;
  }

  std::printf("selftest OK\n");
  std::printf("  示例1: %s -> raw=%u, angle=%.6f deg\n",
              FormatHex(frame1).c_str(), resp1.raw, angle1);
  std::printf("  读请求: %s\n", FormatHex(query).c_str());
  return true;
}

}  // namespace encoder_driver
