// 编码器串口协议：帧构建、Modbus CRC16 校验、响应解析与角度换算。
// 本模块不依赖 ROS，可独立编译测试。
#ifndef ENCODER_DRIVER_ENCODER_FRAME_HPP
#define ENCODER_DRIVER_ENCODER_FRAME_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace encoder_driver {

// 读命令 0x03。
constexpr uint8_t kCmdRead = 0x03;

// 18 位编码器分辨率：2**18 = 262144。
constexpr double kResolution = 262144.0;
constexpr double kFullCircle = 360.0;

// 计算 Modbus CRC16 校验值。
uint16_t ModbusCrc16(const std::vector<uint8_t>& data);

// 在帧末尾追加两字节 Modbus CRC16（低字节在前）。
std::vector<uint8_t> AppendCrc(const std::vector<uint8_t>& frame);

// 校验一帧（含末尾两字节 CRC）是否符合 Modbus CRC16。
bool CheckCrc(const std::vector<uint8_t>& frame);

// 根据 Len 字节推算完整帧的总字节数：ID(1)+CMD(1)+Len(1)+Data(Len)+CRC(2)。
size_t FrameTotalLen(uint8_t len_byte);

// 把一段字节按大端组合成无符号整数。
uint32_t BytesToUint(const std::vector<uint8_t>& data);

// 解析后的响应帧字段。
struct Response {
  uint8_t slave_id = 0;
  uint8_t cmd = 0;
  uint8_t length = 0;
  std::vector<uint8_t> data;
  std::vector<uint8_t> crc;
  uint32_t raw = 0;  // 原始计数值（大端组合）
};

// 解析一帧响应；帧过短时抛 std::invalid_argument。
Response ParseResponse(const std::vector<uint8_t>& frame);

// 原始计数值 -> 角度（默认度）：raw / resolution * full_circle。
double RawToAngle(uint32_t raw,
                  double resolution = kResolution,
                  double full_circle = kFullCircle);

// 构建读请求帧（Modbus RTU 读保持寄存器）：
// [ID][0x03][start_hi][start_lo][count_hi][count_lo][CRClo][CRChi]。
std::vector<uint8_t> BuildReadQuery(uint8_t slave_id,
                                    uint16_t start_reg = 0,
                                    uint16_t reg_count = 2);

// 把字节序列格式化为 "00 01 5F CF" 形式的十六进制字符串。
std::string FormatHex(const std::vector<uint8_t>& data);

// 从串口字节流中按 "ID CMD Len Data CRC" 提取编码器响应帧。
// 串口读取可能分片、粘包或夹带杂质，解析器按字节扫描并只在 CRC
// 校验通过时产出一帧，其余数据丢弃并继续扫描后续字节。
class ResponseParser {
 public:
  ResponseParser(uint8_t slave_id = 0x00, uint8_t cmd = kCmdRead);

  // 喂入一串字节，返回本次解析出的合法帧列表；内部保留缓冲区，
  // 可应对跨读取分段的帧。
  std::vector<std::vector<uint8_t>> Feed(const std::vector<uint8_t>& chunk);

 private:
  int FindHeader() const;
  std::vector<std::vector<uint8_t>> Extract();

  uint8_t slave_id_;
  uint8_t cmd_;
  std::vector<uint8_t> buf_;
};

// 自检：校验 CRC、解析、角度换算与官方示例一致。失败返回 false。
bool SelfTest();

}  // namespace encoder_driver

#endif  // ENCODER_DRIVER_ENCODER_FRAME_HPP
