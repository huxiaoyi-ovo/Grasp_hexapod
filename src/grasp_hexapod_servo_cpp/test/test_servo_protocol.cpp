// 纯 CPU 回归测试：LX-15D 协议封包/校验和/响应解析、角度换算、
// 方向参数解析、板级上电决策。对应 Python 版 test_servo_timing_diagnostics.py
// 的覆盖意图，全部不依赖真实串口。
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "grasp_hexapod_servo_cpp/hiwonder_servo_controller.h"
#include "grasp_hexapod_servo_cpp/hiwonder_servo_cmd.h"
#include "grasp_hexapod_servo_cpp/servo_utils.h"

namespace ghsc = grasp_hexapod_servo_cpp;

namespace {

double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

}  // namespace

// ---- 协议封包与校验和 ----

TEST(TestServoProtocol, MoveTimeWritePacket) {
  const std::vector<uint8_t> packet = ghsc::HiwonderServoController::buildWritePacket(
      2, ghsc::HIWONDER_SERVO_MOVE_TIME_WRITE, {0xF4, 0x01, 0x64, 0x00});
  const std::vector<uint8_t> expected = {0x55, 0x55, 0x02, 0x07, 0x01,
                                         0xF4, 0x01, 0x64, 0x00, 0x9C};
  EXPECT_EQ(packet, expected);
}

TEST(TestServoProtocol, LoadUnloadWritePacket) {
  const std::vector<uint8_t> packet = ghsc::HiwonderServoController::buildWritePacket(
      1, ghsc::HIWONDER_SERVO_LOAD_OR_UNLOAD_WRITE, {1});
  const std::vector<uint8_t> expected = {0x55, 0x55, 0x01, 0x04, 0x1F,
                                         0x01, 0xDA};
  EXPECT_EQ(packet, expected);
}

TEST(TestServoProtocol, PositionReadPacket) {
  const std::vector<uint8_t> packet = ghsc::HiwonderServoController::buildReadPacket(
      3, ghsc::HIWONDER_SERVO_POS_READ);
  const std::vector<uint8_t> expected = {0x55, 0x55, 0x03, 0x03, 0x1C, 0xDD};
  EXPECT_EQ(packet, expected);
}

// ---- 响应帧校验与解析 ----

TEST(TestServoProtocol, ValidPositionResponseParsedToPulse) {
  const std::vector<uint8_t> response = {0x55, 0x55, 0x02, 0x05, 0x1C,
                                         0xF4, 0x01, 0xE7};
  EXPECT_TRUE(ghsc::HiwonderServoController::verifyResponse(response));
  EXPECT_EQ(ghsc::HiwonderServoController::parseTwoByteResult(response), 500);
  EXPECT_EQ(ghsc::HiwonderServoController::parseResult(response), 500);
}

TEST(TestServoProtocol, BadChecksumRejected) {
  const std::vector<uint8_t> response = {0x55, 0x55, 0x02, 0x05, 0x1C,
                                         0xF4, 0x01, 0xE8};
  EXPECT_FALSE(ghsc::HiwonderServoController::verifyResponse(response));
}

TEST(TestServoProtocol, BadPrefixRejected) {
  const std::vector<uint8_t> response = {0x55, 0x54, 0x02, 0x05, 0x1C,
                                         0xF4, 0x01, 0xE7};
  EXPECT_FALSE(ghsc::HiwonderServoController::verifyResponse(response));
}

TEST(TestServoProtocol, ShortResponseRejected) {
  EXPECT_FALSE(
      ghsc::HiwonderServoController::verifyResponse({0x55, 0x55}));
  EXPECT_FALSE(
      ghsc::HiwonderServoController::parseTwoByteResult({0x55, 0x55}));
}

TEST(TestServoProtocol, SingleByteResultParsed) {
  // LOAD_OR_UNLOAD_READ 响应：LEN=4，单字节结果 0x01。
  const std::vector<uint8_t> response = {0x55, 0x55, 0x01, 0x04, 0x20,
                                         0x01, 0xD9};
  EXPECT_TRUE(ghsc::HiwonderServoController::verifyResponse(response));
  EXPECT_EQ(ghsc::HiwonderServoController::parseResult(response), 1);
}

TEST(TestServoProtocol, PairResultParsed) {
  // ANGLE_LIMIT_READ 响应：LEN=7，两个双字节结果 (256, 1000)。
  const std::vector<uint8_t> response = {0x55, 0x55, 0x01, 0x07, 0x15,
                                         0x00, 0x01, 0xE8, 0x03, 0xF6};
  EXPECT_TRUE(ghsc::HiwonderServoController::verifyResponse(response));
  EXPECT_EQ(ghsc::HiwonderServoController::parsePairResult(response),
            (std::pair<int, int>(256, 1000)));
}

// ---- 角度换算（rad <-> 脉冲） ----

TEST(TestServoConversions, MidpointMapsToZeroRadians) {
  EXPECT_EQ(ghsc::radToServo(0.0, 1), 500);
  EXPECT_EQ(ghsc::radToServo(0.0, -1), 500);
  EXPECT_NEAR(ghsc::servoToRad(500, 1), 0.0, 1e-12);
  EXPECT_NEAR(ghsc::servoToRad(500, -1), 0.0, 1e-12);
}

TEST(TestServoConversions, KnownAngleToPulse) {
  EXPECT_EQ(ghsc::radToServo(deg2rad(48.0), 1), 700);
  EXPECT_EQ(ghsc::radToServo(deg2rad(48.0), -1), 300);
  EXPECT_EQ(ghsc::radToServo(deg2rad(-48.0), 1), 300);
  EXPECT_EQ(ghsc::radToServo(deg2rad(-48.0), -1), 700);
}

TEST(TestServoConversions, PulseClampedToRange) {
  EXPECT_EQ(ghsc::radToServo(deg2rad(500.0), 1), 1000);
  EXPECT_EQ(ghsc::radToServo(deg2rad(-500.0), 1), 0);
}

TEST(TestServoConversions, PulseToRadians) {
  EXPECT_NEAR(ghsc::servoToRad(700, 1), deg2rad(48.0), 1e-12);
  EXPECT_NEAR(ghsc::servoToRad(300, -1), deg2rad(48.0), 1e-12);
}

TEST(TestServoConversions, RoundTripPreservesPulse) {
  const std::vector<int> pulses = {0, 250, 500, 750, 1000};
  for (int pulse : pulses) {
    for (int direction : {1, -1}) {
      EXPECT_EQ(ghsc::radToServo(ghsc::servoToRad(pulse, direction),
                                 direction),
                pulse);
    }
  }
}

// Python round() 是银行家舍入（round-half-to-even），nearbyint 与之等价。
TEST(TestServoConversions, RoundingMatchesPythonBankersRounding) {
  EXPECT_EQ(ghsc::roundHalfToEven(502.5), 502);
  EXPECT_EQ(ghsc::roundHalfToEven(501.5), 502);
  EXPECT_EQ(ghsc::roundHalfToEven(500.5), 500);
}

// ---- 方向参数解析与校验 ----

TEST(TestServoDirections, StringFormParsed) {
  auto parsed = ghsc::parseDirectionsString("[1, -1, 1, -1, 1, -1, 1, -1, -1]");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(*parsed,
            (std::vector<int>{1, -1, 1, -1, 1, -1, 1, -1, -1}));
}

TEST(TestServoDirections, BracketlessFormParsed) {
  auto parsed = ghsc::parseDirectionsString("1,-1,1");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(*parsed, (std::vector<int>{1, -1, 1}));
}

TEST(TestServoDirections, MalformedStringRejected) {
  EXPECT_FALSE(ghsc::parseDirectionsString("abc").has_value());
  EXPECT_FALSE(ghsc::parseDirectionsString("").has_value());
  EXPECT_FALSE(ghsc::parseDirectionsString("[]").has_value());
  EXPECT_FALSE(ghsc::parseDirectionsString("[1,2,").has_value());
}

TEST(TestServoDirections, ValidationRules) {
  const std::vector<int> valid = {1, -1, 1, -1, 1, -1, 1, -1, -1};
  EXPECT_TRUE(ghsc::validateDirections(valid, 9));
  EXPECT_FALSE(ghsc::validateDirections(valid, 8));  // 长度不符。
  const std::vector<int> bad_value = {1, 2, 1, -1, 1, -1, 1, -1, -1};
  EXPECT_FALSE(ghsc::validateDirections(bad_value, 9));  // 出现非 ±1。
  const std::vector<int> too_short = {1, -1, 1};
  EXPECT_FALSE(ghsc::validateDirections(too_short, 9));
}

// ---- 板级上电决策 ----

TEST(TestServoPowerDecision, AllReadyAndPoweredOn) {
  EXPECT_TRUE(ghsc::computeRequestedOn({true, true, true},
                                       {true, true, true}));
}

TEST(TestServoPowerDecision, AnyLegWithoutTargetKeepsPowerOff) {
  EXPECT_FALSE(ghsc::computeRequestedOn({true, true, false},
                                        {true, true, true}));
  EXPECT_FALSE(ghsc::computeRequestedOn({true, false, true},
                                        {true, true, true}));
}

TEST(TestServoPowerDecision, AnyLegRequestingUnloadKeepsPowerOff) {
  EXPECT_FALSE(ghsc::computeRequestedOn({true, true, true},
                                        {true, false, true}));
  EXPECT_FALSE(ghsc::computeRequestedOn({true, true, true},
                                        {false, false, false}));
}

TEST(TestServoPowerDecision, SizeMismatchIsNeverOn) {
  EXPECT_FALSE(ghsc::computeRequestedOn({true, true}, {true, true, true}));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
