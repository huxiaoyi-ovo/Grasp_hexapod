// 夹爪状态机纯判定函数回归：位置分类、夹紧偏差三态判定、受限门控。
// 与真实串口/时序无关，纯 CPU 测试。
#include <gtest/gtest.h>

#include "grasp_hexapod_servo_cpp/gripper_manager.h"
#include "grasp_hexapod_servo_cpp/servo_utils.h"

namespace ghsc = grasp_hexapod_servo_cpp;

using ghsc::ClampVerdict;
using ghsc::GripperPositionClass;
using ghsc::GripperState;
using ghsc::clampGripperTravel;
using G = ghsc::GripperManager;

// ---- 位置分类：683 左右为打开，840 左右为夹紧 ----

TEST(TestGripperClassify, OpenAround683) {
  EXPECT_EQ(G::classifyPulse(683, 683, 840, 20), GripperPositionClass::kOpen);
  EXPECT_EQ(G::classifyPulse(663, 683, 840, 20), GripperPositionClass::kOpen);
  EXPECT_EQ(G::classifyPulse(703, 683, 840, 20), GripperPositionClass::kOpen);
}

TEST(TestGripperClassify, ClampAround840) {
  EXPECT_EQ(G::classifyPulse(840, 683, 840, 20), GripperPositionClass::kClamped);
  EXPECT_EQ(G::classifyPulse(820, 683, 840, 20), GripperPositionClass::kClamped);
  EXPECT_EQ(G::classifyPulse(860, 683, 840, 20), GripperPositionClass::kClamped);
}

TEST(TestGripperClassify, BoundaryExcluded) {
  // 容差边界外一格不属于两端。
  EXPECT_EQ(G::classifyPulse(662, 683, 840, 20), GripperPositionClass::kIntermediate);
  EXPECT_EQ(G::classifyPulse(704, 683, 840, 20), GripperPositionClass::kIntermediate);
  EXPECT_EQ(G::classifyPulse(819, 683, 840, 20), GripperPositionClass::kIntermediate);
  EXPECT_EQ(G::classifyPulse(861, 683, 840, 20), GripperPositionClass::kIntermediate);
}

TEST(TestGripperClassify, MidTravelIsIntermediate) {
  EXPECT_EQ(G::classifyPulse(500, 683, 840, 20), GripperPositionClass::kIntermediate);
  EXPECT_EQ(G::classifyPulse(761, 683, 840, 20), GripperPositionClass::kIntermediate);
}

TEST(TestGripperClassify, CustomTargetsAndTolerance) {
  // 参数化目标：换一组开/合脉冲与容差同样成立。
  EXPECT_EQ(G::classifyPulse(300, 300, 700, 10), GripperPositionClass::kOpen);
  EXPECT_EQ(G::classifyPulse(700, 300, 700, 10), GripperPositionClass::kClamped);
  EXPECT_EQ(G::classifyPulse(311, 300, 700, 10), GripperPositionClass::kIntermediate);
}

// ---- 夹紧偏差判定：小偏差算夹紧，偏差 >=50 算大，中间继续等 ----

TEST(TestGripperVerdict, SmallDeviationCountsAsClamped) {
  EXPECT_EQ(G::evaluateClampDeviation(0, 20, 50), ClampVerdict::kClampedOk);
  EXPECT_EQ(G::evaluateClampDeviation(20, 20, 50), ClampVerdict::kClampedOk);
}

TEST(TestGripperVerdict, LargeDeviationFlagsBlock) {
  EXPECT_EQ(G::evaluateClampDeviation(50, 20, 50), ClampVerdict::kLargeDeviation);
  EXPECT_EQ(G::evaluateClampDeviation(80, 20, 50), ClampVerdict::kLargeDeviation);
}

TEST(TestGripperVerdict, MidDeviationKeepsPolling) {
  EXPECT_EQ(G::evaluateClampDeviation(21, 20, 50), ClampVerdict::kInTransit);
  EXPECT_EQ(G::evaluateClampDeviation(49, 20, 50), ClampVerdict::kInTransit);
}

// ---- 受限门控：夹紧失败后必须先 open 复位 ----

TEST(TestGripperRestricted, ClampBlockedOnlyInRestrictedState) {
  EXPECT_TRUE(G::clampAllowed(GripperState::kUnknown));
  EXPECT_TRUE(G::clampAllowed(GripperState::kOffline));
  EXPECT_TRUE(G::clampAllowed(GripperState::kOpen));
  EXPECT_TRUE(G::clampAllowed(GripperState::kClamped));
  EXPECT_FALSE(G::clampAllowed(GripperState::kRestricted));
}

// ---- 盲控行程钳位：方向键目标收敛到真实机械行程，杜绝超程堵转 ----

TEST(TestGripperTravel, DpadTargetsMapToTravelEndpoints) {
  // 方向键 ±1.5 rad 换算 142/858，钳位后恰为打开 683 / 夹紧 840。
  EXPECT_EQ(clampGripperTravel(142, 683, 840), 683);
  EXPECT_EQ(clampGripperTravel(858, 683, 840), 840);
}

TEST(TestGripperTravel, InTravelValueUnchanged) {
  EXPECT_EQ(clampGripperTravel(700, 683, 840), 700);
  EXPECT_EQ(clampGripperTravel(683, 683, 840), 683);
  EXPECT_EQ(clampGripperTravel(840, 683, 840), 840);
}

TEST(TestGripperTravel, OvershootClampedBothSides) {
  // 超出打开位（<683）与超出夹紧位（>840）都收敛到端点。
  EXPECT_EQ(clampGripperTravel(0, 683, 840), 683);
  EXPECT_EQ(clampGripperTravel(280, 683, 840), 683);
  EXPECT_EQ(clampGripperTravel(1000, 683, 840), 840);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
