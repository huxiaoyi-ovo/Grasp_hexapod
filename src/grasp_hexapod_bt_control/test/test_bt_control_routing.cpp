// bt_control_node 纯函数单测：反馈门控与 /cmd_vel 限幅。
#include "grasp_hexapod_bt_control/bt_control_node.h"

#include <array>
#include <cmath>

#include <gtest/gtest.h>

using grasp_hexapod_control_cpp::JointAngles;
using grasp_hexapod_control_cpp::kJointCount;
using grasp_hexapod_control_cpp::kLegCount;
using grasp_hexapod_bt_control::BtControlNode;

namespace {

JointAngles makeQ(double value) {
  JointAngles q;
  for (int leg = 0; leg < kLegCount; ++leg) {
    for (int joint = 0; joint < kJointCount; ++joint) {
      q[leg][joint] = value;
    }
  }
  return q;
}

}  // namespace

// BtControlNode 构造函数依赖 ROS master，这里只测静态/纯函数。
TEST(FeedbackGate, AllFreshSameBoardGivesReady) {
  JointAngles q = makeQ(0.1);
  std::array<double, 6> stamps{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::array<double, 6> last{};
  const auto [ready, complete] = BtControlNode::feedbackFrameState(
      q, stamps, last, 1.05, 0.30, 0.20);
  EXPECT_TRUE(ready);
  EXPECT_TRUE(complete);
}

TEST(FeedbackGate, StaleLegBlocksReady) {
  JointAngles q = makeQ(0.1);
  std::array<double, 6> stamps{1.0, 1.0, 1.0, 1.0, 1.0, 0.5};
  std::array<double, 6> last{};
  const auto [ready, complete] = BtControlNode::feedbackFrameState(
      q, stamps, last, 1.05, 0.30, 0.20);
  EXPECT_FALSE(ready);
  EXPECT_FALSE(complete);
}

TEST(FeedbackGate, SkewedBoardsBlockComplete) {
  JointAngles q = makeQ(0.1);
  std::array<double, 6> stamps{1.0, 1.0, 1.0, 1.3, 1.3, 1.3};
  std::array<double, 6> last{};
  const auto [ready, complete] = BtControlNode::feedbackFrameState(
      q, stamps, last, 1.31, 0.30, 0.20);
  EXPECT_FALSE(ready);  // 0.3 > 0.2 skew limit
  EXPECT_FALSE(complete);
}

TEST(FeedbackIssue, ReportsMissingLegs) {
  JointAngles q = makeQ(0.1);
  std::array<double, 6> stamps{};
  const std::string issue =
      BtControlNode::feedbackIssue(q, stamps, 1.0, 0.3);
  EXPECT_NE(issue.find("left[lf=missing"), std::string::npos);
  EXPECT_NE(issue.find("right[rf=missing"), std::string::npos);
  EXPECT_NE(issue.find("snapshot_skew=nan"), std::string::npos);
}

TEST(ClampCommand, PreservesDirectionAndClampsNorm) {
  const Eigen::Vector4d clamped =
      BtControlNode::clampCommand(3.0, 4.0, 9.9, 0.20, 0.50);
  // 平面模 5 → 按 0.20 限幅并保持方向 (3,4)/5。
  EXPECT_NEAR(clamped[0], 0.20 * 0.6, 1e-9);
  EXPECT_NEAR(clamped[1], 0.20 * 0.8, 1e-9);
  EXPECT_NEAR(clamped[2], 0.0, 1e-9);   // 无 z 轴线速度
  EXPECT_NEAR(clamped[3], 0.50, 1e-9);  // 航向单独限幅
}

TEST(ClampCommand, SmallCommandPassesThrough) {
  const Eigen::Vector4d clamped =
      BtControlNode::clampCommand(0.05, -0.05, -0.1, 0.20, 0.50);
  EXPECT_NEAR(clamped[0], 0.05, 1e-9);
  EXPECT_NEAR(clamped[1], -0.05, 1e-9);
  EXPECT_NEAR(clamped[2], 0.0, 1e-9);
  EXPECT_NEAR(clamped[3], -0.1, 1e-9);
}

TEST(ClampCommand, NonFiniteBecomesZero) {
  const Eigen::Vector4d clamped =
      BtControlNode::clampCommand(std::nan(""), std::nan(""), std::nan(""),
                                  0.20, 0.50);
  EXPECT_EQ(clamped, Eigen::Vector4d::Zero());
}
