#include <limits>

#include <gtest/gtest.h>

#include "grasp_hexapod_bt_control/visual_guidance.h"

namespace grasp_hexapod_bt_control {
namespace {

VisualObservation observation(double x, double z, double stamp) {
  return {true, x, z, stamp, "camera_color_optical_frame"};
}

VisualHealth health(double stamp, int state = VisualHealth::TARGET_VALID) {
  VisualHealth value;
  value.state = state;
  value.stamp = value.rgb_stamp = value.depth_stamp = stamp;
  value.frame = "camera_color_optical_frame";
  return value;
}

void cacheTarget(VisualGuidance& guidance, const VisualObservation& value) {
  guidance.cacheHealth(health(value.stamp));
  guidance.cache(value);
}

TEST(VisualGuidance, SearchUsesYawIndexAndRightTargetTurnsNegative) {
  VisualGuidance guidance(VisualGuidanceConfig{});
  guidance.begin(VisualState::Searching, 1.0);
  cacheTarget(guidance, observation(0.2, 1.0, 1.01));
  const Eigen::Vector4d command = guidance.update(1.02, true);
  EXPECT_EQ(command[0], 0.0);
  EXPECT_EQ(command[1], 0.0);
  EXPECT_EQ(command[2], 0.0);
  EXPECT_LT(command[3], 0.0);
}

TEST(VisualGuidance, ApproachUsesForwardAndYawIndices) {
  VisualGuidance guidance(VisualGuidanceConfig{});
  guidance.begin(VisualState::Approaching, 1.0);
  cacheTarget(guidance, observation(0.01, 1.0, 1.01));
  const Eigen::Vector4d command = guidance.update(1.02, true);
  EXPECT_EQ(command[0], 0.0);
  EXPECT_GT(command[1], 0.0);
  EXPECT_EQ(command[2], 0.0);
  EXPECT_LT(command[3], 0.0);
}

TEST(VisualGuidance, InvalidOrFutureSourceDoesNotPoisonCorrectedSample) {
  VisualGuidance guidance(VisualGuidanceConfig{});
  guidance.begin(VisualState::Approaching, 10.0);
  guidance.cacheHealth(health(10.01));
  guidance.cache(observation(0.0, 1.0, 100.0));
  EXPECT_EQ(guidance.update(10.01, true), Eigen::Vector4d::Zero());
  cacheTarget(guidance, observation(0.0, 1.0, 10.02));
  EXPECT_GT(guidance.update(10.03, true)[1], 0.0);

  VisualObservation nan = observation(0.0, 1.0, 10.04);
  nan.x = std::numeric_limits<double>::quiet_NaN();
  guidance.cache(nan);
  EXPECT_EQ(guidance.update(10.05, true), Eigen::Vector4d::Zero());
  VisualObservation empty_frame = observation(0.0, 1.0, 10.06);
  empty_frame.frame.clear();
  guidance.cache(empty_frame);
  EXPECT_EQ(guidance.update(10.07, true), Eigen::Vector4d::Zero());
}

TEST(VisualGuidance, HeldCallbackZerosButCachedControlReadRemainsUsable) {
  VisualGuidance guidance(VisualGuidanceConfig{});
  guidance.begin(VisualState::Approaching, 1.0);
  cacheTarget(guidance, observation(0.0, 1.0, 1.01));
  EXPECT_GT(guidance.update(1.02, true)[1], 0.0);
  EXPECT_GT(guidance.update(1.10, true)[1], 0.0);
  guidance.cacheHealth(health(1.11));
  guidance.cache(observation(0.0, 1.0, 1.01));
  EXPECT_EQ(guidance.update(1.11, true), Eigen::Vector4d::Zero());
}

TEST(VisualGuidance, SourceAgeAndBearingBlockForwardMotion) {
  VisualGuidance guidance(VisualGuidanceConfig{});
  guidance.begin(VisualState::Approaching, 1.0);
  cacheTarget(guidance, observation(0.3, 1.0, 1.01));
  const Eigen::Vector4d off_axis = guidance.update(1.02, true);
  EXPECT_EQ(off_axis[1], 0.0);
  EXPECT_LT(off_axis[3], 0.0);
  EXPECT_EQ(guidance.update(1.30, true), Eigen::Vector4d::Zero());
}

TEST(VisualGuidance, SearchTimeoutStillAppliesDuringRecoveryDrain) {
  VisualGuidanceConfig config;
  config.confirm_frames = 1.0;
  config.search_timeout_s = 0.2;
  VisualGuidance guidance(config);
  guidance.begin(VisualState::Searching, 1.0);
  cacheTarget(guidance, observation(0.0, 1.0, 1.01));
  guidance.update(1.02, true);
  EXPECT_EQ(guidance.state(), VisualState::Locking);
  guidance.cacheHealth(health(1.03));
  guidance.cache(observation(0.0, 1.0, 1.01));
  guidance.update(1.03, false);
  EXPECT_EQ(guidance.state(), VisualState::Searching);
  guidance.update(1.21, false);
  EXPECT_EQ(guidance.state(), VisualState::Failed);
}

TEST(VisualGuidance, SettleNeedsContinuousFreshSamplesAndStoppedGait) {
  VisualGuidance guidance(VisualGuidanceConfig{});
  guidance.begin(VisualState::Approaching, 1.0);
  cacheTarget(guidance, observation(0.0, 0.5, 1.01));
  guidance.update(1.02, false);
  EXPECT_NE(guidance.state(), VisualState::Success);
  cacheTarget(guidance, observation(0.0, 0.5, 1.30));
  guidance.update(1.31, true);
  EXPECT_NE(guidance.state(), VisualState::Success);
  guidance.update(1.60, true);
  EXPECT_NE(guidance.state(), VisualState::Success);
  cacheTarget(guidance, observation(0.0, 0.5, 1.61));
  guidance.update(1.62, true);
  EXPECT_NE(guidance.state(), VisualState::Success);
  cacheTarget(guidance, observation(0.0, 0.5, 1.90));
  guidance.update(1.91, true);
  EXPECT_EQ(guidance.state(), VisualState::Success);
}

TEST(VisualGuidance, ResumeRequiresSampleNewerThanPauseFloor) {
  VisualGuidance guidance(VisualGuidanceConfig{});
  guidance.begin(VisualState::Searching, 1.0);
  cacheTarget(guidance, observation(0.0, 1.0, 1.01));
  guidance.update(1.02, true);
  EXPECT_EQ(guidance.confirmations(), 1);
  guidance.resume(1.10);
  guidance.update(1.11, true);
  EXPECT_EQ(guidance.confirmations(), 0);
  cacheTarget(guidance, observation(0.0, 1.0, 1.12));
  guidance.update(1.13, true);
  EXPECT_EQ(guidance.confirmations(), 1);
}

TEST(VisualGuidance, InvalidConfigRejected) {
  VisualGuidanceConfig config;
  config.confirm_frames = 2.5;
  EXPECT_THROW(VisualGuidance guidance(config), std::invalid_argument);
}

TEST(VisualGuidance, NoHealthRemainsZeroThenFailsStartup) {
  VisualGuidance guidance(VisualGuidanceConfig{});
  guidance.begin(VisualState::Searching, 1.0);
  EXPECT_EQ(guidance.update(1.1, true), Eigen::Vector4d::Zero());
  guidance.update(3.1, true);
  EXPECT_EQ(guidance.state(), VisualState::Failed);
}

TEST(VisualGuidance, HealthyNegativeSearchYawIsCapped) {
  VisualGuidanceConfig config;
  config.search_yaw_rate = .3;
  config.max_yaw_rate = .15;
  VisualGuidance guidance(config);
  guidance.begin(VisualState::Searching, 1.0);
  guidance.cacheHealth(health(1.01, VisualHealth::NO_TARGET));
  EXPECT_DOUBLE_EQ(guidance.update(1.02, true)[3], .15);
}

TEST(VisualGuidance, HealthyPositiveWithoutTargetIsZero) {
  VisualGuidance guidance(VisualGuidanceConfig{});
  guidance.begin(VisualState::Approaching, 1.0);
  guidance.cacheHealth(health(1.01));
  EXPECT_EQ(guidance.update(1.02, true), Eigen::Vector4d::Zero());
}

TEST(VisualGuidance, FloatStopBandSettlesWithFreshHealthAndTargets) {
  VisualGuidance guidance(VisualGuidanceConfig{});
  guidance.begin(VisualState::Approaching, 1.0);
  cacheTarget(guidance, observation(0., double(float(.60f)), 1.01));
  guidance.update(1.02, true);
  cacheTarget(guidance, observation(0., .601, 1.30));
  guidance.update(1.31, true);
  cacheTarget(guidance, observation(0., .601, 1.60));
  guidance.update(1.61, true);
  EXPECT_EQ(guidance.state(), VisualState::Success);
}

TEST(VisualGuidance, SearchHealthExpiryZerosThenFails) {
  VisualGuidance guidance(VisualGuidanceConfig{});
  guidance.begin(VisualState::Searching, 1.0);
  guidance.cacheHealth(health(1.01, VisualHealth::NO_TARGET));
  EXPECT_GT(guidance.update(1.02, true)[3], 0.);
  EXPECT_EQ(guidance.update(1.30, true), Eigen::Vector4d::Zero());
  guidance.update(1.60, true);
  EXPECT_EQ(guidance.state(), VisualState::Failed);
}

TEST(VisualGuidance, PendingHealthNeverAuthorizesSearch) {
  VisualGuidance guidance(VisualGuidanceConfig{});
  guidance.begin(VisualState::Searching, 1.0);
  guidance.cacheHealth(health(1.01, VisualHealth::TARGET_PENDING));
  EXPECT_EQ(guidance.update(1.02, true), Eigen::Vector4d::Zero());
  guidance.update(3.1, true);
  EXPECT_EQ(guidance.state(), VisualState::Failed);
}

TEST(VisualGuidance, MapFrameDetectedTargetFails) {
  VisualGuidance guidance(VisualGuidanceConfig{});
  guidance.begin(VisualState::Approaching, 1.0);
  guidance.cacheHealth(health(1.01));
  auto target = observation(0., 1., 1.01);
  target.frame = "map";
  guidance.cache(target);
  EXPECT_EQ(guidance.update(1.02, true), Eigen::Vector4d::Zero());
  EXPECT_EQ(guidance.state(), VisualState::Failed);
  EXPECT_EQ(guidance.reason(), "FRAME_MISMATCH");
}

TEST(VisualGuidance, RepeatedHealthCallbacksWithOldSourceDoNotExtendDeadline) {
  VisualGuidance guidance(VisualGuidanceConfig{});
  guidance.begin(VisualState::Searching, 1.0);
  guidance.cacheHealth(health(1.1, VisualHealth::NO_TARGET));
  EXPECT_GT(guidance.update(1.11, true)[3], 0.0);
  for (double now : {1.2, 1.3, 1.4, 1.5}) {
    guidance.cacheHealth(health(1.1, VisualHealth::NO_TARGET));
    guidance.update(now, true);
  }
  guidance.cacheHealth(health(1.1, VisualHealth::NO_TARGET));
  EXPECT_EQ(guidance.update(1.61, true), Eigen::Vector4d::Zero());
  EXPECT_EQ(guidance.state(), VisualState::Failed);
}

TEST(VisualGuidance, FutureOrInfiniteDepthHealthIsRejected) {
  VisualGuidance guidance(VisualGuidanceConfig{});
  guidance.begin(VisualState::Approaching, 2.0);
  VisualHealth future_depth = health(2.01);
  future_depth.depth_stamp = 2.03;
  guidance.cacheHealth(future_depth);
  EXPECT_EQ(guidance.update(2.02, true), Eigen::Vector4d::Zero());

  VisualHealth infinite_depth = health(2.02);
  infinite_depth.depth_stamp = std::numeric_limits<double>::infinity();
  guidance.cacheHealth(infinite_depth);
  EXPECT_EQ(guidance.update(2.03, true), Eigen::Vector4d::Zero());
}

TEST(VisualGuidance, ResumeNeedsFreshHealthAndTargetTogether) {
  VisualGuidance guidance(VisualGuidanceConfig{});
  guidance.begin(VisualState::Approaching, 1.0);
  cacheTarget(guidance, observation(0.0, 1.0, 1.01));
  EXPECT_GT(guidance.update(1.02, true)[1], 0.0);

  guidance.resume(1.10);
  EXPECT_EQ(guidance.update(1.11, true), Eigen::Vector4d::Zero());
  guidance.cacheHealth(health(1.12));
  EXPECT_EQ(guidance.update(1.13, true), Eigen::Vector4d::Zero());
  cacheTarget(guidance, observation(0.0, 1.0, 1.14));
  EXPECT_GT(guidance.update(1.15, true)[1], 0.0);
}

TEST(VisualGuidance, LateFirstNoTargetHealthStartsThenTargetLossTimesOut) {
  VisualGuidance guidance(VisualGuidanceConfig{});
  guidance.begin(VisualState::Approaching, 1.0);
  guidance.cacheHealth(health(2.5, VisualHealth::NO_TARGET));
  EXPECT_EQ(guidance.update(2.51, true), Eigen::Vector4d::Zero());
  EXPECT_NE(guidance.state(), VisualState::Failed);

  for (double stamp : {2.75, 3.0, 3.25, 3.5, 3.51}) {
    guidance.cacheHealth(health(stamp, VisualHealth::NO_TARGET));
    guidance.update(stamp + .01, true);
  }
  EXPECT_EQ(guidance.state(), VisualState::Failed);
}

}  // namespace
}  // namespace grasp_hexapod_bt_control
