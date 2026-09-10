// 轨迹金标：reset/walk 与 compact 攀爬整链逐帧 q_des 对比（理想伺服回放）。
//
// 这是 C++ 移植上实机前的硬门槛：DLS 修正以 16*err*dt 累积进关节角，
// 任何实现差异都会被闭环放大。容差 1e-6 rad（足端 1e-6 m）。
#include "golden_helpers.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace g = grasp_hexapod_control_cpp;
namespace t = grasp_hexapod_control_cpp::testing;

namespace {

g::JointAngles replayFrame(g::GraspController& controller,
                           const g::JointAngles& q_cur,
                           const Eigen::Vector4d& command) {
  return controller.update(q_cur, command);
}

void compareSequence(const t::json& golden, const std::string& name,
                     double joint_tolerance, double foot_tolerance) {
  const t::json* sequence = nullptr;
  for (const t::json& item : golden["sequences"]) {
    if (item["name"] == name) {
      sequence = &item;
      break;
    }
  }
  ASSERT_NE(sequence, nullptr) << "golden sequence missing: " << name;

  const double dt = golden["dt"].get<double>();
  g::GraspController controller(dt, false, false, t::goldenPaths());
  g::JointAngles q_cur = g::makeQStand();
  const Eigen::Vector4d zero_command = Eigen::Vector4d::Zero();

  if (name == "reset_then_walk") {
    // 与 Python 导出脚本一致：扰动关节角 -> B 回站。
    g::JointAngles q_start = g::makeQStand();
    const double start_offsets[6][3] = {
        {10, -12, 8}, {-9, 11, -7}, {12, -10, 9}, {-11, 9, -8}, {9, -13, 11}, {-10, 8, -9}};
    for (int leg = 0; leg < g::kLegCount; ++leg) {
      for (int joint = 0; joint < g::kJointCount; ++joint) {
        q_start[leg][joint] += start_offsets[leg][joint] * M_PI / 180.0;
      }
    }
    controller.resetToStand(q_start);
    q_cur = q_start;
  } else if (name == "climb_left") {
    g::ClimbCompactConfig config = g::loadClimbCompact(t::goldenPaths().climb_compact_json);
    config = g::selectCompactClimbSide(config, "left");
    config.settle_gate.max_foot_target_error_m =
        golden["climb_foot_gate_m"].get<double>();
    controller.enterClimb(q_cur, config, 0, std::nullopt, true);
  } else {
    FAIL() << "unknown golden sequence: " << name;
  }

  const int reset_frames = static_cast<int>(std::lround(2.5 / dt));
  int frame_index = 0;
  double worst_joint = 0.0;
  double worst_foot = 0.0;
  int worst_joint_frame = -1;
  for (const t::json& expected : (*sequence)["frames"]) {
    Eigen::Vector4d command = Eigen::Vector4d::Zero();
    if (name == "reset_then_walk") {
      if (frame_index >= reset_frames && frame_index < reset_frames + 300) {
        command = Eigen::Vector4d(0.05, 0.10, 0.0, 0.15);
      } else if (frame_index >= reset_frames + 300 &&
                 frame_index < reset_frames + 600) {
        command = Eigen::Vector4d(-0.08, -0.05, 0.008, -0.20);
      }
    }
    const g::JointAngles q_des = replayFrame(controller, q_cur, command);
    const g::JointAngles expected_q =
        t::toJoints(expected["q_des"].get<std::vector<double>>());
    const g::FootPositions expected_feet =
        t::toFeet(expected["foot_desired"].get<std::vector<double>>());
    const double joint_diff = t::maxAbsDiff(q_des, expected_q);
    const double foot_diff =
        t::maxAbsDiff(controller.foot_desired_base, expected_feet);
    if (joint_diff > worst_joint) {
      worst_joint = joint_diff;
      worst_joint_frame = frame_index;
    }
    worst_foot = std::max(worst_foot, foot_diff);
    ASSERT_LT(joint_diff, joint_tolerance)
        << name << " frame " << frame_index << " q_des diverged (worst "
        << worst_joint << " at frame " << worst_joint_frame << ")";
    ASSERT_LT(foot_diff, foot_tolerance)
        << name << " frame " << frame_index << " foot target diverged";
    q_cur = q_des;
    ++frame_index;
  }
  EXPECT_EQ(frame_index, static_cast<int>((*sequence)["frames"].size()));

  if (name == "climb_left") {
    EXPECT_EQ(controller.climb_mode.state, g::ClimbState::Done)
        << "C++ climb replay must finish like Python";
  }
  testing::Test::RecordProperty("worst_joint_diff", std::to_string(worst_joint));
  testing::Test::RecordProperty("worst_foot_diff", std::to_string(worst_foot));
  testing::Test::RecordProperty("worst_joint_frame", std::to_string(worst_joint_frame));
}

}  // namespace

TEST(GoldenTrajectory, ResetAndWalkMatchPython) {
  compareSequence(t::loadGolden(), "reset_then_walk", 1e-6, 1e-6);
}

TEST(GoldenTrajectory, ClimbLeftMatchesPython) {
  compareSequence(t::loadGolden(), "climb_left", 1e-6, 1e-6);
}
