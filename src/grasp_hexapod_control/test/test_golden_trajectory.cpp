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
  } else if (name == "climb_front_direct") {
    // front compact 仿真回放：Python 拒绝 front + hardware_execution，因此两侧都用
    // 非 hardware 模式（time-only 阶段推进 + front 的 simulation_joint_velocity_limit
    // 6 rad/s 步进上限），不改写配置自带的 settle_gate。
    g::ClimbCompactConfig config = g::loadClimbCompact(t::frontDirectConfigPath());
    config = g::selectCompactClimbSide(config, "left");
    controller.enterClimb(q_cur, config, 0, std::nullopt, false);
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

  if (name == "climb_left" || name == "climb_front_direct") {
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

TEST(GoldenTrajectory, ClimbFrontDirectMatchesPython) {
  compareSequence(t::loadGolden(), "climb_front_direct", 1e-6, 1e-6);
}

namespace {

double maxStepOfFrames(const g::JointAngles& previous, const g::JointAngles& next) {
  return t::maxAbsDiff(previous, next);
}

}  // namespace

// front + hardware_execution：默认拒绝（与 Python 同一消息）；仅显式授权后接受。
TEST(ClimbFrontHardware, RejectedWithoutFlagAndAcceptedWithFlag) {
  const double dt = t::loadGolden()["dt"].get<double>();
  g::ClimbCompactConfig config = g::loadClimbCompact(t::frontDirectConfigPath());
  ASSERT_TRUE(config.isFront());
  ASSERT_FALSE(config.allow_front_hardware) << "flag must default to false";

  g::GraspController controller(dt, false, false, t::goldenPaths());
  const g::JointAngles q_stand = g::makeQStand();
  try {
    controller.enterClimb(q_stand, config, 0, std::nullopt, true);
    FAIL() << "front + hardware must be rejected without allow_front_hardware";
  } catch (const g::ConfigError& error) {
    EXPECT_STREQ(error.what(), "front compact climbing is simulation-only");
  }
  EXPECT_NE(controller.climb_mode.state, g::ClimbState::Running)
      << "a rejected entry must leave the climb session untouched";

  // 拒绝不得破坏正在进行的侧向会话。
  g::ClimbCompactConfig side = g::loadClimbCompact(t::goldenPaths().climb_compact_json);
  side = g::selectCompactClimbSide(side, "left");
  controller.enterClimb(q_stand, side, 0, std::nullopt, true);
  const std::string side_phase = controller.climb_mode.phase;
  ASSERT_EQ(controller.climb_mode.state, g::ClimbState::Running);
  EXPECT_THROW(controller.climb_mode.enter(q_stand, config, 0, std::nullopt, true),
               g::ConfigError);
  EXPECT_EQ(controller.climb_mode.state, g::ClimbState::Running);
  EXPECT_EQ(controller.climb_mode.phase, side_phase);
  EXPECT_FALSE(controller.climb_mode.currentConfig().isFront());

  // front + 非 hardware 不需要该标志。
  g::GraspController sim_controller(dt, false, false, t::goldenPaths());
  EXPECT_NO_THROW(sim_controller.enterClimb(q_stand, config, 0, std::nullopt, false));

  // 显式授权后接受；授权标志不会被 JSON 设置。
  config.allow_front_hardware = true;
  g::GraspController hardware_controller(dt, false, false, t::goldenPaths());
  EXPECT_NO_THROW(hardware_controller.enterClimb(q_stand, config, 0, std::nullopt, true));
  EXPECT_EQ(hardware_controller.climb_mode.state, g::ClimbState::Running);
  EXPECT_TRUE(hardware_controller.climb_mode.hardware_execution);
}

// hardware_execution 必须使用共享 4 rad/s 步进上限，忽略 simulation_joint_velocity_limit。
TEST(ClimbFrontHardware, HardwareModeIgnoresSimulationVelocityCap) {
  const t::json golden = t::loadGolden();
  const double dt = golden["dt"].get<double>();
  g::ClimbCompactConfig config = g::loadClimbCompact(t::frontDirectConfigPath());
  ASSERT_TRUE(config.simulation_joint_velocity_limit_rad_s.has_value());
  ASSERT_GT(*config.simulation_joint_velocity_limit_rad_s, 4.0);
  const double shared_step = 4.0 * dt;
  const double sim_step = *config.simulation_joint_velocity_limit_rad_s * dt;

  // 金标仿真帧确实需要 > 4 rad/s 的步进（否则本测试无法区分两种上限）。
  const t::json* sequence = nullptr;
  for (const t::json& item : golden["sequences"]) {
    if (item["name"] == "climb_front_direct") {
      sequence = &item;
    }
  }
  ASSERT_NE(sequence, nullptr);
  g::JointAngles previous = g::makeQStand();
  int sim_frames_above_shared_cap = 0;
  for (const t::json& frame : (*sequence)["frames"]) {
    const g::JointAngles q = t::toJoints(frame["q_des"].get<std::vector<double>>());
    const double step = maxStepOfFrames(previous, q);
    EXPECT_LE(step, sim_step + 1e-9);
    if (step > shared_step + 1e-9) {
      ++sim_frames_above_shared_cap;
    }
    previous = q;
  }
  ASSERT_GT(sim_frames_above_shared_cap, 0);

  // 同一配置的 hardware 回放：任何一帧都不得超过 4 rad/s，且共享上限确实发生裁剪。
  config.allow_front_hardware = true;
  g::GraspController controller(dt, false, false, t::goldenPaths());
  g::JointAngles q_cur = g::makeQStand();
  controller.enterClimb(q_cur, config, 0, std::nullopt, true);
  const Eigen::Vector4d zero_command = Eigen::Vector4d::Zero();
  double worst_step = 0.0;
  int clipped_frames = 0;
  for (int frame = 0; frame < 40000; ++frame) {
    const g::JointAngles q_des = controller.update(q_cur, zero_command);
    worst_step = std::max(worst_step, maxStepOfFrames(q_cur, q_des));
    if (controller.last_update_velocity_limit_clip_count > 0) {
      ++clipped_frames;
    }
    q_cur = q_des;
    if (controller.climb_mode.state == g::ClimbState::Done ||
        controller.climb_mode.state == g::ClimbState::Failed) {
      break;
    }
  }
  EXPECT_LE(worst_step, shared_step + 1e-9)
      << "hardware execution must use the shared 4 rad/s cap";
  EXPECT_GT(clipped_frames, 0) << "the shared 4 rad/s cap must actually clip";
}

// 非 hardware 仿真且 preview_time_only_stage_advance=false：走反馈检查点逻辑
// （超时只按控制 dt 推进）。逐帧与 Python 的对照见交叉验证（不进导出脚本：
// 本地 Python 3.13 的补偿 sum() 会让该路径的 Python 侧不可复现）。
TEST(ClimbFrontFeedbackSimulation, FinishesWithFeedbackCheckpointsAndSimulationCap) {
  const double dt = t::loadGolden()["dt"].get<double>();
  g::ClimbCompactConfig config = g::loadClimbCompact(t::frontDirectConfigPath());
  config.settle_gate.preview_time_only_stage_advance = false;
  config.settle_gate.tracking_errors_diagnostic_only = false;
  g::validateClimbCompact(config);

  g::GraspController controller(dt, false, false, t::goldenPaths());
  g::JointAngles q_cur = g::makeQStand();
  controller.enterClimb(q_cur, config, 0, std::nullopt, false);
  ASSERT_FALSE(controller.climb_mode.hardware_execution);
  const Eigen::Vector4d zero_command = Eigen::Vector4d::Zero();
  double worst_step = 0.0;
  for (int frame = 0; frame < 40000; ++frame) {
    const g::JointAngles q_des = controller.update(q_cur, zero_command);
    worst_step = std::max(worst_step, t::maxAbsDiff(q_cur, q_des));
    q_cur = q_des;
    if (controller.climb_mode.state == g::ClimbState::Done ||
        controller.climb_mode.state == g::ClimbState::Failed) {
      break;
    }
  }
  EXPECT_EQ(controller.climb_mode.state, g::ClimbState::Done)
      << controller.climb_mode.failure_reason;
  EXPECT_LE(worst_step, *config.simulation_joint_velocity_limit_rad_s * dt + 1e-9);
  EXPECT_GT(worst_step, 4.0 * dt) << "front simulation cap (6 rad/s) must apply";
}
