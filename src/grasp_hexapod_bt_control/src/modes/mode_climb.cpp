// climb 模式实现：紧凑攀爬入口门控、相对运动监控与速度诊断（逻辑与原
// bt_control_node.cpp 逐段等价，阈值与判断顺序不变）。
#include "grasp_hexapod_bt_control/modes/mode_climb.h"
#include "grasp_hexapod_bt_control/mode_registry.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace grasp_hexapod_bt_control {
namespace modes {

namespace {

using grasp_hexapod_control_cpp::ClimbCompactConfig;
using grasp_hexapod_control_cpp::ClimbState;
using grasp_hexapod_control_cpp::ConfigError;
using grasp_hexapod_control_cpp::kJointCount;
using grasp_hexapod_control_cpp::kLegCount;
using grasp_hexapod_control_cpp::rotationAngle;
using grasp_hexapod_control_cpp::selectCompactClimbSide;
using grasp_hexapod_control_cpp::validateCompactClimbSide;
using grasp_hexapod_control_cpp::worldFromBase;

std::string formatMessage(const char* format, ...) {
  char buffer[1024];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return buffer;
}

// 6×3 关节矩阵的最大绝对差分。
double maxAbsDiffLocal(const JointAngles& a, const JointAngles& b) {
  double worst = 0.0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    worst = std::max(worst, (a[leg] - b[leg]).cwiseAbs().maxCoeff());
  }
  return worst;
}

}  // namespace

ClimbMode::ClimbMode(ModeContext& ctx, const ModeTopics& topics)
    : ModeBase("climb", ctx, topics) {
  ros::NodeHandle& pnh = ctx.privateNodeHandle();
  enable_real_climb_ = pnh.param("enable_real_climb", false);
  climb_side_ = pnh.param<std::string>("climb_side", "left");
  validateCompactClimbSide(climb_side_);
  climb_foot_gate_m_ = pnh.param("climb_foot_gate_m", 0.02);
  if (!std::isfinite(climb_foot_gate_m_) || climb_foot_gate_m_ <= 0.0 ||
      climb_foot_gate_m_ > 0.10) {
    throw ConfigError("~climb_foot_gate_m must be finite and in (0, 0.10]");
  }
  persistence_frames_ = pnh.param("real_climb_persistence_frames", 3);
  max_position_error_ =
      pnh.param("real_climb_max_position_error_m", 0.05);
  max_orientation_error_ =
      pnh.param("real_climb_max_orientation_error_deg", 10.0) * M_PI / 180.0;
  max_angular_speed_ =
      pnh.param("real_climb_max_angular_speed_deg_s", 30.0) * M_PI / 180.0;
}

bool ClimbMode::start(const JointAngles& q_cur, std::string& reason) {
  GraspController& controller = ctx().controller();
  reason = "climb entry rejected";
  if (!enable_real_climb_) {
    ROS_WARN_THROTTLE(2.0, "climb rejected: enable_real_climb is false");
    return false;
  }
  if (ctx().machineState() != MachineState::Hold ||
      controller.mode != GraspController::kApproach) {
    ROS_WARN_THROTTLE(
        2.0, "climb rejected: reset must finish and controls must be fresh");
    return false;
  }
  try {
    ClimbCompactConfig config = controller.climb_mode.loadConfig();
    config = selectCompactClimbSide(config, climb_side_);
    config.settle_gate.max_foot_target_error_m = climb_foot_gate_m_;
    controller.enterClimb(q_cur, config, 0, std::nullopt, true);
  } catch (const ConfigError& error) {
    ROS_WARN("climb rejected: compact entry gate failed: %s", error.what());
    return false;
  }
  ROS_INFO("Compact climb side selected: %s", climb_side_.c_str());

  // 可选相对运动监控：起点参考（RTK/IMU/计划位姿）。
  const NavigationState navigation = ctx().inputs().navigationMotionSnapshot();
  const ImuSnapshot imu = ctx().inputs().imu();
  monitor_active_ = navigation.valid && imu.valid;
  if (monitor_active_) {
    climb_start_navigation_ = navigation;
    have_climb_start_navigation_ = true;
    climb_start_imu_rotation_ = imu.rotation;
    have_climb_start_imu_ = true;
    climb_start_planned_pose_ =
        worldFromBase(*controller.climb_mode.base_pose);
    have_climb_start_planned_ = true;
    ROS_INFO("Optional IMU/RTK relative-motion monitoring enabled");
  } else {
    have_climb_start_navigation_ = false;
    have_climb_start_imu_ = false;
    have_climb_start_planned_ = false;
    ROS_WARN(
        "IMU or RTK/LoRa unavailable at start; "
        "optional relative-motion monitoring disabled for this climb");
  }
  bad_frames_ = 0;
  good_frames_ = 0;
  resetSpeedDiagnostic();
  ctx().setMachineState(MachineState::Running);
  ROS_INFO(
      "climb accepted: diagnostic replay C1-C%d started with source=%s "
      "foot_gate_m=%.3f; joint-feedback gates are not contact/load evidence",
      static_cast<int>(controller.climb_mode.stage_names.size()),
      "hardware_feedback", climb_foot_gate_m_);
  return true;
}

void ClimbMode::pause(const std::string& reason) {
  ModeBase::pause(reason);
  // 冻结攀爬推进但保留阶段状态；恢复时 resume() 继续。
  if (ctx().controller().mode == GraspController::kClimb) {
    ctx().controller().holdClimb();
  }
}

void ClimbMode::resume() {
  ModeBase::resume();
  if (ctx().controller().mode == GraspController::kClimb) {
    ctx().controller().resumeClimb();
  }
}

bool ClimbMode::onNodeHold(const std::string& reason) {
  flushSpeedDiagnostic("hold: " + reason);
  return false;  // 通用路径：取消 approach + holdClimb + Hold。
}

std::optional<JointAngles> ClimbMode::step(const ModeStepInput& input) {
  GraspController& controller = ctx().controller();
  std::optional<std::string> climb_stage;
  if (controller.mode == GraspController::kClimb &&
      controller.climb_mode.state == ClimbState::Running) {
    climb_stage = controller.climb_mode.phase;
  }
  const JointAngles q_des =
      controller.update(input.q_cur, Eigen::Vector4d::Zero());
  warnHardwareClimbPhaseHold();
  infoHardwareClimbActiveTrace();
  if (climb_stage.has_value() && input.feedback_stamp != nullptr) {
    recordSpeedDiagnostic(*climb_stage, input.q_cur, q_des,
                          *input.feedback_stamp, input.now);
  }
  if (controller.mode == GraspController::kClimb) {
    const bool climb_terminal =
        controller.climb_mode.state == ClimbState::Done ||
        controller.climb_mode.state == ClimbState::Failed;
    if (climb_terminal && ctx().machineState() == MachineState::Running) {
      flushSpeedDiagnostic("terminal");
      ctx().setMachineState(MachineState::Hold);
      if (controller.climb_mode.state == ClimbState::Done) {
        ROS_INFO("CLIMB DONE: HOLD; request dock via switch_mode");
      } else {
        ROS_WARN("CLIMB FAILED: HOLD: %s",
                 controller.climb_mode.failure_reason.c_str());
      }
    }
    if (monitor_active_) {
      monitorRealClimb();
    }
  }
  return q_des;
}

std::optional<ModeOutcome> ClimbMode::checkTerminal() {
  const grasp_hexapod_control_cpp::ClimbState state =
      ctx().controller().climb_mode.state;
  if (state == ClimbState::Done) {
    return ModeOutcome{ModeResult::Success, "climb complete"};
  }
  if (state == ClimbState::Failed) {
    return ModeOutcome{ModeResult::Failure,
                       ctx().controller().climb_mode.failure_reason};
  }
  return std::nullopt;
}

// ---------------------------------------------------------------------------
// 相对运动监控（观察 + 持续超限才 HOLD，不直接失败）。
// ---------------------------------------------------------------------------

std::pair<bool, std::string> ClimbMode::realClimbObservation() {
  // 比较部署传感器相对运动与当前计划机身运动。
  GraspController& controller = ctx().controller();
  const NavigationState navigation = ctx().inputs().navigationMotionSnapshot();
  const ImuSnapshot imu = ctx().inputs().imu();
  if (!navigation.valid) {
    return {false, "RTK/LoRa navigation pose is stale or invalid"};
  }
  if (!imu.valid) {
    return {false, "IMU is stale or invalid"};
  }
  if (!have_climb_start_navigation_ || !have_climb_start_imu_ ||
      !have_climb_start_planned_ ||
      !controller.climb_mode.base_pose.has_value()) {
    return {false, "climb safety reference is missing"};
  }
  const Matrix4d start_xiaolan_from_base =
      navigation.pv_from_xiaolan.inverse() * climb_start_navigation_.pv_from_base;
  const Matrix4d current_xiaolan_from_base =
      navigation.pv_from_xiaolan.inverse() * navigation.pv_from_base;
  const Matrix4d actual_relative =
      start_xiaolan_from_base.inverse() * current_xiaolan_from_base;
  const Matrix4d planned_now =
      worldFromBase(*controller.climb_mode.base_pose);
  const Matrix4d planned_relative =
      climb_start_planned_pose_.inverse() * planned_now;
  const double position_error =
      (actual_relative.block<3, 1>(0, 3) - planned_relative.block<3, 1>(0, 3))
          .norm();
  const Matrix3d imu_relative =
      climb_start_imu_rotation_.transpose() * imu.rotation;
  const double orientation_error = rotationAngle(
      planned_relative.block<3, 3>(0, 0).transpose() * imu_relative);
  const double angular_speed = imu.angular_velocity.norm();
  if (position_error > max_position_error_) {
    return {false,
            formatMessage("relative RTK position error %.3f m", position_error)};
  }
  if (orientation_error > max_orientation_error_) {
    return {false, formatMessage("IMU attitude error %.1f deg",
                                 orientation_error * 180.0 / M_PI)};
  }
  if (angular_speed > max_angular_speed_) {
    return {false, formatMessage("IMU angular speed %.1f deg/s",
                                 angular_speed * 180.0 / M_PI)};
  }
  return {true, ""};
}

void ClimbMode::monitorRealClimb() {
  // 持续观察已启用的可选相对运动监控。
  GraspController& controller = ctx().controller();
  if (controller.mode != GraspController::kClimb) {
    return;
  }
  const auto [okay, reason] = realClimbObservation();
  if (okay) {
    good_frames_ += 1;
    bad_frames_ = 0;
    return;
  }
  good_frames_ = 0;
  if (reason.find("stale") != std::string::npos ||
      reason.find("invalid") != std::string::npos ||
      reason.find("missing") != std::string::npos) {
    bad_frames_ = persistence_frames_;
  } else {
    bad_frames_ += 1;
  }
  if (controller.climb_mode.state == ClimbState::Running &&
      bad_frames_ >= persistence_frames_) {
    controller.holdClimb();
    flushSpeedDiagnostic("safety hold");
    ctx().setMachineState(MachineState::Hold);
    ROS_WARN(
        "Diagnostic replay CLIMB HOLD: %s; joint-feedback gates are not "
        "contact/load evidence",
        reason.c_str());
  }
}

// ---------------------------------------------------------------------------
// 速度诊断（observation-only）。
// ---------------------------------------------------------------------------

void ClimbMode::warnHardwareClimbPhaseHold() {
  // 报告实机反馈门冻结，保留完整足端与关节定位。
  const GraspController& controller = ctx().controller();
  if (controller.mode != GraspController::kClimb) {
    return;
  }
  const grasp_hexapod_control_cpp::ClimbMode& climb = controller.climb_mode;
  if (climb.state != ClimbState::Running || !climb.hardware_execution ||
      !climb.last_phase_hold) {
    return;
  }
  ROS_WARN_THROTTLE(
      0.5, "CLIMB PHASE HOLD: source=%s stage=%s %s collision_guard_hold=%s",
      "hardware_feedback", climb.phase.c_str(),
      climb.trackingDiagnosticSummary().c_str(),
      climb.last_collision_guard_hold ? "true" : "false");
}

void ClimbMode::infoHardwareClimbActiveTrace() {
  // 周期输出活动腿的 base_link 规划/反馈对照，不推断因果。
  const GraspController& controller = ctx().controller();
  if (controller.mode != GraspController::kClimb) {
    return;
  }
  const grasp_hexapod_control_cpp::ClimbMode& climb = controller.climb_mode;
  if (climb.state != ClimbState::Running || !climb.hardware_execution) {
    return;
  }
  ROS_INFO_THROTTLE(
      1.0,
      "CLIMB ACTIVE LEG TRACE: source=%s stage=%s %s; diagnostic only: "
      "desired_z/planned lift low suggests planning, while desired-correct "
      "actual lag with same-leg joint error suggests load/execution/feedback; "
      "not causal proof",
      "hardware_feedback", climb.last_diagnostic_stage_name.c_str(),
      climb.activeLegDiagnosticSummary().c_str());
}

void ClimbMode::flushSpeedDiagnostic(const std::string& reason) {
  // 输出并丢弃已完成阶段的聚合，不影响运动。
  const SpeedDiagnostic* item = speed_diagnostic_.get();
  if (item == nullptr) {
    return;
  }
  const double peak_command = item->peak_command_speed_rad_s;
  const double peak_measured = item->peak_measured_speed_rad_s;
  const double peak_ratio =
      peak_command > 0.0 ? peak_measured / peak_command
                         : std::numeric_limits<double>::quiet_NaN();
  const double mean_ratio =
      item->ratio_count > 0
          ? item->ratio_sum / item->ratio_count
          : std::numeric_limits<double>::quiet_NaN();
  ROS_INFO(
      "CLIMB speed diagnostic-only stage=%s reason=%s peak_cmd=%.3f "
      "peak_meas=%.3f R_v=%.3f mean_meas_cmd=%.3f samples=%d "
      "peak_tracking=%.4f clip_joints=%d guard_holds=%d; mean ratio uses "
      "per-joint |cmd_speed| >= 0.05 rad/s and is not contact/load evidence",
      item->stage.c_str(), reason.c_str(), peak_command, peak_measured,
      peak_ratio, mean_ratio, item->ratio_count, item->peak_tracking_error_rad,
      item->velocity_limit_clip_count, item->collision_guard_hold_count);
  speed_diagnostic_.reset();
}

void ClimbMode::recordSpeedDiagnostic(
    const std::string& stage, const JointAngles& q_cur, const JointAngles& q_des,
    const std::array<double, 6>& feedback_stamp, double sample_time) {
  // 聚合实机 CLIMB 反馈；从不门控阶段推进。
  GraspController& controller = ctx().controller();
  if (controller.mode != GraspController::kClimb) {
    return;
  }
  SpeedDiagnostic* item = speed_diagnostic_.get();
  if (item == nullptr || item->stage != stage) {
    flushSpeedDiagnostic("stage transition");
    auto fresh = std::make_unique<SpeedDiagnostic>();
    fresh->stage = stage;
    fresh->previous_q_cur = q_cur;
    fresh->previous_q_des = q_des;
    fresh->previous_feedback_stamp = feedback_stamp;
    fresh->previous_sample_time = sample_time;
    fresh->peak_tracking_error_rad = maxAbsDiffLocal(q_des, q_cur);
    fresh->velocity_limit_clip_count =
        controller.last_update_velocity_limit_clip_count;
    fresh->collision_guard_hold_count =
        controller.last_update_collision_guard_hold_count;
    speed_diagnostic_ = std::move(fresh);
    return;
  }
  const double command_dt = sample_time - item->previous_sample_time;
  bool feedback_dt_positive = true;
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (feedback_stamp[leg] - item->previous_feedback_stamp[leg] <= 0.0) {
      feedback_dt_positive = false;
      break;
    }
  }
  if (command_dt <= 0.0 || !feedback_dt_positive) {
    item->previous_q_cur = q_cur;
    item->previous_q_des = q_des;
    item->previous_feedback_stamp = feedback_stamp;
    item->previous_sample_time = sample_time;
    return;
  }
  double peak_command_speed = 0.0;
  double peak_measured_speed = 0.0;
  double ratio_sum = 0.0;
  int ratio_count = 0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    for (int joint = 0; joint < kJointCount; ++joint) {
      const double command_speed =
          std::abs(q_des[leg][joint] - item->previous_q_des[leg][joint]) /
          command_dt;
      const double measured_speed =
          std::abs(q_cur[leg][joint] - item->previous_q_cur[leg][joint]) /
          (feedback_stamp[leg] - item->previous_feedback_stamp[leg]);
      peak_command_speed = std::max(peak_command_speed, command_speed);
      peak_measured_speed = std::max(peak_measured_speed, measured_speed);
      if (command_speed >= 0.05) {
        ratio_sum += measured_speed / command_speed;
        ++ratio_count;
      }
    }
  }
  item->peak_command_speed_rad_s =
      std::max(item->peak_command_speed_rad_s, peak_command_speed);
  item->peak_measured_speed_rad_s =
      std::max(item->peak_measured_speed_rad_s, peak_measured_speed);
  item->peak_tracking_error_rad =
      std::max(item->peak_tracking_error_rad, maxAbsDiffLocal(q_des, q_cur));
  item->ratio_sum += ratio_sum;
  item->ratio_count += ratio_count;
  item->velocity_limit_clip_count +=
      controller.last_update_velocity_limit_clip_count;
  item->collision_guard_hold_count +=
      controller.last_update_collision_guard_hold_count;
  item->previous_q_cur = q_cur;
  item->previous_q_des = q_des;
  item->previous_feedback_stamp = feedback_stamp;
  item->previous_sample_time = sample_time;
}

REGISTER_MODE("climb", ClimbMode)

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
