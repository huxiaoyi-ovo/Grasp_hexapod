#include "grasp_hexapod_control_cpp/climb_mode.h"

#include "grasp_hexapod_control_cpp/control.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <tuple>
#include <utility>

namespace grasp_hexapod_control_cpp {

namespace {

std::string formatG9(double value) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.9g", value);
  return buffer;
}

std::string formatNamedErrors(const std::vector<std::pair<std::string, double>>& errors) {
  std::string joined;
  for (const auto& [name, value] : errors) {
    if (!joined.empty()) {
      joined += ",";
    }
    joined += name + "=" + formatG9(value);
  }
  return joined.empty() ? "none" : joined;
}

std::string formatXyz(const Vector3d& vector) {
  char buffer[160];
  std::snprintf(buffer, sizeof(buffer), "(%.9g,%.9g,%.9g)", vector.x(), vector.y(),
                vector.z());
  return buffer;
}

}  // namespace

ClimbMode::ClimbMode(GraspController* controller) : controller_(controller) {
  last_worst_foot_leg = kLegNames[0];
  last_worst_joint_leg = kLegNames[0];
  last_worst_joint_name = kJointNames[0];
}

ClimbCompactConfig ClimbMode::loadConfig() const {
  return loadClimbCompact(controller_->paths().climb_compact_json);
}

JointAngles ClimbMode::terminalJoints(
    const std::optional<ClimbCompactConfig>& config) const {
  const ClimbCompactConfig& source = config ? *config : loadConfig();
  validateConfig(source);
  return source.terminal_q_rad;
}

void ClimbMode::validateConfig(const ClimbCompactConfig& config) const {
  validateClimbCompact(config);
}

double ClimbMode::smoothstep(double value) {
  value = std::max(0.0, std::min(1.0, value));
  const double v2 = value * value;
  return v2 * value * (10.0 - 15.0 * value + 6.0 * v2);
}

void ClimbMode::enter(const JointAngles& q_current,
                      const std::optional<ClimbCompactConfig>& config,
                      int start_stage_index, std::optional<int> end_stage_index,
                      bool hardware_execution) {
  config_ = config ? *config : loadConfig();
  validateConfig(config_);
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!q_current[leg].allFinite()) {
      throw ConfigError("q_current must be finite");
    }
  }
  const int stage_count = static_cast<int>(config_.stages.size());
  if (start_stage_index < 0 || start_stage_index >= stage_count) {
    throw ConfigError("compact start stage index is out of range");
  }
  const int end_index = end_stage_index.value_or(stage_count - 1);
  if (start_stage_index > end_index || end_index >= stage_count) {
    throw ConfigError("compact end stage index is out of range");
  }
  if (start_stage_index == 0) {
    double entry_error = 0.0;
    for (int leg = 0; leg < kLegCount; ++leg) {
      entry_error = std::max(entry_error,
                             (q_current[leg] - config_.p0.q_rad[leg]).cwiseAbs().maxCoeff());
    }
    if (entry_error > config_.settle_gate.entry_max_joint_error_rad) {
      throw ConfigError("compact entry joint error exceeds simulation gate");
    }
  }
  this->hardware_execution = hardware_execution;
  stage_index = start_stage_index;
  this->end_stage_index = end_index;
  stage_names.clear();
  for (const CompactStage& stage : config_.stages) {
    stage_names.push_back(stage.name);
  }
  phase = stage_names[start_stage_index];
  phase_time = 0.0;
  stage_elapsed_time = 0.0;
  settle_time = 0.0;
  failure_reason.clear();
  state = ClimbState::Running;
  stage_timing_by_index.clear();
  startStageTiming();
  hardware_stage_last_monotonic_ =
      (hardware_execution && controller_->climb_timeout_uses_wall_time)
          ? std::optional<std::chrono::steady_clock::time_point>(
                std::chrono::steady_clock::now())
          : std::nullopt;

  if (start_stage_index == 0) {
    anchors_world = config_.p0.anchors_world_m;
    base_pose = std::array<double, 5>{config_.p0.base[0], config_.p0.base[1],
                                      config_.p0.base[2], 0.0, config_.p0.base[3]};
  } else {
    const CompactStage& stage = config_.stages[start_stage_index];
    base_pose = stage.pose_start;
    anchors_world = stage.anchor_knots.front();
  }
  applyReference(*base_pose, *anchors_world, true);
  controller_->q_des = q_current;
  updateTrackingDiagnostics(q_current);
  if (start_stage_index != 0 &&
      last_foot_target_error_m > config_.settle_gate.max_foot_target_error_m) {
    throw ConfigError("compact mid-stage entry foot target error exceeds simulation gate");
  }
}

void ClimbMode::hold() {
  if (state == ClimbState::Running) {
    state = ClimbState::Hold;
    hardware_stage_last_monotonic_ = std::nullopt;
  }
}

void ClimbMode::resume() {
  if (state == ClimbState::Hold) {
    state = ClimbState::Running;
    if (hardware_execution && controller_->climb_timeout_uses_wall_time) {
      hardware_stage_last_monotonic_ = std::chrono::steady_clock::now();
    }
  }
}

void ClimbMode::updateHardwareStageElapsedTime() {
  if (!controller_->climb_timeout_uses_wall_time) {
    stage_elapsed_time += controller_->dt;
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (hardware_stage_last_monotonic_.has_value()) {
    const double delta =
        std::chrono::duration<double>(now - *hardware_stage_last_monotonic_).count();
    stage_elapsed_time += std::max(0.0, delta);
  }
  hardware_stage_last_monotonic_ = now;
}

void ClimbMode::applyReference(const std::array<double, 5>& base,
                               const std::array<Vector3d, 6>& anchors,
                               bool sync_previous) {
  base_pose = base;
  anchors_world = anchors;
  const Matrix4d inverse = worldFromBase(base).inverse();
  for (int leg = 0; leg < kLegCount; ++leg) {
    const Eigen::Vector4d homogeneous(anchors[leg].x(), anchors[leg].y(),
                                      anchors[leg].z(), 1.0);
    const Eigen::Vector4d desired = inverse * homogeneous;
    controller_->foot_desired_base[leg] = desired.head<3>();
    if (sync_previous) {
      controller_->foot_desired_base_prev[leg] = desired.head<3>();
    }
  }
}

// 解析分段插值位置：返回 (段索引, 归一化 u, 段时长)；越界返回 nullopt。
namespace {

std::optional<std::tuple<std::size_t, double, double>> locateSegment(
    const std::vector<double>& durations, double elapsed) {
  double remaining = elapsed;
  for (std::size_t index = 0; index < durations.size(); ++index) {
    const double duration = durations[index];
    if (remaining <= duration) {
      return std::make_tuple(index, remaining / duration, duration);
    }
    remaining -= duration;
  }
  return std::nullopt;
}

}  // namespace

std::array<Vector3d, 6> ClimbMode::piecewiseAnchors(
    const std::vector<std::array<Vector3d, 6>>& knots,
    const std::vector<double>& durations, double elapsed) {
  const auto located = locateSegment(durations, elapsed);
  if (!located) {
    return knots.back();
  }
  const auto [index, u, duration] = *located;
  (void)duration;
  std::array<Vector3d, 6> result{};
  const double weight = smoothstep(u);
  for (int leg = 0; leg < kLegCount; ++leg) {
    result[leg] = knots[index][leg] * (1.0 - weight) + knots[index + 1][leg] * weight;
  }
  return result;
}

std::vector<Vector3d> ClimbMode::piecewiseSlots(
    const std::vector<std::vector<Vector3d>>& knots,
    const std::vector<double>& durations,
    const std::vector<std::vector<Vector3d>>* velocities, double elapsed) {
  const auto located = locateSegment(durations, elapsed);
  if (!located) {
    return knots.back();
  }
  const auto [index, u, duration] = *located;
  std::vector<Vector3d> result(knots[index].size());
  for (std::size_t slot = 0; slot < knots[index].size(); ++slot) {
    if (velocities != nullptr) {
      const double u3 = u * u * u;
      const double u4 = u3 * u;
      const double u5 = u4 * u;
      const double h00 = 1 - 10 * u3 + 15 * u4 - 6 * u5;
      const double h01 = 10 * u3 - 15 * u4 + 6 * u5;
      const double h10 = u - 6 * u3 + 8 * u4 - 3 * u5;
      const double h11 = -4 * u3 + 7 * u4 - 3 * u5;
      result[slot] = h00 * knots[index][slot] + h01 * knots[index + 1][slot] +
                     duration * (h10 * (*velocities)[index][slot] +
                                 h11 * (*velocities)[index + 1][slot]);
    } else {
      const double weight = smoothstep(u);
      result[slot] =
          knots[index][slot] * (1.0 - weight) + knots[index + 1][slot] * weight;
    }
  }
  return result;
}

std::tuple<std::array<double, 5>, std::array<Vector3d, 6>, double>
ClimbMode::stageReference() const {
  const CompactStage& stage = config_.stages[*stage_index];
  const std::vector<double>& durations = stage.segment_durations_s;
  const double total = std::accumulate(durations.begin(), durations.end(), 0.0);
  double phase_value = std::max(0.0, std::min(1.0, phase_time / total));
  double pose_phase = phase_value;
  if (stage.pose_curve == "quintic_first_segment") {
    pose_phase = std::max(0.0, std::min(1.0, phase_time / durations[0]));
  }
  const double pose_weight = smoothstep(pose_phase);
  std::array<double, 5> pose{};
  for (int i = 0; i < 5; ++i) {
    pose[i] = stage.pose_start[i] * (1.0 - pose_weight) + stage.pose_end[i] * pose_weight;
  }

  std::array<Vector3d, 6> anchors{};
  if (stage.anchor_curve == "relative_base_high_step") {
    anchors = relativeBaseHighStep(stage, pose, pose_weight, phase_value);
  } else if (stage.anchor_curve == "piecewise_base_quintic") {
    anchors = piecewiseBase(stage, pose);
  } else {
    anchors = piecewiseAnchors(stage.anchor_knots, durations, phase_time);
  }
  if (phase_time >= total) {
    pose = stage.pose_end;
    anchors = stage.anchor_knots.back();
  }
  return {pose, anchors, total};
}

std::array<Vector3d, 6> ClimbMode::relativeBaseHighStep(
    const CompactStage& stage, const std::array<double, 5>& pose, double pose_weight,
    double phase) const {
  const std::vector<std::array<Vector3d, 6>>& knots = stage.anchor_knots;
  const Matrix4d start_inverse = worldFromBase(stage.pose_start).inverse();
  const Matrix4d end_inverse = worldFromBase(stage.pose_end).inverse();
  std::array<Vector3d, 6> start_base{};
  std::array<Vector3d, 6> end_base{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    const Eigen::Vector4d homogeneous(knots[0][leg].x(), knots[0][leg].y(),
                                      knots[0][leg].z(), 1.0);
    start_base[leg] = (start_inverse * homogeneous).head<3>();
    const Eigen::Vector4d homogeneous_end(knots[1][leg].x(), knots[1][leg].y(),
                                          knots[1][leg].z(), 1.0);
    end_base[leg] = (end_inverse * homogeneous_end).head<3>();
  }
  std::array<Vector3d, 6> desired_base{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    desired_base[leg] =
        (1.0 - pose_weight) * start_base[leg] + pose_weight * end_base[leg];
  }
  // 六次单峰曲线的两端速度、加速度均为零；避免把 Approach 30 mm 梯形抬腿
  // 中的分段速度拐点放大到攀爬高步。
  const double lift_weight =
      64.0 * std::pow(phase, 3) * std::pow(1.0 - phase, 3);
  const Matrix4d current_world_from_base = worldFromBase(pose);
  for (int leg : stage.active_legs) {
    desired_base[leg].z() += *stage.relative_swing_height_m * lift_weight;
  }
  std::array<Vector3d, 6> anchors = knots[0];
  for (int leg : stage.active_legs) {
    const Eigen::Vector4d homogeneous(desired_base[leg].x(), desired_base[leg].y(),
                                      desired_base[leg].z(), 1.0);
    anchors[leg] = (current_world_from_base * homogeneous).head<3>();
  }
  return anchors;
}

std::array<Vector3d, 6> ClimbMode::piecewiseBase(const CompactStage& stage,
                                                 const std::array<double, 5>& pose) const {
  const std::vector<std::vector<Vector3d>>* velocities =
      stage.has_active_base_velocities ? &stage.active_base_velocities_m_s : nullptr;
  const std::vector<Vector3d> active_base =
      piecewiseSlots(stage.active_base_knots_m, stage.segment_durations_s, velocities,
                     phase_time);
  const Matrix4d world = worldFromBase(pose);
  std::array<Vector3d, 6> anchors =
      piecewiseAnchors(stage.anchor_knots, stage.segment_durations_s, phase_time);
  for (std::size_t slot = 0; slot < stage.active_legs.size(); ++slot) {
    const int leg = stage.active_legs[slot];
    const Eigen::Vector4d homogeneous(active_base[slot].x(), active_base[slot].y(),
                                      active_base[slot].z(), 1.0);
    anchors[leg] = (world * homogeneous).head<3>();
  }
  return anchors;
}

void ClimbMode::advanceStage() {
  if (*stage_index == *end_stage_index) {
    state = ClimbState::Done;
    hardware_stage_last_monotonic_ = std::nullopt;
    return;
  }
  *stage_index += 1;
  phase = stage_names[*stage_index];
  phase_time = 0.0;
  stage_elapsed_time = 0.0;
  settle_time = 0.0;
  startStageTiming();
  if (hardware_execution && controller_->climb_timeout_uses_wall_time) {
    hardware_stage_last_monotonic_ = std::chrono::steady_clock::now();
  }
}

void ClimbMode::startStageTiming() {
  const CompactStage& stage = config_.stages[*stage_index];
  ClimbStageTiming timing;
  timing.planned_motion_s =
      std::accumulate(stage.segment_durations_s.begin(), stage.segment_durations_s.end(),
                      0.0);
  timing.effective_settle_required_s = effectiveSettleRequired(stage);
  stage_timing_by_index[*stage_index] = timing;
}

void ClimbMode::recordStageTiming(const std::string& category) {
  static const std::map<std::string, double ClimbStageTiming::*> fields = {
      {"motion", &ClimbStageTiming::motion_progress_s},
      {"checkpoint_feedback", &ClimbStageTiming::internal_checkpoint_feedback_hold_s},
      {"terminal_persistence", &ClimbStageTiming::terminal_persistence_wait_s},
      {"terminal_feedback", &ClimbStageTiming::terminal_feedback_hold_s},
      {"collision", &ClimbStageTiming::collision_guard_hold_s},
      {"transition", &ClimbStageTiming::stage_transition_control_cycle_s}};
  const auto found = fields.find(category);
  if (found == fields.end()) {
    throw std::invalid_argument("invalid climb timing category");
  }
  if (stage_timing_by_index.find(*stage_index) == stage_timing_by_index.end()) {
    startStageTiming();
  }
  ClimbStageTiming& timing = stage_timing_by_index[*stage_index];
  timing.actual_stage_elapsed_s += controller_->dt;
  (timing.*found->second) += controller_->dt;
}

double ClimbMode::effectiveSettleRequired(const CompactStage& stage) const {
  const double persistence = stage.has_settle_persistence
                                 ? stage.settle_persistence_s
                                 : config_.settle_gate.persistence_s;
  return std::max(stage.settle_s, persistence);
}

void ClimbMode::updateTrackingDiagnostics(const JointAngles& q_current) {
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!q_current[leg].allFinite()) {
      throw ConfigError("q_current must be finite");
    }
  }
  const JointAngles& q_des = controller_->q_des;
  double tracking_error = 0.0;
  int worst_leg = 0;
  int worst_joint = 0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    for (int joint = 0; joint < kJointCount; ++joint) {
      last_joint_tracking_errors_rad[leg][joint] =
          std::abs(q_current[leg][joint] - q_des[leg][joint]);
      if (last_joint_tracking_errors_rad[leg][joint] >
          last_joint_tracking_errors_rad[worst_leg][worst_joint]) {
        worst_leg = leg;
        worst_joint = joint;
      }
    }
    tracking_error = std::max(
        tracking_error, last_joint_tracking_errors_rad[leg].maxCoeff());
  }
  const FootPositions actual_base = controller_->kinematic.hipToBase(
      controller_->kinematic.forward(q_current));
  const FootPositions& desired_base = controller_->foot_desired_base;
  double foot_error = 0.0;
  int worst_foot = 0;  // np.argmax 语义：仅严格大于时更新（取第一个最大值）
  for (int leg = 0; leg < kLegCount; ++leg) {
    last_actual_foot_base_m[leg] = actual_base[leg];
    last_desired_foot_base_m[leg] = desired_base[leg];
    last_foot_error_xyz_m[leg] = actual_base[leg] - desired_base[leg];
    last_foot_errors_m[leg] = last_foot_error_xyz_m[leg].norm();
    if (last_foot_errors_m[leg] > foot_error) {
      worst_foot = leg;
      foot_error = last_foot_errors_m[leg];
    }
  }
  if (!std::isfinite(tracking_error) || !std::isfinite(foot_error)) {
    throw ConfigError("compact tracking diagnostics must be finite");
  }
  const CompactSettleGate& gate = config_.settle_gate;
  last_feet_over_gate.clear();
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (last_foot_errors_m[leg] > gate.max_foot_target_error_m) {
      last_feet_over_gate.emplace_back(kLegNames[leg], last_foot_errors_m[leg]);
    }
  }
  last_joints_over_tracking_gate.clear();
  for (int leg = 0; leg < kLegCount; ++leg) {
    for (int joint = 0; joint < kJointCount; ++joint) {
      if (last_joint_tracking_errors_rad[leg][joint] >
          gate.max_joint_tracking_error_rad) {
        last_joints_over_tracking_gate.emplace_back(
            std::string(kLegNames[leg]) + "_" + kJointNames[joint],
            last_joint_tracking_errors_rad[leg][joint]);
      }
    }
  }

  last_tracking_error_rad = tracking_error;
  last_foot_target_error_m = foot_error;
  last_diagnostic_phase_time_s = phase_time;
  last_diagnostic_stage_elapsed_s = stage_elapsed_time;
  const CompactStage& stage = config_.stages[*stage_index];
  last_diagnostic_stage_name = stage.name;
  last_diagnostic_stage_duration_s = std::accumulate(
      stage.segment_durations_s.begin(), stage.segment_durations_s.end(), 0.0);
  last_diagnostic_active_legs = stage.active_legs;
  last_worst_foot_leg = kLegNames[worst_foot];
  last_worst_foot_error_m = last_foot_errors_m[worst_foot];
  last_worst_joint_leg = kLegNames[worst_leg];
  last_worst_joint_name = kJointNames[worst_joint];
  last_worst_joint_error_rad = last_joint_tracking_errors_rad[worst_leg][worst_joint];
  last_worst_joint_q_current_rad = q_current[worst_leg][worst_joint];
  last_worst_joint_q_des_rad = q_des[worst_leg][worst_joint];
  // 关节角误差和足端 FK 误差来自同一组回读，并且会随姿态受到不同 Jacobian
  // 放大。实机负载下前者存在稳定偏差时，不能再把它作为与足端任务误差并列
  // 的完成条件；足端误差才是阶段推进的任务量，关节误差仅保留给日志。
  last_settled = foot_error <= gate.max_foot_target_error_m;
}

std::string ClimbMode::trackingDiagnosticSummary() const {
  const CompactSettleGate& gate = config_.settle_gate;
  std::string feet_vectors;
  for (const auto& [name, value] : last_feet_over_gate) {
    (void)value;
    const int leg_index = legIndexFromName(name);
    if (!feet_vectors.empty()) {
      feet_vectors += ";";
    }
    feet_vectors += std::string(kLegNames[leg_index]) +
                    "[actual_base_xyz_m=" + formatXyz(last_actual_foot_base_m[leg_index]) +
                    ",desired_base_xyz_m=" +
                    formatXyz(last_desired_foot_base_m[leg_index]) +
                    ",error_xyz_m=" + formatXyz(last_foot_error_xyz_m[leg_index]) + "]";
  }
  if (feet_vectors.empty()) {
    feet_vectors = "none";
  }
  const std::string worst_motor = last_worst_joint_leg + "_" + last_worst_joint_name;
  return "worst_foot=" + last_worst_foot_leg +
         " foot_target_error_m=" + formatG9(last_foot_target_error_m) +
         " foot_gate_m=" + formatG9(gate.max_foot_target_error_m) +
         " feet_over_gate=" + formatNamedErrors(last_feet_over_gate) +
         " feet_over_gate_base_link_xyz=" + feet_vectors +
         " worst_motor=" + worst_motor +
         " tracking_error_rad=" + formatG9(last_worst_joint_error_rad) +
         " q_cur_rad=" + formatG9(last_worst_joint_q_current_rad) +
         " q_des_rad=" + formatG9(last_worst_joint_q_des_rad) +
         " motors_over_" + formatG9(gate.max_joint_tracking_error_rad) + "rad=" +
         formatNamedErrors(last_joints_over_tracking_gate);
}

std::string ClimbMode::activeLegDiagnosticSummary() const {
  std::string vectors;
  for (int leg : last_diagnostic_active_legs) {
    if (!vectors.empty()) {
      vectors += ";";
    }
    vectors += std::string(kLegNames[leg]) +
               "[actual_base_xyz_m=" + formatXyz(last_actual_foot_base_m[leg]) +
               ",desired_base_xyz_m=" + formatXyz(last_desired_foot_base_m[leg]) +
               ",error_xyz_m=" + formatXyz(last_foot_error_xyz_m[leg]) + "]";
  }
  if (vectors.empty()) {
    vectors = "none";
  }
  std::string active_names;
  for (int leg : last_diagnostic_active_legs) {
    if (!active_names.empty()) {
      active_names += ",";
    }
    active_names += kLegNames[leg];
  }
  if (active_names.empty()) {
    active_names = "none";
  }
  return "base_link diagnostic_stage=" + last_diagnostic_stage_name +
         " diagnostic_phase_time_s=" + formatG9(last_diagnostic_phase_time_s) +
         " stage_duration_s=" + formatG9(last_diagnostic_stage_duration_s) +
         " diagnostic_stage_elapsed_s=" + formatG9(last_diagnostic_stage_elapsed_s) +
         " active_legs=" + active_names + " active_leg_base_link_xyz=" + vectors;
}

std::string ClimbMode::trackingFailureReason() const {
  return phase + ": TRACKING_OR_KINEMATIC_TARGET_TIMEOUT_NOT_CONTACT_PROOF " +
         trackingDiagnosticSummary();
}

void ClimbMode::update(const Eigen::Vector4d& command, const JointAngles& q_current) {
  (void)command;
  if (state == ClimbState::Idle) {
    return;
  }
  if (state == ClimbState::Hold || state == ClimbState::Failed) {
    updateTrackingDiagnostics(q_current);
    return;
  }

  if (hardware_execution) {
    const CompactSettleGate& gate = config_.settle_gate;
    const CompactStage& stage = config_.stages[*stage_index];
    const double duration = std::accumulate(stage.segment_durations_s.begin(),
                                            stage.segment_durations_s.end(), 0.0);
    updateHardwareStageElapsedTime();

    // 先检查上一控制帧已经下发的目标。实机跟不上或公共碰撞守卫保持时冻结
    // 当前轨迹相位，继续追踪同一目标，避免摆动腿尚未到达最高点，规划时钟
    // 就进入平移或下降。
    updateTrackingDiagnostics(q_current);
    const bool collision_hold = controller_->last_update_collision_guard_hold_count > 0;
    last_collision_guard_hold = collision_hold;
    if (phase_time < duration) {
      // 段边界（去掉最后一段的总时长）。
      std::vector<double> boundaries;
      boundaries.reserve(stage.segment_durations_s.size());
      double accumulated = 0.0;
      for (double segment : stage.segment_durations_s) {
        accumulated += segment;
        boundaries.push_back(accumulated);
      }
      std::vector<double> internal(boundaries.begin(), boundaries.end() - 1);
      bool at_checkpoint = false;
      for (double boundary : internal) {
        if (std::abs(phase_time - boundary) <= 1e-12) {
          at_checkpoint = true;
          break;
        }
      }
      const bool checkpoint_foot_hold = at_checkpoint && !last_settled;
      last_phase_hold = collision_hold || checkpoint_foot_hold;
      const char* timing_category =
          collision_hold ? "collision"
                         : (checkpoint_foot_hold ? "checkpoint_feedback" : "motion");
      recordStageTiming(timing_category);
      if (!last_phase_hold) {
        // np.searchsorted(boundaries, phase_time, side="right")。
        const auto next = std::upper_bound(boundaries.begin(), boundaries.end(),
                                           phase_time);
        const double next_boundary = *next;
        phase_time = std::min(next_boundary, phase_time + controller_->dt);
      }
      const auto [base, anchors, total] = stageReference();
      (void)total;
      applyReference(base, anchors);
      if (stage_elapsed_time >= duration + gate.timeout_s) {
        state = ClimbState::Failed;
        failure_reason = trackingFailureReason();
      }
      return;
    }

    // 到达终点后的下一帧才开始累计稳定时间，避免把抵达终点前一帧对旧目标
    // 的误差错误计入完成门。
    const auto [base, anchors, total] = stageReference();
    (void)total;
    applyReference(base, anchors);
    last_phase_hold = !last_settled || collision_hold;
    const double settle_required = effectiveSettleRequired(stage);
    const char* timing_category;
    if (collision_hold) {
      timing_category = "collision";
    } else if (!last_settled) {
      timing_category = "terminal_feedback";
    } else if (settle_time + controller_->dt >= settle_required) {
      // 完成 persistence 的这一帧仍保持当前终点参考；下一帧才会下发后续
      // 阶段首个目标，单列以量化 30 Hz 周期衔接成本。
      timing_category = "transition";
    } else {
      timing_category = "terminal_persistence";
    }
    recordStageTiming(timing_category);
    settle_time =
        (last_settled && !collision_hold) ? settle_time + controller_->dt : 0.0;
    if (settle_time >= settle_required) {
      const bool continuous_air = stage.continuous_air_transition;
      advanceStage();
      if (continuous_air && state == ClimbState::Running) {
        const double next_duration = std::accumulate(
            config_.stages[*stage_index].segment_durations_s.begin(),
            config_.stages[*stage_index].segment_durations_s.end(), 0.0);
        phase_time = std::min(controller_->dt, next_duration);
        const auto [next_base, next_anchors, next_total] = stageReference();
        (void)next_total;
        applyReference(next_base, next_anchors);
      }
    } else if (stage_elapsed_time >= duration + gate.timeout_s) {
      state = ClimbState::Failed;
      failure_reason = trackingFailureReason();
    }
    return;
  }

  const auto [base, anchors, duration] = stageReference();
  applyReference(base, anchors);
  updateTrackingDiagnostics(q_current);
  if (state == ClimbState::Done) {
    return;
  }
  if (phase_time < duration) {
    phase_time = std::min(duration, phase_time + controller_->dt);
    return;
  }
  const CompactSettleGate& gate = config_.settle_gate;
  const bool preview_time_only =
      gate.preview_time_only_stage_advance && !hardware_execution;
  if (preview_time_only) {
    settle_time += controller_->dt;
  } else {
    settle_time = last_settled ? settle_time + controller_->dt : 0.0;
  }
  const double settle_required = config_.stages[*stage_index].settle_s;
  if (settle_time >= settle_required) {
    const bool continuous_air =
        config_.stages[*stage_index].continuous_air_transition;
    advanceStage();
    if (continuous_air && state == ClimbState::Running) {
      const double next_duration = std::accumulate(
          config_.stages[*stage_index].segment_durations_s.begin(),
          config_.stages[*stage_index].segment_durations_s.end(), 0.0);
      phase_time = std::min(controller_->dt, next_duration);
    }
  } else if (!preview_time_only && phase_time >= duration + gate.timeout_s) {
    state = ClimbState::Failed;
    failure_reason = phase + ": TRACKING_OR_KINEMATIC_TARGET_TIMEOUT_NOT_CONTACT_PROOF "
                            "tracking_error_rad=" + formatG9(last_tracking_error_rad) +
                            " foot_target_error_m=" + formatG9(last_foot_target_error_m);
  } else {
    phase_time += controller_->dt;
  }
}

}  // namespace grasp_hexapod_control_cpp
