#include "grasp_hexapod_control_cpp/dock_mode.h"

#include "grasp_hexapod_control_cpp/control.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <map>

namespace grasp_hexapod_control_cpp {

namespace {

std::string formatCn(const char* format, ...) {
  char buffer[512];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return buffer;
}

// np.ptp：最大值 - 最小值。
double peakToPeak(const FootPositions& feet, int axis) {
  double low = feet[0][axis];
  double high = feet[0][axis];
  for (int leg = 1; leg < kLegCount; ++leg) {
    low = std::min(low, feet[leg][axis]);
    high = std::max(high, feet[leg][axis]);
  }
  return high - low;
}

std::vector<Vector3d> asVector(const FootPositions& feet) {
  return std::vector<Vector3d>(feet.begin(), feet.end());
}

}  // namespace

DockMode::DockMode(GraspController* controller, PerceptionInterface* perception,
                   bool require_lock_confirmation, double linear_speed_m_s,
                   double update_rate_hz, double perception_rate_hz,
                   double leg_lift_speed_m_s, double sit_settle_duration_s,
                   std::function<void(bool, const std::string&)> logger)
    : controller_(controller),
      perception_(perception),
      require_lock_confirmation_(require_lock_confirmation),
      logger_(std::move(logger)) {
  linear_speed_m_s_ = linear_speed_m_s;
  leg_lift_speed_m_s_ = leg_lift_speed_m_s;
  sit_settle_duration_s_ = sit_settle_duration_s;
  update_period = 1.0 / update_rate_hz;
  perception_period = 1.0 / perception_rate_hz;
  update_dt = controller_->dt;
  if (!std::isfinite(linear_speed_m_s_) || linear_speed_m_s_ <= 0.0) {
    throw ConfigError("dock linear_speed_m_s must be finite and positive");
  }
  if (!std::isfinite(leg_lift_speed_m_s_) || leg_lift_speed_m_s_ <= 0.0) {
    throw ConfigError("dock leg_lift_speed_m_s must be finite and positive");
  }
  if (!std::isfinite(sit_settle_duration_s_) || sit_settle_duration_s_ < 0.0) {
    throw ConfigError("dock sit_settle_duration_s must be finite and nonnegative");
  }
}

void DockMode::setState(const std::string& next_state, const std::string& next_reason) {
  const bool changed = next_state != state;
  state = next_state;
  reason = next_reason;
  if (changed && logger_) {
    static const std::map<std::string, std::string> labels = {
        {kIdle, "待机"},
        {kClimbTerminalEntry, "恢复攀爬末端姿态"},
        {kBodyRaise, "对接初始姿态抬升"},
        {kSearchingTag, "平面扫描AprilTag"},
        {kWaitingTag, "等待AprilTag"},
        {kPrealign, "视觉预对准"},
        {kPreDescentSettle, "下降前稳定"},
        {kDescent, "机械导向下降"},
        {kSitSettle, "下坐稳定等待"},
        {kLegLift, "腿部腾空"},
        {kAligned, "等待锁紧确认"},
        {kSuccess, "对接成功"},
        {kFailed, "对接失败"}};
    const auto label = labels.find(next_state);
    const std::string text = formatCn("DockMode阶段: %s (%s) - %s",
                                      label != labels.end() ? label->second.c_str()
                                                            : next_state.c_str(),
                                      next_state.c_str(),
                                      next_reason.empty() ? "无" : next_reason.c_str());
    logger_(next_state == kFailed, text);
  }
}

void DockMode::enter(const JointAngles& current_joints,
                     const JointAngles* climb_terminal_joints) {
  const JointAngles target =
      climb_terminal_joints != nullptr ? *climb_terminal_joints : current_joints;
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!current_joints[leg].allFinite() || !target[leg].allFinite()) {
      throw ConfigError("dock entry joints must be finite");
    }
  }
  active = true;
  perception_->reset();
  entry_start_ = current_joints;
  entry_target_ = target;
  entry_elapsed_ = 0.0;
  body_raise_start_feet_.reset();
  body_raise_progress_ = 0.0;
  search_anchor_feet_.reset();
  search_body_offset_.setZero();
  search_angle_ = -0.5 * M_PI;
  search_on_circle_ = false;
  descent_total_ = 0.0;
  descent_remaining_ = 0.0;
  descent_duration_ = 0.0;
  pre_descent_elapsed_ = 0.0;
  pre_descent_feet_.reset();
  sit_settle_elapsed_ = 0.0;
  sit_settle_feet_.reset();
  leg_lift_start_feet_.reset();
  leg_lift_progress_ = 0.0;
  cached_pose.reset();
  cached_ids.clear();
  last_visual_target.reset();
  update_elapsed = std::max(0.0, update_period - controller_->dt);
  update_dt = update_period;
  last_update_result.reset();
  perception_elapsed = std::max(0.0, perception_period - controller_->dt);
  perception_sampled = false;
  last_perception_reason = "no perception result";
  using_last_complete_frame = false;
  const JointAngles velocity_limit = jointVelocityLimit();
  double max_ratio = 0.0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    for (int joint = 0; joint < kJointCount; ++joint) {
      const double ratio =
          std::abs(target[leg][joint] - current_joints[leg][joint]) /
          velocity_limit[leg][joint];
      max_ratio = std::max(max_ratio, ratio);
    }
  }
  entry_duration_ = std::max(0.5, 1.875 * max_ratio);
  setState(kClimbTerminalEntry, "正在进入攀爬结束关节姿态");
}

void DockMode::exit() {
  active = false;
  entry_start_.reset();
  entry_target_.reset();
  body_raise_start_feet_.reset();
  body_raise_progress_ = 0.0;
  search_anchor_feet_.reset();
  search_body_offset_.setZero();
  search_angle_ = -0.5 * M_PI;
  search_on_circle_ = false;
  descent_total_ = 0.0;
  descent_remaining_ = 0.0;
  descent_duration_ = 0.0;
  pre_descent_elapsed_ = 0.0;
  pre_descent_feet_.reset();
  sit_settle_elapsed_ = 0.0;
  sit_settle_feet_.reset();
  leg_lift_start_feet_.reset();
  leg_lift_progress_ = 0.0;
  cached_pose.reset();
  cached_ids.clear();
  last_visual_target.reset();
  update_elapsed = 0.0;
  last_update_result.reset();
  perception_elapsed = 0.0;
  perception_sampled = false;
  last_perception_reason = "no perception result";
  using_last_complete_frame = false;
  setState(kIdle, "");
}

void DockMode::failExecution(const std::string& reason) {
  if (active) {
    setState(kFailed, reason);
  }
}

std::optional<std::pair<Matrix4d, std::vector<int>>> DockMode::perceptionPose(
    const DockRobotState& robot_state, std::string& reason_out) {
  if (!robot_state.lock_from_pin.has_value()) {
    perception_elapsed += update_dt;
    const bool refresh =
        !perception_sampled || perception_elapsed + 1e-12 >= perception_period;
    if (!refresh) {
      if (!cached_pose.has_value()) {
        reason_out = last_perception_reason;
        return std::nullopt;
      }
      reason_out = using_last_complete_frame ? "使用最后完整AprilTag帧推算" : "";
      return std::make_pair(*cached_pose, cached_ids);
    }

    perception_elapsed = 0.0;
    const PerceptionResult observed = perception_->latest();
    perception_sampled = true;
    if (!observed.valid) {
      last_perception_reason = observed.reason;
      if (!cached_pose.has_value()) {
        reason_out = observed.reason;
        return std::nullopt;
      }
      using_last_complete_frame = true;
      reason_out = "使用最后完整AprilTag帧推算";
      return std::make_pair(*cached_pose, cached_ids);
    }
    cached_pose = observed.lock_from_pin;
    cached_ids = observed.decoded_ids;
    last_perception_reason = "";
    using_last_complete_frame = false;
  } else {
    using_last_complete_frame = false;
  }

  const Matrix4d pose = *robot_state.lock_from_pin;
  if (!pose.allFinite()) {
    reason_out = "non-finite lock_from_pin";
    return std::nullopt;
  }
  cached_pose = pose;
  cached_ids = robot_state.decoded_ids.value_or(std::vector<int>());
  reason_out = "";
  return std::make_pair(pose, cached_ids);
}

FootPositions DockMode::actualFeet(const JointAngles& joints) const {
  return controller_->kinematic.forwardBase(joints);
}

FootPositions DockMode::syncedFeet(const JointAngles& current) const {
  return controller_->syncActualFeet(current);
}

DockResult DockMode::makeResult(std::optional<FootPositions> feet,
                                std::optional<JointAngles> joints) const {
  DockResult result;
  result.foot_positions_base = std::move(feet);
  result.active = active;
  result.success = state == kSuccess;
  result.failed = state == kFailed;
  result.request_lock = state == kAligned;
  result.reason = reason;
  result.joint_positions = std::move(joints);
  result.state = state;
  return result;
}

DockResult DockMode::updateEntry(const JointAngles& current) {
  entry_elapsed_ = std::min(entry_duration_, entry_elapsed_ + update_dt);
  const double phase = entry_elapsed_ / entry_duration_;
  const double blend = smoothStep(phase);
  JointAngles command{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    command[leg] = (1.0 - blend) * (*entry_start_)[leg] + blend * (*entry_target_)[leg];
  }
  if (phase >= 1.0) {
    double error = 0.0;
    for (int leg = 0; leg < kLegCount; ++leg) {
      error = std::max(error, (current[leg] - (*entry_target_)[leg]).cwiseAbs().maxCoeff());
    }
    const char* quality =
        error <= kEntryTrackingTolerance ? "达标" : "仅供参考";
    body_raise_start_feet_ = syncedFeet(current);
    body_raise_progress_ = 0.0;
    setState(kBodyRaise,
             formatCn("攀爬结束姿态指令已完成，关节误差%.2fdeg（%s）；开始足端向下移动40mm",
                      error * 180.0 / M_PI, quality));
  }
  return makeResult(std::nullopt, command);
}

DockResult DockMode::bodyRaiseStep(const JointAngles& current) {
  if (!body_raise_start_feet_.has_value()) {
    body_raise_start_feet_ = syncedFeet(current);
  }
  body_raise_progress_ = std::min(
      kBodyRaiseHeight, body_raise_progress_ + linear_speed_m_s_ * update_dt);
  FootPositions feet = *body_raise_start_feet_;
  for (int leg = 0; leg < kLegCount; ++leg) {
    feet[leg].z() -= body_raise_progress_;
  }
  if (body_raise_progress_ >= kBodyRaiseHeight) {
    last_visual_target = feet;
    search_anchor_feet_ = feet;
    search_body_offset_.setZero();
    search_angle_ = -0.5 * M_PI;
    search_on_circle_ = false;
    setState(kSearchingTag,
             "初始姿态已抬升40mm；开始以20mm半径、20mm/s圆周扫描");
  } else {
    setState(kBodyRaise, formatCn("六足同步向下：%.1f/40.0mm",
                                  body_raise_progress_ * 1000.0));
  }
  return makeResult(feet, std::nullopt);
}

DockResult DockMode::tagSearchStep(const JointAngles& current,
                                   const std::string& reason) {
  if (!search_anchor_feet_.has_value()) {
    search_anchor_feet_ = syncedFeet(current);
  }
  if (search_on_circle_) {
    search_angle_ -= kTagSearchSpeed * update_dt / kTagSearchRadius;
  }
  const Eigen::Vector2d target =
      kTagSearchRadius * Eigen::Vector2d(std::cos(search_angle_), std::sin(search_angle_));
  const Eigen::Vector2d delta = target - search_body_offset_;
  const double distance = delta.norm();
  const double step = std::min(kTagSearchSpeed * update_dt, distance);
  if (distance > 1e-9) {
    search_body_offset_ += delta * step / distance;
  }
  if (distance <= step + 1e-9) {
    search_body_offset_ = target;
    search_on_circle_ = true;
  }
  const Matrix4d offset =
      translation(-search_body_offset_.x(), -search_body_offset_.y(), 0.0);
  const FootPositions feet = [&] {
    std::vector<Vector3d> transformed =
        transformPoints(offset, asVector(*search_anchor_feet_));
    FootPositions result{};
    std::copy(transformed.begin(), transformed.end(), result.begin());
    return result;
  }();
  last_visual_target = feet;
  setState(kSearchingTag,
           formatCn("机身圆周扫描偏移=(%.1f,%.1f)mm；%s",
                    search_body_offset_.x() * 1000.0,
                    search_body_offset_.y() * 1000.0, reason.c_str()));
  return makeResult(feet, std::nullopt);
}

FootPositions DockMode::visualStep(const JointAngles& current, const Matrix4d& pose) {
  Vector3d correction = -pose.block<3, 1>(0, 3);
  // 只取 XY 分量做水平修正。
  Eigen::Vector2d horizontal = correction.head<2>();
  const double distance = horizontal.norm();
  if (distance > linear_speed_m_s_ * update_dt) {
    horizontal *= linear_speed_m_s_ * update_dt / distance;
  }
  const Matrix4d increment = translation(horizontal.x(), horizontal.y(), 0.0);
  if (!last_visual_target.has_value()) {
    last_visual_target = actualFeet(current);
  }
  cached_pose = increment * pose;
  std::vector<Vector3d> transformed =
      transformPoints(increment, asVector(*last_visual_target));
  FootPositions result{};
  std::copy(transformed.begin(), transformed.end(), result.begin());
  last_visual_target = result;
  return result;
}

DockResult DockMode::preDescentSettleStep(const JointAngles& current) {
  pre_descent_elapsed_ = std::min(kPreDescentSettleDuration,
                                  pre_descent_elapsed_ + update_dt);
  if (pre_descent_elapsed_ >= kPreDescentSettleDuration) {
    setState(kDescent, "下降前姿态已稳定，开始机械导向下降");
    return descentStep(current);
  }
  setState(kPreDescentSettle,
           formatCn("保持预对接姿态：%.2f/%.2fs", pre_descent_elapsed_,
                    kPreDescentSettleDuration));
  return makeResult(*pre_descent_feet_, std::nullopt);
}

DockResult DockMode::descentStep(const JointAngles& current) {
  const double step = std::min(linear_speed_m_s_ * update_dt, descent_remaining_);
  descent_remaining_ -= step;
  const Matrix4d offset = translation(0.0, 0.0, step);
  std::vector<Vector3d> transformed =
      transformPoints(offset, asVector(syncedFeet(current)));
  FootPositions feet{};
  std::copy(transformed.begin(), transformed.end(), feet.begin());
  if (descent_remaining_ <= 1e-9) {
    sit_settle_elapsed_ = 0.0;
    sit_settle_feet_ = feet;
    leg_lift_start_feet_.reset();
    leg_lift_progress_ = 0.0;
    setState(kSitSettle, formatCn("下坐完成，稳定等待%.1fs", sit_settle_duration_s_));
  } else {
    setState(kDescent, formatCn("机械导向下降中，剩余%.1fmm，预计%.2fs",
                                descent_remaining_ * 1000.0,
                                descent_remaining_ / linear_speed_m_s_));
  }
  return makeResult(feet, std::nullopt);
}

DockResult DockMode::sitSettleStep(const JointAngles& current) {
  sit_settle_elapsed_ =
      std::min(sit_settle_duration_s_, sit_settle_elapsed_ + update_dt);
  if (sit_settle_elapsed_ >= sit_settle_duration_s_) {
    setState(kLegLift,
             "下坐稳定完成，开始将六腿收至同一高度（至少抬升60mm）");
    return legLiftStep(current);
  }
  setState(kSitSettle, formatCn("下坐稳定等待中：%.2f/%.2fs", sit_settle_elapsed_,
                                sit_settle_duration_s_));
  return makeResult(*sit_settle_feet_, std::nullopt);
}

DockResult DockMode::legLiftStep(const JointAngles& current) {
  if (!leg_lift_start_feet_.has_value()) {
    leg_lift_start_feet_ = syncedFeet(current);
  }
  double highest = (*leg_lift_start_feet_)[0].z();
  double lowest = (*leg_lift_start_feet_)[0].z();
  for (int leg = 1; leg < kLegCount; ++leg) {
    highest = std::max(highest, (*leg_lift_start_feet_)[leg].z());
    lowest = std::min(lowest, (*leg_lift_start_feet_)[leg].z());
  }
  const double target_z = highest + kLegLiftHeight;
  const double travel = target_z - lowest;

  if (leg_lift_progress_ >= travel) {
    FootPositions feet = *leg_lift_start_feet_;
    for (int leg = 0; leg < kLegCount; ++leg) {
      feet[leg].z() = target_z;
    }
    const double actual_spread = peakToPeak(actualFeet(current), 2);
    if (actual_spread > kLegLiftLevelTolerance) {
      setState(kLegLift, formatCn("统一高度收敛中：实际高度差%.1fmm",
                                  actual_spread * 1000.0));
      return makeResult(feet, std::nullopt);
    }
    if (require_lock_confirmation_) {
      setState(kAligned, "六腿已收至同一高度，等待锁紧机构确认");
    } else {
      setState(kSuccess, "六腿已收至同一高度，对接结束");
    }
    return makeResult(std::nullopt, current);
  }

  leg_lift_progress_ = std::min(travel, leg_lift_progress_ + leg_lift_speed_m_s_ * update_dt);
  FootPositions feet = *leg_lift_start_feet_;
  for (int leg = 0; leg < kLegCount; ++leg) {
    feet[leg].z() = std::min((*leg_lift_start_feet_)[leg].z() + leg_lift_progress_, target_z);
  }
  setState(kLegLift, formatCn("六腿同步抬起中：%.1f/%.1fmm，目标同高",
                              leg_lift_progress_ * 1000.0, travel * 1000.0));
  return makeResult(feet, std::nullopt);
}

DockResult DockMode::update(const DockRobotState& robot_state) {
  if (!active) {
    throw std::runtime_error("enter() must be called before update()");
  }
  if (isTerminalState(state)) {
    return updateOnce(robot_state);
  }

  update_elapsed += controller_->dt;
  if (last_update_result.has_value() && update_elapsed + 1e-12 < update_period) {
    return *last_update_result;
  }
  update_dt = update_elapsed;
  update_elapsed = 0.0;
  last_update_result = updateOnce(robot_state);
  return *last_update_result;
}

DockResult DockMode::updateOnce(const DockRobotState& robot_state) {
  // 终态优先返回，避免壳层进入 HOLD、不再注入 DockRobotState 后把已经确认
  // 的 SUCCESS 覆盖成关节反馈缺失 FAILED。
  if (isTerminalState(state)) {
    if (!robot_state.joints.has_value()) {
      return makeResult(std::nullopt, std::nullopt);
    }
    return makeResult(std::nullopt, *robot_state.joints);
  }

  if (!robot_state.joints.has_value()) {
    failExecution("对接执行缺少统一控制链路的关节反馈");
    return makeResult(std::nullopt, std::nullopt);
  }
  const JointAngles& current = *robot_state.joints;
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!current[leg].allFinite()) {
      failExecution("关节反馈包含非有限值");
      return makeResult(std::nullopt, std::nullopt);
    }
  }

  if (state == kClimbTerminalEntry) {
    return updateEntry(current);
  }
  if (state == kBodyRaise) {
    return bodyRaiseStep(current);
  }

  // 锁存预对接姿态后不再依赖 AprilTag，避免标签离开视野时中断机械对接。
  if (state == kPreDescentSettle) {
    return preDescentSettleStep(current);
  }
  if (state == kDescent) {
    return descentStep(current);
  }
  if (state == kSitSettle) {
    return sitSettleStep(current);
  }
  if (state == kLegLift) {
    return legLiftStep(current);
  }

  if (robot_state.lock_confirmed.has_value() && *robot_state.lock_confirmed) {
    setState(kSuccess, "锁紧机构已确认，对接成功");
    return makeResult(std::nullopt, current);
  }
  if (state == kAligned) {
    return makeResult(std::nullopt, current);
  }

  std::string perception_reason;
  const auto pose_and_ids = perceptionPose(robot_state, perception_reason);
  if (!pose_and_ids.has_value()) {
    if (state == kSearchingTag) {
      return tagSearchStep(current, perception_reason);
    }
    setState(kWaitingTag, "等待完整AprilTag：" + perception_reason);
    // 本次对接尚未得到过完整标签时保持当前位置。
    return makeResult(std::nullopt, current);
  }
  const Matrix4d& pose = pose_and_ids->first;
  const std::vector<int>& decoded_ids = pose_and_ids->second;

  if (state == kSearchingTag) {
    // 扫描一旦发现完整标签，立即以当前实际足端开始视觉闭环。
    last_visual_target = syncedFeet(current);
  }

  const double horizontal = pose.block<2, 1>(0, 3).norm();
  const double tilt =
      std::acos(std::max(-1.0, std::min(1.0, pose(2, 2))));
  const bool ready = horizontal <= kPrealignPositionReference;
  static const std::map<int, const char*> tag_directions = {
      {0, "+y"}, {1, "+x"}, {2, "-y"}, {3, "-x"}};
  std::string tags;
  for (int tag_id : decoded_ids) {
    if (!tags.empty()) {
      tags += ",";
    }
    tags += formatCn("ID%d(%s)", tag_id, tag_directions.at(tag_id));
  }
  if (tags.empty()) {
    tags = "外部TF";
  }
  if (!perception_reason.empty()) {
    tags += "[最后完整帧推算]";
  }
  if (ready) {
    // 水平轨迹已进入导向锥可接管区域；冻结实际足端，后续只下降。
    const double tf_distance = std::max(0.0, -pose(2, 3));
    descent_total_ = tf_distance;
    descent_remaining_ = descent_total_;
    descent_duration_ = descent_total_ / linear_speed_m_s_;
    pre_descent_elapsed_ = 0.0;
    pre_descent_feet_ = syncedFeet(current);
    last_visual_target = pre_descent_feet_;
    setState(kPreDescentSettle,
             formatCn("%s到达下降参考：水平%.1fmm，倾斜%.2fdeg；稳定等待0.5s",
                      tags.c_str(), horizontal * 1000.0, tilt * 180.0 / M_PI));
    return makeResult(*pre_descent_feet_, std::nullopt);
  }

  setState(kPrealign,
           formatCn("%s水平调整：水平%.1fmm，倾斜%.2fdeg（仅显示）",
                    tags.c_str(), horizontal * 1000.0, tilt * 180.0 / M_PI));
  // 姿态和高度保持不变，只累计 X/Y 目标。
  const FootPositions feet = visualStep(current, pose);
  return makeResult(feet, std::nullopt);
}

bool DockMode::descentHasStarted() const {
  return state == kDescent || state == kSitSettle || state == kLegLift ||
         state == kAligned || state == kSuccess;
}

}  // namespace grasp_hexapod_control_cpp
