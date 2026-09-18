#include "grasp_hexapod_control_cpp/ring_dock_mode.h"

#include "grasp_hexapod_control_cpp/math_utils.h"
#include "grasp_hexapod_control_cpp/kinematics.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <map>

namespace grasp_hexapod_ring {

namespace {

bool liftCollisionFree(const GraspKinematic& model, const JointAngles& q);

std::string formatCn(const char* format, ...) {
  char buffer[512];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return buffer;
}

Vector3d liftJointTravel(const JointAngles& start, const JointAngles& target) {
  Vector3d travel = Vector3d::Zero();
  for (int leg = 0; leg < kLegCount; ++leg)
    travel = travel.cwiseMax((target[leg] - start[leg]).cwiseAbs());
  return travel;
}

// 中腿沿同一条时间曲线先膝后踝；规划采样和实际执行共用，避免路径不一致。
JointAngles interpolateLift(const JointAngles& start, const JointAngles& target,
                            double progress, bool knee_first) {
  Vector3d blend = Vector3d::Constant(progress);
  if (knee_first) {
    const auto travel = liftJointTravel(start, target);
    const double distance = progress * (travel[1] + travel[2]);
    blend[1] = travel[1] > 0. ? std::clamp(distance / travel[1], 0., 1.) : 1.;
    blend[2] = travel[2] > 0. ? std::clamp((distance - travel[1]) / travel[2], 0., 1.) : 1.;
  }
  JointAngles q;
  for (int leg = 0; leg < kLegCount; ++leg)
    q[leg] = start[leg] + blend.cwiseProduct(target[leg] - start[leg]);
  return q;
}

std::vector<Vector3d> asVector(const FootPositions& feet) {
  return std::vector<Vector3d>(feet.begin(), feet.end());
}

}  // namespace

DockMode::DockMode(double dt, std::function<FootPositions(const JointAngles&, bool)> feet,
                   std::function<PerceptionResult()> latest, std::function<void()> reset,
                   bool require_lock_confirmation, double linear_speed_m_s,
                   double update_rate_hz, double perception_rate_hz,
                   double leg_lift_speed_m_s, double sit_settle_duration_s,
                   double leg_lift_height_m, double leg_lift_level_tolerance_m,
                   std::function<void(bool, const std::string&)> logger,
                   std::optional<double> alignment_speed_m_s)
    : dt_(dt), feet_(std::move(feet)), latest_(std::move(latest)), reset_(std::move(reset)),
      require_lock_confirmation_(require_lock_confirmation),
      logger_(std::move(logger)) {
  linear_speed_m_s_ = linear_speed_m_s;
  alignment_speed_m_s_ = alignment_speed_m_s.value_or(linear_speed_m_s);
  if (!std::isfinite(alignment_speed_m_s_) || alignment_speed_m_s_ <= 0.0) {
    throw ConfigError("dock alignment_speed_m_s must be finite and positive");
  }
  leg_lift_speed_m_s_ = leg_lift_speed_m_s;
  sit_settle_duration_s_ = sit_settle_duration_s;
  leg_lift_height_m_ = leg_lift_height_m;
  leg_lift_level_tolerance_m_ = leg_lift_level_tolerance_m;
  if (!std::isfinite(dt_) || dt_ <= 0 || !std::isfinite(update_rate_hz) || update_rate_hz <= 0 || !std::isfinite(perception_rate_hz) || perception_rate_hz <= 0) throw ConfigError("dock rates must be finite and positive");
  update_period = 1.0 / update_rate_hz;
  perception_period = 1.0 / perception_rate_hz;
  update_dt = dt_;
  if (!std::isfinite(linear_speed_m_s_) || linear_speed_m_s_ <= 0.0) {
    throw ConfigError("dock linear_speed_m_s must be finite and positive");
  }
  if (!std::isfinite(leg_lift_speed_m_s_) || leg_lift_speed_m_s_ <= 0.0) {
    throw ConfigError("dock leg_lift_speed_m_s must be finite and positive");
  }
  if (!std::isfinite(sit_settle_duration_s_) || sit_settle_duration_s_ < 0.0) {
    throw ConfigError("dock sit_settle_duration_s must be finite and nonnegative");
  }
  if (!std::isfinite(leg_lift_height_m_) || leg_lift_height_m_ <= 0.0) {
    throw ConfigError("dock leg_lift_height_m must be finite and positive");
  }
  if (!std::isfinite(leg_lift_level_tolerance_m_) ||
      leg_lift_level_tolerance_m_ < 0.0) {
    throw ConfigError(
        "dock leg_lift_level_tolerance_m must be finite and nonnegative");
  }
}

void DockMode::setState(const std::string& next_state, const std::string& next_reason) {
  const bool changed = next_state != state;
  state = next_state;
  reason = next_reason;
  if (changed && logger_) {
    static const std::map<std::string, std::string> labels = {
        {kIdle, "待机"},
        {kClimbTerminalEntry, "保持对接入口姿态"},
        {kMiddleLift, "提前收起中腿"},
        {kBodyRaise, "四腿支撑抬升"},
        {kSearchingTag, "同心圆圆周搜索"},
        {kWaitingTag, "等待同心圆"},
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
      current_joints;
  (void)climb_terminal_joints;
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!current_joints[leg].allFinite() || !target[leg].allFinite()) {
      throw ConfigError("dock entry joints must be finite");
    }
  }
  active = true;
  reset_();
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
  descent_command_q_.reset();
  descent_path_.clear();
  descent_index_ = descent_lift_index_ = 1;
  descent_segment_elapsed_ = 0.;
  sit_settle_elapsed_ = 0.0;
  sit_settle_feet_.reset();
  leg_lift_start_feet_.reset();
  leg_lift_progress_ = 0.0;
  resetLegLift();
  middle_hold_q_.reset();
  last_command_q_.reset();
  support_wait_elapsed_ = body_raise_elapsed_ = descent_elapsed_ = 0.0;
  cached_pose.reset();
  cached_ids.clear();
  last_visual_target.reset();
  update_elapsed = std::max(0.0, update_period - dt_);
  update_dt = update_period;
  last_update_result.reset();
  perception_elapsed = std::max(0.0, perception_period - dt_);
  perception_sampled = false;
  last_perception_reason = "no perception result";
  using_last_complete_frame = false;
  entry_duration_ = 0.5;
  setState(kClimbTerminalEntry, "保持当前关节姿态，准备竖直抬升");
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
  descent_command_q_.reset();
  descent_path_.clear();
  descent_index_ = descent_lift_index_ = 1;
  descent_segment_elapsed_ = 0.;
  sit_settle_elapsed_ = 0.0;
  sit_settle_feet_.reset();
  leg_lift_start_feet_.reset();
  leg_lift_progress_ = 0.0;
  resetLegLift();
  middle_hold_q_.reset();
  last_command_q_.reset();
  support_wait_elapsed_ = body_raise_elapsed_ = descent_elapsed_ = 0.0;
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
    if (lift_last_feedback_q_) lift_stop_q_ = *lift_last_feedback_q_;
    setState(kFailed, reason);
  }
}

std::optional<std::pair<Matrix4d, std::vector<int>>> DockMode::perceptionPose(
    const DockRobotState& robot_state, std::string& reason_out) {
  Matrix4d pose;
  if (!robot_state.lock_from_pin.has_value()) {
    perception_elapsed += update_dt;
    const bool refresh = !perception_sampled || perception_elapsed + 1e-12 >= perception_period;
    if (!refresh) {
      reason_out = last_perception_reason;
      if (!cached_pose) return std::nullopt;
      return std::make_pair(*cached_pose, cached_ids);
    }
    perception_elapsed = 0.0;
    perception_sampled = true;
    const auto observed = latest_();
    if (!observed.valid) {
      cached_pose.reset();
      last_perception_reason = observed.reason;
      reason_out = observed.reason;
      return std::nullopt;
    }
    pose = observed.lock_from_pin;
    last_perception_reason.clear();
  } else {
    pose = *robot_state.lock_from_pin;
  }
  if (!pose.allFinite()) {
    reason_out = "non-finite lock_from_pin";
    return std::nullopt;
  }
  cached_pose = pose;
  reason_out.clear();
  return std::make_pair(pose, cached_ids);
}

FootPositions DockMode::actualFeet(const JointAngles& joints) const {
  return feet_(joints, false);
}

FootPositions DockMode::syncedFeet(const JointAngles& current) const {
  return feet_(current, true);
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
  if (entry_elapsed_ >= entry_duration_) {
    double error = 0.;
    for (int leg = 0; leg < kLegCount; ++leg)
      error = std::max(error, (current[leg] - (*entry_target_)[leg]).cwiseAbs().maxCoeff());
    if (error > kEntryTrackingTolerance) {
      support_wait_elapsed_ += update_dt;
      if (support_wait_elapsed_ > .8) return stopLegLift(current, "入口姿态未到位");
    } else {
      support_wait_elapsed_ = 0.;
      setState(kMiddleLift, "保持四腿，中腿先膝98deg后踝75deg收起");
    }
  }
  return makeResult(std::nullopt, *entry_target_);
}

FootPositions DockMode::freezeMiddleFeet(FootPositions feet) const {
  if (middle_hold_q_) {
    const auto locked = actualFeet(*middle_hold_q_);
    for (int leg : {2, 5}) feet[leg] = locked[leg];
  }
  return feet;
}

DockResult DockMode::bodyRaiseStep(const JointAngles& current) {
  if (!body_raise_start_feet_) body_raise_start_feet_ = syncedFeet(current);
  body_raise_elapsed_ += update_dt;
  FootPositions feet = *body_raise_start_feet_;
  for (int leg = 0; leg < kLegCount; ++leg)
    if (!isMiddleLeg(leg)) feet[leg].z() -= body_raise_progress_;
  feet = freezeMiddleFeet(feet);
  if (body_raise_elapsed_ > 3. * kBodyRaiseHeight / linear_speed_m_s_ + 3.)
    return stopLegLift(current, "四腿抬升跟踪超时");
  if (body_raise_progress_ >= kBodyRaiseHeight - 1e-9) {
    last_visual_target = feet;
    search_anchor_feet_ = feet;
    search_body_offset_.setZero();
    search_angle_ = .5 * M_PI; // 机身 +Y 为前方。
    search_on_circle_ = false;
    setState(kSearchingTag, "四腿抬升完成，开始30mm半径圆周搜索，中腿保持");
  } else {
    body_raise_progress_ = std::min(kBodyRaiseHeight,
        body_raise_progress_ + linear_speed_m_s_ * std::min(update_dt, .05));
    feet = *body_raise_start_feet_;
    for (int leg = 0; leg < kLegCount; ++leg)
      if (!isMiddleLeg(leg)) feet[leg].z() -= body_raise_progress_;
    feet = freezeMiddleFeet(feet);
  }
  return makeResult(feet, std::nullopt);
}

DockResult DockMode::tagSearchStep(const JointAngles& current,
                                  const std::string& reason) {
  if (!search_anchor_feet_) search_anchor_feet_ = freezeMiddleFeet(syncedFeet(current));
  const double step = kTagSearchSpeed * update_dt;
  if (search_on_circle_) search_angle_ -= step / kTagSearchRadius;
  const Eigen::Vector2d target = kTagSearchRadius *
      Eigen::Vector2d(std::cos(search_angle_), std::sin(search_angle_));
  const Eigen::Vector2d delta = target - search_body_offset_;
  const double distance = delta.norm();
  if (distance <= step) {
    search_body_offset_ = target;
    search_on_circle_ = true;
  } else {
    search_body_offset_ += delta * (step / distance);
  }
  // 固定搜索起点：支撑足端反向平移，中腿保持原关节姿态。
  FootPositions feet = *search_anchor_feet_;
  for (int leg = 0; leg < kLegCount; ++leg)
    if (!isMiddleLeg(leg)) feet[leg].head<2>() -= search_body_offset_;
  feet = freezeMiddleFeet(feet);
  last_visual_target = feet;
  setState(kSearchingTag, formatCn("圆周搜索偏移=(%.1f,%.1f)mm；%s",
      search_body_offset_.x()*1000., search_body_offset_.y()*1000., reason.c_str()));
  return makeResult(feet, std::nullopt);
}

FootPositions DockMode::visualStep(const JointAngles& current, const Matrix4d& pose) {
  Vector3d correction = -pose.block<3, 1>(0, 3);
  // 只取 XY 分量做水平修正。
  Eigen::Vector2d horizontal = correction.head<2>();
  const double distance = horizontal.norm();
  if (distance > alignment_speed_m_s_ * update_dt) {
    horizontal *= alignment_speed_m_s_ * update_dt / distance;
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
  result = freezeMiddleFeet(result);
  last_visual_target = result;
  return result;
}

DockResult DockMode::preDescentSettleStep(const JointAngles& current) {
  pre_descent_elapsed_ += update_dt;
  if (pre_descent_elapsed_ >= kPreDescentSettleDuration) {
    descent_elapsed_ = 0.;
    setState(kDescent, "稳定等待结束，开始机械导向下降；中腿保持");
    return descentStep(current);
  }
  return makeResult(std::nullopt, *descent_command_q_);
}

// 按足端高度求膝角，同时选择大腿、踝角，使二维水平偏移尽量小。
// 直接输出求得的关节姿态，避免公共DLS再次把踝目标推过边界。
bool DockMode::descentPose(double progress, const JointAngles& previous_q, JointAngles& target) const {
  const GraspKinematic model;
  const auto initial = model.baseToHip(*pre_descent_feet_);
  const auto lower = jointLower(), upper = jointUpper();
  target = previous_q;
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (isMiddleLeg(leg)) continue;
    const auto previous = target[leg];
    const double wanted_z = initial[leg].z() + progress;
    double best = std::numeric_limits<double>::infinity();
    // 沿用收腿规划的踝角支路和115deg规划余量。
    for (double ankle = 5.; ankle <= 115.; ankle += .5) {
      auto trial = previous;
      trial[2] = -ankle * M_PI / 180. / kJointAxisSigns[leg][2];
      const auto z_at = [&](double knee) {
        trial[1] = knee;
        return model.forwardLeg(leg, trial).z();
      };
      const double zp = z_at(M_PI / 2.), zm = z_at(-M_PI / 2.);
      const double c = (zp + zm) / 2., a = (zp - zm) / 2.;
      const double b = z_at(0.) - c, radius = std::hypot(a, b);
      if (radius < 1e-9 || std::abs(wanted_z - c) > radius) continue;
      const double alpha = std::asin(std::clamp((wanted_z - c) / radius, -1., 1.));
      for (double root : {alpha - std::atan2(b, a), M_PI - alpha - std::atan2(b, a)}) {
        trial[1] = std::remainder(root, 2. * M_PI);
        if (std::abs(trial[1]) > kLiftKneePlanLimit) continue;
        // 大腿角参与求解：按二维水平距离选方向，不固定径向或横向。
        trial[0] = 0.;
        const auto foot_zero = model.forwardLeg(leg, trial);
        const double yaw = std::remainder(std::atan2(initial[leg].y(), initial[leg].x()) -
            std::atan2(foot_zero.y(), foot_zero.x()), 2. * M_PI) / kJointAxisSigns[leg][0];
        trial[0] = std::clamp(yaw, lower[leg][0], upper[leg][0]);
        const double horizontal = (model.forwardLeg(leg, trial) - initial[leg]).head<2>().squaredNorm();
        if (horizontal > kDescentHorizontalAdjustment * kDescentHorizontalAdjustment) continue;
        const double score = horizontal + 1e-4 * (trial - previous).squaredNorm();
        if (score < best) { best = score; target[leg] = trial; }
      }
    }
    if (!std::isfinite(best)) return false;
  }
  return true;
}

bool DockMode::planDescent(const JointAngles& current) {
  const GraspKinematic model;
  // 指令起点沿用上一帧；反馈仍交给统一跟踪检查，避免115->117deg重设目标。
  JointAngles start = last_command_q_.value_or(current);
  if (middle_hold_q_) for (int leg : {2, 5}) start[leg] = (*middle_hold_q_)[leg];
  pre_descent_feet_ = model.forwardBase(start);
  descent_path_.clear();
  descent_path_.push_back({start, 0., 0.});
  const auto validSegment = [&](const JointAngles& previous, const JointAngles& target, bool limit_horizontal = true) {
    double delta = 0.;
    for (int leg = 0; leg < kLegCount; ++leg)
      delta = std::max(delta, (target[leg] - previous[leg]).cwiseAbs().maxCoeff());
    // 复用现有连杆碰撞和水平幅度检查，提前验证段内插值。
    const int samples = std::max(1, static_cast<int>(std::ceil(delta * 180. / M_PI)));
    for (int n = 1; n <= samples; ++n) {
      JointAngles q;
      for (int leg = 0; leg < kLegCount; ++leg)
        q[leg] = previous[leg] + (target[leg] - previous[leg]) * (double(n) / samples);
      if (!liftCollisionFree(model, q)) return false;
      const auto feet = model.forwardBase(q);
      for (int leg = 0; leg < kLegCount; ++leg)
        if (limit_horizontal && !isMiddleLeg(leg) && (feet[leg] - (*pre_descent_feet_)[leg]).head<2>().norm() >
            kDescentHorizontalAdjustment + 1e-9) return false;
    }
    return true;
  };
  const auto append = [&](const JointAngles& target, double height, bool limit_horizontal = true) {
    const auto previous = descent_path_.back().joints;
    double delta = 0., travel = 0.;
    for (int leg = 0; leg < kLegCount; ++leg)
      delta = std::max(delta, (target[leg] - previous[leg]).cwiseAbs().maxCoeff());
    const auto before = model.forwardBase(previous), after = model.forwardBase(target);
    for (int leg = 0; leg < kLegCount; ++leg)
      travel = std::max(travel, (after[leg] - before[leg]).norm());
    if (!validSegment(previous, target, limit_horizontal)) return false;
    descent_path_.push_back({target, height,
        std::max({dt_, delta / kLiftJointSpeed, travel / linear_speed_m_s_})});
    return true;
  };
  // 先离线计算可执行的支撑下降，再把末段收拢一起接上；执行时不再求解。
  double height = 0.;
  while (height < descent_total_ - 1e-9) {
    const double next = std::min(descent_total_, height + .002);
    JointAngles target;
    if (!descentPose(next, descent_path_.back().joints, target) || !append(target, next)) break;
    height = next;
  }
  // 下降终点不再回退：在完整的可执行下降之后，先膝、后踝。
  // 收腿取消30mm水平约束，沿用连杆碰撞检查，不增加阶段等待或评分。
  const auto fold_start = model.forwardBase(descent_path_.back().joints);
  descent_lift_index_ = descent_path_.size();
  for (int joint : {1, 2}) {
    while (true) {
      const auto previous = descent_path_.back().joints;
      auto target = previous;
      bool moved = false;
      for (int leg : {0, 1, 3, 4}) {
        const double goal = (joint == 1 ? kLiftKneePlanLimit : -75. * M_PI / 180.) /
                            kJointAxisSigns[leg][joint];
        const double step = std::clamp(goal - previous[leg][joint], -M_PI / 180., M_PI / 180.);
        if (std::abs(step) < 1e-9) continue;
        auto trial = target;
        trial[leg][joint] += step;
        if (validSegment(previous, trial, false)) { target = trial; moved = true; }
      }
      if (!moved) break;
      append(target, height, false);
    }
  }
  const auto final = descent_path_.back().joints;
  // 保留末段终点，即使某个方向已无运动余量，也沿用统一的完成处理。
  append(final, height, false);
  const auto final_feet = model.forwardBase(final);
  std::array<double, kLegCount> lifts{};
  for (int leg = 0; leg < kLegCount; ++leg) lifts[leg] = final_feet[leg].z() - fold_start[leg].z();
  resetLegLift();
  descent_command_q_ = start;
  descent_index_ = 1; descent_segment_elapsed_ = 0.;
  descent_duration_ = 0.;
  for (const auto& point : descent_path_) descent_duration_ += point.duration;
  if (logger_) logger_(false, formatCn("完整轨迹已规划：请求下降%.1fmm，支撑下降%.1fmm后连续收腿落座，预计%.2fs；末端踝角%.1f/%.1f/%.1f/%.1fdeg",
      descent_total_ * 1000., height * 1000., descent_duration_,
      final[0][2]*180./M_PI, final[1][2]*180./M_PI, final[3][2]*180./M_PI, final[4][2]*180./M_PI));
  if (logger_) logger_(false, formatCn("预规划收腿抬升量(lb/lf/rb/rf)：%.1f/%.1f/%.1f/%.1fmm（相对机身）",
      lifts[0]*1000., lifts[1]*1000., lifts[3]*1000., lifts[4]*1000.));
  return true;
}

DockResult DockMode::descentStep(const JointAngles& current) {
  if (descent_path_.empty()) return stopLegLift(current, "缺少预规划下降轨迹");
  double remaining_dt = std::min(update_dt, .05);
  while (descent_index_ < descent_path_.size() && remaining_dt > 1e-12) {
    const auto& previous = descent_path_[descent_index_ - 1];
    const auto& next = descent_path_[descent_index_];
    const double advance = std::min(remaining_dt, next.duration - descent_segment_elapsed_);
    descent_segment_elapsed_ += advance; remaining_dt -= advance;
    const double blend = std::min(1., descent_segment_elapsed_ / next.duration);
    for (int leg = 0; leg < kLegCount; ++leg)
      (*descent_command_q_)[leg] = previous.joints[leg] + blend * (next.joints[leg] - previous.joints[leg]);
    descent_remaining_ = std::max(0., descent_total_ -
        (previous.height + blend * (next.height - previous.height)));
    if (descent_index_ >= descent_lift_index_ && state != kLegLift)
      setState(kLegLift, "执行预规划收拢落座段，中腿保持");
    if (blend >= 1.) { ++descent_index_; descent_segment_elapsed_ = 0.; }
  }
  if (descent_index_ == descent_path_.size()) {
    double error = 0.;
    for (int leg = 0; leg < kLegCount; ++leg)
      if (!isMiddleLeg(leg)) error = std::max(error,
          (current[leg] - descent_path_.back().joints[leg]).cwiseAbs().maxCoeff());
    lift_settled_time_ = error <= kLiftTrackingTolerance ? lift_settled_time_ + update_dt : 0.;
    if (lift_settled_time_ >= .3) {
      lift_target_q_ = descent_path_.back().joints;
      setState(require_lock_confirmation_ ? kAligned : kSuccess,
          require_lock_confirmation_ ? "预规划落座动作完成，等待锁紧确认" : "预规划下降收拢完成，中腿保持");
    }
  }
  return makeResult(std::nullopt, *descent_command_q_);
}

DockResult DockMode::sitSettleStep(const JointAngles& current) {
  sit_settle_elapsed_ += update_dt;
  if (sit_settle_elapsed_ >= sit_settle_duration_s_) {
    setState(kLegLift, "下坐稳定完成，四腿分别收拢，不要求同高；中腿保持");
    return legLiftStep(current);
  }
  return makeResult(std::nullopt, descent_command_q_.value_or(current));
}

namespace {

// 与公共控制器一致的简化胶囊模型；直接关节轨迹绕过公共 DLS，必须单独检查。
// 这不包含对接台/实物止挡，膝关节的局部软限位仍不可省略。
bool liftCollisionFree(const GraspKinematic& model, const JointAngles& q) {
  const auto p = model.collisionPointsBase(q);
  const double radii[] = {std::hypot(.0125, .0125),
                         std::hypot(.011, .011), std::hypot(.011, .011),
                         std::hypot(.011, .011)};
  const Vector3d body_start(0., 0., 0.), body_end(0., 0., .121);
  for (int leg = 0; leg < kLegCount; ++leg) {
    for (int segment = 2; segment < 4; ++segment) {
      if (segmentDistance(p[leg][0], p[leg][1], p[leg][segment],
                          p[leg][segment + 1]) < radii[0] + radii[segment] + .003)
        return false;
    }
    for (int segment = 1; segment < 4; ++segment) {
      if (segmentDistance(p[leg][segment], p[leg][segment + 1],
                          body_start, body_end) < .097 + radii[segment] + .003)
        return false;
    }
    for (int other = 0; other < leg; ++other) {
      for (int first = 0; first < 4; ++first) {
        for (int second = 0; second < 4; ++second) {
          if (segmentDistance(p[leg][first], p[leg][first + 1],
                              p[other][second], p[other][second + 1]) <
              radii[first] + radii[second] + .003) return false;
        }
      }
    }
  }
  return true;
}

}  // namespace

bool DockMode::trackingHealthy(const JointAngles& current) {
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (std::abs(current[leg][1]) > kLiftKneeLimit ||
        std::abs(current[leg][2]) > kLiftAnkleLimit) {
      stopLegLift(current, formatCn("%s对接反馈越界：膝%.1fdeg 踝%.1fdeg", kLegNames[leg],
                                  current[leg][1] * 180. / M_PI, current[leg][2] * 180. / M_PI));
      return false;
    }
  }
  return true;
}

JointAngles DockMode::acceptCommand(const JointAngles& candidate, const JointAngles& current) {
  if (state == kFailed) return lift_stop_q_.value_or(current);
  for (const auto& q : current) {
    if (!q.allFinite()) {
      failExecution("对接输出缺少有限反馈");
      return lift_stop_q_.value_or(current);
    }
  }
  lift_last_feedback_q_ = current;
  // SUCCESS后的壳层HOLD不再注入DockRobotState，仍在输出出口监测中腿/各关节。
  if (state == kSuccess) {
    update_dt = dt_;
    if (!trackingHealthy(current)) return *lift_stop_q_;
  }
  JointAngles command = candidate;
  if (middle_hold_q_) for (int leg : {2, 5}) command[leg] = (*middle_hold_q_)[leg];
  const auto lower = jointLower(), upper = jointUpper();
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!command[leg].allFinite() || command[leg][0] < lower[leg][0] ||
        command[leg][0] > upper[leg][0] || std::abs(command[leg][1]) > kLiftKneeLimit ||
        std::abs(command[leg][2]) > kLiftAnkleLimit) {
      stopLegLift(current, formatCn("%s对接候选目标越界，撤销本帧运动", kLegNames[leg]));
      return *lift_stop_q_;
    }
  }
  const GraspKinematic model;
  if (!liftCollisionFree(model, command)) {
    stopLegLift(current, "对接候选姿态触发连杆碰撞模型");
    return *lift_stop_q_;
  }
  last_command_q_ = command;
  return command;
}

void DockMode::resetLegLift() {
  lift_start_q_.reset();
  lift_target_q_.reset();
  lift_command_q_.reset();
  lift_last_feedback_q_.reset();
  lift_stop_q_.reset();
  lift_elapsed_ = lift_wall_elapsed_ = lift_settled_time_ = 0.0;
  lift_duration_ = 0.0;
}

bool DockMode::planLegLift(const JointAngles& current, std::array<double, kLegCount>* lift_heights) {
  const GraspKinematic model;
  const auto initial = model.forwardBase(current);
  const auto lower = jointLower();
  const auto upper = jointUpper();
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!current[leg].allFinite() || !initial[leg].allFinite() ||
        current[leg][0] < lower[leg][0] || current[leg][0] > upper[leg][0] ||
        std::abs(current[leg][1]) > kLiftKneeLimit ||
        std::abs(current[leg][2]) > kLiftAnkleLimit) {
      reason = formatCn("%s收腿规划入口越界，拒绝规划；膝%.1fdeg 踝%.1fdeg",
                        kLegNames[leg], current[leg][1] * 180. / M_PI,
                        current[leg][2] * 180. / M_PI);
      return false;
    }
  }
  if (!liftCollisionFree(model, current)) {
    reason = "收腿入口触发连杆碰撞模型，拒绝继续收腿";
    return false;
  }
  const bool middle_only = state == kMiddleLift;
  // 中腿直接使用关节目标；兼容的四腿收腿分支仍按足端高度求解。
  const double preferred_lift = middle_only ? .040 : std::min(.030, leg_lift_height_m_);
  JointAngles target = current;
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (isMiddleLeg(leg) != middle_only) {
      if (isMiddleLeg(leg) && middle_hold_q_) target[leg] = (*middle_hold_q_)[leg];
      continue;
    }
    if (middle_only) {
      target[leg][1] = kLiftKneePlanLimit / kJointAxisSigns[leg][1];
      target[leg][2] = -75. * M_PI / 180. / kJointAxisSigns[leg][2];
      continue;
    }
    // 各腿独立寻找接近75deg且尽量高的姿态，不被另一条腿的低抬升量拖住。
    std::array<double, 3> best_score{INFINITY, INFINITY, INFINITY};
    const auto initial_hip = model.forwardLeg(leg, current[leg]);
    const auto anchor = model.baseToHip(pre_descent_feet_.value_or(initial));
    // 固定大腿，解析求膝角；保留既有5..115deg同支路搜索范围。
    for (int ankle_deg = 5; ankle_deg <= 115; ++ankle_deg) {
      Eigen::Vector3d trial = current[leg];
      trial[2] = -ankle_deg * M_PI / 180. / kJointAxisSigns[leg][2];
      const auto z_at = [&](double knee) {
        trial[1] = knee;
        return model.forwardLeg(leg, trial).z() + kHipXyz[leg].z();
      };
      const double zp = z_at(M_PI / 2.), zm = z_at(-M_PI / 2.);
      const double c = (zp + zm) / 2., a = (zp - zm) / 2.;
      const double b = z_at(0.) - c, radius = std::hypot(a, b);
      if (radius < 1e-9) continue;
      for (double height = preferred_lift; height >= .005 - 1e-9; height -= .001) {
        const double target_z = initial[leg].z() + height;
        if (std::abs(target_z - c) > radius) continue;
        const double alpha = std::asin(std::clamp((target_z - c) / radius, -1., 1.));
        for (double root : {alpha - std::atan2(b, a), M_PI - alpha - std::atan2(b, a)}) {
          trial[1] = std::remainder(root, 2. * M_PI);
          if (std::abs(trial[1]) > kLiftKneePlanLimit) continue;
          const auto foot_hip = model.forwardLeg(leg, trial);
          const double horizontal = (foot_hip - initial_hip).head<2>().norm();
          if (horizontal > .060) continue;
          if (!middle_only && pre_descent_feet_ &&
              (foot_hip - anchor[leg]).head<2>().norm() > kDescentHorizontalAdjustment) continue;
          const double ankle_error = (ankle_deg - 75.) * M_PI / 180.;
          const double movement = (trial - current[leg]).squaredNorm() + std::pow(horizontal / .060, 2);
          const std::array<double, 3> score = middle_only
              ? std::array<double, 3>{std::abs(ankle_error), -height, movement}
              : std::array<double, 3>{2. * ankle_error * ankle_error + std::pow(1. - height / preferred_lift, 2),
                                      std::abs(ankle_error), movement};
          if (score < best_score) { best_score = score; target[leg] = trial; }
        }
      }
    }
    if (!std::isfinite(best_score[0])) {
      reason = "限定搜索范围内未找到通过现有限位的收拢姿态";
      return false;
    }
  }
  bool feasible = true;
  const auto travel = liftJointTravel(current, target);
  const double max_delta = middle_only ? travel[1] + travel[2] : travel.maxCoeff();
  // 按实际先膝后踝路径采样，每次关节变化不超过1deg。
  const int samples = std::max(1, static_cast<int>(std::ceil(max_delta * 180. / M_PI)));
  double max_foot_derivative = 0.;
  auto previous_feet = initial;
  for (int sample = 1; sample <= samples && feasible; ++sample) {
    const double t = static_cast<double>(sample) / samples;
    const auto q = interpolateLift(current, target, t, middle_only);
    const auto feet = model.forwardBase(q);
    if (!liftCollisionFree(model, q)) { feasible = false; break; }
    for (int leg = 0; leg < kLegCount; ++leg) {
      if (feet[leg].z() < initial[leg].z() - .001 ||
          (!middle_only && (feet[leg] - initial[leg]).head<2>().norm() > .060)) feasible = false;
      max_foot_derivative = std::max(max_foot_derivative,
                                    (feet[leg] - previous_feet[leg]).norm() * samples);
    }
    previous_feet = feet;
  }
  if (!feasible) { reason = "收拢轨迹未通过现有连杆/足端范围检查"; return false; }
  lift_start_q_ = current;
  lift_target_q_ = target;
  lift_command_q_ = current;
  leg_lift_start_feet_ = initial;
  // 仅提前收起中腿提速2倍；同步缩放两项限速，保持原路径和先膝后踝顺序。
  const double speed_scale = middle_only ? 3.0 : 1.;
  lift_duration_ = std::max({.5, 1.875 * max_delta / (kLiftJointSpeed * speed_scale),
                            1.875 * max_foot_derivative / (leg_lift_speed_m_s_ * speed_scale)});
  const auto final_feet = model.forwardBase(target);
  std::array<double, kLegCount> lifts{};
  for (int leg = 0; leg < kLegCount; ++leg) lifts[leg] = final_feet[leg].z() - initial[leg].z();
  if (lift_heights) *lift_heights = lifts;
  reason = formatCn("%s轨迹就绪：预计%.2fs，膝规划不超过98deg",
                    middle_only ? "中腿" : "四腿", lift_duration_);
  if (logger_ && middle_only) logger_(false, reason);
  return true;
}

DockResult DockMode::stopLegLift(const JointAngles& current, const std::string& message) {
  // 撤销追赶目标，不自动回站姿、退让或卸力。保持反馈角不保证解除接触载荷。
  lift_stop_q_ = current;
  lift_command_q_ = current;
  setState(kFailed, message + "；已撤销收腿目标并保持反馈姿态，请检查机构");
  return makeResult(std::nullopt, current);
}

DockResult DockMode::legLiftStep(const JointAngles& current) {
  lift_last_feedback_q_ = current;
  if (!lift_target_q_ && !planLegLift(last_command_q_.value_or(current))) return stopLegLift(current, reason);
  lift_wall_elapsed_ += update_dt;
  const GraspKinematic model;
  if (!liftCollisionFree(model, current))
    return stopLegLift(current, "收腿反馈姿态触发连杆碰撞模型");
  // 本阶段运动腿决定轨迹推进，沿用原有轨迹时钟与完成判断。
  const bool middle_only = state == kMiddleLift;
  double max_error = 0.;
  double final_error = 0.;
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (isMiddleLeg(leg) != middle_only) continue;
    max_error = std::max(max_error,
        ((*lift_command_q_)[leg] - current[leg]).cwiseAbs().maxCoeff());
    final_error = std::max(final_error,
        ((*lift_target_q_)[leg] - current[leg]).cwiseAbs().maxCoeff());
  }
  if (lift_wall_elapsed_ > 3. * lift_duration_ + 3.)
    return stopLegLift(current, "收腿跟踪超时");
  // 误差大时冻结轨迹时钟，避免目标持续跑到实际姿态前方。
  if (max_error <= kLiftTrackingTolerance)
    lift_elapsed_ = std::min(lift_duration_, lift_elapsed_ + std::min(update_dt, .05));
  const double blend = smoothStep(lift_elapsed_ / lift_duration_);
  const auto command = interpolateLift(*lift_start_q_, *lift_target_q_, blend, middle_only);
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (std::abs(command[leg][1]) > kLiftKneeLimit ||
        std::abs(command[leg][2]) > kLiftAnkleLimit)
      return stopLegLift(current, "收腿输出越界");
  }
  lift_command_q_ = command;
  if (lift_elapsed_ >= lift_duration_ && final_error <= kLiftTrackingTolerance) {
    lift_settled_time_ += update_dt;
    if (lift_settled_time_ >= .3) {
      if (state == kMiddleLift) {
        middle_hold_q_ = command;
        body_raise_start_feet_ = freezeMiddleFeet(syncedFeet(current));
        body_raise_progress_ = body_raise_elapsed_ = support_wait_elapsed_ = 0.;
        resetLegLift();
        setState(kBodyRaise, "左右中腿到位并锁定，开始四腿支撑抬升");
      } else {
        setState(require_lock_confirmation_ ? kAligned : kSuccess,
                 require_lock_confirmation_ ? "四腿收拢完成，等待锁紧确认" : "四腿收拢完成，中腿保持，对接结束");
      }
    }
  } else {
    lift_settled_time_ = 0.;
  }
  return makeResult(std::nullopt, command);
}

DockResult DockMode::update(const DockRobotState& robot_state) {
  if (!active) {
    throw std::runtime_error("enter() must be called before update()");
  }
  if (isTerminalState(state)) {
    return updateOnce(robot_state);
  }

  update_elapsed += dt_;
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
    if (lift_stop_q_) return makeResult(std::nullopt, *lift_stop_q_);
    if (lift_target_q_ && state == kSuccess)
      return makeResult(std::nullopt, *lift_target_q_);
    if (!robot_state.joints.has_value()) {
      return makeResult(std::nullopt, std::nullopt);
    }
    return makeResult(std::nullopt, *robot_state.joints);
  }

  if (!robot_state.joints.has_value()) {
    failExecution("对接执行缺少统一控制链路的关节反馈");
    return makeResult(std::nullopt, lift_stop_q_);
  }
  const JointAngles& current = *robot_state.joints;
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!current[leg].allFinite()) {
      failExecution("关节反馈包含非有限值");
      return makeResult(std::nullopt, lift_stop_q_);
    }
  }

  lift_last_feedback_q_ = current;
  if (!trackingHealthy(current)) return makeResult(std::nullopt, lift_stop_q_);
  if (state == kMiddleLift) return legLiftStep(current);
  if (state == kClimbTerminalEntry) {
    return updateEntry(current);
  }
  if (state == kBodyRaise) {
    return bodyRaiseStep(current);
  }

  // 锁存预对接姿态后不再依赖圆环，避免目标离开视野时中断机械对接。
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
    return descent_path_.empty() ? legLiftStep(current) : descentStep(current);
  }

  if (state == kAligned) {
    if (robot_state.lock_confirmed.value_or(false)) {
      setState(kSuccess, "锁紧机构已确认，对接成功");
    }
    return makeResult(std::nullopt, lift_target_q_.value_or(current));
  }
  std::string perception_reason;
  const auto pose_and_ids = perceptionPose(robot_state, perception_reason);
  if (!pose_and_ids) {
    if (state == kSearchingTag) return tagSearchStep(current, perception_reason);
    if (!last_visual_target) last_visual_target = freezeMiddleFeet(syncedFeet(current));
    setState(kWaitingTag, "等待同心圆：" + perception_reason);
    return makeResult(*last_visual_target, std::nullopt);
  }
  const Matrix4d& pose = pose_and_ids->first;
  const double horizontal = pose.block<2, 1>(0, 3).norm();
  if (horizontal <= kPrealignPositionReference) {
    descent_total_ = descent_distance_ ? std::max(0.0, *descent_distance_)
                                      : std::max(0.0, -pose(2, 3)) + hole_depth_;
    descent_remaining_ = descent_total_;
    descent_duration_ = descent_total_ / linear_speed_m_s_;
    pre_descent_elapsed_ = 0.0;
    if (!planDescent(current)) return stopLegLift(current, reason);
    support_wait_elapsed_ = 0.;
    last_visual_target = pre_descent_feet_;
    setState(kPreDescentSettle, formatCn("同心圆到达下降参考：水平%.1fmm；稳定等待0.5s", horizontal * 1000.0));
    return makeResult(std::nullopt, *descent_command_q_);
  }
  setState(kPrealign, formatCn("四腿水平调整：水平%.1fmm，中腿保持", horizontal * 1000.0));
  return makeResult(visualStep(current, pose), std::nullopt);
}

bool DockMode::descentHasStarted() const {
  return state == kDescent || state == kSitSettle || state == kLegLift ||
         state == kAligned || state == kSuccess;
}

}  // namespace grasp_hexapod_control_cpp
