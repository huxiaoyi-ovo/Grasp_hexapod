#include "grasp_hexapod_bt_control/tag_guidance.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace grasp_hexapod_bt_control {
namespace {
bool positive(double value) {
  return std::isfinite(value) && value > 0.;
}

double bounded(double value, double limit) {
  return std::max(-limit, std::min(limit, value));
}

std::string meters(double value) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.3f", value);
  return text;
}
}  // namespace

std::array<bool, 6> effectiveStance(bool gait_started, bool transfer_active,
                                    const std::array<bool, 6>& raw_mask) {
  if (transfer_active || !gait_started) return {{true, true, true, true, true, true}};
  return raw_mask;
}

double wrapAngle(double angle) {
  const double two_pi = 2. * std::acos(-1.);
  angle = std::fmod(angle, two_pi);
  if (angle > two_pi / 2.) angle -= two_pi;
  if (angle <= -two_pi / 2.) angle += two_pi;
  return angle;
}

double tagYaw(const Eigen::Matrix3d& rotation) {
  return std::atan2(rotation(2, 0), rotation(0, 0));
}

bool tagYawFromQuaternion(double qx, double qy, double qz, double qw, double& yaw) {
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (!std::isfinite(norm) || norm < 1e-6) return false;
  const Eigen::Quaterniond q(qw / norm, qx / norm, qy / norm, qz / norm);
  const Eigen::Matrix3d rotation = q.toRotationMatrix();
  if (!rotation.allFinite() || std::abs(rotation(1, 0)) > kMaxTagXAxisOutOfPlane) return false;
  yaw = tagYaw(rotation);
  return true;
}

TagGuidance::TagGuidance(TagGuidanceConfig c) : config_(c) {
  if (c.tag_id < 0 || c.confirm_frames < 1 || !positive(c.max_age_s) || c.expected_frame.empty() ||
      !std::isfinite(c.ref_x_m) || !positive(c.p0_z_m) || !std::isfinite(c.ref_yaw_rad) ||
      !positive(c.lateral_gain) || !positive(c.forward_gain) || !positive(c.yaw_gain) ||
      !positive(c.max_lateral_speed_m_s) || !positive(c.max_forward_speed_m_s) ||
      !positive(c.max_yaw_rate) || !positive(c.position_tolerance_m) ||
      !positive(c.yaw_tolerance_rad) || !positive(c.settle_s) || !positive(c.loss_timeout_s) ||
      !positive(c.align_timeout_s) || !std::isfinite(c.final_overshoot_m) ||
      c.final_overshoot_m < 0. || !(c.p0_z_m - c.final_overshoot_m > 0.) ||
      !positive(c.handover_loss_s) || !std::isfinite(c.min_reliable_z_m) ||
      c.min_reliable_z_m < 0. || !positive(c.max_blind_distance_m) ||
      !positive(c.blind_speed_m_s) || !positive(c.heading_gain) ||
      !positive(c.max_heading_error_rad) || !positive(c.odom_mismatch_m) ||
      !positive(c.blind_timeout_s) || !(c.imu_yaw_sign == 1. || c.imu_yaw_sign == -1.)) {
    throw std::invalid_argument("invalid tag guidance configuration");
  }
}

void TagGuidance::cache(const TagObservation& value) {
  std::lock_guard<std::mutex> guard(lock_);
  latest_.value = value;
  ++latest_.generation;
}

TagGuidance::Cached TagGuidance::latest() const {
  std::lock_guard<std::mutex> guard(lock_);
  return latest_;
}

void TagGuidance::begin(double now) {
  state_ = TagState::Coarse;
  run_source_floor_ = now;
  last_new_stamp_ = now;
  last_valid_ = now;
  guiding_started_ = 0.0;
  settle_started_ = 0.0;
  accepted_available_ = false;
  confirmations_ = 0;
  error_.setZero();
  reason_.clear();
  have_last_valid_ = false;
  handover_ = TagHandover();
  odom_ = cmd_integral_ = last_forward_cmd_ = heading_error_ = odom_at_last_valid_ = 0.0;
  last_imu_valid_ = imu_at_last_valid_ok_ = false;
  last_imu_heading_ = imu_heading_at_last_valid_ = 0.0;
  blind_started_ = yaw_hold_ = 0.0;
  have_last_update_ = false;
  prev_feet_valid_ = false;
}

void TagGuidance::resume(double now) {
  run_source_floor_ = now;
  last_new_stamp_ = now;
  last_valid_ = now;
  settle_started_ = 0.0;
  accepted_available_ = false;
  have_last_valid_ = false;
  confirmations_ = 0;
  // Blind finish: yaw_hold_, odometry and the command integral are kept, but
  // nothing accumulates across the pause (no dt, new foot baseline).
  have_last_update_ = false;
  prev_feet_valid_ = false;
  last_forward_cmd_ = 0.0;
}

bool TagGuidance::fresh(const TagObservation& v, double now) const {
  return std::isfinite(v.stamp) && v.stamp > run_source_floor_ && now >= v.stamp &&
         now - v.stamp <= config_.max_age_s;
}

bool TagGuidance::valid(const TagObservation& v, double now, double& yaw) const {
  return v.frame == config_.expected_frame && std::isfinite(v.x) && std::isfinite(v.y) &&
         std::isfinite(v.z) && v.z > 0.0 && fresh(v, now) &&
         tagYawFromQuaternion(v.qx, v.qy, v.qz, v.qw, yaw);
}

void TagGuidance::processCache(double now) {
  const Cached cached = latest();
  if (cached.generation == processed_generation_) return;
  processed_generation_ = cached.generation;
  const TagObservation& v = cached.value;
  if (v.id != config_.tag_id) return;  // other tags never count and never reset
  double yaw = 0.0;
  if (!valid(v, now, yaw)) {
    // Stale / wrong frame / non-finite / impossible pose: not usable.
    accepted_available_ = false;
    if (state_ == TagState::Coarse) confirmations_ = 0;
    return;
  }
  // Duplicate or reordered stamps are ignored without invalidating the
  // previous good sample.
  if (v.stamp <= source_high_watermark_) return;
  source_high_watermark_ = v.stamp;
  accepted_ = v;
  accepted_yaw_ = yaw;
  accepted_available_ = true;
  have_last_valid_ = true;
  last_x_ = v.x;
  last_z_ = v.z;
  last_yaw_ = yaw;
  odom_at_last_valid_ = odom_;  // odometry accumulates continuously from handoff
  imu_at_last_valid_ok_ = last_imu_valid_;  // IMU heading of the most recent update
  imu_heading_at_last_valid_ = last_imu_heading_;
  last_new_stamp_ = v.stamp;
  if (state_ == TagState::Coarse) {
    ++confirmations_;
  } else {
    last_valid_ = now;
  }
}

Eigen::Vector4d TagGuidance::servo(double x, double z, double yaw, bool allow_forward) const {
  const double ex = x - config_.ref_x_m;
  const double ez = z - zTarget();
  const double eyaw = wrapAngle(yaw - config_.ref_yaw_rad);
  // Signs (see tests): tag to the right than the reference -> move right;
  // tag farther than the target -> move forward (never backward).  A CCW
  // (positive) body yaw rotates the scene CW in the body frame, which decreases
  // the tag yaw defined above, so positive eyaw needs a positive (CCW) yaw
  // command.
  const double position_deadband = kServoDeadbandFraction * config_.position_tolerance_m;
  const double right = std::abs(ex) <= position_deadband
      ? 0. : bounded(config_.lateral_gain * ex, config_.max_lateral_speed_m_s);
  const double forward = (!allow_forward || ez <= position_deadband)
      ? 0. : std::min(config_.forward_gain * ez, config_.max_forward_speed_m_s);
  const double yaw_rate = std::abs(eyaw) <= kServoDeadbandFraction * config_.yaw_tolerance_rad
      ? 0. : bounded(config_.yaw_gain * eyaw, config_.max_yaw_rate);
  return Eigen::Vector4d(right, forward, 0., yaw_rate);
}

void TagGuidance::fail(const std::string& reason) {
  state_ = TagState::Failed;
  reason_ = reason;
  last_forward_cmd_ = 0.0;
}

// Body forward increment = -mean(delta foot_y) over the legs that were on the
// ground in both this and the previous frame.  Runs every frame after handoff
// so that a baseline exists at handover; reset by resume().
void TagGuidance::trackOdometry(const TagGuidanceInput& input) {
  bool finite = input.feet_valid;
  for (const auto& foot : input.feet_base) finite = finite && foot.allFinite();
  if (!finite) {
    prev_feet_valid_ = false;
    return;
  }
  const std::array<bool, 6> stance =
      effectiveStance(input.gait_started, input.transfer_active, input.stance_mask);
  if (prev_feet_valid_) {
    double sum = 0.0;
    int count = 0;
    for (int leg = 0; leg < 6; ++leg) {
      if (!stance[leg] || !prev_stance_[leg]) continue;
      sum += input.feet_base[leg].y() - prev_feet_[leg].y();
      ++count;
    }
    if (count > 0) odom_ += -sum / count;
  }
  prev_feet_ = input.feet_base;
  prev_stance_ = stance;
  prev_feet_valid_ = true;
}

Eigen::Vector4d TagGuidance::settling(double now, bool gait_stopped) {
  if (!gait_stopped) {
    settle_started_ = 0.0;
    return Eigen::Vector4d::Zero();
  }
  if (!settle_started_) settle_started_ = now;
  if (now - settle_started_ >= config_.settle_s) state_ = TagState::Success;
  return Eigen::Vector4d::Zero();
}

// The tag is no longer reliable while Advancing: evaluate the last valid sample.
Eigen::Vector4d TagGuidance::doHandover(double now, const TagGuidanceInput& input) {
  const Eigen::Vector4d zero = Eigen::Vector4d::Zero();
  if (!have_last_valid_) {
    fail("tag lost before alignment");
    return zero;
  }
  const double ex = last_x_ - config_.ref_x_m;
  const double eyaw = wrapAngle(last_yaw_ - config_.ref_yaw_rad);
  if (std::abs(ex) > config_.position_tolerance_m || std::abs(eyaw) > config_.yaw_tolerance_rad) {
    fail("tag lost before alignment");
    return zero;
  }
  const double travelled = odom_ - odom_at_last_valid_;
  const double remaining = last_z_ - zTarget() - travelled;
  handover_ = TagHandover{true, last_z_, ex, eyaw, travelled, remaining};
  if (remaining <= 0.) {
    state_ = TagState::Settling;
    settle_started_ = 0.0;
    return zero;
  }
  if (remaining > config_.max_blind_distance_m) {
    fail("tag lost too far from P0 (remaining " + meters(remaining) + " m)");
    return zero;
  }
  if (!input.imu_valid || !std::isfinite(input.imu_yaw)) {
    fail("imu unavailable for blind finish");
    return zero;
  }
  state_ = TagState::Blind;
  // Tag-aligned heading target: the IMU heading when the last valid tag sample
  // was taken, plus the tag yaw error then (positive eyaw needs CCW rotation).
  // This removes the residual tag yaw error and any rotation during the loss
  // window instead of freezing them.  If that IMU heading was not valid, the
  // current one stands in.
  const double heading_now = config_.imu_yaw_sign * input.imu_yaw;
  const double heading_at_sample = imu_at_last_valid_ok_ ? imu_heading_at_last_valid_ : heading_now;
  yaw_hold_ = wrapAngle(heading_at_sample + eyaw);
  handover_.heading_target_offset = wrapAngle(yaw_hold_ - heading_now);
  heading_error_ = 0.0;
  blind_started_ = now;
  odom_ = 0.0;
  cmd_integral_ = 0.0;
  last_forward_cmd_ = 0.0;
  return blind(now, input);
}

Eigen::Vector4d TagGuidance::blind(double now, const TagGuidanceInput& input) {
  const Eigen::Vector4d zero = Eigen::Vector4d::Zero();
  if (!input.imu_valid || !std::isfinite(input.imu_yaw)) {
    fail("imu unavailable for blind finish");
    return zero;
  }
  heading_error_ = wrapAngle(yaw_hold_ - config_.imu_yaw_sign * input.imu_yaw);
  if (std::abs(heading_error_) > config_.max_heading_error_rad) {
    fail("blind heading error");
    return zero;
  }
  if (now - blind_started_ > config_.blind_timeout_s) {
    fail("blind finish timeout");
    return zero;
  }
  if (std::abs(odom_ - cmd_integral_) > config_.odom_mismatch_m) {
    fail("blind odometry mismatch (blocked or slipping legs?)");
    return zero;
  }
  if (odom_ >= handover_.remaining) {
    // Forward stops here; odometry keeps being tracked while the gait drains.
    state_ = TagState::Settling;
    settle_started_ = 0.0;
    last_forward_cmd_ = 0.0;
    return zero;
  }
  last_forward_cmd_ = config_.blind_speed_m_s;
  return Eigen::Vector4d(
      0., config_.blind_speed_m_s, 0.,
      bounded(config_.heading_gain * heading_error_, config_.max_yaw_rate));
}

Eigen::Vector4d TagGuidance::update(double now, bool gait_stopped) {
  TagGuidanceInput input;
  input.gait_started = !gait_stopped;
  return update(now, input);
}

Eigen::Vector4d TagGuidance::update(double now, const TagGuidanceInput& input) {
  last_imu_valid_ = input.imu_valid && std::isfinite(input.imu_yaw);
  if (last_imu_valid_) last_imu_heading_ = wrapAngle(config_.imu_yaw_sign * input.imu_yaw);
  processCache(now);
  const Eigen::Vector4d zero = Eigen::Vector4d::Zero();
  if (state_ == TagState::Success || state_ == TagState::Failed) return zero;
  const bool gait_stopped = !input.gait_started && !input.transfer_active;
  if (state_ == TagState::Coarse) {
    // "Consecutive" also means no gap: a run that stops getting new samples
    // for longer than the freshness window starts over.
    if (confirmations_ > 0 && now - last_new_stamp_ > config_.max_age_s) confirmations_ = 0;
    if (confirmations_ >= config_.confirm_frames) {
      state_ = TagState::Aligning;
      guiding_started_ = now;
      last_valid_ = now;
      settle_started_ = 0.0;
    }
    return zero;
  }

  // Integrate the forward command applied over the last interval (monitor
  // only) and the leg odometry.  Neither accumulates across a pause.
  if (have_last_update_ && state_ == TagState::Blind) {
    cmd_integral_ += last_forward_cmd_ * std::max(0., now - last_update_time_);
  }
  have_last_update_ = true;
  last_update_time_ = now;
  trackOdometry(input);

  if (state_ == TagState::Blind) return blind(now, input);
  if (state_ == TagState::Settling) return settling(now, gait_stopped);

  // Aligning / Advancing.
  if (now - guiding_started_ > config_.align_timeout_s) {
    fail("tag alignment timeout");
    return zero;
  }
  if (state_ == TagState::Aligning) {
    if (now - last_valid_ > config_.loss_timeout_s) {
      fail("tag lost");
      return zero;
    }
  } else {
    const bool lost = now - last_valid_ > config_.handover_loss_s;
    const bool too_close = config_.min_reliable_z_m > 0. && have_last_valid_ &&
                           last_z_ < config_.min_reliable_z_m;
    if (lost || too_close) return doHandover(now, input);
  }
  if (!accepted_available_ || !fresh(accepted_, now)) {
    settle_started_ = 0.0;
    return zero;
  }
  error_ = Eigen::Vector3d(accepted_.x - config_.ref_x_m, accepted_.z - zTarget(),
                           wrapAngle(accepted_yaw_ - config_.ref_yaw_rad));
  const bool gate = std::abs(error_[0]) <= 2. * config_.position_tolerance_m &&
                    std::abs(error_[2]) <= 2. * config_.yaw_tolerance_rad;
  if (state_ == TagState::Aligning && gate) state_ = TagState::Advancing;
  const bool advancing = state_ == TagState::Advancing;
  const Eigen::Vector4d command =
      servo(accepted_.x, accepted_.z, accepted_yaw_, advancing && gate);
  // One-sided in depth: closer to Xiaolan than z_target is acceptable.
  const bool inside = std::abs(error_[0]) <= config_.position_tolerance_m &&
                      error_[1] <= config_.position_tolerance_m &&
                      std::abs(error_[2]) <= config_.yaw_tolerance_rad;
  if (!advancing || !inside || !gait_stopped) {
    settle_started_ = 0.0;
    return command;
  }
  if (!settle_started_) settle_started_ = now;
  if (now - settle_started_ >= config_.settle_s) state_ = TagState::Success;
  return zero;
}

}  // namespace grasp_hexapod_bt_control
