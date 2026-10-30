#include "grasp_hexapod_bt_control/tag_guidance.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace grasp_hexapod_bt_control {
namespace {
bool positive(double value) {
  return std::isfinite(value) && value > 0.;
}

double bounded(double value, double limit) {
  return std::max(-limit, std::min(limit, value));
}
}  // namespace

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
      !std::isfinite(c.ref_x_m) || !positive(c.ref_z_m) || !std::isfinite(c.ref_yaw_rad) ||
      !positive(c.lateral_gain) || !positive(c.forward_gain) || !positive(c.yaw_gain) ||
      !positive(c.max_lateral_speed_m_s) || !positive(c.max_forward_speed_m_s) ||
      !positive(c.max_yaw_rate) || !positive(c.position_tolerance_m) ||
      !positive(c.yaw_tolerance_rad) || !positive(c.settle_s) || !positive(c.loss_timeout_s) ||
      !positive(c.align_timeout_s)) {
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
}

void TagGuidance::resume(double now) {
  run_source_floor_ = now;
  last_new_stamp_ = now;
  last_valid_ = now;
  settle_started_ = 0.0;
  accepted_available_ = false;
  confirmations_ = 0;
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
  last_new_stamp_ = v.stamp;
  if (state_ == TagState::Coarse) {
    ++confirmations_;
  } else {
    last_valid_ = now;
  }
}

Eigen::Vector4d TagGuidance::servo(double x, double z, double yaw) const {
  const double ex = x - config_.ref_x_m;
  const double ez = z - config_.ref_z_m;
  const double eyaw = wrapAngle(yaw - config_.ref_yaw_rad);
  // Signs (see tests): tag to the right/farther than the reference -> move
  // right/forward.  A CCW (positive) body yaw rotates the scene CW in the
  // body frame, which decreases the tag yaw defined above, so positive eyaw
  // needs a positive (CCW) yaw command.
  const double right = std::abs(ex) <= config_.position_tolerance_m
      ? 0. : bounded(config_.lateral_gain * ex, config_.max_lateral_speed_m_s);
  const double forward = std::abs(ez) <= config_.position_tolerance_m
      ? 0. : bounded(config_.forward_gain * ez, config_.max_forward_speed_m_s);
  const double yaw_rate = std::abs(eyaw) <= config_.yaw_tolerance_rad
      ? 0. : bounded(config_.yaw_gain * eyaw, config_.max_yaw_rate);
  return Eigen::Vector4d(right, forward, 0., yaw_rate);
}

Eigen::Vector4d TagGuidance::update(double now, bool gait_stopped) {
  processCache(now);
  const Eigen::Vector4d zero = Eigen::Vector4d::Zero();
  if (state_ == TagState::Success || state_ == TagState::Failed) return zero;
  if (state_ == TagState::Coarse) {
    // "Consecutive" also means no gap: a run that stops getting new samples
    // for longer than the freshness window starts over.
    if (confirmations_ > 0 && now - last_new_stamp_ > config_.max_age_s) confirmations_ = 0;
    if (confirmations_ >= config_.confirm_frames) {
      state_ = TagState::Guiding;
      guiding_started_ = now;
      last_valid_ = now;
      settle_started_ = 0.0;
    }
    return zero;
  }
  if (now - guiding_started_ > config_.align_timeout_s) {
    state_ = TagState::Failed;
    reason_ = "tag alignment timeout";
    return zero;
  }
  if (now - last_valid_ > config_.loss_timeout_s) {
    state_ = TagState::Failed;
    reason_ = "tag lost";
    return zero;
  }
  if (!accepted_available_ || !fresh(accepted_, now)) {
    settle_started_ = 0.0;
    return zero;
  }
  const Eigen::Vector4d command = servo(accepted_.x, accepted_.z, accepted_yaw_);
  error_ = Eigen::Vector3d(accepted_.x - config_.ref_x_m, accepted_.z - config_.ref_z_m,
                           wrapAngle(accepted_yaw_ - config_.ref_yaw_rad));
  const bool inside = std::abs(error_[0]) <= config_.position_tolerance_m &&
                      std::abs(error_[1]) <= config_.position_tolerance_m &&
                      std::abs(error_[2]) <= config_.yaw_tolerance_rad;
  if (!inside || !gait_stopped) {
    settle_started_ = 0.0;
    return command;
  }
  if (!settle_started_) settle_started_ = now;
  if (now - settle_started_ >= config_.settle_s) state_ = TagState::Success;
  return zero;
}

}  // namespace grasp_hexapod_bt_control
