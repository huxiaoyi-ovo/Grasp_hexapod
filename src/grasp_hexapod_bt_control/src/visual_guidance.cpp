#include "grasp_hexapod_bt_control/visual_guidance.h"

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

VisualGuidanceConfig loadVisualGuidanceConfig(ros::NodeHandle& p) {
  VisualGuidanceConfig c;
  c.max_age_s = p.param("visual_target_max_age_s", .25);
  c.confirm_frames = p.param("visual_confirm_frames", 3.);
  c.lock_angle_rad = p.param("visual_lock_angle_rad", .10);
  c.align_angle_rad = p.param("visual_align_angle_rad", .15);
  c.search_yaw_rate = p.param("visual_search_yaw_rate", .15);
  c.yaw_gain = p.param("visual_yaw_gain", .8);
  c.max_yaw_rate = p.param("visual_max_yaw_rate", .15);
  c.max_forward_speed_m_s = p.param("visual_max_forward_speed_m_s", .03);
  c.distance_gain = p.param("visual_distance_gain", .3);
  c.stop_distance_m = p.param("visual_stop_distance_m", .60);
  c.settle_s = p.param("visual_settle_s", .25);
  c.loss_timeout_s = p.param("visual_loss_timeout_s", 1.);
  c.search_timeout_s = p.param("visual_search_timeout_s", 60.);
  c.approach_timeout_s = p.param("visual_approach_timeout_s", 60.);
  c.stop_tolerance_m = p.param("visual_stop_tolerance_m", .02);
  c.input_startup_timeout_s = p.param("visual_input_startup_timeout_s", 2.);
  c.input_loss_timeout_s = p.param("visual_input_loss_timeout_s", .5);
  c.diagnostics_period_s = p.param("visual_diagnostics_period_s", 2.);
  c.expected_frame = p.param<std::string>("visual_expected_target_frame", "camera_color_optical_frame");
  return c;
}

VisualGuidance::VisualGuidance(VisualGuidanceConfig c) : config_(c) {
  if (!positive(c.max_age_s) || !positive(c.confirm_frames) ||
      std::floor(c.confirm_frames) != c.confirm_frames || !positive(c.lock_angle_rad) ||
      !positive(c.align_angle_rad) || c.lock_angle_rad > c.align_angle_rad ||
      !positive(c.search_yaw_rate) || !positive(c.yaw_gain) || !positive(c.max_yaw_rate) ||
      !positive(c.max_forward_speed_m_s) || !positive(c.distance_gain) ||
      !positive(c.stop_distance_m) || !positive(c.stop_tolerance_m) || c.stop_tolerance_m >= c.stop_distance_m || !positive(c.settle_s) || !positive(c.loss_timeout_s) || !positive(c.input_startup_timeout_s) || !positive(c.input_loss_timeout_s) || c.input_loss_timeout_s <= c.max_age_s || !std::isfinite(c.diagnostics_period_s) || c.diagnostics_period_s < 1. || c.expected_frame.empty() ||
      !positive(c.search_timeout_s) || !positive(c.approach_timeout_s)) {
    throw std::invalid_argument("invalid visual guidance configuration");
  }
}
void VisualGuidance::cache(const VisualObservation& value) {
  std::lock_guard<std::mutex> guard(lock_);
  // The callback only stores a sample.  The control thread validates timestamp
  // and contents before changing the accepted source high-watermark.
  latest_.value = value;
  ++latest_.generation;
  ++target_rx_count_;
}

void VisualGuidance::cacheHealth(const VisualHealth& health) {
  std::lock_guard<std::mutex> guard(lock_);
  health_ = health;
  ++health_generation_;
}

VisualGuidance::CachedObservation VisualGuidance::latest() const {
  std::lock_guard<std::mutex> guard(lock_);
  return latest_;
}

void VisualGuidance::begin(VisualState s, double now) {
  state_ = s;
  started_ = now;
  run_source_floor_ = now;
  last_new_stamp_ = run_source_floor_;
  last_valid_ = 0.0;
  settle_started_ = 0.0;
  accepted_available_ = false;
  accepted_generation_ = 0;
  confirmations_ = 0;
  recovery_pending_ = false;
  reason_.clear();
  last_good_health_stamp_ = 0.;
  diagnostics_window_start_ = now;
  last_report_time_ = now;
  last_report_state_ = s;
  reported_ = false;
  {
    std::lock_guard<std::mutex> guard(lock_);
    report_target_rx_count_ = target_rx_count_;
    report_fresh_accepted_count_ = fresh_accepted_count_;
  }
}

bool VisualGuidance::freshHealth(double now) const {
  const double stamp = std::min(health_.rgb_stamp, health_.depth_stamp);
  return std::isfinite(health_.stamp) && std::isfinite(health_.rgb_stamp) && std::isfinite(health_.depth_stamp) &&
      health_.rgb_stamp > run_source_floor_ && health_.depth_stamp > run_source_floor_ && health_.stamp > run_source_floor_ &&
      std::abs(stamp - health_.stamp) <= 1e-6 && now >= health_.stamp && now >= health_.rgb_stamp && now >= health_.depth_stamp && now - stamp <= config_.max_age_s &&
      health_.frame == config_.expected_frame;
}

void VisualGuidance::resume(double now) {
  run_source_floor_ = now;
  last_new_stamp_ = now;
  last_valid_ = now;
  settle_started_ = 0.0;
  confirmations_ = 0;
  accepted_available_ = false;
  accepted_generation_ = 0;
  recovery_pending_ = false;
  last_good_health_stamp_ = 0.;
  if (state_ == VisualState::Locking) {
    state_ = VisualState::Searching;
  } else if (state_ == VisualState::Settling) {
    state_ = VisualState::Approaching;
  }
}

bool VisualGuidance::fresh(const VisualObservation& v, double now) const {
  return v.detected && std::isfinite(v.x) && std::isfinite(v.z) &&
         std::isfinite(v.stamp) && v.stamp > 0.0 && v.z > 0.0 &&
         !v.frame.empty() &&
         v.stamp > run_source_floor_ && now >= v.stamp &&
         now - v.stamp <= config_.max_age_s;
}

void VisualGuidance::processCache(double now) {
  const CachedObservation cached = latest();
  if (cached.generation == processed_generation_) {
    return;
  }
  processed_generation_ = cached.generation;
  accepted_available_ = false;
  if (cached.value.detected && cached.value.frame != config_.expected_frame) {
    state_ = VisualState::Failed;
    reason_ = "FRAME_MISMATCH";
    return;
  }

  // New callbacks expose producer-held or reordered source stamps.  They are
  // invalidated here, after finite/current-time validation, so bad future
  // timestamps never poison the accepted source ordering.
  if (!fresh(cached.value, now) ||
      cached.value.stamp <= source_high_watermark_) {
    return;
  }
  source_high_watermark_ = cached.value.stamp;
  accepted_ = cached.value;
  accepted_generation_ = cached.generation;
  accepted_available_ = true;
}

Eigen::Vector4d VisualGuidance::update(double now, bool gait_stopped) {
  processCache(now);
  const auto zero = Eigen::Vector4d::Zero();
  if (state_ == VisualState::Success || state_ == VisualState::Failed) return zero;
  if ((state_ == VisualState::Searching || state_ == VisualState::Locking) && now - started_ > config_.search_timeout_s) {
    state_ = VisualState::Failed; reason_ = "visual search timeout"; return zero;
  }
  if ((state_ == VisualState::Approaching || state_ == VisualState::Settling) && now - started_ > config_.approach_timeout_s) {
    state_ = VisualState::Failed; reason_ = "visual target lost or approach timeout"; return zero;
  }
  bool health_ok;
  int health_state;
  bool first_good = false;
  {
    std::lock_guard<std::mutex> guard(lock_);
    health_ok = freshHealth(now);
    health_state = health_.state;
    first_good = last_good_health_stamp_ == 0.;
    if (health_ok && (health_state == VisualHealth::NO_TARGET || health_state == VisualHealth::TARGET_VALID) && health_.stamp > last_good_health_stamp_) {
      last_good_health_stamp_ = health_.stamp;
    }
  }
  const bool authorizes = health_ok && (health_state == VisualHealth::NO_TARGET || health_state == VisualHealth::TARGET_VALID);
  if (authorizes && first_good) last_valid_ = now;
  if (!authorizes) {
    confirmations_ = 0;
    settle_started_ = 0.;
    if (state_ == VisualState::Locking) { state_ = VisualState::Searching; recovery_pending_ = true; }
    if (state_ == VisualState::Settling) state_ = VisualState::Approaching;
    if (last_good_health_stamp_ == 0. && now - run_source_floor_ > config_.input_startup_timeout_s) {
      state_ = VisualState::Failed;
      reason_ = "perception startup unavailable";
    } else if (last_good_health_stamp_ > 0. && now - last_good_health_stamp_ > config_.input_loss_timeout_s) {
      state_ = VisualState::Failed;
      reason_ = "perception stale/unusable";
    }
    return Eigen::Vector4d::Zero();
  }
  if (health_state == VisualHealth::NO_TARGET) {
    accepted_available_ = false;
  }
  const VisualObservation v = accepted_;
  const bool valid = accepted_available_ && fresh(v, now);
  // Subsequent control reads of the same callback stay valid but do not count
  // toward a new confirmation or settling sample.
  const bool distinct_new = valid && v.stamp > last_new_stamp_;
  if (distinct_new) {
    last_new_stamp_ = v.stamp;
    last_valid_ = now;
    std::lock_guard<std::mutex> guard(lock_);
    ++fresh_accepted_count_;
  }
  const double theta = valid ? std::atan2(v.x, v.z) : 0.;
  if (state_ == VisualState::Searching) {
    if (now - started_ > config_.search_timeout_s) {
      state_ = VisualState::Failed;
      reason_ = "visual search timeout";
      return zero;
    }
    if (recovery_pending_) {
      if (gait_stopped) {
        recovery_pending_ = false;
      } else {
        return zero;
      }
    }
    if (!valid) {
      confirmations_ = 0;
      return health_state == VisualHealth::NO_TARGET ? Eigen::Vector4d(0., 0., 0., bounded(config_.search_yaw_rate, config_.max_yaw_rate)) : zero;
    }
    if (std::abs(theta) > config_.lock_angle_rad) {
      confirmations_ = 0;
      return Eigen::Vector4d(0., 0., 0.,
                             bounded(-config_.yaw_gain * theta,
                                     config_.max_yaw_rate));
    }
    if (distinct_new) {
      ++confirmations_;
    }
    if (confirmations_ >= static_cast<int>(config_.confirm_frames)) {
      state_ = VisualState::Locking;
      settle_started_ = 0.;
    }
    return zero;
  }
  if (state_ == VisualState::Locking) {
    if (now - started_ > config_.search_timeout_s) {
      state_ = VisualState::Failed;
      reason_ = "visual search timeout";
      return zero;
    }
    if (!valid || std::abs(theta) > config_.lock_angle_rad) {
      state_ = VisualState::Searching;
      confirmations_ = 0;
      recovery_pending_ = true;
      return zero;
    }
    if (distinct_new && gait_stopped) {
      if (!settle_started_) {
        settle_started_ = now;
      }
      if (now - settle_started_ >= config_.settle_s) {
        state_ = VisualState::Success;
      }
    } else if (!gait_stopped) {
      settle_started_ = 0.;
    }
    return zero;
  }
  if (state_ == VisualState::Approaching || state_ == VisualState::Settling) {
    const bool lost = last_valid_ == 0. ? now - started_ > config_.loss_timeout_s
                                         : now - last_valid_ > config_.loss_timeout_s;
    if (now - started_ > config_.approach_timeout_s || lost) {
      state_ = VisualState::Failed;
      reason_ = "visual target lost or approach timeout";
      return zero;
    }
    if (!valid) {
      settle_started_ = 0.;
      return zero;
    }
    if (v.z <= config_.stop_distance_m + config_.stop_tolerance_m + 1e-6 && std::abs(theta) <= config_.lock_angle_rad) {
      state_ = VisualState::Settling;
      if (distinct_new && gait_stopped) {
        if (!settle_started_) settle_started_ = now;
        if (now - settle_started_ >= config_.settle_s) state_ = VisualState::Success;
      } else if (!gait_stopped) {
        settle_started_ = 0.;
      }
      return zero;
    }
    state_ = VisualState::Approaching;
    settle_started_ = 0.;
    const double yaw = bounded(-config_.yaw_gain * theta, config_.max_yaw_rate);
    const double forward = std::abs(theta) > config_.align_angle_rad ? 0. : std::min(config_.max_forward_speed_m_s, config_.distance_gain * std::max(0., v.z - config_.stop_distance_m));
    return Eigen::Vector4d(0., forward, 0., yaw);
  }
  return zero;
}

void VisualGuidance::report(double now, const std::string& mode,
                            const Eigen::Vector4d& command,
                            bool gait_stopped) {
  VisualHealth health;
  VisualObservation raw_target;
  VisualObservation accepted_target;
  bool accepted_available;
  bool current_health_ok;
  uint64_t target_rx_count;
  uint64_t fresh_accepted_count;
  double health_stamp;
  {
    std::lock_guard<std::mutex> guard(lock_);
    health = health_;
    raw_target = latest_.value;
    accepted_target = accepted_;
    accepted_available = accepted_available_;
    current_health_ok = freshHealth(now);
    target_rx_count = target_rx_count_;
    fresh_accepted_count = fresh_accepted_count_;
    health_stamp = last_good_health_stamp_;
  }
  const bool terminal = state_ == VisualState::Success || state_ == VisualState::Failed;
  const bool changed = state_ != last_report_state_;
  const double elapsed = now - diagnostics_window_start_;
  const bool immediate_terminal = terminal && state_ != last_report_state_;
  const bool first_report = !reported_;
  const bool summary_due = elapsed >= config_.diagnostics_period_s;
  const bool phase_due = changed && now - last_report_time_ >= .5;
  if (!first_report && !immediate_terminal && !summary_due && !phase_due) {
    return;
  }
  const double window = elapsed < .1 ? 0. : elapsed;
  const double target_rx_hz = window > 0.
      ? static_cast<double>(target_rx_count - report_target_rx_count_) / window : 0.;
  const double fresh_hz = window > 0.
      ? static_cast<double>(fresh_accepted_count - report_fresh_accepted_count_) / window : 0.;
  const double health_age_ms = health_stamp > 0. ? 1000. * (now - health_stamp) : -1.;
  const double target_age_ms = raw_target.stamp > 0.
      ? 1000. * (now - raw_target.stamp) : -1.;
  const bool target_valid = current_health_ok &&
      health.state == VisualHealth::TARGET_VALID && accepted_available &&
      fresh(accepted_target, now) &&
      accepted_target.frame == config_.expected_frame;
  const char* guidance_state = "SEARCHING";
  if (health_stamp == 0. && !terminal) {
    guidance_state = "WAIT_INPUT";
  } else {
    switch (state_) {
      case VisualState::Searching: guidance_state = "SEARCHING"; break;
      case VisualState::Locking: guidance_state = "LOCKING"; break;
      case VisualState::Approaching: guidance_state = "APPROACHING"; break;
      case VisualState::Settling: guidance_state = "SETTLING"; break;
      case VisualState::Success: guidance_state = "SUCCESS"; break;
      case VisualState::Failed: guidance_state = "FAILED"; break;
    }
  }
  const char* health_label = "UNKNOWN";
  switch (health.state) {
    case VisualHealth::WAITING: health_label = "WAITING"; break;
    case VisualHealth::NO_TARGET: health_label = "NO_TARGET"; break;
    case VisualHealth::TARGET_VALID: health_label = "TARGET_VALID"; break;
    case VisualHealth::TARGET_PENDING: health_label = "TARGET_PENDING"; break;
    case VisualHealth::FAULT: health_label = "FAULT"; break;
  }
  const double bearing_deg = target_valid
      ? std::atan2(accepted_target.x, accepted_target.z) * 180. / std::acos(-1.) : 0.;
  ROS_INFO("visual mode=%s state=%s policy_reason=%s health_state=%d:%s health_reason=%s health_age_ms=%.1f target_valid=%d target_age_ms=%.1f x=%.3f z=%.3f bearing_deg=%.2f cmd_forward_m_s=%.3f cmd_yaw_rad_s=%.3f gait_stopped=%d producer_detection_hz=%.2f depth_hz=%.2f fresh_target_hz=%.2f infer_ms=%.2f measured_target_rx_hz=%.2f fresh_accepted_hz=%.2f",
           mode.c_str(), guidance_state, reason_.empty() ? "none" : reason_.c_str(),
           health.state, health_label,
           health.reason.empty() ? "none" : health.reason.c_str(),
           health_age_ms, target_valid ? 1 : 0, target_age_ms,
           target_valid ? accepted_target.x : 0.,
           target_valid ? accepted_target.z : 0., bearing_deg,
           command[1], command[3], gait_stopped ? 1 : 0,
           health.detection_hz, health.depth_hz, health.target_hz,
           health.inference_ms, target_rx_hz, fresh_hz);
  last_report_time_ = now;
  last_report_state_ = state_;
  reported_ = true;
  if (elapsed >= config_.diagnostics_period_s) {
    diagnostics_window_start_ = now;
    report_target_rx_count_ = target_rx_count;
    report_fresh_accepted_count_ = fresh_accepted_count;
  }
}
}  // namespace grasp_hexapod_bt_control
