#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <mutex>
#include <ros/ros.h>
#include <string>

namespace grasp_hexapod_bt_control {

struct VisualGuidanceConfig {
  double max_age_s = .25, confirm_frames = 3, lock_angle_rad = .10;
  double align_angle_rad = .15, search_yaw_rate = .15, yaw_gain = .8;
  double max_yaw_rate = .15, max_forward_speed_m_s = .03;
  double distance_gain = .3, stop_distance_m = .60, settle_s = .25;
  double loss_timeout_s = 1., search_timeout_s = 60., approach_timeout_s = 60.;
  double stop_tolerance_m = .02, input_startup_timeout_s = 2., input_loss_timeout_s = .5;
  double diagnostics_period_s = 2.;
  std::string expected_frame = "camera_color_optical_frame";
};

struct VisualObservation {
  bool detected = false;
  double x = 0.0;
  double z = 0.0;
  double stamp = 0.0;
  std::string frame;
};
struct VisualHealth {
  enum State { WAITING = 0, NO_TARGET = 1, TARGET_VALID = 2, TARGET_PENDING = 3, FAULT = 4 };
  int state = WAITING;
  std::string reason, frame;
  double stamp = 0., rgb_stamp = 0., depth_stamp = 0.;
  double detection_hz = 0., depth_hz = 0., target_hz = 0., inference_ms = 0.;
};
VisualGuidanceConfig loadVisualGuidanceConfig(ros::NodeHandle& pnh);
enum class VisualState { Searching, Locking, Approaching, Settling, Success, Failed };

// Thread-safe cache is deliberately separate from deterministic policy so tests need no ROS.
class VisualGuidance {
 public:
  explicit VisualGuidance(VisualGuidanceConfig config);
  void cache(const VisualObservation& observation);
  void cacheHealth(const VisualHealth& health);
  void begin(VisualState state, double now);
  // A resumed mode must not use a pre-pause observation to lock or settle.
  // This preserves the original run deadline rather than extending its budget.
  void resume(double now);
  Eigen::Vector4d update(double now, bool gait_stopped);
  void report(double now, const std::string& mode,
              const Eigen::Vector4d& command, bool gait_stopped);
  VisualState state() const { return state_; }
  std::string reason() const { return reason_; }
  int confirmations() const { return confirmations_; }

 private:
  struct CachedObservation {
    VisualObservation value;
    uint64_t generation = 0;
  };

  bool fresh(const VisualObservation& value, double now) const;
  CachedObservation latest() const;
  bool freshHealth(double now) const;
  void processCache(double now);
  VisualGuidanceConfig config_;
  mutable std::mutex lock_;
  CachedObservation latest_;
  VisualHealth health_;
  uint64_t health_generation_ = 0;
  double last_good_health_stamp_ = 0.;
  VisualObservation accepted_;
  VisualState state_ = VisualState::Searching;
  double started_ = 0.0;
  double run_source_floor_ = 0.0;
  double source_high_watermark_ = 0.0;
  double last_new_stamp_ = 0.0;
  double last_valid_ = 0.0;
  double settle_started_ = 0.0;
  uint64_t processed_generation_ = 0;
  uint64_t accepted_generation_ = 0;
  bool accepted_available_ = false;
  bool recovery_pending_ = false;
  int confirmations_ = 0;
  std::string reason_;
  double diagnostics_window_start_ = 0.;
  double last_report_time_ = 0.;
  VisualState last_report_state_ = VisualState::Searching;
  bool reported_ = false;
  uint64_t target_rx_count_ = 0;
  uint64_t fresh_accepted_count_ = 0;
  uint64_t report_target_rx_count_ = 0;
  uint64_t report_fresh_accepted_count_ = 0;
};

}  // namespace grasp_hexapod_bt_control
