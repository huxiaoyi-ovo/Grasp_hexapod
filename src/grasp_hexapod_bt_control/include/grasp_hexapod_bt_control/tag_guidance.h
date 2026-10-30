#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <mutex>
#include <string>

namespace grasp_hexapod_bt_control {

// ROS-free AprilTag handoff + servo policy used by the visual `approach` mode.
//
// Frames: all tag poses are in the camera optical frame (x right, y down,
// z forward).  Body command order is [right m/s, forward m/s, up m/s,
// yaw rad/s] with base_link +x right, +y forward, +z up, yaw CCW about +z.
// The camera optical axis is assumed to point along base +y and camera x along
// base +x (forward-facing camera, pitch ignored).  The taught reference and
// the 0.20 m coarse stop are camera-frame diagnostics, not body clearance.
//
// Flow after the YOLO->tag handoff:
//   Aligning : servo lateral + yaw to the taught tag_ref_x / tag_ref_yaw; no
//              forward motion until inside the 2x tolerance gate.
//   Advancing: keep correcting lateral/yaw and drive forward on the tag depth
//              toward z_target = p0_z - final_overshoot.  Forward is gated off
//              again whenever the pose leaves the 2x gate.  If the tag stays
//              reliable to the end the run settles on the tag (Success: |ex|,
//              |eyaw| <= tol and ez <= tol; closer than z_target is accepted).
//   Blind    : the camera cannot image the tag close to the climb start pose P0,
//              so when the tag is lost (or too close) the last valid sample is
//              used to compute the remaining distance, which is then driven
//              open loop with leg odometry (stance feet from joint feedback)
//              and an IMU heading hold.
//   Settling : zero command until the gait is stopped for settle_s -> Success.
// p0_z_m is a VIRTUAL depth: taught depth minus the measured back-off distance.
// The servo stops commanding inside this fraction of the tolerance, so the
// pose parks well inside the 1x tolerance used by the handover alignment check
// and the success check (and the 2x advance gate).
constexpr double kServoDeadbandFraction = 0.5;

struct TagGuidanceConfig {
  int tag_id = 0;
  int confirm_frames = 5;
  double max_age_s = .25;
  std::string expected_frame = "camera_color_optical_frame";
  // Taught reference of the tag in the camera frame at P0.  These defaults are
  // PLACEHOLDERS: they MUST be taught on the real rig (record_tag_reference.py).
  // p0_z_m = taught depth - back-off distance (virtual depth at P0).
  double ref_x_m = 0.0, p0_z_m = 0.15, ref_yaw_rad = 0.0;
  double lateral_gain = .5, forward_gain = .5, yaw_gain = .8;
  double max_lateral_speed_m_s = .02, max_forward_speed_m_s = .02, max_yaw_rate = .10;
  double position_tolerance_m = .01, yaw_tolerance_rad = .035;
  double settle_s = .5, loss_timeout_s = 1., align_timeout_s = 60.;
  // Advance/handover/blind finish.
  double final_overshoot_m = .01;     // aim this much closer than P0
  double handover_loss_s = .3;        // no fresh valid tag for this long -> handover
  double min_reliable_z_m = 0.;       // last valid z below this -> handover (0 = off)
  double max_blind_distance_m = .25;  // refuse a blind finish longer than this
  double blind_speed_m_s = .015;
  double heading_gain = .8;           // rad/s per rad of heading error (clamped by max_yaw_rate)
  double max_heading_error_rad = .087;
  double odom_mismatch_m = .03;       // |leg odometry - integrated command|
  double blind_timeout_s = 30.;
  // imu_yaw used by the blind finish = imu_yaw_sign * input.imu_yaw.  The raw IMU
  // yaw axis may point along -z of base_link; check on the robot (README).
  double imu_yaw_sign = 1.;           // +1 or -1 only
};

struct TagObservation {
  int id = -1;
  double x = 0.0, y = 0.0, z = 0.0;           // tag origin in camera frame [m]
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;  // tag frame in camera frame
  double stamp = 0.0;
  std::string frame;
};

// Wrap to (-pi, pi].
double wrapAngle(double angle);
// yaw = atan2(R(2,0), R(0,0)): R is the rotation of the tag frame in the camera
// frame.  The tag x axis points along image +x under both apriltag z
// conventions (z into or out of the tag face), so this does not depend on that
// convention.  yaw = 0 when the tag faces the camera squarely; positive yaw =
// the outward normal tilts toward camera +x (right).  Equivalent to
// atan2(n_x, -n_z) for the outward normal n of the planar case.
double tagYaw(const Eigen::Matrix3d& rotation);
// Poses whose tag x axis leaves the camera x-z plane by more than this
// (|R(1,0)| > limit) are treated as bogus / heavily rolled and rejected.
constexpr double kMaxTagXAxisOutOfPlane = 0.5;
// Same, from a (not necessarily normalized) quaternion; false if non-finite or
// rejected by kMaxTagXAxisOutOfPlane.
bool tagYawFromQuaternion(double qx, double qy, double qz, double qw, double& yaw);

enum class TagState { Coarse, Aligning, Advancing, Blind, Settling, Success, Failed };

// Per-frame inputs gathered by the caller (all ROS-free).  Feet are the six
// foot centres in base_link from the joint FEEDBACK (forward kinematics);
// stance_mask/gait flags come straight from the approach gait.
struct TagGuidanceInput {
  bool gait_started = false;     // gait running (or finishing a step)
  bool transfer_active = false;  // double-support transfer between tripods
  std::array<bool, 6> stance_mask{{true, true, true, true, true, true}};  // true = stance
  bool feet_valid = false;
  std::array<Eigen::Vector3d, 6> feet_base{};
  bool imu_valid = false;
  double imu_yaw = 0.0;          // raw atan2(R(1,0), R(0,0)) of the IMU rotation [rad]
};

// Stance mask the odometry may trust: all six legs are on the ground during the
// transfer and when the gait is stopped (the raw mask is stale then); only
// while a step is in progress is the raw mask used.
std::array<bool, 6> effectiveStance(bool gait_started, bool transfer_active,
                                    const std::array<bool, 6>& raw_mask);

// Last valid tag sample used at handover (valid after the Advancing handover).
struct TagHandover {
  bool occurred = false;
  // remaining = z_last - z_target - travelled; travelled = leg odometry since
  // the last valid sample.
  double z_last = 0.0, ex = 0.0, eyaw = 0.0, travelled = 0.0, remaining = 0.0;
  // Blind heading target minus the IMU heading at handover (the initial heading
  // error): the tag-aligned hold is imu_heading_at_last_valid + eyaw_last.
  double heading_target_offset = 0.0;
};

// Cache is thread-safe (callback thread); everything else runs on the control
// thread and never blocks.
class TagGuidance {
 public:
  explicit TagGuidance(TagGuidanceConfig config);
  void cache(const TagObservation& observation);
  void begin(double now);
  // A resumed run must not use a pre-pause detection to hand off or settle.
  // The tag-stage deadline is kept (not extended); the loss clock restarts.
  void resume(double now);
  // Coarse: counts consecutive new valid detections, returns zero; the caller
  // keeps using its coarse command until handedOff().  Afterwards: body command.
  Eigen::Vector4d update(double now, const TagGuidanceInput& input);
  // Convenience for tag-only callers/tests: gait_stopped -> gait_started=false;
  // no IMU / feet, so a blind finish would fail with "imu unavailable".
  Eigen::Vector4d update(double now, bool gait_stopped);
  // Pure servo law on a measured tag pose (camera frame); no state.  Forward is
  // zero unless allow_forward; then clamp(gain*(z - z_target), 0, max) with the
  // position deadband.
  Eigen::Vector4d servo(double x, double z, double yaw, bool allow_forward) const;
  double zTarget() const { return config_.p0_z_m - config_.final_overshoot_m; }

  TagState state() const { return state_; }
  bool handedOff() const { return state_ != TagState::Coarse; }
  const TagHandover& handover() const { return handover_; }
  // Blind finish monitors (valid from handover on).
  double blindRemaining() const { return handover_.remaining; }
  double odometry() const { return odom_; }
  double commandIntegral() const { return cmd_integral_; }
  double yawHold() const { return yaw_hold_; }
  double headingError() const { return heading_error_; }
  std::string reason() const { return reason_; }
  int confirmations() const { return confirmations_; }
  int tagId() const { return config_.tag_id; }
  // Last servo errors [ex, ez, eyaw], ez = z - z_target (valid while Aligning/Advancing).
  Eigen::Vector3d lastError() const { return error_; }
  // Latest accepted tag measurement in the camera optical frame: x [m], z [m], yaw [rad].
  Eigen::Vector3d lastMeasurement() const { return Eigen::Vector3d(accepted_.x, accepted_.z, accepted_yaw_); }

 private:
  struct Cached {
    TagObservation value;
    uint64_t generation = 0;
  };
  Cached latest() const;
  bool valid(const TagObservation& value, double now, double& yaw) const;
  bool fresh(const TagObservation& value, double now) const;
  void processCache(double now);
  void fail(const std::string& reason);
  void trackOdometry(const TagGuidanceInput& input);
  Eigen::Vector4d doHandover(double now, const TagGuidanceInput& input);
  Eigen::Vector4d blind(double now, const TagGuidanceInput& input);
  Eigen::Vector4d settling(double now, bool gait_stopped);

  TagGuidanceConfig config_;
  mutable std::mutex lock_;
  Cached latest_;
  uint64_t processed_generation_ = 0;
  TagObservation accepted_;
  double accepted_yaw_ = 0.0;
  bool accepted_available_ = false;
  TagState state_ = TagState::Coarse;
  double run_source_floor_ = 0.0;
  double source_high_watermark_ = 0.0;
  double last_new_stamp_ = 0.0;
  double last_valid_ = 0.0;
  double guiding_started_ = 0.0;
  double settle_started_ = 0.0;
  int confirmations_ = 0;
  Eigen::Vector3d error_ = Eigen::Vector3d::Zero();
  std::string reason_;
  // Last valid sample (survives later invalid detections; cleared by resume()).
  bool have_last_valid_ = false;
  double last_x_ = 0.0, last_z_ = 0.0, last_yaw_ = 0.0;
  double odom_at_last_valid_ = 0.0;
  // IMU heading (sign applied) of the latest update with a valid IMU, and the one
  // current when the last valid tag sample was accepted.
  bool last_imu_valid_ = false;
  double last_imu_heading_ = 0.0;
  bool imu_at_last_valid_ok_ = false;
  double imu_heading_at_last_valid_ = 0.0;
  TagHandover handover_;
  // Blind finish.  yaw_hold_ is the tag-aligned heading target (IMU heading frame).
  double blind_started_ = 0.0;
  double yaw_hold_ = 0.0;
  double heading_error_ = 0.0;
  double odom_ = 0.0;
  double cmd_integral_ = 0.0;
  double last_forward_cmd_ = 0.0;
  bool have_last_update_ = false;
  double last_update_time_ = 0.0;
  bool prev_feet_valid_ = false;
  std::array<Eigen::Vector3d, 6> prev_feet_{};
  std::array<bool, 6> prev_stance_{};
};

}  // namespace grasp_hexapod_bt_control
