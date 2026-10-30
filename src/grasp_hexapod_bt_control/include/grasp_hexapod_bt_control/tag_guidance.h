#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
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
struct TagGuidanceConfig {
  int tag_id = 0;
  int confirm_frames = 5;
  double max_age_s = .25;
  std::string expected_frame = "camera_color_optical_frame";
  // Taught reference pose of the tag in the camera frame.  These defaults are
  // PLACEHOLDERS: they MUST be taught on the real rig (record_tag_reference.py).
  double ref_x_m = 0.0, ref_z_m = 0.15, ref_yaw_rad = 0.0;
  double lateral_gain = .5, forward_gain = .5, yaw_gain = .8;
  double max_lateral_speed_m_s = .02, max_forward_speed_m_s = .02, max_yaw_rate = .10;
  double position_tolerance_m = .01, yaw_tolerance_rad = .035;
  double settle_s = .5, loss_timeout_s = 1., align_timeout_s = 60.;
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

enum class TagState { Coarse, Guiding, Success, Failed };

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
  // keeps using its coarse command until handedOff().  Guiding: servo command.
  Eigen::Vector4d update(double now, bool gait_stopped);
  // Pure servo law on a measured tag pose (camera frame); no state.
  Eigen::Vector4d servo(double x, double z, double yaw) const;

  TagState state() const { return state_; }
  bool handedOff() const { return state_ != TagState::Coarse; }
  std::string reason() const { return reason_; }
  int confirmations() const { return confirmations_; }
  int tagId() const { return config_.tag_id; }
  // Last servo errors [ex, ez, eyaw] (valid while Guiding).
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
};

}  // namespace grasp_hexapod_bt_control
