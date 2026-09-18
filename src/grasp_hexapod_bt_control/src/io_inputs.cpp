// io_inputs.h 实现：回调只写缓存，snapshot 只读。
#include "grasp_hexapod_bt_control/io_inputs.h"

#include <cmath>

namespace grasp_hexapod_bt_control {

// ---------------------------------------------------------------------------
// NavigationInput。
// ---------------------------------------------------------------------------

bool NavigationInput::validFrameImpl(const std::string& frame_id) const {
  if (frame_id.empty() ||
      (frame_id[0] == '/' ? frame_id.substr(1) : frame_id) == "pv_map") {
    return true;
  }
  ROS_WARN_THROTTLE(2.0, "Navigation frame must be pv_map");
  return false;
}

void NavigationInput::baseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& message) {
  if (!validFrameImpl(message->header.frame_id)) {
    return;
  }
  Matrix4d transform;
  const geometry_msgs::Point& p = message->pose.position;
  const geometry_msgs::Quaternion& q = message->pose.orientation;
  if (!poseToTransform(Eigen::Quaterniond(q.w, q.x, q.y, q.z),
                       Vector3d(p.x, p.y, p.z), transform)) {
    return;
  }
  std::lock_guard<std::mutex> guard(lock_);
  const double stamp = message->header.stamp.toSec();
  if (stamp < base_stamp_) {
    return;
  }
  pv_from_base_ = transform;
  base_stamp_ = stamp;
  have_base_ = true;
}

void NavigationInput::xiaolanCallback(
    const geometry_msgs::PoseStamped::ConstPtr& message) {
  if (!validFrameImpl(message->header.frame_id)) {
    return;
  }
  Matrix4d transform;
  const geometry_msgs::Point& p = message->pose.position;
  const geometry_msgs::Quaternion& q = message->pose.orientation;
  if (!poseToTransform(Eigen::Quaterniond(q.w, q.x, q.y, q.z),
                       Vector3d(p.x, p.y, p.z), transform)) {
    return;
  }
  std::lock_guard<std::mutex> guard(lock_);
  const double stamp = message->header.stamp.toSec();
  if (stamp < xiaolan_stamp_) {
    return;
  }
  pv_from_xiaolan_ = transform;
  xiaolan_stamp_ = stamp;
  have_xiaolan_ = true;
}

void NavigationInput::boundaryCallback(
    const geometry_msgs::PolygonStamped::ConstPtr& message) {
  if (!validFrameImpl(message->header.frame_id)) {
    return;
  }
  std::vector<Vector2d> boundary;
  boundary.reserve(message->polygon.points.size());
  bool finite = true;
  for (const geometry_msgs::Point32& point : message->polygon.points) {
    boundary.emplace_back(point.x, point.y);
    finite = finite && std::isfinite(point.x) && std::isfinite(point.y);
  }
  if (!finite) {
    return;
  }
  std::lock_guard<std::mutex> guard(lock_);
  const double stamp = message->header.stamp.toSec();
  if (stamp < boundary_stamp_) {
    return;
  }
  pv_boundary_ = std::move(boundary);
  boundary_stamp_ = stamp;
}

void NavigationInput::landingCallback(
    const std_msgs::Bool::ConstPtr& message) {
  std::lock_guard<std::mutex> guard(lock_);
  landing_confirmed_ = message->data;
}

NavigationState NavigationInput::snapshot() const {
  const double now = nowSeconds();
  std::lock_guard<std::mutex> guard(lock_);
  NavigationState state;
  state.stamp = std::min(base_stamp_, xiaolan_stamp_);
  const double pose_skew = std::abs(base_stamp_ - xiaolan_stamp_);
  state.valid = have_base_ && have_xiaolan_ && pv_boundary_.size() >= 3 &&
                state.stamp > 0.0 && 0.0 <= now - state.stamp &&
                now - state.stamp <= max_age_ && pose_skew <= max_pose_skew_ &&
                boundary_stamp_ > 0.0 && 0.0 <= now - boundary_stamp_ &&
                now - boundary_stamp_ <= max_boundary_age_;
  state.landing_confirmed = landing_confirmed_;
  state.pv_from_base = pv_from_base_;
  state.pv_from_xiaolan = pv_from_xiaolan_;
  state.pv_boundary = pv_boundary_;
  return state;
}

NavigationState NavigationInput::motionSnapshot() const {
  const double now = nowSeconds();
  std::lock_guard<std::mutex> guard(lock_);
  NavigationState state;
  state.stamp = std::min(base_stamp_, xiaolan_stamp_);
  const double pose_skew = std::abs(base_stamp_ - xiaolan_stamp_);
  state.valid = have_base_ && have_xiaolan_ && state.stamp > 0.0 &&
                0.0 <= now - state.stamp && now - state.stamp <= max_age_ &&
                pose_skew <= max_pose_skew_;
  state.landing_confirmed = landing_confirmed_;
  state.pv_from_base = pv_from_base_;
  state.pv_from_xiaolan = pv_from_xiaolan_;
  state.pv_boundary = pv_boundary_;
  return state;
}

// ---------------------------------------------------------------------------
// ImuInput。
// ---------------------------------------------------------------------------

void ImuInput::callback(const sensor_msgs::Imu::ConstPtr& message) {
  const double stamp = message->header.stamp.toSec();
  const geometry_msgs::Quaternion& q = message->orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  Vector3d angular_velocity(message->angular_velocity.x,
                            message->angular_velocity.y,
                            message->angular_velocity.z);
  if (norm <= 0.0 || stamp <= 0.0 || !angular_velocity.allFinite()) {
    return;
  }
  const double x = q.x / norm, y = q.y / norm, z = q.z / norm, w = q.w / norm;
  Matrix3d rotation;
  rotation << 1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
      2.0 * (x * z + y * w), 2.0 * (x * y + z * w),
      1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
      2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y);
  if (!rotation.allFinite()) {
    return;
  }
  std::lock_guard<std::mutex> guard(lock_);
  rotation_ = rotation;
  angular_velocity_ = angular_velocity;
  stamp_ = stamp;
  have_rotation_ = true;
}

ImuSnapshot ImuInput::snapshot() const {
  const double now = nowSeconds();
  std::lock_guard<std::mutex> guard(lock_);
  ImuSnapshot snap;
  snap.valid = have_rotation_ && stamp_ > 0.0 && 0.0 <= now - stamp_ &&
               now - stamp_ <= max_age_;
  snap.rotation = rotation_;
  snap.angular_velocity = angular_velocity_;
  return snap;
}

// ---------------------------------------------------------------------------
// BoolInput。
// ---------------------------------------------------------------------------

void BoolInput::callback(const std_msgs::Bool::ConstPtr& message) {
  std::lock_guard<std::mutex> guard(lock_);
  value_ = message->data;
  received_at_ = nowSeconds();
}

std::optional<bool> BoolInput::snapshot() const {
  std::lock_guard<std::mutex> guard(lock_);
  return value_;
}

std::pair<std::optional<bool>, double> BoolInput::snapshotWithTime() const {
  std::lock_guard<std::mutex> guard(lock_);
  return {value_, received_at_};
}

}  // namespace grasp_hexapod_bt_control
