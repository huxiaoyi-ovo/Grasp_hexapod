#include "grasp_hexapod_control_cpp/dock_perception_ros.h"

#include "grasp_hexapod_control_cpp/config_io.h"
#include "grasp_hexapod_control_cpp/math_utils.h"

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <set>

namespace grasp_hexapod_control_cpp {

namespace {

constexpr int kTagIds[4] = {0, 1, 2, 3};
constexpr double kMaxPositionError = 0.03;
constexpr double kMaxAngleError = 15.0 * M_PI / 180.0;

Matrix3d orthonormalProjection(const Matrix3d& input) {
  // np.linalg.svd 平均旋转矩阵；det 修正保证右手系。
  Eigen::JacobiSVD<Matrix3d> svd(input, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix3d left = svd.matrixU();
  const Matrix3d& right = svd.matrixV();
  if ((left * right).determinant() < 0.0) {
    left.col(2) = -left.col(2);
  }
  return left * right.transpose();
}

}  // namespace

Matrix4d rigidTransform(Vector3d translation, Matrix3d rotation) {
  Matrix4d result = Matrix4d::Identity();
  result.block<3, 3>(0, 0) = rotation;
  result.block<3, 1>(0, 3) = translation;
  return result;
}

Matrix4d invertTransform(const Matrix4d& transform) {
  const Matrix3d rotation = transform.block<3, 3>(0, 0);
  Matrix4d result = Matrix4d::Identity();
  result.block<3, 3>(0, 0) = rotation.transpose();
  result.block<3, 1>(0, 3) = -rotation.transpose() * transform.block<3, 1>(0, 3);
  return result;
}

std::pair<double, double> poseDifference(const Matrix4d& left, const Matrix4d& right) {
  const double position =
      (left.block<3, 1>(0, 3) - right.block<3, 1>(0, 3)).norm();
  const Matrix3d rotation =
      left.block<3, 3>(0, 0).transpose() * right.block<3, 3>(0, 0);
  return {position, rotationAngle(rotation)};
}

std::vector<std::pair<int, Matrix4d>> consistentPoses(
    const std::vector<std::pair<int, Matrix4d>>& poses, double max_position,
    double max_angle) {
  if (poses.size() < 2) {
    return poses;
  }
  const int count = static_cast<int>(poses.size());
  for (int size = count; size >= 2; --size) {
    // 枚举全部 C(count, size) 组合（count<=4，开销可忽略）。
    std::vector<int> selection(static_cast<std::size_t>(size));
    std::iota(selection.begin(), selection.end(), 0);
    bool have_best = false;
    double best_score = 0.0;
    std::vector<std::pair<int, Matrix4d>> best_group;
    while (true) {
      std::vector<std::pair<int, Matrix4d>> group;
      group.reserve(selection.size());
      for (int index : selection) {
        group.push_back(poses[static_cast<std::size_t>(index)]);
      }
      bool all_agree = true;
      double score = 0.0;
      for (std::size_t a = 0; a < group.size() && all_agree; ++a) {
        for (std::size_t b = a + 1; b < group.size(); ++b) {
          const auto [position, angle] = poseDifference(group[a].second, group[b].second);
          if (position > max_position || angle > max_angle) {
            all_agree = false;
            break;
          }
          score += position / max_position + angle / max_angle;
        }
      }
      if (all_agree && (!have_best || score < best_score)) {
        have_best = true;
        best_score = score;
        best_group = group;
      }
      // 下一个组合（next_permutation 风格的选择索引递增）。
      int position_index = size - 1;
      while (position_index >= 0 &&
             selection[static_cast<std::size_t>(position_index)] ==
                 count - (size - position_index)) {
        --position_index;
      }
      if (position_index < 0) {
        break;
      }
      ++selection[static_cast<std::size_t>(position_index)];
      for (int tail = position_index + 1; tail < size; ++tail) {
        selection[static_cast<std::size_t>(tail)] =
            selection[static_cast<std::size_t>(tail - 1)] + 1;
      }
    }
    if (have_best) {
      return best_group;
    }
  }
  return {};
}

std::tuple<Matrix4d, double, double> fusePoses(
    const std::vector<std::pair<int, Matrix4d>>& poses) {
  Matrix4d result = poses.front().second;
  Vector3d translation = Vector3d::Zero();
  Matrix3d rotation_sum = Matrix3d::Zero();
  for (const auto& [tag_id, pose] : poses) {
    (void)tag_id;
    translation += pose.block<3, 1>(0, 3);
    rotation_sum += pose.block<3, 3>(0, 0);
  }
  result.block<3, 1>(0, 3) = translation / static_cast<double>(poses.size());
  result.block<3, 3>(0, 0) = orthonormalProjection(rotation_sum / static_cast<double>(poses.size()));
  double max_position = 0.0;
  double max_angle = 0.0;
  for (const auto& [tag_id, pose] : poses) {
    (void)tag_id;
    const auto [position, angle] = poseDifference(result, pose);
    max_position = std::max(max_position, position);
    max_angle = std::max(max_angle, angle);
  }
  return {result, max_position, max_angle};
}

bool poseIsPlausible(const Matrix4d& pose, const Matrix4d& lock_from_camera) {
  if (!pose.allFinite()) {
    return false;
  }
  const Matrix3d rotation = pose.block<3, 3>(0, 0);
  if (!((rotation.transpose() * rotation - Matrix3d::Identity()).cwiseAbs().maxCoeff() <=
        1e-3)) {
    return false;
  }
  if (rotation.determinant() < 0.999) {
    return false;
  }
  const Matrix4d camera_from_pin = invertTransform(lock_from_camera) * pose;
  const double z = camera_from_pin(2, 3);
  return 0.01 <= z && z <= 1.5 &&
         camera_from_pin.block<2, 1>(0, 3).norm() <= 0.5;
}

double confidenceScore(int decoded_count, double position_spread, double angle_spread) {
  const double base = 0.6 + 0.12 * (decoded_count - 1);
  const double agreement =
      1.0 - 0.25 * (std::min(position_spread / kMaxPositionError, 1.0) +
                    std::min(angle_spread / kMaxAngleError, 1.0));
  return std::max(0.0, std::min(1.0, base * agreement));
}

Matrix4d DockPerceptionRos::matrixFromTransformStamped(
    const geometry_msgs::TransformStamped& transform_stamped) {
  const geometry_msgs::Quaternion& q = transform_stamped.transform.rotation;
  const geometry_msgs::Vector3& t = transform_stamped.transform.translation;
  Matrix4d pose = Matrix4d::Identity();
  const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
  pose.block<3, 3>(0, 0) = rotation.toRotationMatrix();
  pose(0, 3) = t.x;
  pose(1, 3) = t.y;
  pose(2, 3) = t.z;
  return pose;
}

PerceptionResult DockPerceptionRos::invalid(const std::string& reason) {
  PerceptionResult result;
  result.stamp = have_stamp_ ? stamp_.toSec() : 0.0;
  result.reason = reason;
  result_ = result;
  return result_;
}

void DockPerceptionRos::reset() {
  result_ = PerceptionResult();
}

DockPerceptionRos::DockPerceptionRos(double max_age, const std::string& lock_frame,
                                     const std::string& pin_frame_prefix,
                                     const std::string& dock_system_path,
                                     tf2_ros::Buffer* buffer)
    : max_age_(max_age), lock_frame_(lock_frame) {
  const DockSystem dock_system = loadDockSystem(dock_system_path);
  lock_from_camera_ = dock_system.lock_from_camera;
  for (const int tag_id : kTagIds) {
    pin_frames_.emplace_back(tag_id, pin_frame_prefix + std::to_string(tag_id));
  }
  if (buffer != nullptr) {
    tf_buffer_ = buffer;
  } else {
    owned_buffer_ = std::make_unique<tf2_ros::Buffer>();
    listener_ = std::make_unique<tf2_ros::TransformListener>(*owned_buffer_);
    tf_buffer_ = owned_buffer_.get();
  }
}

PerceptionResult DockPerceptionRos::latest() {
  const ros::Time now = ros::Time::now();
  std::vector<std::pair<int, Matrix4d>> candidates;
  std::map<int, ros::Time> stamps;
  for (const auto& [tag_id, pin_frame] : pin_frames_) {
    geometry_msgs::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
          lock_frame_, pin_frame, ros::Time(0), ros::Duration(0.0));
    } catch (const tf2::TransformException&) {
      continue;
    } catch (const std::exception&) {
      continue;
    }
    const ros::Time stamp = transform_stamped.header.stamp;
    stamp_ = stamp;
    have_stamp_ = true;
    if (std::abs((now - stamp).toSec()) > max_age_) {
      continue;
    }
    const Matrix4d pose = matrixFromTransformStamped(transform_stamped);
    if (pose.allFinite()) {
      candidates.emplace_back(tag_id, pose);
      stamps[tag_id] = stamp;
    }
  }
  if (candidates.empty()) {
    return invalid("no fresh complete dock TF");
  }
  std::sort(candidates.begin(), candidates.end(),
            [&stamps](const std::pair<int, Matrix4d>& a,
                      const std::pair<int, Matrix4d>& b) {
              return stamps[a.first].toSec() > stamps[b.first].toSec();
            });
  std::vector<std::pair<int, Matrix4d>> poses = consistentPoses(candidates);
  const bool poses_agree = !poses.empty();
  // 一致性只作为观测质量诊断。只要存在完整且有限的标签位姿，
  // 就使用时间戳最新的候选，不按 ID 编号设置优先级。
  if (poses.empty()) {
    poses.assign(candidates.begin(), candidates.begin() + 1);
  }
  const auto [pose, position_spread, angle_spread] = fusePoses(poses);
  std::vector<int> decoded;
  for (const auto& [tag_id, fused_pose] : poses) {
    (void)fused_pose;
    decoded.push_back(tag_id);
  }
  std::sort(decoded.begin(), decoded.end());
  ros::Time latest_stamp = stamps[decoded.front()];
  for (int tag_id : decoded) {
    latest_stamp = std::max(latest_stamp, stamps[tag_id]);
  }
  stamp_ = latest_stamp;
  have_stamp_ = true;
  const bool plausible = poseIsPlausible(pose, lock_from_camera_);

  static const std::map<int, const char*> tag_directions = {
      {0, "+y"}, {1, "+x"}, {2, "-y"}, {3, "-x"}};
  std::string tag_text;
  for (int tag_id : decoded) {
    if (!tag_text.empty()) {
      tag_text += ",";
    }
    tag_text += "ID" + std::to_string(tag_id) +
                "(" + tag_directions.at(tag_id) + ")";
  }
  std::string quality_reason = "AprilTag " + tag_text + " pose";
  if (!poses_agree) {
    quality_reason = "tag poses disagree; using " + tag_text;
  } else if (!plausible) {
    quality_reason += "; outside nominal camera workspace";
  }

  PerceptionResult result;
  result.valid = true;
  result.lock_from_pin = pose;
  result.stamp = stamp_.toSec();
  result.decoded_ids = decoded;
  result.confidence = confidenceScore(static_cast<int>(decoded.size()),
                                      position_spread, angle_spread);
  result.position_spread = position_spread;
  result.angle_spread = angle_spread;
  result.reason = quality_reason;
  result_ = result;
  return result_;
}

}  // namespace grasp_hexapod_control_cpp
