// AprilTag 感知的 tf2 实现（对应 Python dock_mode.py::DockPerception 与
// 同文件中的模块级纯函数：consistent_poses / fuse_poses / pose_is_plausible /
// confidence_score）。
//
// 本头文件依赖 roscpp/tf2，属于 ROS 壳层；核心库通过 PerceptionInterface
// 抽象使用感知结果。
#pragma once

#include "grasp_hexapod_control_cpp/dock_mode.h"

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <utility>
#include <vector>

namespace grasp_hexapod_control_cpp {

// 构造 4x4 刚体变换（对应 dock_mode.rigid_transform）。
Matrix4d rigidTransform(Vector3d translation, Matrix3d rotation);
inline Matrix4d rigidTransform(Vector3d translation) {
  return rigidTransform(translation, Matrix3d::Identity());
}
// 解析求取刚体变换的逆（R^T / -R^T t）。
Matrix4d invertTransform(const Matrix4d& transform);

// 位姿差：(位置差 m, 旋转角 rad)。
std::pair<double, double> poseDifference(const Matrix4d& left, const Matrix4d& right);

// 多标签同时可见时，选择彼此一致的最大候选集合（组合搜索，评分取最小）。
std::vector<std::pair<int, Matrix4d>> consistentPoses(
    const std::vector<std::pair<int, Matrix4d>>& poses,
    double max_position = 0.03, double max_angle = 15.0 * M_PI / 180.0);

// 融合一致位姿（平移取均值 + SVD 正交投影），返回 (位姿, 位置离散, 角度离散)。
std::tuple<Matrix4d, double, double> fusePoses(
    const std::vector<std::pair<int, Matrix4d>>& poses);

// 观测合理性：旋转正交(1e-3)、det>=0.999、相机系 z∈[0.01,1.5]、水平<=0.5。
bool poseIsPlausible(const Matrix4d& pose, const Matrix4d& lock_from_camera);

// 观测质量得分 ∈ [0,1]。
double confidenceScore(int decoded_count, double position_spread,
                       double angle_spread);

class DockPerceptionRos : public PerceptionInterface {
 public:
  // buffer 传空则自建 Buffer + TransformListener。
  DockPerceptionRos(double max_age, const std::string& lock_frame,
                    const std::string& pin_frame_prefix,
                    const std::string& dock_system_path,
                    tf2_ros::Buffer* buffer = nullptr);

  void reset() override;
  PerceptionResult latest() override;

 private:
  static Matrix4d matrixFromTransformStamped(
      const geometry_msgs::TransformStamped& transform_stamped);
  PerceptionResult invalid(const std::string& reason);

  double max_age_;
  Matrix4d lock_from_camera_;
  std::string lock_frame_;
  std::vector<std::pair<int, std::string>> pin_frames_;  // (tag_id, frame)
  std::unique_ptr<tf2_ros::Buffer> owned_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  tf2_ros::Buffer* tf_buffer_;
  bool have_stamp_ = false;
  ros::Time stamp_;
  PerceptionResult result_;
};

}  // namespace grasp_hexapod_control_cpp
