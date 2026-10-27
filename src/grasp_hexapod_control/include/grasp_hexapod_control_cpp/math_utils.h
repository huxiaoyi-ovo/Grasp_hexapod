// 控制代码共用的小工具（对应 Python scripts/utils/__init__.py 与
// control.py 中的静态几何函数）。
//
// 输入/输出：Eigen 位姿、点集、关节数组和 NavigationState。
// 边界：只做无状态计算和数据整理，不承担步态、ROS 或硬件控制。
#pragma once

#include "grasp_hexapod_control_cpp/types.h"

namespace grasp_hexapod_control_cpp {

// ---------------------------------------------------------------------------
// 齐次变换构造（对应 kinematics.py 的 translation/rotation_x/rotation_z）。
// ---------------------------------------------------------------------------

Matrix4d translation(double x, double y, double z);
Matrix4d rotationX(double angle);
Matrix4d rotationZ(double angle);

// ---------------------------------------------------------------------------
// 角度与位姿工具（对应 utils/__init__.py）。
// ---------------------------------------------------------------------------

// 把角度压到 [-pi, pi)，单位 rad。Python `%` 对正除数总返回非负余数，
// 与 std::fmod 符号规则不同，必须按 Python 语义实现。
double wrapAngle(double angle);

// 读取 4×4 齐次变换中的偏航角，单位 rad。
inline double yawFromTransform(const Matrix4d& transform) {
  return std::atan2(transform(1, 0), transform(0, 0));
}

// 正交旋转矩阵的最小夹角（run_real._rotation_angle / dock_mode.rotation_angle）。
inline double rotationAngle(const Matrix3d& rotation) {
  const double cosine =
      std::max(-1.0, std::min(1.0, (rotation.trace() - 1.0) / 2.0));
  return std::acos(cosine);
}

// ROS Pose 四元数 (x,y,z,w) + 平移 -> 齐次变换；无效四元数返回 false。
bool poseToTransform(const Eigen::Quaterniond& quaternion,
                     const Vector3d& position, Matrix4d& transform_out);

// 用 4×4 齐次变换批量转换 N 个三维点（transform @ [p;1]）。
std::vector<Vector3d> transformPoints(const Matrix4d& transform,
                                      const std::vector<Vector3d>& points);
// 二维版本：等价于补零到三维后变换再取 xy。
std::vector<Vector2d> transformPoints2(const Matrix4d& transform,
                                       const std::vector<Vector2d>& points);

// 五次平滑插值 10t^3 - 15t^4 + 6t^5（controller._smooth_step）。
inline double smoothStep(double phase) {
  const double p2 = phase * phase;
  const double p3 = p2 * phase;
  return 10.0 * p3 - 15.0 * p3 * phase + 6.0 * p3 * p2;
}

// 两条三维有限线段之间的最短距离（controller._segment_distance）。
double segmentDistance(const Vector3d& a_start, const Vector3d& a_end,
                       const Vector3d& b_start, const Vector3d& b_end);

// ---------------------------------------------------------------------------
// 二维多边形工具（对应 utils/__init__.py）。
// ---------------------------------------------------------------------------

// 有序二维多边形的有符号面积（鞋带公式）。
double polygonArea(const std::vector<Vector2d>& polygon);

// 每个二维点到多边形边界的最短距离。
std::vector<double> distanceToPolygonBoundary(
    const std::vector<Vector2d>& points,
    const std::vector<Vector2d>& polygon);

// 判断二维点是否位于多边形内部；落在边界上（<=1e-9）也视为内部。
std::vector<bool> pointsInPolygon(const std::vector<Vector2d>& points,
                                  const std::vector<Vector2d>& polygon);

// 按最大 spacing 对二维线段均匀采样，包含两个端点。
std::vector<Vector2d> sampleSegment(const Vector2d& start, const Vector2d& end,
                                    double spacing);

// ---------------------------------------------------------------------------
// NavigationState（对应 utils/__init__.py 的 dataclass）。
// ---------------------------------------------------------------------------

struct NavigationState {
  double stamp = 0.0;
  bool valid = false;
  bool landing_confirmed = false;
  Matrix4d pv_from_base = Matrix4d::Identity();
  Matrix4d pv_from_xiaolan = Matrix4d::Identity();
  std::vector<Vector2d> pv_boundary;
};

}  // namespace grasp_hexapod_control_cpp
