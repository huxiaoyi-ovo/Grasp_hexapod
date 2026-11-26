// 抓取六足的公共类型与几何常量（对应 Python kinematics.py 顶部常量）。
//
// 腿顺序契约贯穿全链：lb、lf、lm、rb、rf、rm；每腿 (thigh, knee, ankle)，
// 关节角单位 rad。全部数值计算使用 float64，与 numpy 默认一致。
#pragma once

#include <Eigen/Dense>

#include <array>
#include <stdexcept>
#include <string>
#include <vector>

namespace grasp_hexapod_control_cpp {

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;

// Python 侧用 ValueError 表达配置/输入错误；调用点按同样消息捕获并记录。
using ConfigError = std::invalid_argument;

// 6 条腿 × 每腿 3 关节，行顺序与 GraspController 一致。
using JointAngles = std::array<Eigen::Vector3d, 6>;
using FootPositions = std::array<Eigen::Vector3d, 6>;

inline constexpr int kLegCount = 6;
inline constexpr int kJointCount = 3;

inline const std::array<const char*, 6> kLegNames = {
    "lb", "lf", "lm", "rb", "rf", "rm"};
inline const std::array<const char*, 3> kJointNames = {
    "thigh", "knee", "ankle"};

// LEG_NAMES.index(name)；未知腿名抛错。
inline int legIndexFromName(const std::string& name) {
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (name == kLegNames[leg]) {
      return leg;
    }
  }
  throw ConfigError("unknown leg name: " + name);
}

// 六个 thigh_joint 在 base_link 中的位置和固定 yaw。
inline const std::array<Eigen::Vector3d, 6> kHipXyz = {
    Eigen::Vector3d(-0.04250, -0.07361, 0.02525),
    Eigen::Vector3d(-0.04250, 0.07361, 0.02525),
    Eigen::Vector3d(-0.08500, 0.00000, 0.02525),
    Eigen::Vector3d(0.04250, -0.07361, 0.02525),
    Eigen::Vector3d(0.04250, 0.07361, 0.02525),
    Eigen::Vector3d(0.08500, 0.00000, 0.02525)};
inline const std::array<double, 6> kHipYaw = {
    -2.0944, 2.0944, -3.1416, -1.0472, 1.0472, 0.0};

// 实际几何旋转角 = JOINT_AXIS_SIGNS * Isaac/实机关节角。
inline const std::array<Eigen::Vector3d, 6> kJointAxisSigns = {
    Eigen::Vector3d(-1.0, -1.0, -1.0),
    Eigen::Vector3d(-1.0, -1.0, -1.0),
    Eigen::Vector3d(-1.0, -1.0, -1.0),
    Eigen::Vector3d(-1.0, 1.0, 1.0),
    Eigen::Vector3d(-1.0, 1.0, 1.0),
    Eigen::Vector3d(-1.0, 1.0, 1.0)};

inline const Eigen::Vector3d kFootOffsetAnkle =
    Eigen::Vector3d(0.11660, -0.08015, -0.00025);
inline constexpr double kThighToKneeLength = 0.05236;
inline constexpr double kKneeToAnkleLength = 0.07745;
inline constexpr double kFootRadius = 0.0065;

// 攀爬准备姿态（旧默认站姿，数值逐位保持）：标准工作中心保持 108.971 mm 外展半径，
// 同时让末段严格竖直。所有攀爬入口、P0 和攀爬/对接几何都以它为准。
inline constexpr double kClimbPrepKneeAngleDeg = 43.035535181;
inline constexpr double kClimbPrepAnkleAngleDeg = -98.531183330;

inline JointAngles makeQClimbPrep() {
  const double prep_knee = kClimbPrepKneeAngleDeg * M_PI / 180.0;
  const double prep_ankle = kClimbPrepAnkleAngleDeg * M_PI / 180.0;
  JointAngles q_prep{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    q_prep[leg] = Eigen::Vector3d(
        0.0 / kJointAxisSigns[leg].x(),
        prep_knee / kJointAxisSigns[leg].y(),
        prep_ankle / kJointAxisSigns[leg].z());
  }
  return q_prep;
}

// 行走站姿 Q_STAND（冻结字面值，与 Python Q_STAND 逐位一致）。
// 定义：髋坐标系足端 = (rho*cos(beta), rho*sin(beta), -0.115)，rho = 0.125 m，
// beta = s*(-10 deg)，s = +1 lb、-1 lf、0 lm、-1 rb、+1 rf、0 rm（前后足向外侧转 10 度）。
// 行：lb, lf, lm, rb, rf, rm；列（控制器关节顺序）：thigh, knee, ankle。机身高约 96 mm。
inline JointAngles makeQStand() {
  JointAngles q_stand{};
  q_stand[0] = Eigen::Vector3d(0.17453630454812805, -0.34908021850852294, 1.318643074432592);
  q_stand[1] = Eigen::Vector3d(-0.1745295458507378, -0.34908021850852256, 1.3186430744325917);
  q_stand[2] = Eigen::Vector3d(3.3793486951084017e-06, -0.3490802185085229, 1.3186430744325923);
  q_stand[3] = Eigen::Vector3d(-0.1745295458507378, 0.34908021850852256, -1.3186430744325917);
  q_stand[4] = Eigen::Vector3d(0.17453630454812805, 0.34908021850852294, -1.318643074432592);
  q_stand[5] = Eigen::Vector3d(3.3793486951084017e-06, 0.3490802185085229, -1.3186430744325923);
  return q_stand;
}

// 控制器顺序的 URDF 机械限位；公共控制层使用同一组边界。
inline JointAngles jointLower() {
  JointAngles lower{};
  const std::array<double, 6> thigh_lower = {
      -1.047065850398866, -1.571, -0.698, -1.571, -1.047065850398866, -0.698};
  for (int leg = 0; leg < kLegCount; ++leg) {
    lower[leg] = Eigen::Vector3d(thigh_lower[leg], -2.094, -2.094);
  }
  return lower;
}

inline JointAngles jointUpper() {
  JointAngles upper{};
  const std::array<double, 6> thigh_upper = {
      1.571, 1.047065850398866, 0.698, 1.047065850398866, 1.571, 0.698};
  for (int leg = 0; leg < kLegCount; ++leg) {
    upper[leg] = Eigen::Vector3d(thigh_upper[leg], 2.094, 2.094);
  }
  return upper;
}

// 30 Hz 控制链以 4 rad/s 限制每个控制周期的目标关节步进。
// 这是共享目标步长保护，不是 LX-15D 的实测负载速度。
inline JointAngles jointVelocityLimit() {
  JointAngles limit{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    limit[leg] = Eigen::Vector3d(4.0, 4.0, 4.0);
  }
  return limit;
}

}  // namespace grasp_hexapod_control_cpp
