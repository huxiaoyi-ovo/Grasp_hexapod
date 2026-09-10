// 抓取六足的纯运动学模型（对应 Python kinematics.py 的 GraspKinematic）。
//
// 只包含确定性数学模型：FK、连杆关键点、解析 Jacobian、DLS 阻尼逆以及
// hip/base_link 坐标转换；不包含步态、任务状态机、仿真或硬件通信。
// 未移植的 run_sim/离线工具专用接口（COM、最小奇异值、关节限位余量等）
// 留在 Python 侧，实机链路不使用。
#pragma once

#include "grasp_hexapod_control_cpp/types.h"

namespace grasp_hexapod_control_cpp {

class GraspKinematic {
 public:
  GraspKinematic();

  // 一条腿足端球心在该腿髋坐标系中的位置。
  Vector3d forwardLeg(int leg_index, const Eigen::Vector3d& joint_angles) const;

  // 六个足端在各自髋坐标系中的位置。
  FootPositions forward(const JointAngles& joint_angles) const;

  // 六条腿的髋、膝、踝、足端点（base_link），shape=(6,4,3)。
  std::array<std::array<Vector3d, 4>, 6> linkPointsBase(
      const JointAngles& joint_angles) const;

  // 碰撞专用：髋、膝、踝、ankle L 拐点和足端（base_link），shape=(6,5,3)。
  std::array<std::array<Vector3d, 5>, 6> collisionPointsBase(
      const JointAngles& joint_angles) const;

  // 一条腿足端位置对三个关节角的 3×3 雅可比（hip 系）。
  Matrix3d jacobianLeg(int leg_index,
                       const Eigen::Vector3d& joint_angles) const;

  // 六条腿的雅可比。
  std::array<Matrix3d, 6> jacobian(const JointAngles& joint_angles) const;

  // DLS 阻尼雅可比逆：J^T (J J^T + λ²I)^{-1}。
  std::array<Matrix3d, 6> dampedInverseJacobian(const JointAngles& joint_angles,
                                                double damping = 0.01) const;

  // 六个足端从各自髋坐标系转换到 base_link / 反向。
  FootPositions hipToBase(const FootPositions& foot_positions_hip) const;
  FootPositions baseToHip(const FootPositions& foot_positions_base) const;

  // 六个足端球心在 base_link 中的位置。
  FootPositions forwardBase(const JointAngles& joint_angles) const;

 private:
  // base 到各连杆的变换（thigh/knee/ankle/foot），shape=(6,4)。
  std::array<std::array<Matrix4d, 4>, 6> linkTransformsBase(
      const JointAngles& joint_angles) const;

  std::array<Matrix4d, 6> base_from_hip_;
  std::array<Matrix4d, 6> hip_from_base_;
  Matrix4d thigh_to_knee_origin_;
  Matrix4d knee_to_ankle_origin_;
  Matrix4d ankle_to_foot_;
};

}  // namespace grasp_hexapod_control_cpp
