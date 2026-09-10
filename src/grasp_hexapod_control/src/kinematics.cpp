#include "grasp_hexapod_control_cpp/kinematics.h"

#include "grasp_hexapod_control_cpp/math_utils.h"

namespace grasp_hexapod_control_cpp {

GraspKinematic::GraspKinematic() {
  for (int leg = 0; leg < kLegCount; ++leg) {
    base_from_hip_[leg] =
        translation(kHipXyz[leg].x(), kHipXyz[leg].y(), kHipXyz[leg].z()) *
        rotationZ(kHipYaw[leg]);
    hip_from_base_[leg] = base_from_hip_[leg].inverse();
  }

  thigh_to_knee_origin_ =
      translation(kThighToKneeLength, 0.0, 0.0) * rotationX(1.5708);
  knee_to_ankle_origin_ =
      translation(kKneeToAnkleLength, 0.0, 0.00025);
  ankle_to_foot_ = translation(kFootOffsetAnkle.x(), kFootOffsetAnkle.y(),
                               kFootOffsetAnkle.z());
}

Vector3d GraspKinematic::forwardLeg(
    int leg_index, const Eigen::Vector3d& joint_angles) const {
  const double q_thigh = joint_angles.x();
  const double q_knee = joint_angles.y();
  const double q_ankle = joint_angles.z();
  const Eigen::Vector3d& signs = kJointAxisSigns[leg_index];
  const Matrix4d hip_from_foot =
      rotationZ(signs.x() * q_thigh) * thigh_to_knee_origin_ *
      rotationZ(signs.y() * q_knee) * knee_to_ankle_origin_ *
      rotationZ(signs.z() * q_ankle) * ankle_to_foot_;
  return hip_from_foot.block<3, 1>(0, 3);
}

FootPositions GraspKinematic::forward(const JointAngles& joint_angles) const {
  FootPositions result;
  for (int leg = 0; leg < kLegCount; ++leg) {
    result[leg] = forwardLeg(leg, joint_angles[leg]);
  }
  return result;
}

std::array<std::array<Vector3d, 4>, 6> GraspKinematic::linkPointsBase(
    const JointAngles& joint_angles) const {
  std::array<std::array<Vector3d, 4>, 6> points;
  for (int leg = 0; leg < kLegCount; ++leg) {
    const Eigen::Vector3d theta =
        kJointAxisSigns[leg].cwiseProduct(joint_angles[leg]);
    Matrix4d transform = base_from_hip_[leg];
    points[leg][0] = transform.block<3, 1>(0, 3);

    transform = transform * rotationZ(theta.x()) * thigh_to_knee_origin_;
    points[leg][1] = transform.block<3, 1>(0, 3);

    transform = transform * rotationZ(theta.y()) * knee_to_ankle_origin_;
    points[leg][2] = transform.block<3, 1>(0, 3);

    transform = transform * rotationZ(theta.z()) * ankle_to_foot_;
    points[leg][3] = transform.block<3, 1>(0, 3);
  }
  return points;
}

std::array<std::array<Matrix4d, 4>, 6> GraspKinematic::linkTransformsBase(
    const JointAngles& joint_angles) const {
  std::array<std::array<Matrix4d, 4>, 6> transforms;
  for (int leg = 0; leg < kLegCount; ++leg) {
    const Eigen::Vector3d theta =
        kJointAxisSigns[leg].cwiseProduct(joint_angles[leg]);
    const Matrix4d thigh = base_from_hip_[leg] * rotationZ(theta.x());
    const Matrix4d knee = thigh * thigh_to_knee_origin_ * rotationZ(theta.y());
    const Matrix4d ankle =
        knee * knee_to_ankle_origin_ * rotationZ(theta.z());
    const Matrix4d foot = ankle * ankle_to_foot_;
    transforms[leg][0] = thigh;
    transforms[leg][1] = knee;
    transforms[leg][2] = ankle;
    transforms[leg][3] = foot;
  }
  return transforms;
}

std::array<std::array<Vector3d, 5>, 6> GraspKinematic::collisionPointsBase(
    const JointAngles& joint_angles) const {
  const std::array<std::array<Matrix4d, 4>, 6> transforms =
      linkTransformsBase(joint_angles);
  std::array<std::array<Vector3d, 5>, 6> points;
  for (int leg = 0; leg < kLegCount; ++leg) {
    points[leg][0] = transforms[leg][0].block<3, 1>(0, 3);
    points[leg][1] = transforms[leg][1].block<3, 1>(0, 3);
    points[leg][2] = transforms[leg][2].block<3, 1>(0, 3);
    const Eigen::Vector4d ankle_corner_local(kFootOffsetAnkle.x(), 0.0,
                                             kFootOffsetAnkle.z(), 1.0);
    const Eigen::Vector4d ankle_corner =
        transforms[leg][2] * ankle_corner_local;
    points[leg][3] = ankle_corner.head<3>();
    points[leg][4] = transforms[leg][3].block<3, 1>(0, 3);
  }
  return points;
}

Matrix3d GraspKinematic::jacobianLeg(
    int leg_index, const Eigen::Vector3d& joint_angles) const {
  const Eigen::Vector3d theta =
      kJointAxisSigns[leg_index].cwiseProduct(joint_angles);

  std::array<Vector3d, 3> joint_origins;
  std::array<Vector3d, 3> joint_axes;

  Matrix4d transform = Matrix4d::Identity();
  joint_origins[0] = transform.block<3, 1>(0, 3);
  joint_axes[0] = transform.block<3, 1>(0, 2);

  transform = transform * rotationZ(theta.x()) * thigh_to_knee_origin_;
  joint_origins[1] = transform.block<3, 1>(0, 3);
  joint_axes[1] = transform.block<3, 1>(0, 2);

  transform = transform * rotationZ(theta.y()) * knee_to_ankle_origin_;
  joint_origins[2] = transform.block<3, 1>(0, 3);
  joint_axes[2] = transform.block<3, 1>(0, 2);

  transform = transform * rotationZ(theta.z()) * ankle_to_foot_;
  const Vector3d foot_position = transform.block<3, 1>(0, 3);

  Matrix3d jacobian;
  for (int joint = 0; joint < kJointCount; ++joint) {
    // Jacobian 列 = 轴符号 * (z 轴 × (足端 - 关节原点))。
    const Vector3d column =
        kJointAxisSigns[leg_index][joint] *
        joint_axes[joint].cross(foot_position - joint_origins[joint]);
    jacobian.col(joint) = column;
  }
  return jacobian;
}

std::array<Matrix3d, 6> GraspKinematic::jacobian(
    const JointAngles& joint_angles) const {
  std::array<Matrix3d, 6> jacobians;
  for (int leg = 0; leg < kLegCount; ++leg) {
    jacobians[leg] = jacobianLeg(leg, joint_angles[leg]);
  }
  return jacobians;
}

std::array<Matrix3d, 6> GraspKinematic::dampedInverseJacobian(
    const JointAngles& joint_angles, double damping) const {
  const std::array<Matrix3d, 6> jacobians = jacobian(joint_angles);
  std::array<Matrix3d, 6> result;
  const Matrix3d identity = Matrix3d::Identity();
  for (int leg = 0; leg < kLegCount; ++leg) {
    const Matrix3d& jacobian = jacobians[leg];
    const Matrix3d damped =
        (jacobian * jacobian.transpose() + damping * damping * identity);
    result[leg] = jacobian.transpose() * damped.inverse();
  }
  return result;
}

FootPositions GraspKinematic::hipToBase(
    const FootPositions& foot_positions_hip) const {
  FootPositions result;
  for (int leg = 0; leg < kLegCount; ++leg) {
    const Eigen::Vector4d homogeneous(foot_positions_hip[leg].x(),
                                      foot_positions_hip[leg].y(),
                                      foot_positions_hip[leg].z(), 1.0);
    const Eigen::Vector4d transformed = base_from_hip_[leg] * homogeneous;
    result[leg] = transformed.head<3>();
  }
  return result;
}

FootPositions GraspKinematic::baseToHip(
    const FootPositions& foot_positions_base) const {
  FootPositions result;
  for (int leg = 0; leg < kLegCount; ++leg) {
    const Eigen::Vector4d homogeneous(foot_positions_base[leg].x(),
                                      foot_positions_base[leg].y(),
                                      foot_positions_base[leg].z(), 1.0);
    const Eigen::Vector4d transformed = hip_from_base_[leg] * homogeneous;
    result[leg] = transformed.head<3>();
  }
  return result;
}

FootPositions GraspKinematic::forwardBase(
    const JointAngles& joint_angles) const {
  return hipToBase(forward(joint_angles));
}

}  // namespace grasp_hexapod_control_cpp
