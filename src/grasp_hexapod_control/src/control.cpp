#include "grasp_hexapod_control_cpp/control.h"

#include <algorithm>
#include <cmath>

namespace grasp_hexapod_control_cpp {

namespace {

// 当前简化 URDF 碰撞盒对应的控制器碰撞模型（control.py 顶部常量）。
constexpr double kLinkCollisionRadii[3] = {
    std::hypot(0.025 / 2.0, 0.025 / 2.0),
    std::hypot(0.022 / 2.0, 0.022 / 2.0),
    std::hypot(0.022 / 2.0, 0.022 / 2.0)};
constexpr double kCollisionMargin = 0.003;
constexpr double kBodyCollisionRadius = 0.097;
constexpr double kBodyCollisionZMin = 0.0;
constexpr double kBodyCollisionZMax = 0.121;
constexpr double kWorkspaceBetaLimit = 30.0 * M_PI / 180.0;
constexpr double kWorkspaceNumericalTolerance = 1e-9;

// 攀爬准备过渡（行走站姿 -> Q_CLIMB_PREP）的冻结参数。
constexpr double kClimbPrepClearance = 0.030;      // 摆动足净空（m）
constexpr double kClimbPrepLiftDuration = 0.60;    // S1/S3 单组迈步时长（s）
constexpr double kClimbPrepHoldDuration = 0.10;    // S2/S4 保持（s）
constexpr double kClimbPrepLowerDuration = 1.20;   // S5 机身下降（s）
constexpr double kClimbPrepJointMargin = 0.05;     // 关节限位余量（rad）
constexpr double kClimbPrepSpeedLimit = 3.0;       // 逐帧关节速度上限（rad/s）

double minFootClearance() {
  static const double value = 2.0 * kFootRadius + kCollisionMargin;
  return value;
}

// 单腿数值逆解（与 ApproachMode::legIk 相同的 Newton 迭代与收敛判据）。
bool legIkNewton(const GraspKinematic& kinematic, int leg, const Vector3d& target_hip,
                 const Vector3d& q_seed, Vector3d* q) {
  Vector3d angles = q_seed;
  for (int iteration = 0; iteration < 50; ++iteration) {
    const Vector3d error = target_hip - kinematic.forwardLeg(leg, angles);
    if (error.norm() < 1e-10) {
      break;
    }
    const Matrix3d jacobian = kinematic.jacobianLeg(leg, angles);
    const Matrix3d normal = jacobian.transpose() * jacobian + 1e-9 * Matrix3d::Identity();
    angles += normal.partialPivLu().solve(jacobian.transpose() * error);
  }
  *q = angles;
  return (target_hip - kinematic.forwardLeg(leg, angles)).norm() <= 1e-6;
}

}  // namespace

// ---------------------------------------------------------------------------
// MissionStateMachine
// ---------------------------------------------------------------------------

MissionStateMachine::MissionStateMachine(GraspController* controller)
    : controller_(controller) {}

ApproachPlan MissionStateMachine::start(
    const NavigationState& navigation_state, const ClimbCompactConfig& climb_config,
    int start_stage_index, std::optional<int> end_stage_index,
    double approach_timeout_s, double approach_position_tolerance_m,
    double approach_yaw_tolerance_rad, bool climb_hardware_execution) {
  if (state == kApproach || state == kFailLanding || state == kPrepareClimb ||
      state == kClimb) {
    throw ConfigError("full mission is already running");
  }
  if (controller_->climb_prep_active || controller_->climb_prep_posture_held) {
    throw ConfigError("return to the walking stance (B) before starting a mission");
  }
  validateClimbCompact(climb_config);
  controller_->setMode(GraspController::kApproach);
  controller_->reset_active = false;
  this->climb_config = climb_config;
  climb_start_stage_index = start_stage_index;
  this->climb_end_stage_index =
      end_stage_index.value_or(static_cast<int>(climb_config.stages.size()) - 1);
  this->climb_hardware_execution = climb_hardware_execution;
  approach_elapsed_s = 0.0;
  approach_timeout_s = approach_timeout_s;
  if (approach_timeout_s <= 0.0) {
    throw ConfigError("approach_timeout_s must be positive");
  }
  this->approach_position_tolerance_m = approach_position_tolerance_m;
  this->approach_yaw_tolerance_rad = approach_yaw_tolerance_rad;
  if (approach_position_tolerance_m <= 0.0 || approach_yaw_tolerance_rad <= 0.0) {
    throw ConfigError("full mission approach tolerances must be positive");
  }
  controller_->approach_mode.position_tolerance_ = approach_position_tolerance_m;
  controller_->approach_mode.yaw_tolerance_ = approach_yaw_tolerance_rad;
  reason.clear();
  prepare_elapsed_s = 0.0;
  prepare_settle_s = 0.0;
  last_prepare_joint_error_rad = std::numeric_limits<double>::infinity();
  last_prepare_world_foot_error_m = std::numeric_limits<double>::infinity();
  last_prepare_position_error_m = std::numeric_limits<double>::infinity();
  last_prepare_yaw_error_rad = std::numeric_limits<double>::infinity();
  prepare_retry_count = 0;
  posture_prepared = false;
  prepare_previous_pose_pv.reset();
  prepare_previous_stamp = 0.0;
  last_prepare_linear_speed_m_s = std::numeric_limits<double>::infinity();
  last_prepare_angular_speed_rad_s = std::numeric_limits<double>::infinity();
  final_position_error_m = std::numeric_limits<double>::infinity();
  final_orientation_error_rad = std::numeric_limits<double>::infinity();
  final_world_foot_error_m = std::numeric_limits<double>::infinity();
  pending_failure_reason.clear();

  const ApproachPlan result =
      controller_->startAutonomousApproach(navigation_state);
  if (result.failed) {
    state = kFailed;
    reason = result.reason;
  } else {
    state = kApproach;
    approach_target_pose_pv = result.target_pose_pv;
  }
  return result;
}

void MissionStateMachine::cancel(const std::string& reason_text) {
  if (state == kApproach) {
    controller_->approach_mode.cancelAutonomousApproach(reason_text);
  }
  state = kIdle;
  reason = reason_text;
  climb_config.reset();
}

std::optional<ApproachPlan> MissionStateMachine::update(
    const JointAngles& q_cur, const NavigationState* navigation_state) {
  const Eigen::Vector4d command = Eigen::Vector4d::Zero();
  if (state == kApproach) {
    approach_elapsed_s += controller_->dt;
    const ApproachPlan& result = controller_->approach_mode.update(
        command, navigation_state);
    if (result.failed) {
      pending_failure_reason = result.reason;
      if (controller_->approach_mode.gait_started ||
          controller_->approach_mode.transfer_active_) {
        state = kFailLanding;
      } else {
        state = kFailed;
        reason = result.reason;
      }
    } else if (approach_elapsed_s >= approach_timeout_s) {
      controller_->approach_mode.cancelAutonomousApproach("approach timeout");
      pending_failure_reason = "approach timeout";
      if (controller_->approach_mode.gait_started ||
          controller_->approach_mode.transfer_active_) {
        state = kFailLanding;
      } else {
        state = kFailed;
        reason = pending_failure_reason;
      }
    } else if (result.ready_for_climb) {
      approach_target_pose_pv = result.target_pose_pv;
      prepare_elapsed_s = 0.0;
      prepare_settle_s = 0.0;
      if (!posture_prepared) {
        // 落脚未完成时继续以零指令更新接近步态，六足落地后再开始准备过渡。
        if (controller_->approach_mode.gait_started ||
            controller_->approach_mode.transfer_active_) {
          return result;
        }
        std::string prep_reason;
        if (!controller_->beginClimbPrep(q_cur, &prep_reason)) {
          state = kFailed;
          reason = prep_reason;
          return result;
        }
        posture_prepared = true;
      }
      if (navigation_state != nullptr) {
        prepare_previous_pose_pv = navigation_state->pv_from_base;
        prepare_previous_stamp = navigation_state->stamp;
      }
      state = kPrepareClimb;
    }
    return result;
  }

  if (state == kFailLanding) {
    const ApproachPlan& result = controller_->approach_mode.update(command, nullptr);
    if (!controller_->approach_mode.gait_started &&
        !controller_->approach_mode.transfer_active_) {
      state = kFailed;
      reason = pending_failure_reason;
    }
    return result;
  }

  if (state == kPrepareClimb) {
    prepare_elapsed_s += controller_->dt;
    double joint_error = 0.0;
    for (int leg = 0; leg < kLegCount; ++leg) {
      joint_error =
          std::max(joint_error,
                   (q_cur[leg] - controller_->q_climb_prep[leg]).cwiseAbs().maxCoeff());
    }
    last_prepare_joint_error_rad = joint_error;
    const CompactSettleGate& gate = climb_config->settle_gate;
    if (!controller_->climb_prep_active) {
      if (navigation_state == nullptr) {
        state = kFailed;
        reason = "navigation state was lost before climb entry";
        return std::nullopt;
      }
      const NavigationState& navigation = *navigation_state;
      if (!navigation.valid) {
        state = kFailed;
        reason = "navigation state became invalid before climb entry";
        return std::nullopt;
      }
      const double observation_dt = navigation.stamp - prepare_previous_stamp;
      if (observation_dt <= 0.0) {
        return std::nullopt;
      }
      if (prepare_previous_pose_pv.has_value()) {
        last_prepare_linear_speed_m_s =
            (navigation.pv_from_base.block<3, 1>(0, 3) -
             prepare_previous_pose_pv->block<3, 1>(0, 3))
                .norm() /
            observation_dt;
        const Matrix3d relative_rotation =
            prepare_previous_pose_pv->block<3, 3>(0, 0).transpose() *
            navigation.pv_from_base.block<3, 3>(0, 0);
        const double rotation_cosine = std::max(
            -1.0, std::min(1.0, (relative_rotation.trace() - 1.0) / 2.0));
        last_prepare_angular_speed_rad_s = std::acos(rotation_cosine) / observation_dt;
      }
      prepare_previous_pose_pv = navigation.pv_from_base;
      prepare_previous_stamp = navigation.stamp;
      const double position_error =
          (navigation.pv_from_base.block<2, 1>(0, 3) -
           approach_target_pose_pv->block<2, 1>(0, 3))
              .norm();
      const double yaw_error = std::abs(
          wrapAngle(yawFromTransform(*approach_target_pose_pv) -
                    yawFromTransform(navigation.pv_from_base)));
      last_prepare_position_error_m = position_error;
      last_prepare_yaw_error_rad = yaw_error;
      const bool pose_ready =
          navigation.valid && position_error <= approach_position_tolerance_m &&
          yaw_error <= approach_yaw_tolerance_rad;
      const FootPositions feet_base = controller_->kinematic.hipToBase(
          controller_->kinematic.forward(q_cur));
      const std::vector<Vector3d> feet_world = transformPoints(
          navigation.pv_from_base, std::vector<Vector3d>(feet_base.begin(), feet_base.end()));
      double world_foot_error = 0.0;
      for (int leg = 0; leg < kLegCount; ++leg) {
        world_foot_error =
            std::max(world_foot_error,
                     (feet_world[leg] - climb_config->p0.anchors_world_m[leg]).norm());
      }
      last_prepare_world_foot_error_m = world_foot_error;
      if (!pose_ready) {
        // 准备姿态下步态已挂起，重启接近不可能再移动机身：直接失败，不重试
        //（原重试上限分支随之不可达，已删除）。
        state = kFailed;
        reason = "pose drifted after climb-prep; press B to return to the walking stance";
        return std::nullopt;
      }
      if (last_prepare_joint_error_rad <= gate.entry_max_joint_error_rad &&
          last_prepare_world_foot_error_m <= gate.max_foot_target_error_m &&
          last_prepare_linear_speed_m_s <= prepare_linear_speed_limit_m_s &&
          last_prepare_angular_speed_rad_s <= prepare_angular_speed_limit_rad_s) {
        // 一个跨很久的新时间戳不能单帧伪造持续稳定。
        prepare_settle_s += std::min(observation_dt, 2.0 * controller_->dt);
      } else {
        prepare_settle_s = 0.0;
      }
      if (prepare_settle_s >= gate.persistence_s) {
        controller_->enterClimb(q_cur, *climb_config, climb_start_stage_index,
                                climb_end_stage_index, climb_hardware_execution);
        state = kClimb;
      }
    }
    if (state == kPrepareClimb &&
        prepare_elapsed_s >= controller_->reset_duration + 8.0) {
      state = kFailed;
      reason = "climb entry posture timeout: joint_error_rad=" +
               std::to_string(last_prepare_joint_error_rad);
    }
    return std::nullopt;
  }

  if (state == kClimb) {
    controller_->climb_mode.update(command, q_cur);
    if (controller_->climb_mode.state == ClimbState::Done) {
      if (climbTerminalReady(q_cur, navigation_state)) {
        // DOCK 先作为终端保持与后续 DockMode 的明确接管点，不能解释为
        // 已经完成物理对接。
        controller_->setMode(GraspController::kDock);
        state = kDock;
      } else {
        state = kFailed;
        reason = "climb terminal observation gate failed: position_error_m=" +
                 std::to_string(final_position_error_m) + " orientation_error_deg=" +
                 std::to_string(final_orientation_error_rad * 180.0 / M_PI) +
                 " world_foot_error_m=" + std::to_string(final_world_foot_error_m);
      }
    } else if (controller_->climb_mode.state == ClimbState::Failed) {
      state = kFailed;
      reason = controller_->climb_mode.failure_reason;
    }
    return std::nullopt;
  }

  return std::nullopt;
}

bool MissionStateMachine::climbTerminalReady(const JointAngles& q_cur,
                                             const NavigationState* navigation_state) {
  if (navigation_state == nullptr) {
    return false;
  }
  const NavigationState& navigation = *navigation_state;
  if (!navigation.valid) {
    return false;
  }
  const CompactStage& final_stage =
      climb_config->stages[*climb_end_stage_index];
  const Matrix4d target_pose = worldFromBase(final_stage.pose_end);
  const Matrix4d& actual_pose = navigation.pv_from_base;
  final_position_error_m =
      (actual_pose.block<3, 1>(0, 3) - target_pose.block<3, 1>(0, 3)).norm();
  const Matrix3d relative_rotation =
      target_pose.block<3, 3>(0, 0).transpose() * actual_pose.block<3, 3>(0, 0);
  const double rotation_cosine =
      std::max(-1.0, std::min(1.0, (relative_rotation.trace() - 1.0) / 2.0));
  final_orientation_error_rad = std::acos(rotation_cosine);
  const FootPositions feet_base =
      controller_->kinematic.hipToBase(controller_->kinematic.forward(q_cur));
  const std::vector<Vector3d> feet_world = transformPoints(
      actual_pose, std::vector<Vector3d>(feet_base.begin(), feet_base.end()));
  double world_foot_error = 0.0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    world_foot_error =
        std::max(world_foot_error,
                 (feet_world[leg] - final_stage.anchor_knots.back()[leg]).norm());
  }
  final_world_foot_error_m = world_foot_error;
  return final_position_error_m <= final_position_tolerance_m &&
         final_orientation_error_rad <= final_orientation_tolerance_rad &&
         final_world_foot_error_m <= final_world_foot_tolerance_m;
}

// ---------------------------------------------------------------------------
// GraspController
// ---------------------------------------------------------------------------

GraspController::GraspController(double dt, bool enable_link_collision_check,
                                 bool climb_timeout_uses_wall_time,
                                 const ControlPaths& paths)
    : dt(dt),
      enable_link_collision_check(enable_link_collision_check),
      climb_timeout_uses_wall_time(climb_timeout_uses_wall_time),
      workspace_boundary(loadWorkspaceBoundary(paths.workspace_bounds_csv)),
      approach_mode(this),
      climb_mode(this),
      mission(this),
      paths_(paths) {
  q_init = makeQStand();
  q_des = q_init;

  foot_init_hip = kinematic.forward(q_init);
  foot_init_base = kinematic.hipToBase(foot_init_hip);

  foot_current_hip = foot_init_hip;

  foot_desired_base = foot_init_base;
  foot_desired_base_prev = foot_desired_base;
  foot_desired_hip = foot_init_hip;

  double z_sum = 0.0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    z_sum += foot_init_base[leg].z();
  }
  base_height_at_stand = kFootRadius - z_sum / kLegCount;

  q_climb_prep = makeQClimbPrep();
  foot_climb_prep_base = kinematic.hipToBase(kinematic.forward(q_climb_prep));
  double prep_z_sum = 0.0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    prep_z_sum += foot_climb_prep_base[leg].z();
  }
  base_height_at_climb_prep = kFootRadius - prep_z_sum / kLegCount;

  reset_start_q = q_init;
  last_link_collision_free.fill(true);

  approach_mode.initFromController();
  climb_terminal_q = climb_mode.terminalJoints(std::nullopt);
  mode = kApproach;

  const std::array<bool, 6> workspace_ok = workspaceFeasible(foot_init_base);
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!workspace_ok[leg]) {
      throw ConfigError("Q_STAND is outside the safe workspace");
    }
  }
  const std::array<bool, 6> collision_ok = linkCollisionFree(q_init);
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!collision_ok[leg]) {
      throw ConfigError("Q_STAND contains a link collision");
    }
  }
  const std::array<bool, 6> prep_workspace_ok = workspaceFeasible(foot_climb_prep_base);
  const std::array<bool, 6> prep_collision_ok = linkCollisionFree(q_climb_prep);
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!prep_workspace_ok[leg]) {
      throw ConfigError("Q_CLIMB_PREP is outside the safe workspace");
    }
    if (!prep_collision_ok[leg]) {
      throw ConfigError("Q_CLIMB_PREP contains a link collision");
    }
  }
}

std::array<bool, 6> GraspController::workspaceFeasible(
    const FootPositions& foot_positions_base) const {
  const FootPositions foot_positions_hip = kinematic.baseToHip(foot_positions_base);
  std::array<bool, 6> feasible{};
  const double boundary_z_first = workspace_boundary.z.front();
  const double boundary_z_last = workspace_boundary.z.back();
  for (int leg = 0; leg < kLegCount; ++leg) {
    const Vector3d& hip = foot_positions_hip[leg];
    const double rho = hip.head<2>().norm();
    const double beta = std::atan2(hip.y(), hip.x());
    const double z = hip.z();
    const double rho_min = interpMonotonic(z, workspace_boundary.z, workspace_boundary.rho_min);
    const double rho_max = interpMonotonic(z, workspace_boundary.z, workspace_boundary.rho_max);
    feasible[leg] =
        (z >= boundary_z_first - kWorkspaceNumericalTolerance) &&
        (z <= boundary_z_last + kWorkspaceNumericalTolerance) &&
        (rho >= rho_min - kWorkspaceNumericalTolerance) &&
        (rho <= rho_max + kWorkspaceNumericalTolerance) &&
        (std::abs(beta) <= kWorkspaceBetaLimit + kWorkspaceNumericalTolerance);
  }
  return feasible;
}

FootPositions GraspController::projectWorkspace(
    const FootPositions& foot_positions_base) const {
  const FootPositions foot_positions_hip = kinematic.baseToHip(foot_positions_base);
  const double boundary_z_first = workspace_boundary.z.front();
  const double boundary_z_last = workspace_boundary.z.back();
  FootPositions projected{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    const Vector3d& hip = foot_positions_hip[leg];
    const double z = std::max(boundary_z_first,
                             std::min(boundary_z_last, hip.z()));
    const double beta =
        std::max(-kWorkspaceBetaLimit,
                 std::min(kWorkspaceBetaLimit, std::atan2(hip.y(), hip.x())));
    double rho = hip.head<2>().norm();
    const double rho_min =
        interpMonotonic(z, workspace_boundary.z, workspace_boundary.rho_min);
    const double rho_max =
        interpMonotonic(z, workspace_boundary.z, workspace_boundary.rho_max);
    rho = std::max(rho_min, std::min(rho_max, rho));
    projected[leg] = Vector3d(rho * std::cos(beta), rho * std::sin(beta), z);
  }
  return kinematic.hipToBase(projected);
}

std::array<bool, 6> GraspController::footCollisionFree(
    const FootPositions& foot_positions_base) const {
  std::array<bool, 6> collision_free{};
  collision_free.fill(true);
  for (int first_leg = 0; first_leg < kLegCount; ++first_leg) {
    for (int second_leg = first_leg + 1; second_leg < kLegCount; ++second_leg) {
      const double distance =
          (foot_positions_base[first_leg] - foot_positions_base[second_leg]).norm();
      if (distance < minFootClearance()) {
        collision_free[first_leg] = false;
        collision_free[second_leg] = false;
      }
    }
  }
  return collision_free;
}

std::array<bool, 6> GraspController::sameLegCollisionFree(
    const std::array<std::array<Vector3d, 5>, 6>& collision_points) const {
  std::array<bool, 6> collision_free{};
  collision_free.fill(true);
  const double self_clearance =
      kLinkCollisionRadii[0] + kLinkCollisionRadii[2] + kCollisionMargin;
  for (int leg = 0; leg < kLegCount; ++leg) {
    const Vector3d& thigh_start = collision_points[leg][0];
    const Vector3d& thigh_end = collision_points[leg][1];
    for (int segment = 0; segment < 2; ++segment) {
      if (segmentDistance(thigh_start, thigh_end, collision_points[leg][2 + segment],
                          collision_points[leg][3 + segment]) < self_clearance) {
        collision_free[leg] = false;
        break;
      }
    }
  }
  return collision_free;
}

std::array<bool, 6> GraspController::linkCollisionFree(
    const JointAngles& joint_angles) const {
  const std::array<std::array<Vector3d, 4>, 6> points =
      kinematic.linkPointsBase(joint_angles);
  const std::array<std::array<Vector3d, 5>, 6> collision_points =
      kinematic.collisionPointsBase(joint_angles);
  std::array<bool, 6> collision_free = sameLegCollisionFree(collision_points);

  // 六条腿两两检查，共 15 组；每组检查 3×3 个胶囊组合。
  for (int first_leg = 0; first_leg < kLegCount; ++first_leg) {
    for (int second_leg = first_leg + 1; second_leg < kLegCount; ++second_leg) {
      bool pair_collision = false;
      for (int first_link = 0; first_link < 3 && !pair_collision; ++first_link) {
        for (int second_link = 0; second_link < 3; ++second_link) {
          const double distance = segmentDistance(
              points[first_leg][first_link], points[first_leg][first_link + 1],
              points[second_leg][second_link], points[second_leg][second_link + 1]);
          const double clearance = kLinkCollisionRadii[first_link] +
                                   kLinkCollisionRadii[second_link] + kCollisionMargin;
          if (distance < clearance) {
            pair_collision = true;
            break;
          }
        }
      }
      if (pair_collision) {
        collision_free[first_leg] = false;
        collision_free[second_leg] = false;
      }
    }
  }

  // thigh 根部从机身内部安装，因此只检查 knee 和 ankle。
  const Vector3d body_start(0.0, 0.0, kBodyCollisionZMin);
  const Vector3d body_end(0.0, 0.0, kBodyCollisionZMax);
  for (int leg = 0; leg < kLegCount; ++leg) {
    for (int link_index : {1, 2}) {
      const double distance = segmentDistance(
          points[leg][link_index], points[leg][link_index + 1], body_start, body_end);
      const double clearance =
          kBodyCollisionRadius + kLinkCollisionRadii[link_index] + kCollisionMargin;
      if (distance < clearance) {
        collision_free[leg] = false;
      }
    }
  }
  return collision_free;
}

JointAngles GraspController::collisionGuard(const JointAngles& q_candidate,
                                            const JointAngles& q_current) {
  if (mode == kApproach) {
    // 普通行走的足端轨迹已经过工作空间投影和足间距。
    // 连续从站姿出发时不再每帧执行整机胶囊扫描。
    last_link_collision_free.fill(true);
    return q_candidate;
  }
  if (!enable_link_collision_check) {
    // 仅绕过耗时的整机连杆胶囊碰撞检查。候选姿态在到达这里前仍经过
    // 工作空间、足端间距和关节限位约束。
    last_link_collision_free.fill(true);
    return q_candidate;
  }

  last_link_collision_free = linkCollisionFree(q_candidate);
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!last_link_collision_free[leg]) {
      return q_current;
    }
  }
  return q_candidate;
}

FootPositions GraspController::syncActualFeet(const JointAngles& q_cur) {
  const FootPositions feet_base =
      kinematic.hipToBase(kinematic.forward(q_cur));
  foot_desired_base = feet_base;
  foot_desired_base_prev = feet_base;
  q_des = q_cur;
  return feet_base;
}

bool GraspController::commitWorkspaceCandidate(const FootPositions& candidate_base) {
  // 可行候选直接使用；只有越界时才做连续投影，
  // 避免每帧重复执行无意义的坐标裁剪。
  FootPositions safe_candidate = candidate_base;
  if (enable_workspace_check) {
    const std::array<bool, 6> feasible = workspaceFeasible(candidate_base);
    bool all_feasible = true;
    for (int leg = 0; leg < kLegCount; ++leg) {
      all_feasible = all_feasible && feasible[leg];
    }
    if (!all_feasible) {
      safe_candidate = projectWorkspace(candidate_base);
    }
  }

  const std::array<bool, 6> foot_free = footCollisionFree(safe_candidate);
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!foot_free[leg]) {
      return false;
    }
  }
  foot_desired_base = safe_candidate;
  return true;
}

void GraspController::resetToStand(const JointAngles& q_cur) {
  // B 是全局恢复动作，必须退出攀爬状态再执行站立轨迹。
  mission.cancel("reset to stand");
  abortClimb();
  exitDock(q_cur);
  setMode(kApproach);
  // 回行走站姿同时取消攀爬准备过渡与保持。
  climb_prep_active = false;
  climb_prep_posture_held = false;
  climb_prep_schedule_.clear();
  climb_prep_frame_ = 0;
  prepareClimbEntry(q_cur);
}

void GraspController::prepareClimbEntry(const JointAngles& q_cur) {
  reset_start_q = q_cur;
  reset_time = 0.0;
  reset_active = true;
  approach_mode.reset();
}

bool GraspController::planClimbPrep(const JointAngles& q_start,
                                    std::vector<JointAngles>* schedule,
                                    std::array<int, 5>* segment_frames,
                                    std::string* reason) const {
  const auto fail = [&](const std::string& message) {
    *reason = message;
    return false;
  };
  const int lift_frames = static_cast<int>(std::lround(kClimbPrepLiftDuration / dt));
  const int hold_frames = static_cast<int>(std::lround(kClimbPrepHoldDuration / dt));
  const int lower_frames = static_cast<int>(std::lround(kClimbPrepLowerDuration / dt));
  *segment_frames = {lift_frames, hold_frames, lift_frames, hold_frames, lower_frames};
  if (lift_frames < 2 || hold_frames < 1 || lower_frames < 2) {
    return fail("climb prep frame count is too small for the control period");
  }

  // 中间目标：足端 xy 取准备姿态，z 保持当前地面高度，IK 以 Q_CLIMB_PREP 为初值。
  const FootPositions start_base = kinematic.hipToBase(kinematic.forward(q_start));
  FootPositions mid_base = foot_climb_prep_base;
  for (int leg = 0; leg < kLegCount; ++leg) {
    mid_base[leg].z() = start_base[leg].z();
  }
  const FootPositions mid_hip = kinematic.baseToHip(mid_base);
  JointAngles q_mid{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!legIkNewton(kinematic, leg, mid_hip[leg], q_climb_prep[leg], &q_mid[leg])) {
      return fail("climb prep mid-target IK failed on leg " + std::to_string(leg));
    }
  }

  // 摆动腿：五次平滑 + 仅膝关节抬腿鼓包；幅值二分到 21 个采样点上峰值净空 30 mm。
  const auto base_pose = [&](int leg, double u) {
    return Vector3d(q_start[leg] + (q_mid[leg] - q_start[leg]) * smoothStep(u));
  };
  std::array<double, 6> lift_amplitude{};  // 带符号 s*A
  for (int leg = 0; leg < kLegCount; ++leg) {
    const Vector3d q_half = base_pose(leg, 0.5);
    const double sign = kinematic.jacobianLeg(leg, q_half)(2, 1) >= 0.0 ? 1.0 : -1.0;
    const double ground_z_hip = mid_hip[leg].z();
    const auto peak_height = [&](double amplitude) {
      double peak = -1e300;
      for (int sample = 0; sample <= 20; ++sample) {
        const double u = sample / 20.0;
        Vector3d q_lift = base_pose(leg, u);
        q_lift[1] += sign * amplitude * 64.0 * std::pow(u, 3) * std::pow(1.0 - u, 3);
        peak = std::max(peak, kinematic.forwardLeg(leg, q_lift).z() - ground_z_hip);
      }
      return peak;
    };
    double low = 0.0;
    double high = 1.2;
    for (int iteration = 0; iteration < 60; ++iteration) {
      const double middle = 0.5 * (low + high);
      if (peak_height(middle) < kClimbPrepClearance) {
        low = middle;
      } else {
        high = middle;
      }
    }
    double amplitude = 0.5 * (low + high);
    if (peak_height(1.2) < kClimbPrepClearance) {
      amplitude = 1.2;
    }
    lift_amplitude[leg] = sign * amplitude;
  }
  const auto swing_pose = [&](int leg, double u) {
    Vector3d q = base_pose(leg, u);
    q[1] += lift_amplitude[leg] * 64.0 * std::pow(u, 3) * std::pow(1.0 - u, 3);
    return q;
  };

  const int total_frames = 2 * lift_frames + 2 * hold_frames + lower_frames;
  schedule->assign(static_cast<std::size_t>(total_frames) + 1, q_start);
  std::size_t frame = 0;
  const auto is_group_a = [](int leg) { return leg == 0 || leg == 1 || leg == 5; };
  // S1：A 组迈步，B 组保持。
  for (int step = 1; step <= lift_frames; ++step) {
    JointAngles q = (*schedule)[frame];
    for (int leg = 0; leg < kLegCount; ++leg) {
      if (is_group_a(leg)) {
        q[leg] = swing_pose(leg, static_cast<double>(step) / lift_frames);
      }
    }
    (*schedule)[++frame] = q;
  }
  // S2：全部保持。
  for (int step = 1; step <= hold_frames; ++step) {
    (*schedule)[frame + 1] = (*schedule)[frame];
    ++frame;
  }
  // S3：B 组迈步，A 组停在 q_m。
  for (int step = 1; step <= lift_frames; ++step) {
    JointAngles q = (*schedule)[frame];
    for (int leg = 0; leg < kLegCount; ++leg) {
      if (!is_group_a(leg)) {
        q[leg] = swing_pose(leg, static_cast<double>(step) / lift_frames);
      }
    }
    (*schedule)[++frame] = q;
  }
  // S4：全部保持。
  for (int step = 1; step <= hold_frames; ++step) {
    (*schedule)[frame + 1] = (*schedule)[frame];
    ++frame;
  }
  // S5：机身下降。足端 xy 取准备姿态，z 由 q_m 对应高度平滑过渡到准备高度，
  // 逐帧 IK 以上一帧为初值；末帧精确取 Q_CLIMB_PREP。
  for (int step = 1; step <= lower_frames; ++step) {
    const double blend = smoothStep(static_cast<double>(step) / lower_frames);
    FootPositions target_base = foot_climb_prep_base;
    for (int leg = 0; leg < kLegCount; ++leg) {
      target_base[leg].z() =
          mid_base[leg].z() + blend * (foot_climb_prep_base[leg].z() - mid_base[leg].z());
    }
    const FootPositions target_hip = kinematic.baseToHip(target_base);
    JointAngles q = (*schedule)[frame];
    if (step == lower_frames) {
      q = q_climb_prep;
    } else {
      for (int leg = 0; leg < kLegCount; ++leg) {
        if (!legIkNewton(kinematic, leg, target_hip[leg], (*schedule)[frame][leg], &q[leg])) {
          return fail("climb prep lowering IK failed on leg " + std::to_string(leg) +
                      " at step " + std::to_string(step));
        }
      }
    }
    (*schedule)[++frame] = q;
  }

  // 接受前整段仿真校验：关节余量、整机连杆碰撞、逐帧关节速度。
  const JointAngles lower = jointLower();
  const JointAngles upper = jointUpper();
  for (std::size_t k = 1; k < schedule->size(); ++k) {
    const JointAngles& q = (*schedule)[k];
    for (int leg = 0; leg < kLegCount; ++leg) {
      for (int joint = 0; joint < kJointCount; ++joint) {
        const double margin =
            std::min(q[leg][joint] - lower[leg][joint], upper[leg][joint] - q[leg][joint]);
        if (!(margin >= kClimbPrepJointMargin)) {
          return fail("climb prep joint margin below 0.05 rad at frame " + std::to_string(k) +
                      " leg " + std::to_string(leg));
        }
        if (!((std::abs(q[leg][joint] - (*schedule)[k - 1][leg][joint]) / dt) <=
              kClimbPrepSpeedLimit)) {
          return fail("climb prep joint speed above 3.0 rad/s at frame " + std::to_string(k) +
                      " leg " + std::to_string(leg));
        }
      }
    }
    const std::array<bool, 6> collision_free = linkCollisionFree(q);
    for (int leg = 0; leg < kLegCount; ++leg) {
      if (!collision_free[leg]) {
        return fail("climb prep link collision at frame " + std::to_string(k) + " leg " +
                    std::to_string(leg));
      }
    }
  }
  return true;
}

bool GraspController::beginClimbPrep(const JointAngles& q_cur, std::string* reason) {
  std::string local_reason;
  std::string* out = reason != nullptr ? reason : &local_reason;
  const auto fail = [&](const std::string& message) {
    *out = message;
    return false;
  };
  if (mode != kApproach) {
    return fail("climb prep requires APPROACH mode");
  }
  if (reset_active) {
    return fail("climb prep cannot start while the stand reset is active");
  }
  if (approach_mode.gait_started || approach_mode.transfer_active_) {
    return fail("climb prep requires the approach gait to be idle");
  }
  if (climb_prep_active) {
    return fail("climb prep is already active");
  }
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!q_cur[leg].allFinite()) {
      return fail("q_cur must be finite");
    }
  }
  std::vector<JointAngles> schedule;
  std::array<int, 5> segment_frames{};
  if (!planClimbPrep(q_cur, &schedule, &segment_frames, out)) {
    return false;
  }
  climb_prep_schedule_ = std::move(schedule);
  climb_prep_segment_frames = segment_frames;
  climb_prep_frame_ = 0;
  climb_prep_active = true;
  climb_prep_posture_held = false;
  return true;
}

bool GraspController::climbEntryNeedsPrep(const JointAngles& q_cur,
                                          const ClimbCompactConfig& config,
                                          int start_stage_index) const {
  if (start_stage_index != 0 || climb_prep_posture_held) {
    return false;
  }
  double entry_error = 0.0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    entry_error = std::max(entry_error, (q_cur[leg] - config.p0.q_rad[leg]).cwiseAbs().maxCoeff());
  }
  return entry_error > config.settle_gate.entry_max_joint_error_rad;
}

ApproachPlan GraspController::startAutonomousApproach(
    const NavigationState& navigation_state,
    const std::optional<std::string>& target_side) {
  if (mode != kApproach) {
    throw ConfigError("Autonomous approach requires APPROACH mode");
  }
  return approach_mode.planAutonomousApproach(navigation_state, target_side);
}

ApproachPlan GraspController::startFullMission(
    const NavigationState& navigation_state, const ClimbCompactConfig& climb_config,
    int start_stage_index, std::optional<int> end_stage_index,
    double approach_timeout_s, double approach_position_tolerance_m,
    double approach_yaw_tolerance_rad, bool climb_hardware_execution) {
  return mission.start(navigation_state, climb_config, start_stage_index,
                       end_stage_index, approach_timeout_s,
                       approach_position_tolerance_m, approach_yaw_tolerance_rad,
                       climb_hardware_execution);
}

void GraspController::setMode(const std::string& next_mode) {
  if (next_mode != kApproach && next_mode != kClimb && next_mode != kDock) {
    throw ConfigError("Unknown control mode: " + next_mode);
  }
  mode = next_mode;
  joint_override = {};
}

void GraspController::enterClimb(const JointAngles& q_cur,
                                 const std::optional<ClimbCompactConfig>& config,
                                 int start_stage_index,
                                 std::optional<int> end_stage_index,
                                 bool hardware_execution) {
  reset_active = false;
  // 攀爬入口接管准备姿态：过渡与保持标志在此消耗。
  climb_prep_active = false;
  climb_prep_posture_held = false;
  exitDock(q_cur);
  setMode(kClimb);
  climb_mode.enter(q_cur, config, start_stage_index, end_stage_index,
                   hardware_execution);
  climb_terminal_q = climb_mode.terminalJoints(climb_mode.currentConfig());
}

void GraspController::attachDockMode(DockMode* dock) {
  if (dock == nullptr) {
    throw ConfigError("dock_mode is required");
  }
  dock_mode = dock;
}

void GraspController::enterDock(const JointAngles& q_cur) {
  if (dock_mode == nullptr) {
    throw std::runtime_error("DockMode is not attached");
  }
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!q_cur[leg].allFinite()) {
      throw ConfigError("q_cur must be finite");
    }
  }
  // 不依赖 ClimbMode 当前状态；B 回站后仍使用缓存的攀爬末姿态。
  const JointAngles terminal_q = climb_terminal_q;
  abortClimb();
  syncActualFeet(q_cur);
  setMode(kDock);
  dock_mode->enter(q_cur, &terminal_q);
}

void GraspController::exitDock(const JointAngles& q_cur) {
  if (dock_mode != nullptr && dock_mode->active) {
    dock_mode->exit();
  }
  syncActualFeet(q_cur);
}

JointAngles GraspController::replayClimbPrefix(const JointAngles& q_cur,
                                               const ClimbCompactConfig& config,
                                               int end_stage_index, int max_ticks) {
  enterClimb(q_cur, config, 0, end_stage_index);
  const Eigen::Vector4d command = Eigen::Vector4d::Zero();
  JointAngles q = q_cur;
  for (int tick = 0; tick < max_ticks; ++tick) {
    if (climb_mode.state != ClimbState::Running) {
      break;
    }
    q = update(q, command);
  }
  if (climb_mode.state != ClimbState::Done) {
    throw std::runtime_error("compact CPU prefix replay failed: " +
                             climb_mode.failure_reason);
  }
  return q;
}

void GraspController::holdClimb() {
  if (mode == kClimb) {
    climb_mode.hold();
  }
}

void GraspController::resumeClimb() {
  if (mode == kClimb) {
    climb_mode.resume();
  }
}

void GraspController::abortClimb() {
  // 停止当前攀爬会话；B 复位后只能重新通过入口开始。
  climb_mode.hold();
  climb_mode.state = ClimbState::Idle;
  climb_mode.clearJointOverride();
  joint_override = {};
  climb_prep_posture_held = false;
}

JointAngles GraspController::update(const JointAngles& q_cur,
                                    const Eigen::Vector4d& command,
                                    const NavigationState* navigation_state,
                                    const DockRobotState* dock_robot_state) {
  joint_override = {};
  if (mission.state != MissionStateMachine::kIdle) {
    last_mode_result = mission.update(q_cur, navigation_state);
  } else if (mode == kApproach) {
    last_mode_result = approach_mode.update(command, navigation_state);
  } else if (mode == kClimb) {
    climb_mode.update(command, q_cur);
    last_mode_result.reset();
  } else if (mode == kDock) {
    if (dock_mode == nullptr) {
      throw std::runtime_error("DockMode is not attached");
    }
    const DockRobotState state =
        dock_robot_state != nullptr ? *dock_robot_state : DockRobotState{};
    const DockResult result = dock_mode->update(state);
    if (result.joint_positions.has_value()) {
      const JointAngles& joint_target = *result.joint_positions;
      for (int leg = 0; leg < kLegCount; ++leg) {
        if (!joint_target[leg].allFinite()) {
          dock_mode->failExecution("dock joint target is non-finite");
          return q_cur;
        }
      }
      // 入口轨迹已经按公共关节速度生成，不再叠加 Dock 专属关节限位。
      q_des = joint_target;
      return q_des;
    }

    if (!result.foot_positions_base.has_value()) {
      q_des = q_cur;
      return q_des;
    }
    const FootPositions& foot_target = *result.foot_positions_base;
    for (int leg = 0; leg < kLegCount; ++leg) {
      if (!foot_target[leg].allFinite()) {
        dock_mode->failExecution("dock foot target is non-finite");
        return q_cur;
      }
    }
    // 与 ApproachMode 和 ClimbMode 相同：模式只写足端目标，统一 DLS 求关节。
    foot_desired_base = foot_target;
    return calJointPoses(q_cur);
  } else {
    throw ConfigError("Unknown control mode: " + mode);
  }

  const JointAngles q_result = calJointPoses(q_cur);
  if (mode == kClimb && climb_mode.state == ClimbState::Done) {
    // 实际完成攀爬后，用公共 DLS 的最新末帧覆盖默认快照。
    climb_terminal_q = q_result;
  }
  return q_result;
}

JointAngles GraspController::calJointPoses(const JointAngles& q_cur) {
  last_update_velocity_limit_clip_count = 0;
  last_update_collision_guard_hold_count = 0;

  // 每个控制周期更新。
  foot_current_hip = kinematic.forward(q_cur);

  if (reset_active) {
    const double next_reset_time = std::min(reset_time + dt, reset_duration);
    double phase = next_reset_time / reset_duration;
    if (phase >= 1.0 - 1e-12) {
      phase = 1.0;
    }
    const double blend = smoothStep(phase);

    // 五次曲线在起点和终点的速度、加速度都为零，
    // 因而 B 回正和 A 使能准备不会产生瞬时关节目标跳变。
    JointAngles q_scheduled{};
    for (int leg = 0; leg < kLegCount; ++leg) {
      q_scheduled[leg] =
          (1.0 - blend) * reset_start_q[leg] + blend * q_init[leg];
    }
    reset_time = next_reset_time;
    q_des = q_scheduled;

    // 回正轨迹结束即进入 HOLD；HOLD 仍持续发送站姿目标。
    // 不用承重关节的伺服稳态误差阻塞整机状态机。
    if (phase >= 1.0) {
      reset_active = false;
      foot_desired_base = foot_init_base;
      foot_desired_hip = foot_init_hip;
      approach_mode.finishReset();
    }
    return q_des;
  }

  // 攀爬准备过渡：逐帧取预先校验过的关节调度，不经 DLS（与回正同样处理）。
  if (climb_prep_active) {
    ++climb_prep_frame_;
    q_des = climb_prep_schedule_[climb_prep_frame_];
    foot_desired_base = kinematic.hipToBase(kinematic.forward(q_des));
    foot_desired_base_prev = foot_desired_base;
    if (climb_prep_frame_ + 1 >= climb_prep_schedule_.size()) {
      climb_prep_active = false;
      climb_prep_posture_held = true;
    }
    return q_des;
  }
  // 过渡完成后保持 Q_CLIMB_PREP，直到 B 复位、中止或攀爬入口。
  if (climb_prep_posture_held) {
    q_des = q_climb_prep;
    foot_desired_base = foot_climb_prep_base;
    foot_desired_base_prev = foot_climb_prep_base;
    return q_des;
  }

  // 步态规划在 base_link 工作，统一转换一下坐标系。
  foot_desired_hip = kinematic.baseToHip(foot_desired_base);

  const std::array<Matrix3d, 6> damped_inverse =
      kinematic.dampedInverseJacobian(q_cur);

  // 前馈: Jacobian 定义在各腿 hip 系，因此先把上一帧目标转到同一坐标系
  // 再差分；禁止将 base_link 速度直接乘 hip Jacobian。
  const FootPositions foot_desired_hip_prev = kinematic.baseToHip(foot_desired_base_prev);
  JointAngles q_candidate{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    const Vector3d position_error =
        foot_desired_hip[leg] - foot_current_hip[leg];
    const Vector3d joint_correction = damped_inverse[leg] * position_error;
    const Vector3d vel_ff = (foot_desired_hip[leg] - foot_desired_hip_prev[leg]) / dt;
    const Vector3d joint_ff = damped_inverse[leg] * vel_ff;
    q_candidate[leg] =
        q_cur[leg] + 16.0 * joint_correction * dt + joint_ff * dt;
    if (joint_override[leg].has_value()) {
      // 关节空间段：该腿跳过 DLS 结果，直接取配置关节目标。
      q_candidate[leg] = *joint_override[leg];
    }
  }
  foot_desired_base_prev = foot_desired_base;

  const JointAngles lower = jointLower();
  const JointAngles upper = jointUpper();
  JointAngles velocity_limit = jointVelocityLimit();
  // 仅仿真的 front 候选可请求受限的更高目标步进上限；侧向、非攀爬与
  // 任何 hardware_execution 路径一律保持共享 4 rad/s（对应 Python cal_joint_poses）。
  if (mode == kClimb && climb_mode.currentConfig().isFront() &&
      !climb_mode.hardware_execution &&
      climb_mode.currentConfig().simulation_joint_velocity_limit_rad_s.has_value()) {
    const double requested_cap =
        *climb_mode.currentConfig().simulation_joint_velocity_limit_rad_s;
    for (int leg = 0; leg < kLegCount; ++leg) {
      velocity_limit[leg] = Eigen::Vector3d::Constant(requested_cap);
    }
  }
  for (int leg = 0; leg < kLegCount; ++leg) {
    for (int joint = 0; joint < kJointCount; ++joint) {
      double value = std::max(lower[leg][joint],
                              std::min(upper[leg][joint], q_candidate[leg][joint]));
      // 舵机能力兜底: 单帧关节增量限幅。
      const double step = velocity_limit[leg][joint] * dt;
      if (std::abs(value - q_cur[leg][joint]) > step) {
        ++last_update_velocity_limit_clip_count;
      }
      value = std::max(q_cur[leg][joint] - step,
                       std::min(q_cur[leg][joint] + step, value));
      q_candidate[leg][joint] = value;
    }
  }

  // 普通行走已在足端层完成工作空间/间距检查；
  // CLIMB/DOCK 可按配置再做完整连杆胶囊检查。
  const JointAngles accepted = collisionGuard(q_candidate, q_cur);
  bool held = false;
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!(accepted[leg].array() == q_candidate[leg].array()).all()) {
      held = true;
      break;
    }
  }
  last_update_collision_guard_hold_count = held ? 1 : 0;
  q_des = accepted;
  return q_des;
}

}  // namespace grasp_hexapod_control_cpp
