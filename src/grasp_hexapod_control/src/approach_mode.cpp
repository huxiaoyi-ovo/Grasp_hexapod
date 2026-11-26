#include "grasp_hexapod_control_cpp/approach_mode.h"

#include "grasp_hexapod_control_cpp/control.h"

#include <algorithm>
#include <cmath>

namespace grasp_hexapod_control_cpp {

namespace {

// np.linspace(start, stop, count)（含端点）。
std::vector<double> linspace(double start, double stop, int count) {
  std::vector<double> values(static_cast<std::size_t>(count));
  const double step = (stop - start) / (count - 1);
  for (int i = 0; i < count; ++i) {
    values[static_cast<std::size_t>(i)] = start + i * step;
  }
  return values;
}

}  // namespace

ApproachMode::ApproachMode(GraspController* controller) : controller_(controller) {}

void ApproachMode::initFromController() {
  GraspController* controller = controller_;
  double radius_sum = 0.0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    radius_sum += controller->foot_init_base[leg].head<2>().norm();
  }
  nominal_foot_radius_ = radius_sum / kLegCount;
  robot_safety_radius_ = 0.0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    robot_safety_radius_ =
        std::max(robot_safety_radius_, controller->foot_init_base[leg].head<2>().norm());
  }

  gaits_.fill(false);
  for (int leg : kTripodAIndices) {
    gaits_[leg] = true;
  }

  transfer_frames_ = std::max(1, static_cast<int>(std::lround(0.050 / controller->dt)));
  transfer_duration_ = transfer_frames_ * controller->dt;

  swing_start_base_ = controller->foot_init_base;
  swing_target_base_ = controller->foot_init_base;

  for (int leg = 0; leg < kLegCount; ++leg) {
    swing_q_start_[leg] = controller->q_init[leg];
    swing_q_end_[leg] = controller->q_init[leg];
    swing_qd_start_[leg].setZero();
    swing_qd_end_[leg].setZero();
    swing_lift_amplitude_[leg] = 0.0;
    last_override_q_[leg] = controller->q_init[leg];
    swing_override_emitted_[leg] = false;
  }
  last_override_valid_ = false;
}

void ApproachMode::configureFixedApproach(
    const Matrix4d& xiaolan_from_base, const std::string& target_side,
    double boundary_margin, double linear_speed, double yaw_rate,
    const std::vector<Vector2d>& xiaolan_keepout_polygon_xy_m,
    double xiaolan_body_clearance_m) {
  if (target_side != "left" && target_side != "right") {
    throw ConfigError("fixed approach target_side must be left or right");
  }
  const bool bottom_row_identity =
      std::abs(xiaolan_from_base(3, 0)) <= 1e-9 &&
      std::abs(xiaolan_from_base(3, 1)) <= 1e-9 &&
      std::abs(xiaolan_from_base(3, 2)) <= 1e-9 &&
      std::abs(xiaolan_from_base(3, 3) - 1.0) <= 1e-9;
  if (!xiaolan_from_base.allFinite() || !bottom_row_identity ||
      !((xiaolan_from_base.block<3, 3>(0, 0).transpose() *
         xiaolan_from_base.block<3, 3>(0, 0) - Matrix3d::Identity())
            .cwiseAbs()
            .maxCoeff() <= 1e-6) ||
      std::abs(xiaolan_from_base.block<3, 3>(0, 0).determinant() - 1.0) > 1e-6) {
    throw ConfigError("fixed approach pose must be a finite rigid transform");
  }
  xiaolan_from_fixed_approaches_[target_side] = xiaolan_from_base;
  boundary_margin_ = boundary_margin;
  auto_linear_speed_ = linear_speed;
  auto_yaw_rate_ = yaw_rate;
  xiaolan_keepout_polygon_xy_ = xiaolan_keepout_polygon_xy_m;
  if (xiaolan_keepout_polygon_xy_.size() < 3) {
    // 空多边形合法（未配置 keepout），1~2 个点非法。
    if (!xiaolan_keepout_polygon_xy_.empty()) {
      throw ConfigError("xiaolan keepout polygon must contain at least 3 points");
    }
  }
  for (const Vector2d& point : xiaolan_keepout_polygon_xy_) {
    if (!point.allFinite()) {
      throw ConfigError("xiaolan keepout polygon must be finite");
    }
  }
  if (xiaolan_keepout_polygon_xy_.size() >= 3 &&
      std::abs(polygonArea(xiaolan_keepout_polygon_xy_)) <= 1e-9) {
    throw ConfigError("xiaolan keepout polygon must enclose area");
  }
  xiaolan_body_clearance_ = xiaolan_body_clearance_m;
  if (std::min(boundary_margin_, xiaolan_body_clearance_) <= 0.0) {
    throw ConfigError("approach safety margins must be positive");
  }
}

ApproachPlan ApproachMode::planAutonomousApproach(
    const NavigationState& navigation_state,
    const std::optional<std::string>& target_side) {
  navigation_state_ = navigation_state;
  const NavigationState& navigation = *navigation_state_;

  if (!navigation.valid) {
    return failApproach("navigation data is invalid");
  }
  if (!navigation.landing_confirmed) {
    return failApproach("landing is not confirmed");
  }
  if (navigation.pv_boundary.size() < 3) {
    return failApproach("pv boundary is unavailable");
  }
  if (xiaolan_from_fixed_approaches_.empty()) {
    return failApproach("fixed approach poses are not configured");
  }
  if (target_side.has_value() && *target_side != "left" && *target_side != "right") {
    return failApproach("requested approach side is invalid");
  }
  if (target_side.has_value() &&
      xiaolan_from_fixed_approaches_.find(*target_side) ==
          xiaolan_from_fixed_approaches_.end()) {
    return failApproach("fixed " + *target_side + " approach pose is not configured");
  }

  const Vector2d current_xy = navigation.pv_from_base.block<2, 1>(0, 3);
  const double current_yaw = yawFromTransform(navigation.pv_from_base);
  if (xiaolan_keepout_polygon_xy_.size() < 3) {
    return failApproach("xiaolan keepout polygon is not configured");
  }
  const std::vector<Vector2d> obstacle_polygon_pv =
      transformPoints2(navigation.pv_from_xiaolan, xiaolan_keepout_polygon_xy_);
  std::vector<std::string> candidate_sides;
  if (target_side.has_value()) {
    candidate_sides.push_back(*target_side);
  } else {
    for (const auto& [side, pose] : xiaolan_from_fixed_approaches_) {
      (void)pose;
      candidate_sides.push_back(side);
    }
  }

  struct Candidate {
    double distance;
    double yaw_change;
    std::string side;
    Matrix4d target_pose;
    double clearance;
  };
  std::vector<Candidate> candidates;
  for (const std::string& side : candidate_sides) {
    const Matrix4d target_pose =
        navigation.pv_from_xiaolan * xiaolan_from_fixed_approaches_.at(side);
    const Vector2d target_xy = target_pose.block<2, 1>(0, 3);
    const double target_yaw = yawFromTransform(target_pose);
    const auto [feasible, clearance] = remainingRouteFeasible(
        navigation, current_xy, current_yaw, {target_xy}, target_yaw);
    if (!feasible ||
        !xiaolanDirectPathFeasible(current_xy, target_xy, obstacle_polygon_pv)) {
      continue;
    }
    candidates.push_back(
        {(target_xy - current_xy).norm(),
         std::abs(wrapAngle(target_yaw - current_yaw)), side, target_pose, clearance});
  }
  if (candidates.empty()) {
    const std::string requested = target_side.has_value() ? *target_side : "left/right";
    return failApproach("no direct " + requested +
                        " approach satisfies panel/Xiaolan constraints");
  }
  const Candidate& best = *std::min_element(
      candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        if (a.distance != b.distance) {
          return a.distance < b.distance;
        }
        if (a.yaw_change != b.yaw_change) {
          return a.yaw_change < b.yaw_change;
        }
        return a.side < b.side;
      });

  ApproachPlan plan;
  plan.active = true;
  plan.state = "align";
  plan.target_side = best.side;
  plan.target_pose_pv = best.target_pose;
  plan.waypoints_pv = {best.target_pose.block<2, 1>(0, 3)};
  plan.minimum_clearance = best.clearance;
  approach_plan = plan;
  return approach_plan;
}

bool ApproachMode::xiaolanDirectPathFeasible(
    const Vector2d& start_xy, const Vector2d& target_xy,
    const std::vector<Vector2d>& obstacle) const {
  const std::vector<Vector2d> samples =
      sampleSegment(start_xy, target_xy, path_sample_spacing_);
  const std::vector<bool> outside = pointsInPolygon(samples, obstacle);
  const std::vector<double> clearance =
      distanceToPolygonBoundary(samples, obstacle);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (outside[i] || clearance[i] < xiaolan_body_clearance_) {
      return false;
    }
  }
  return true;
}

double ApproachMode::gaitMotionMargin() const {
  const double support_duration = phase_duration_ + 2.0 * transfer_duration_;
  return 0.5 * support_duration *
         std::max(auto_linear_speed_, robot_safety_radius_ * auto_yaw_rate_);
}

std::pair<bool, double> ApproachMode::poseFeasible(
    const Vector2d& center_xy, double yaw,
    const std::vector<Vector2d>& boundary) const {
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  std::vector<Vector2d> checked_points{center_xy};
  for (int leg = 0; leg < kLegCount; ++leg) {
    const Vector2d local = controller_->foot_init_base[leg].head<2>();
    // feet_xy = foot_init_xy @ rotation.T + center（行向量右乘转置）。
    checked_points.emplace_back(
        center_xy.x() + cosine * local.x() + sine * local.y(),
        center_xy.y() - sine * local.x() + cosine * local.y());
  }
  const std::vector<double> clearance =
      distanceToPolygonBoundary(checked_points, boundary);
  const std::vector<bool> inside = pointsInPolygon(checked_points, boundary);
  const double required_margin = boundary_margin_ + gaitMotionMargin();
  double min_clearance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < checked_points.size(); ++i) {
    min_clearance = std::min(min_clearance, clearance[i]);
    if (!inside[i] || clearance[i] < required_margin) {
      return {false, min_clearance};
    }
  }
  return {true, min_clearance};
}

std::pair<bool, double> ApproachMode::rotationFeasible(
    const Vector2d& center_xy, double start_yaw, double target_yaw,
    const std::vector<Vector2d>& boundary) const {
  const double yaw_change = wrapAngle(target_yaw - start_yaw);
  const int count =
      std::max(2, static_cast<int>(std::ceil(std::abs(yaw_change) / (5.0 * M_PI / 180.0))) + 1);
  double minimum_clearance = std::numeric_limits<double>::infinity();
  for (double yaw : linspace(start_yaw, start_yaw + yaw_change, count)) {
    const auto [feasible, clearance] = poseFeasible(center_xy, yaw, boundary);
    minimum_clearance = std::min(minimum_clearance, clearance);
    if (!feasible) {
      return {false, minimum_clearance};
    }
  }
  return {true, minimum_clearance};
}

std::pair<bool, double> ApproachMode::translationFeasible(
    const Vector2d& start_xy, const Vector2d& end_xy, double yaw,
    const std::vector<Vector2d>& boundary) const {
  const int count = std::max(
      2, static_cast<int>(std::ceil((end_xy - start_xy).norm() / path_sample_spacing_)) + 1);
  double minimum_clearance = std::numeric_limits<double>::infinity();
  for (double t : linspace(0.0, 1.0, count)) {
    const Vector2d center = start_xy + t * (end_xy - start_xy);
    const auto [feasible, clearance] = poseFeasible(center, yaw, boundary);
    minimum_clearance = std::min(minimum_clearance, clearance);
    if (!feasible) {
      return {false, minimum_clearance};
    }
  }
  return {true, minimum_clearance};
}

std::pair<bool, double> ApproachMode::remainingRouteFeasible(
    const NavigationState& navigation, const Vector2d& current_xy, double current_yaw,
    const std::vector<Vector2d>& waypoints_pv, double target_yaw) const {
  const std::vector<Vector2d>& boundary = navigation.pv_boundary;
  const auto [rotation_ok, minimum_clearance] =
      rotationFeasible(current_xy, current_yaw, target_yaw, boundary);
  if (!rotation_ok) {
    return {false, minimum_clearance};
  }
  Vector2d previous_xy = current_xy;
  double clearance = minimum_clearance;
  for (const Vector2d& waypoint_xy : waypoints_pv) {
    const auto [feasible, step_clearance] =
        translationFeasible(previous_xy, waypoint_xy, target_yaw, boundary);
    clearance = std::min(clearance, step_clearance);
    if (!feasible) {
      return {false, clearance};
    }
    previous_xy = waypoint_xy;
  }
  return {true, clearance};
}

void ApproachMode::cancelAutonomousApproach(const std::string& reason) {
  approach_plan.active = false;
  approach_plan.failed = false;
  approach_plan.state = "cancelled";
  approach_plan.reason = reason;
}

ApproachPlan ApproachMode::failApproach(const std::string& reason) {
  ApproachPlan failed_plan;
  failed_plan.state = "failed";
  failed_plan.failed = true;
  failed_plan.reason = reason;
  approach_plan = failed_plan;
  return approach_plan;
}

Eigen::Vector4d ApproachMode::autonomousCommand(
    const NavigationState& navigation_state) {
  navigation_state_ = navigation_state;
  const NavigationState& navigation = *navigation_state_;
  Eigen::Vector4d command = Eigen::Vector4d::Zero();

  if (!navigation.valid) {
    failApproach("navigation data became invalid");
    return command;
  }
  if (!navigation.landing_confirmed) {
    failApproach("landing confirmation was lost");
    return command;
  }
  if (!approach_plan.active) {
    return command;
  }

  const Matrix4d& target_pose = approach_plan.target_pose_pv;
  const Vector2d current_xy = navigation.pv_from_base.block<2, 1>(0, 3);
  const double current_yaw = yawFromTransform(navigation.pv_from_base);
  const double target_yaw = yawFromTransform(target_pose);
  const double yaw_error = wrapAngle(target_yaw - current_yaw);
  std::vector<Vector2d> waypoints(approach_plan.waypoints_pv.begin() +
                                      approach_plan.waypoint_index,
                                  approach_plan.waypoints_pv.end());
  const auto [feasible, minimum_clearance] = remainingRouteFeasible(
      navigation, current_xy, current_yaw, waypoints, target_yaw);
  approach_plan.minimum_clearance = minimum_clearance;
  if (!feasible) {
    failApproach("remaining approach path became unsafe");
    return command;
  }
  if (!xiaolanDirectPathFeasible(current_xy, target_pose.block<2, 1>(0, 3),
                                 transformPoints2(navigation.pv_from_xiaolan,
                                                  xiaolan_keepout_polygon_xy_))) {
    failApproach("remaining direct route reaches Xiaolan safety envelope");
    return command;
  }

  if (approach_plan.state == "align") {
    if (std::abs(yaw_error) > yaw_tolerance_) {
      command[3] = std::max(-auto_yaw_rate_,
                            std::min(auto_yaw_rate_, 1.5 * yaw_error));
      return command;
    }
    approach_plan.state = "translate";
  }

  if (approach_plan.state == "translate") {
    if (approach_plan.waypoint_index >=
        static_cast<int>(approach_plan.waypoints_pv.size())) {
      approach_plan.state = "settle";
      return command;
    }
    // 偏航漂移明显时先停下重新对齐，避免边走边扭影响相机稳定。
    if (std::abs(yaw_error) > 2.0 * yaw_tolerance_) {
      approach_plan.state = "align";
      return command;
    }
    const Vector2d target_xy =
        approach_plan.waypoints_pv[approach_plan.waypoint_index];
    const Vector2d position_error_pv = target_xy - current_xy;
    const double position_error = position_error_pv.norm();
    if (position_error > position_tolerance_) {
      // 比例项 2.5：满速保持到误差 8cm 以内才减速，缩短末段蠕行；
      // 过冲风险换来的提速，位置容差只有 3cm，不宜再放大。
      const double speed = std::min(auto_linear_speed_, 2.5 * position_error);
      const Vector2d velocity_pv = speed * position_error_pv / position_error;
      // pv 速度转到当前 base_link；+x 右、+y 前。
      const double cosine = std::cos(current_yaw);
      const double sine = std::sin(current_yaw);
      command[0] = cosine * velocity_pv.x() + sine * velocity_pv.y();
      command[1] = -sine * velocity_pv.x() + cosine * velocity_pv.y();
      return command;
    }
    approach_plan.state = "settle";
  }

  if (approach_plan.state == "settle") {
    // 零指令会让当前摆动组落地；六足停稳后才允许切换 CLIMB。
    if (!gait_started && !transfer_active_) {
      // 半周期冻结会产生少量停车距离，停稳后重新测量并修正，
      // 不能只凭发出零指令前的位置宣布到达。
      if (std::abs(yaw_error) > yaw_tolerance_) {
        approach_plan.state = "align";
      } else if (approach_plan.waypoint_index <
                     static_cast<int>(approach_plan.waypoints_pv.size()) &&
                 (approach_plan.waypoints_pv[approach_plan.waypoint_index] - current_xy)
                         .norm() > position_tolerance_) {
        approach_plan.state = "translate";
      } else if (approach_plan.waypoint_index <
                 static_cast<int>(approach_plan.waypoints_pv.size())) {
        approach_plan.waypoint_index += 1;
        approach_plan.state = "align";
      } else {
        approach_plan.active = false;
        approach_plan.state = "ready";
        approach_plan.ready_for_climb = true;
      }
    }
  }
  return command;
}

bool ApproachMode::commitCandidate(const FootPositions& candidate_base) {
  const bool navigation_guard_active = approach_plan.active || approach_plan.failed;
  if (navigation_guard_active && navigation_state_.has_value()) {
    if (navigation_state_->pv_boundary.size() < 3) {
      failApproach("pv boundary became unavailable");
      return false;
    }
    const std::vector<Vector3d> candidate(candidate_base.begin(), candidate_base.end());
    const std::vector<Vector3d> feet_pv =
        transformPoints(navigation_state_->pv_from_base, candidate);
    std::vector<Vector2d> feet_pv_xy;
    for (const Vector3d& foot : feet_pv) {
      feet_pv_xy.emplace_back(foot.x(), foot.y());
    }
    const std::vector<bool> inside =
        pointsInPolygon(feet_pv_xy, navigation_state_->pv_boundary);
    const std::vector<double> clearance =
        distanceToPolygonBoundary(feet_pv_xy, navigation_state_->pv_boundary);
    for (std::size_t i = 0; i < feet_pv_xy.size(); ++i) {
      if (!inside[i] || clearance[i] < boundary_margin_) {
        failApproach("planned foot position reaches pv boundary margin");
        return false;
      }
    }
  }
  return controller_->commitWorkspaceCandidate(candidate_base);
}

// ---------------------------------------------------------------------
// 摆动腿关节空间轨迹
// ---------------------------------------------------------------------
Vector3d ApproachMode::basePointToHip(int leg, const Vector3d& point_base) const {
  FootPositions feet{};
  feet.fill(point_base);
  return controller_->kinematic.baseToHip(feet)[leg];
}

Vector3d ApproachMode::hipPointToBase(int leg, const Vector3d& point_hip) const {
  FootPositions feet{};
  feet.fill(point_hip);
  return controller_->kinematic.hipToBase(feet)[leg];
}

Vector3d ApproachMode::baseVectorToHip(int leg, const Vector3d& point_base,
                                       const Vector3d& vector_base) const {
  return basePointToHip(leg, point_base + vector_base) - basePointToHip(leg, point_base);
}

bool ApproachMode::legIk(int leg, const Vector3d& target_hip, const Vector3d& q_seed,
                         Vector3d* q) const {
  const GraspKinematic& kinematic = controller_->kinematic;
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

void ApproachMode::hermiteQuintic(const Vector3d& q0, const Vector3d& q1,
                                  const Vector3d& qd0, const Vector3d& qd1,
                                  double duration, double u, Vector3d* q, Vector3d* dq) {
  const Vector3d scaled_velocity = qd0 * duration;
  const Vector3d residual = q1 - q0 - scaled_velocity;
  const Vector3d velocity_change = (qd1 - qd0) * duration;
  const Vector3d c3 = 10.0 * residual - 4.0 * velocity_change;
  const Vector3d c4 = -15.0 * residual + 7.0 * velocity_change;
  const Vector3d c5 = 6.0 * residual - 3.0 * velocity_change;
  *q = q0 + scaled_velocity * u + c3 * std::pow(u, 3) + c4 * std::pow(u, 4) +
       c5 * std::pow(u, 5);
  *dq = (scaled_velocity + 3.0 * c3 * u * u + 4.0 * c4 * std::pow(u, 3) +
         5.0 * c5 * std::pow(u, 4)) /
        duration;
}

void ApproachMode::swingJointState(int leg, double u, Vector3d* q, Vector3d* dq) const {
  hermiteQuintic(swing_q_start_[leg], swing_q_end_[leg], swing_qd_start_[leg],
                 swing_qd_end_[leg], phase_duration_, u, q, dq);
  // b(u)=64u^3(1-u)^3：b(0)=b(1)=0，两端速度、加速度为零，b(0.5)=1。
  const double bump = 64.0 * std::pow(u, 3) * std::pow(1.0 - u, 3);
  const double bump_rate =
      192.0 * u * u * (1.0 - u) * (1.0 - u) * (1.0 - 2.0 * u) / phase_duration_;
  (*q)[1] += swing_lift_amplitude_[leg] * bump;
  (*dq)[1] += swing_lift_amplitude_[leg] * bump_rate;
}

bool ApproachMode::planSwing(const std::vector<int>& swing_indices) {
  GraspController* controller = controller_;
  const GraspKinematic& kinematic = controller->kinematic;
  const double alpha = swing_boundary_velocity_ratio_;
  const JointAngles lower = jointLower();
  const JointAngles upper = jointUpper();

  for (int leg : swing_indices) {
    const Vector3d start_base = swing_start_base_[leg];
    const Vector3d target_base = swing_target_base_[leg];
    const Vector3d start_hip = basePointToHip(leg, start_base);
    const Vector3d target_hip = basePointToHip(leg, target_base);
    Vector3d q_start;
    Vector3d q_end;
    const bool start_ok = legIk(leg, start_hip, controller->q_des[leg], &q_start);
    const bool end_ok = legIk(leg, target_hip, controller->q_init[leg], &q_end);
    if (!(start_ok && end_ok)) {
      return false;
    }

    const Vector3d start_velocity(swing_start_velocity_xy_[leg].x(),
                                  swing_start_velocity_xy_[leg].y(), 0.0);
    const Vector3d target_velocity(swing_target_velocity_xy_[leg].x(),
                                   swing_target_velocity_xy_[leg].y(), 0.0);
    const Vector3d qd_start =
        alpha * kinematic.jacobianLeg(leg, q_start)
                    .partialPivLu()
                    .solve(baseVectorToHip(leg, start_base, start_velocity));
    const Vector3d qd_end =
        alpha * kinematic.jacobianLeg(leg, q_end)
                    .partialPivLu()
                    .solve(baseVectorToHip(leg, target_base, target_velocity));
    swing_q_start_[leg] = q_start;
    swing_q_end_[leg] = q_end;
    swing_qd_start_[leg] = qd_start;
    swing_qd_end_[leg] = qd_end;

    // 膝关节抬腿：幅值二分到 21 个均匀采样点上足端最高点
    // 高出落点地面 step_height。
    std::array<Vector3d, 21> q_base_samples;
    std::array<double, 21> bump_samples;
    for (int sample = 0; sample <= 20; ++sample) {
      const double u = sample / 20.0;
      Vector3d dq_unused;
      hermiteQuintic(q_start, q_end, qd_start, qd_end, phase_duration_, u,
                     &q_base_samples[sample], &dq_unused);
      bump_samples[sample] = 64.0 * std::pow(u, 3) * std::pow(1.0 - u, 3);
    }
    Vector3d q_mid;
    Vector3d dq_unused;
    hermiteQuintic(q_start, q_end, qd_start, qd_end, phase_duration_, 0.5, &q_mid,
                   &dq_unused);
    const double sign = kinematic.jacobianLeg(leg, q_mid)(2, 1) >= 0.0 ? 1.0 : -1.0;
    const double ground_z_hip = target_hip.z();
    const auto peak_height = [&](double lift_amplitude) {
      double peak = -1e300;
      for (int sample = 0; sample <= 20; ++sample) {
        Vector3d q_lift = q_base_samples[sample];
        q_lift[1] += sign * lift_amplitude * bump_samples[sample];
        peak = std::max(peak, kinematic.forwardLeg(leg, q_lift).z() - ground_z_hip);
      }
      return peak;
    };
    double low = 0.0;
    double high = 1.2;
    for (int iteration = 0; iteration < 60; ++iteration) {
      const double middle = 0.5 * (low + high);
      if (peak_height(middle) < step_height_) {
        low = middle;
      } else {
        high = middle;
      }
    }
    double amplitude = 0.5 * (low + high);
    if (peak_height(1.2) < step_height_) {
      amplitude = 1.2;
    }

    // 速度限幅：幅值取 min(高度所需幅值, 该腿 41 个均匀采样点上
    // 三个关节解析峰值速度不超过限值的最大幅值)。
    std::array<Vector3d, 41> base_rate;
    std::array<double, 41> bump_rate;
    for (int sample = 0; sample <= 40; ++sample) {
      const double u = sample / 40.0;
      Vector3d q_unused;
      hermiteQuintic(q_start, q_end, qd_start, qd_end, phase_duration_, u, &q_unused,
                     &base_rate[sample]);
      bump_rate[sample] = 192.0 * u * u * (1.0 - u) * (1.0 - u) * (1.0 - 2.0 * u) /
                          phase_duration_;
    }
    const auto peak_speed = [&](double lift_amplitude) {
      double peak = 0.0;
      for (int sample = 0; sample <= 40; ++sample) {
        Vector3d rate = base_rate[sample];
        rate[1] += sign * lift_amplitude * bump_rate[sample];
        peak = std::max(peak, rate.cwiseAbs().maxCoeff());
      }
      return peak;
    };
    if (peak_speed(0.0) > swing_joint_speed_limit_) {
      // 基础路径本身已超速，不能靠减小抬腿解决：保持高度所需幅值。
      speed_governor_infeasible += 1;
    } else if (peak_speed(1.2) > swing_joint_speed_limit_) {
      double feasible = 0.0;
      double infeasible = 1.2;
      for (int iteration = 0; iteration < 60; ++iteration) {
        const double middle = 0.5 * (feasible + infeasible);
        if (peak_speed(middle) <= swing_joint_speed_limit_) {
          feasible = middle;
        } else {
          infeasible = middle;
        }
      }
      amplitude = std::min(amplitude, feasible);
    }
    swing_lift_amplitude_[leg] = sign * amplitude;

    for (int sample = 0; sample <= 20; ++sample) {
      Vector3d q;
      Vector3d dq;
      swingJointState(leg, sample / 20.0, &q, &dq);
      for (int joint = 0; joint < kJointCount; ++joint) {
        if (q[joint] < lower[leg][joint] || q[joint] > upper[leg][joint]) {
          return false;
        }
      }
    }
  }
  return true;
}

void ApproachMode::clearOverrideState() {
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (swing_override_emitted_[leg]) {
      controller_->joint_override[leg].reset();
    }
    swing_override_emitted_[leg] = false;
    last_override_q_[leg].setZero();
  }
  last_override_valid_ = false;
}

void ApproachMode::emitOverrides(bool committed, const std::array<bool, 6>& is_swing,
                                 const std::array<Vector3d, 6>& swing_targets) {
  GraspController* controller = controller_;
  if (committed) {
    const FootPositions feet_hip = controller->kinematic.baseToHip(controller->foot_desired_base);
    JointAngles targets{};
    bool all_converged = true;
    for (int leg = 0; leg < kLegCount; ++leg) {
      if (is_swing[leg]) {
        targets[leg] = swing_targets[leg];
        continue;
      }
      const Vector3d seed =
          last_override_valid_ ? last_override_q_[leg] : controller->q_des[leg];
      if (!legIk(leg, feet_hip[leg], seed, &targets[leg])) {
        all_converged = false;
        break;
      }
    }
    if (all_converged) {
      last_override_q_ = targets;
      last_override_valid_ = true;
    }
  }
  for (int leg = 0; leg < kLegCount; ++leg) {
    controller->joint_override[leg] =
        last_override_valid_ ? last_override_q_[leg] : controller->q_des[leg];
    swing_override_emitted_[leg] = true;
  }
}

void ApproachMode::abortStep() {
  // 摆动规划失败：不起步，全部保持支撑，按停止路径清零指令。
  gait_started = false;
  phase_time_ = 0.0;
  requested_command_.setZero();
  foot_velocity_xy_.fill(Eigen::Vector2d::Zero());
}

std::array<Eigen::Vector2d, 6> ApproachMode::stanceVelocity(
    const std::array<Eigen::Vector2d, 6>& foot_xy, const Eigen::Vector4d& command) {
  // 地面固定足端在 base_link 中的刚体相对速度。
  const double velocity_right = command[0];
  const double velocity_forward = command[1];
  const double yaw_rate = command[3];
  std::array<Eigen::Vector2d, 6> velocity{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    velocity[leg].x() = -velocity_right + yaw_rate * foot_xy[leg].y();
    velocity[leg].y() = -velocity_forward - yaw_rate * foot_xy[leg].x();
  }
  return velocity;
}

Eigen::Vector4d ApproachMode::scaledCommand(const Eigen::Vector4d& command) const {
  Eigen::Vector4d scaled = command;
  const double foot_speed_budget =
      scaled.head<2>().norm() + nominal_foot_radius_ * std::abs(scaled[3]);
  if (foot_speed_budget > max_foot_planar_speed_) {
    const double scale = max_foot_planar_speed_ / foot_speed_budget;
    scaled.head<2>() *= scale;
    scaled[3] *= scale;
  }
  return scaled;
}

Eigen::Vector4d ApproachMode::slewCommand(const Eigen::Vector4d& requested,
                                          const Eigen::Vector4d& target) const {
  Eigen::Vector4d slewed = requested;
  Eigen::Vector2d linear_delta = target.head<2>() - slewed.head<2>();
  const double linear_norm = linear_delta.norm();
  const double linear_step = max_linear_acceleration_ * controller_->dt;
  if (linear_norm > linear_step) {
    linear_delta *= linear_step / linear_norm;
  }
  slewed.head<2>() += linear_delta;

  const double yaw_delta = std::max(
      -max_yaw_acceleration_ * controller_->dt,
      std::min(max_yaw_acceleration_ * controller_->dt, target[3] - slewed[3]));
  slewed[3] += yaw_delta;
  slewed[2] = target[2];
  return slewed;
}

void ApproachMode::slewRequestedCommand(const Eigen::Vector4d& command) {
  slew_target_command_ = scaledCommand(command);
  requested_command_ = slewCommand(requested_command_, slew_target_command_);
}

std::vector<Eigen::Vector4d> ApproachMode::predictCommandProfile(int count) const {
  std::vector<Eigen::Vector4d> profile;
  profile.reserve(static_cast<std::size_t>(count));
  Eigen::Vector4d current = requested_command_;
  for (int index = 0; index < count; ++index) {
    profile.push_back(current);
    current = slewCommand(current, slew_target_command_);
  }
  return profile;
}

void ApproachMode::updateSwingTarget(const std::vector<int>& swing_indices,
                                     int frame_offset) {
  GraspController* controller = controller_;
  FootPositions target = controller->foot_init_base;
  // 落地腿的支撑期 = 随后的共同支撑 + 下一摆动相 + 再下一次共同支撑；
  // 落点取支撑期位移的一半反向偏置，使支撑行程以 home 为中心。
  const int phase_frames =
      static_cast<int>(std::ceil(phase_duration_ / controller->dt - 1e-9));
  const int first_stance = frame_offset + phase_frames;
  const int stance_count = transfer_frames_ + phase_frames + transfer_frames_;
  const std::vector<Eigen::Vector4d> profile =
      predictCommandProfile(first_stance + stance_count);
  std::array<Eigen::Vector2d, 6> home_xy{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    home_xy[leg] = controller->foot_init_base[leg].head<2>();
  }
  std::array<Eigen::Vector2d, 6> stance_displacement{};
  stance_displacement.fill(Eigen::Vector2d::Zero());
  for (int index = first_stance; index < first_stance + stance_count; ++index) {
    const std::array<Eigen::Vector2d, 6> stance_at_home =
        stanceVelocity(home_xy, profile[static_cast<std::size_t>(index)]);
    for (int leg = 0; leg < kLegCount; ++leg) {
      stance_displacement[leg] += stance_at_home[leg] * controller->dt;
    }
  }
  for (int leg : swing_indices) {
    target[leg].head<2>() -= 0.5 * stance_displacement[leg];
    target[leg].z() = controller->foot_init_base[leg].z() - body_height_offset_;
  }

  const std::array<bool, 6> target_feasible = controller->workspaceFeasible(target);
  bool all_swing_feasible = true;
  for (int leg : swing_indices) {
    all_swing_feasible = all_swing_feasible && target_feasible[leg];
  }
  if (!all_swing_feasible) {
    target = controller->projectWorkspace(target);
  }
  for (int leg : swing_indices) {
    swing_target_base_[leg] = target[leg];
  }
  // 落点边界速度 = 第一帧支撑指令在（投影后的）新落点位置上的刚体速度。
  std::array<Eigen::Vector2d, 6> target_xy{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    target_xy[leg] = swing_target_base_[leg].head<2>();
  }
  const std::array<Eigen::Vector2d, 6> target_velocity =
      stanceVelocity(target_xy, profile[static_cast<std::size_t>(first_stance)]);
  for (int leg : swing_indices) {
    swing_target_velocity_xy_[leg] = target_velocity[leg];
  }
}

void ApproachMode::reset() {
  stance_group_index_ = 0;
  gaits_.fill(false);
  for (int leg : kTripodAIndices) {
    gaits_[leg] = true;
  }

  phase_time_ = 0.0;
  transfer_time_ = 0.0;
  transfer_active_ = false;
  requested_command_.setZero();
  slew_target_command_.setZero();
  stop_requested_ = false;
  gait_started = false;
  first_step_ = true;
  foot_velocity_xy_.fill(Eigen::Vector2d::Zero());
  swing_start_velocity_xy_.fill(Eigen::Vector2d::Zero());
  swing_target_velocity_xy_.fill(Eigen::Vector2d::Zero());
  clearOverrideState();
  approach_plan = ApproachPlan();
  navigation_state_.reset();
}

void ApproachMode::finishReset() {
  clearOverrideState();
  body_height_offset_ = 0.0;
  swing_start_base_ = controller_->foot_init_base;
  swing_target_base_ = controller_->foot_init_base;
}

bool ApproachMode::beginStep(int frame_offset) {
  GraspController* controller = controller_;
  phase_time_ = 0.0;

  std::vector<int> swing_indices;
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!gaits_[leg]) {
      swing_indices.push_back(leg);
    }
  }
  swing_start_base_ = controller->foot_desired_base;
  swing_target_base_ = controller->foot_init_base;
  swing_start_velocity_xy_ = foot_velocity_xy_;
  updateSwingTarget(swing_indices, frame_offset);
  if (!planSwing(swing_indices)) {
    return false;
  }
  return true;
}

const ApproachPlan& ApproachMode::update(const Eigen::Vector4d& command,
                                         const NavigationState* navigation_state) {
  Eigen::Vector4d effective_command = command;
  if (approach_plan.failed) {
    effective_command = Eigen::Vector4d::Zero();
  } else if (approach_plan.active) {
    if (navigation_state == nullptr) {
      effective_command = Eigen::Vector4d::Zero();
      failApproach("navigation state was not supplied");
    } else {
      effective_command = autonomousCommand(*navigation_state);
    }
  }
  updateGait(effective_command);
  return approach_plan;
}

void ApproachMode::updateGait(const Eigen::Vector4d& command) {
  GraspController* controller = controller_;
  // 控制器每帧开头已清空 joint_override；本帧的发出标记重新计。
  swing_override_emitted_.fill(false);

  // 回正由控制器中的关节插值接管。
  if (controller->reset_active) {
    return;
  }

  // 每帧过滤操作者请求；本相使用的命令只会在下一次换相时锁存。
  slewRequestedCommand(command);

  const double velocity_right = command[0];
  const double velocity_forward = command[1];
  const double velocity_up = command[2];
  const double yaw_rate = command[3];

  body_height_offset_ += velocity_up * controller->dt;
  body_height_offset_ = std::max(body_height_offset_min_,
                                 std::min(body_height_offset_max_,
                                          body_height_offset_));

  FootPositions ground_z{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    ground_z[leg] = controller->foot_init_base[leg];
    ground_z[leg].z() = controller->foot_init_base[leg].z() - body_height_offset_;
  }
  FootPositions candidate_base = controller->foot_desired_base;

  // 单独的 vz 只调节机身高度，不触发三角换相。
  const Eigen::Vector3d planar_command(velocity_right, velocity_forward, yaw_rate);
  if (!gait_started) {
    for (int leg = 0; leg < kLegCount; ++leg) {
      candidate_base[leg].z() = ground_z[leg].z();
    }
    const bool idle_committed = commitCandidate(candidate_base);
    emitOverrides(idle_committed, std::array<bool, 6>{}, std::array<Vector3d, 6>{});
    if (!idle_committed) {
      return;
    }
    foot_velocity_xy_.fill(Eigen::Vector2d::Zero());

    if (planar_command.norm() < 1e-8) {
      return;
    }
    gait_started = true;
    stop_requested_ = false;
    if (!beginStep(0)) {
      abortStep();
      return;
    }
  } else {
    // 零指令时先让当前摆动组落地，再停止。
    stop_requested_ = planar_command.norm() < 1e-8;
  }

  if (transfer_active_) {
    const double transfer_dt =
        std::min(controller->dt, transfer_duration_ - transfer_time_);
    std::array<Eigen::Vector2d, 6> candidate_xy{};
    for (int leg = 0; leg < kLegCount; ++leg) {
      candidate_xy[leg] = candidate_base[leg].head<2>();
    }
    const std::array<Eigen::Vector2d, 6> stance = stanceVelocity(candidate_xy, requested_command_);
    for (int leg = 0; leg < kLegCount; ++leg) {
      candidate_base[leg].head<2>() += stance[leg] * transfer_dt;
      candidate_base[leg].z() = ground_z[leg].z();
    }
    const std::array<Eigen::Vector2d, 6> velocity_before = foot_velocity_xy_;
    foot_velocity_xy_ = stance;

    const bool transfer_committed = commitCandidate(candidate_base);
    emitOverrides(transfer_committed, std::array<bool, 6>{}, std::array<Vector3d, 6>{});
    if (!transfer_committed) {
      foot_velocity_xy_ = velocity_before;
      return;
    }
    transfer_time_ += transfer_dt;

    const bool transfer_finished =
        transfer_time_ >= transfer_duration_ - 1e-12;
    if (!transfer_finished) {
      return;
    }

    transfer_active_ = false;
    transfer_time_ = 0.0;
    stance_group_index_ = 1 - stance_group_index_;
    gaits_.fill(false);
    if (stance_group_index_ == 0) {
      for (int leg : kTripodAIndices) {
        gaits_[leg] = true;
      }
    } else {
      for (int leg : kTripodBIndices) {
        gaits_[leg] = true;
      }
    }
    first_step_ = false;

    // 零指令且限加速度指令已降到零才停；否则继续迈步，
    // 前瞻落点会让最后几步自然缩短。
    if (stop_requested_ && requested_command_.head<2>().norm() < 1e-6 &&
        std::abs(requested_command_[3]) < 1e-6) {
      gait_started = false;
      phase_time_ = 0.0;
      requested_command_.setZero();
      foot_velocity_xy_.fill(Eigen::Vector2d::Zero());
      return;
    }
    if (!beginStep(1)) {
      abortStep();
      return;
    }
    return;
  }

  const double phase_dt = std::min(controller->dt, phase_duration_ - phase_time_);
  std::vector<int> stance_indices;
  std::vector<int> swing_indices;
  for (int leg = 0; leg < kLegCount; ++leg) {
    (gaits_[leg] ? stance_indices : swing_indices).push_back(leg);
  }

  std::array<Eigen::Vector2d, 6> candidate_xy{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    candidate_xy[leg] = candidate_base[leg].head<2>();
  }
  const std::array<Eigen::Vector2d, 6> stance = stanceVelocity(candidate_xy, requested_command_);
  for (int leg : stance_indices) {
    candidate_base[leg].head<2>() += stance[leg] * phase_dt;
    candidate_base[leg].z() = ground_z[leg].z();
  }
  const std::array<Eigen::Vector2d, 6> velocity_before = foot_velocity_xy_;
  for (int leg : stance_indices) {
    foot_velocity_xy_[leg] = stance[leg];
  }

  const double next_phase_time = phase_time_ + phase_dt;
  const double phase = std::max(0.0, std::min(1.0, next_phase_time / phase_duration_));
  const bool phase_finished = next_phase_time >= phase_duration_ - 1e-12;
  // 摆动腿走关节空间轨迹：候选足端取规划关节角的 FK，使边界、
  // 工作空间和碰撞检查看到真实的规划足端。
  std::array<Vector3d, 6> swing_joint_targets{};
  FootPositions swing_hip_feet{};
  FootPositions swing_hip_moved{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    swing_hip_feet[leg].setZero();
    swing_hip_moved[leg].setZero();
    swing_joint_targets[leg].setZero();
  }
  for (int leg : swing_indices) {
    if (phase_finished) {
      swing_joint_targets[leg] = swing_q_end_[leg];
      continue;
    }
    Vector3d q_leg;
    Vector3d dq_leg;
    swingJointState(leg, phase, &q_leg, &dq_leg);
    swing_joint_targets[leg] = q_leg;
    swing_hip_feet[leg] = controller->kinematic.forwardLeg(leg, q_leg);
    swing_hip_moved[leg] =
        swing_hip_feet[leg] + controller->kinematic.jacobianLeg(leg, q_leg) * dq_leg;
  }
  if (!phase_finished) {
    const FootPositions swing_feet_base = controller->kinematic.hipToBase(swing_hip_feet);
    const FootPositions swing_feet_moved = controller->kinematic.hipToBase(swing_hip_moved);
    for (int leg : swing_indices) {
      candidate_base[leg] = swing_feet_base[leg];
      foot_velocity_xy_[leg] =
          (swing_feet_moved[leg] - swing_feet_base[leg]).head<2>();
    }
  } else {
    for (int leg : swing_indices) {
      candidate_base[leg] = swing_target_base_[leg];
      candidate_base[leg].z() = ground_z[leg].z();
      foot_velocity_xy_[leg] = swing_target_velocity_xy_[leg];
    }
  }

  const bool committed = commitCandidate(candidate_base);
  if (!committed) {
    foot_velocity_xy_ = velocity_before;
  }
  // 控制器每帧清空覆盖：摆动腿取调度，支撑腿取已提交足端的精确 IK；
  // commit 失败时重复上一帧目标以保持足端。
  std::array<bool, 6> is_swing{};
  for (int leg : swing_indices) {
    is_swing[leg] = true;
  }
  emitOverrides(committed, is_swing, swing_joint_targets);
  if (!committed) {
    return;
  }
  phase_time_ = next_phase_time;

  if (phase_finished) {
    for (int leg : swing_indices) {
      swing_target_base_[leg].z() = ground_z[leg].z();
    }
    transfer_active_ = true;
    transfer_time_ = 0.0;
  }
}

}  // namespace grasp_hexapod_control_cpp
