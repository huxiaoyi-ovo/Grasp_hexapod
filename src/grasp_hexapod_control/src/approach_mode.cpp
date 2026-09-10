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
      const double speed = std::min(auto_linear_speed_, 1.5 * position_error);
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

double ApproachMode::smoothStepStatic(double phase) {
  const double p2 = phase * phase;
  const double p3 = p2 * phase;
  return 10.0 * p3 - 15.0 * p3 * phase + 6.0 * p3 * p2;
}

double ApproachMode::trapezoidRise(double u) const {
  // 梯形升起剖面(归一化 0→1): smoothstep 加速-匀速-减速。
  const double a = lift_accel_fraction_;
  const double v = 1.0 / (1.0 - a);
  if (u <= a) {
    return v * a * 0.5 * smoothStepStatic(u / a);
  }
  if (u <= 1.0 - a) {
    return v * (a * 0.5 + (u - a));
  }
  return v * (1.0 - 1.5 * a + 0.5 * a * smoothStepStatic((u - 1.0 + a) / a));
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> ApproachMode::quinticSegment(
    const Eigen::Vector2d& start, const Eigen::Vector2d& target,
    const Eigen::Vector2d& start_velocity, const Eigen::Vector2d& target_velocity,
    double duration, double phase) {
  Eigen::Vector2d position;
  Eigen::Vector2d velocity;
  for (int axis = 0; axis < 2; ++axis) {
    const double start_scaled_velocity = start_velocity[axis] * duration;
    const double velocity_change = (target_velocity[axis] - start_velocity[axis]) * duration;
    const double residual = target[axis] - start[axis] - start_scaled_velocity;

    const double c3 = 10.0 * residual - 4.0 * velocity_change;
    const double c4 = -15.0 * residual + 7.0 * velocity_change;
    const double c5 = 6.0 * residual - 3.0 * velocity_change;

    position[axis] = start[axis] + start_scaled_velocity * phase +
                     c3 * std::pow(phase, 3) + c4 * std::pow(phase, 4) +
                     c5 * std::pow(phase, 5);
    velocity[axis] = (start_scaled_velocity + 3.0 * c3 * phase * phase +
                      4.0 * c4 * std::pow(phase, 3) + 5.0 * c5 * std::pow(phase, 4)) /
                     duration;
  }
  return {position, velocity};
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

void ApproachMode::slewRequestedCommand(const Eigen::Vector4d& command) {
  Eigen::Vector4d slewed = command;
  const double foot_speed_budget =
      slewed.head<2>().norm() + nominal_foot_radius_ * std::abs(slewed[3]);
  if (foot_speed_budget > max_foot_planar_speed_) {
    const double scale = max_foot_planar_speed_ / foot_speed_budget;
    slewed.head<2>() *= scale;
    slewed[3] *= scale;
  }
  Eigen::Vector2d linear_delta = slewed.head<2>() - requested_command_.head<2>();
  const double linear_norm = linear_delta.norm();
  const double linear_step = max_linear_acceleration_ * controller_->dt;
  if (linear_norm > linear_step) {
    linear_delta *= linear_step / linear_norm;
  }
  requested_command_.head<2>() += linear_delta;

  const double yaw_delta = std::max(
      -max_yaw_acceleration_ * controller_->dt,
      std::min(max_yaw_acceleration_ * controller_->dt,
               slewed[3] - requested_command_[3]));
  requested_command_[3] += yaw_delta;
  requested_command_[2] = slewed[2];
}

void ApproachMode::updateSwingTarget(const std::vector<int>& swing_indices) {
  GraspController* controller = controller_;
  FootPositions target = controller->foot_init_base;
  std::array<Eigen::Vector2d, 6> home_xy{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    home_xy[leg] = controller->foot_init_base[leg].head<2>();
  }
  const std::array<Eigen::Vector2d, 6> stance_at_home = stanceVelocity(home_xy, active_phase_command_);
  const double support_duration = phase_duration_ + 2.0 * transfer_duration_;
  for (int leg : swing_indices) {
    target[leg].head<2>() +=
        0.5 * support_duration * (-stance_at_home[leg]);
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
  // 目标速度在（投影后的）新落点位置上求值。
  std::array<Eigen::Vector2d, 6> target_xy{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    target_xy[leg] = swing_target_base_[leg].head<2>();
  }
  const std::array<Eigen::Vector2d, 6> target_velocity = stanceVelocity(target_xy, active_phase_command_);
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
  active_phase_command_.setZero();
  stop_requested_ = false;
  gait_started = false;
  first_step_ = true;
  foot_velocity_xy_.fill(Eigen::Vector2d::Zero());
  swing_start_velocity_xy_.fill(Eigen::Vector2d::Zero());
  swing_target_velocity_xy_.fill(Eigen::Vector2d::Zero());
  approach_plan = ApproachPlan();
  navigation_state_.reset();
}

void ApproachMode::finishReset() {
  body_height_offset_ = 0.0;
  swing_start_base_ = controller_->foot_init_base;
  swing_target_base_ = controller_->foot_init_base;
}

void ApproachMode::beginStep() {
  GraspController* controller = controller_;
  phase_time_ = 0.0;
  active_phase_command_ = requested_command_;

  std::vector<int> swing_indices;
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!gaits_[leg]) {
      swing_indices.push_back(leg);
    }
  }
  swing_start_base_ = controller->foot_desired_base;
  swing_target_base_ = controller->foot_init_base;
  swing_start_velocity_xy_ = foot_velocity_xy_;
  updateSwingTarget(swing_indices);
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
    if (!commitCandidate(candidate_base)) {
      return;
    }
    foot_velocity_xy_.fill(Eigen::Vector2d::Zero());

    if (planar_command.norm() < 1e-8) {
      return;
    }
    gait_started = true;
    stop_requested_ = false;
    beginStep();
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
    const std::array<Eigen::Vector2d, 6> stance = stanceVelocity(candidate_xy, active_phase_command_);
    for (int leg = 0; leg < kLegCount; ++leg) {
      candidate_base[leg].head<2>() += stance[leg] * transfer_dt;
      candidate_base[leg].z() = ground_z[leg].z();
    }
    const std::array<Eigen::Vector2d, 6> velocity_before = foot_velocity_xy_;
    foot_velocity_xy_ = stance;

    if (!commitCandidate(candidate_base)) {
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

    if (stop_requested_) {
      gait_started = false;
      phase_time_ = 0.0;
      requested_command_.setZero();
      active_phase_command_.setZero();
      foot_velocity_xy_.fill(Eigen::Vector2d::Zero());
      return;
    }
    beginStep();
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
  const std::array<Eigen::Vector2d, 6> stance = stanceVelocity(candidate_xy, active_phase_command_);
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
  for (int leg : swing_indices) {
    const auto [position, velocity] = quinticSegment(
        swing_start_base_[leg].head<2>(), swing_target_base_[leg].head<2>(),
        swing_start_velocity_xy_[leg], swing_target_velocity_xy_[leg],
        phase_duration_, phase);
    candidate_base[leg].head<2>() = position;
    foot_velocity_xy_[leg] = velocity;
  }

  double lift_height;
  if (phase < swing_lift_fraction_) {
    lift_height = step_height_ * trapezoidRise(phase / swing_lift_fraction_);
  } else if (phase > 1.0 - swing_lift_fraction_) {
    lift_height =
        step_height_ * trapezoidRise((1.0 - phase) / swing_lift_fraction_);
  } else {
    lift_height = step_height_;
  }
  for (int leg : swing_indices) {
    candidate_base[leg].z() = ground_z[leg].z() + lift_height;
  }

  const bool phase_finished = next_phase_time >= phase_duration_ - 1e-12;
  if (phase_finished) {
    for (int leg : swing_indices) {
      candidate_base[leg] = swing_target_base_[leg];
      candidate_base[leg].z() = ground_z[leg].z();
      foot_velocity_xy_[leg] = swing_target_velocity_xy_[leg];
    }
  }

  if (!commitCandidate(candidate_base)) {
    foot_velocity_xy_ = velocity_before;
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
