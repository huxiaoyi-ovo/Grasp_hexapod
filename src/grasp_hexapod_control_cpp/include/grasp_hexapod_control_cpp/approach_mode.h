// 接近模式的目标选择、固定路径和三角步态规划器
// （对应 Python approach_mode.py::ApproachMode）。
//
// 输入 command=(向右, 向前, 向上, 偏航)，单位 m/s、rad/s；NavigationState 提供
// pv_map 中的六足、小蓝和光伏板边界。输出更新 controller.foot_desired_base
// 并给出接近任务状态。左右接近位姿必须经过实测后由外部配置。
#pragma once

#include "grasp_hexapod_control_cpp/math_utils.h"
#include "grasp_hexapod_control_cpp/types.h"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace grasp_hexapod_control_cpp {

class GraspController;

// 三角步态分组：A 组 lb、lf、rm；B 组 lm、rb、rf。
inline constexpr int kTripodAIndices[] = {0, 1, 5};
inline constexpr int kTripodBIndices[] = {2, 3, 4};

struct ApproachPlan {
  bool active = false;
  std::string state = "idle";
  std::string target_side;
  Matrix4d target_pose_pv = Matrix4d::Identity();
  std::vector<Vector2d> waypoints_pv;
  int waypoint_index = 0;
  double minimum_clearance = 0.0;
  bool ready_for_climb = false;
  bool failed = false;
  std::string reason;
};

class ApproachMode {
 public:
  explicit ApproachMode(GraspController* controller);

  // GraspController 构造体（foot_init_base 就绪后）调用；构造函数只存指针。
  void initFromController();

  // 增加一个固定左右接近位姿和共用导航参数（实机来源必须先过外部标定门）。
  void configureFixedApproach(
      const Matrix4d& xiaolan_from_base, const std::string& target_side,
      double boundary_margin, double linear_speed, double yaw_rate,
      const std::vector<Vector2d>& xiaolan_keepout_polygon_xy_m,
      double xiaolan_body_clearance_m);

  // 在左右入口中选择可直达者；不搜索绕障路线。
  ApproachPlan planAutonomousApproach(const NavigationState& navigation_state,
                                      const std::optional<std::string>& target_side);

  void cancelAutonomousApproach(const std::string& reason);

  // 根据手柄或自动接近指令更新六足目标，并返回接近任务状态。
  const ApproachPlan& update(const Eigen::Vector4d& command,
                             const NavigationState* navigation_state);

  // 回正期间清空三角步态状态，完成后从 A 组重新起步。
  void reset();
  // 标准站姿恢复完成后同步模式内部的足端基准。
  void finishReset();

  ApproachPlan approach_plan;

  // 回正由控制器中的关节插值接管时步态暂停。
  bool gait_started = false;
  // MissionStateMachine 在 full mission 启动时覆盖这两个门限。
  double position_tolerance_ = 0.03;
  double yaw_tolerance_ = 5.0 * M_PI / 180.0;
  bool transfer_active_ = false;

 private:
  ApproachPlan failApproach(const std::string& reason);
  bool xiaolanDirectPathFeasible(const Vector2d& start_xy, const Vector2d& target_xy,
                                 const std::vector<Vector2d>& obstacle) const;
  double gaitMotionMargin() const;
  std::pair<bool, double> poseFeasible(const Vector2d& center_xy, double yaw,
                                       const std::vector<Vector2d>& boundary) const;
  std::pair<bool, double> rotationFeasible(const Vector2d& center_xy, double start_yaw,
                                           double target_yaw,
                                           const std::vector<Vector2d>& boundary) const;
  std::pair<bool, double> translationFeasible(const Vector2d& start_xy,
                                              const Vector2d& end_xy, double yaw,
                                              const std::vector<Vector2d>& boundary) const;
  std::pair<bool, double> remainingRouteFeasible(
      const NavigationState& navigation, const Vector2d& current_xy,
      double current_yaw, const std::vector<Vector2d>& waypoints_pv,
      double target_yaw) const;
  Eigen::Vector4d autonomousCommand(const NavigationState& navigation_state);
  bool commitCandidate(const FootPositions& candidate_base);
  static double smoothStepStatic(double phase);
  double trapezoidRise(double u) const;
  // 五次轨迹：两端位置、速度连续，两端加速度为零（按轴计算）。
  static std::pair<Eigen::Vector2d, Eigen::Vector2d> quinticSegment(
      const Eigen::Vector2d& start, const Eigen::Vector2d& target,
      const Eigen::Vector2d& start_velocity, const Eigen::Vector2d& target_velocity,
      double duration, double phase);
  static std::array<Eigen::Vector2d, 6> stanceVelocity(
      const std::array<Eigen::Vector2d, 6>& foot_xy, const Eigen::Vector4d& command);
  void slewRequestedCommand(const Eigen::Vector4d& command);
  void updateSwingTarget(const std::vector<int>& swing_indices);
  void beginStep();
  void updateGait(const Eigen::Vector4d& command);

  GraspController* controller_;

  // 左右入口都必须是相对 xiaolan_frame 的外部标定量。
  std::map<std::string, Matrix4d> xiaolan_from_fixed_approaches_;
  std::vector<Vector2d> xiaolan_keepout_polygon_xy_;
  double xiaolan_body_clearance_ = 0.13;
  std::optional<NavigationState> navigation_state_;
  double boundary_margin_ = 0.03;
  double path_sample_spacing_ = 0.02;
  double auto_linear_speed_ = 0.12;
  double auto_yaw_rate_ = 0.8;
  double robot_safety_radius_ = 0.0;

  // True 表示支撑腿，False 表示摆动腿。
  std::array<bool, 6> gaits_{};
  int stance_group_index_ = 0;

  // 实机优先保持足端轨迹和舵机跟踪平滑，不追求极限步频。
  double phase_duration_ = 0.70;
  double phase_time_ = 0.0;

  // 共同支撑目标为 50 ms，并量化到最接近的整数控制帧。
  int transfer_frames_ = 1;
  double transfer_duration_ = 0.0;
  double transfer_time_ = 0.0;

  double step_height_ = 0.040;
  // 前 45% 平滑抬起、中间 10% 保持高位、后 45% 平滑落下。
  double swing_lift_fraction_ = 0.45;
  // 梯形剖面加减速段占比，峰值速度由单程抬腿时间决定。
  double lift_accel_fraction_ = 0.15;
  double max_linear_acceleration_ = 0.40;
  double max_yaw_acceleration_ = 2.0;
  // 合并平移和偏航后，限制标准足端的平面速度预算。
  double max_foot_planar_speed_ = 0.12;
  double nominal_foot_radius_ = 0.0;
  Eigen::Vector4d requested_command_ = Eigen::Vector4d::Zero();
  Eigen::Vector4d active_phase_command_ = Eigen::Vector4d::Zero();
  bool stop_requested_ = false;

  double body_height_offset_ = 0.0;
  double body_height_offset_min_ = -0.0075;
  double body_height_offset_max_ = 0.015;

  bool first_step_ = true;

  FootPositions swing_start_base_{};
  FootPositions swing_target_base_{};
  std::array<Eigen::Vector2d, 6> foot_velocity_xy_{};
  std::array<Eigen::Vector2d, 6> swing_start_velocity_xy_{};
  std::array<Eigen::Vector2d, 6> swing_target_velocity_xy_{};
};

}  // namespace grasp_hexapod_control_cpp
