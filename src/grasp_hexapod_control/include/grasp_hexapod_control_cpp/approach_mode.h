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
  // 离线诊断：基础路径本身已超速、速度限幅无解的腿步数。
  int speed_governor_infeasible = 0;
  // Read-only: true = stance; stale during transfer/stopped, callers treat those as all-stance.
  const std::array<bool, 6>& stanceMask() const { return gaits_; }

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
  // 摆动腿关节空间轨迹：beginStep 规划一次，每帧求值。
  Vector3d basePointToHip(int leg, const Vector3d& point_base) const;
  Vector3d hipPointToBase(int leg, const Vector3d& point_hip) const;
  Vector3d baseVectorToHip(int leg, const Vector3d& point_base,
                           const Vector3d& vector_base) const;
  // 单腿数值逆解（雅可比 Newton）；返回是否收敛。
  bool legIk(int leg, const Vector3d& target_hip, const Vector3d& q_seed,
             Vector3d* q) const;
  // 关节五次 Hermite：两端位置、速度给定，两端加速度为零。
  static void hermiteQuintic(const Vector3d& q0, const Vector3d& q1,
                             const Vector3d& qd0, const Vector3d& qd1,
                             double duration, double u, Vector3d* q,
                             Vector3d* dq);
  // 第 leg 腿相位 u 的关节角与关节速度（膝关节叠加抬腿鼓包）。
  void swingJointState(int leg, double u, Vector3d* q, Vector3d* dq) const;
  bool planSwing(const std::vector<int>& swing_indices);
  // 清空关节覆盖状态，并撤销本模式发出的关节覆盖（回正前后调用）。
  void clearOverrideState();
  // 写本帧六条腿的关节覆盖：committed 为真时摆动腿取调度、其余腿对已提交
  // 足端做 IK（以上一帧覆盖为初值）；任一腿 IK 失败或 committed 为假，
  // 重复发出上一帧覆盖（没有上一帧时保持 q_des）。
  void emitOverrides(bool committed, const std::array<bool, 6>& is_swing,
                     const std::array<Vector3d, 6>& swing_targets);
  void abortStep();
  static std::array<Eigen::Vector2d, 6> stanceVelocity(
      const std::array<Eigen::Vector2d, 6>& foot_xy, const Eigen::Vector4d& command);
  // 按足端平面速度预算缩放指令（平移和偏航同比例）。
  Eigen::Vector4d scaledCommand(const Eigen::Vector4d& command) const;
  // 纯函数：requested 按实机可控加速度向 target 推进一帧。
  Eigen::Vector4d slewCommand(const Eigen::Vector4d& requested,
                              const Eigen::Vector4d& target) const;
  void slewRequestedCommand(const Eigen::Vector4d& command);
  // 纯函数：从当前 requested_command_ 起向保持不变的目标指令逐帧外推，
  // 第 i 项是 i 帧之后的指令（第 0 项即当前指令）。
  std::vector<Eigen::Vector4d> predictCommandProfile(int count) const;
  // frame_offset：本相第一帧相对当前帧的帧数（起步帧为 0，换相帧为 1）。
  void updateSwingTarget(const std::vector<int>& swing_indices, int frame_offset);
  // 摆动规划失败返回 false，且不起步。
  bool beginStep(int frame_offset);
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

  // 摆动腿走关节空间轨迹，步态周期放慢以保证关节速度不超过 3 rad/s。
  // 步距 = v*(T+2τ)：0.14 m/s 约 8.4 cm，落点由工作空间投影兜底。
  double phase_duration_ = 0.50;
  double phase_time_ = 0.0;

  // 共同支撑目标为 50 ms，并量化到最接近的整数控制帧。
  int transfer_frames_ = 1;
  double transfer_duration_ = 0.0;
  double transfer_time_ = 0.0;

  // 摆动全程足端相对支撑地面的最高净空（按 21 个采样点的峰值标定）。
  double step_height_ = 0.040;
  // 摆动两端关节速度 = alpha × 支撑腿刚体速度对应的关节速度。
  double swing_boundary_velocity_ratio_ = 0.5;
  // 从静止起步时第一组支撑腿从 home 出发，斜坡时间约为 T+τ 才能让首程
  // 约为半个步距，留在工作空间内。
  double max_linear_acceleration_ = 0.25;
  double max_yaw_acceleration_ = 1.2;
  // 合并平移和偏航后，限制标准足端的平面速度预算。
  // 这是实际限速瓶颈：navigation_linear_speed 配 0.20 也被这里砍回限幅值。
  double max_foot_planar_speed_ = 0.14;
  double nominal_foot_radius_ = 0.0;
  Eigen::Vector4d requested_command_ = Eigen::Vector4d::Zero();
  // 最近一帧经足端速度预算缩放后的原始指令；落点前瞻按它保持不变外推。
  Eigen::Vector4d slew_target_command_ = Eigen::Vector4d::Zero();
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

  // 摆动腿关节空间计划：基础五次 Hermite 端点/端速度 + 仅膝关节的抬腿鼓包。
  JointAngles swing_q_start_{};
  JointAngles swing_q_end_{};
  JointAngles swing_qd_start_{};
  JointAngles swing_qd_end_{};
  // 带符号的膝关节抬腿幅值 s*A（rad）。
  std::array<double, 6> swing_lift_amplitude_{};
  // 摆动腿的抬腿幅值受关节速度限制（rad/s），分析峰值取 41 个均匀采样点。
  double swing_joint_speed_limit_ = 3.0;
  // 步行期间六条腿全部走关节覆盖：摆动腿取调度，其余腿取已提交足端的精确 IK。
  // last_override_q_ 是上一帧成功提交的六腿关节目标；commit 或 IK 失败时重复发出。
  JointAngles last_override_q_{};
  bool last_override_valid_ = false;
  // 本帧由本模式写入 joint_override 的腿。
  std::array<bool, 6> swing_override_emitted_{};
};

}  // namespace grasp_hexapod_control_cpp
