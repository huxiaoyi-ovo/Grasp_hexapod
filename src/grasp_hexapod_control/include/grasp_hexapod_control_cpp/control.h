// 六足总控制器与公共安全执行层（对应 Python control.py）。
//
// 管理 APPROACH/CLIMB/DOCK 三种模式，维护公共足端目标，执行工作空间、
// 足端/连杆碰撞检查，并用 DLS 把笛卡尔足端误差转换为关节目标。
// 不直接调用 ROS 或舵机 SDK；仿真和实机共用本控制逻辑。
#pragma once

#include "grasp_hexapod_control_cpp/approach_mode.h"
#include "grasp_hexapod_control_cpp/climb_compact.h"
#include "grasp_hexapod_control_cpp/climb_mode.h"
#include "grasp_hexapod_control_cpp/config_io.h"
#include "grasp_hexapod_control_cpp/dock_mode.h"
#include "grasp_hexapod_control_cpp/kinematics.h"
#include "grasp_hexapod_control_cpp/math_utils.h"
#include "grasp_hexapod_control_cpp/types.h"

#include <optional>
#include <string>

namespace grasp_hexapod_control_cpp {

// 配置文件路径（由壳层/测试注入，核心库不做 ROS 包路径解析）。
struct ControlPaths {
  std::string climb_compact_json;
  std::string workspace_bounds_csv;
};

class GraspController;

// 调度自动接近、攀爬和对接交接状态（control.py::MissionStateMachine）。
class MissionStateMachine {
 public:
  static constexpr const char* kIdle = "IDLE";
  static constexpr const char* kApproach = "APPROACH";
  static constexpr const char* kFailLanding = "FAIL_LANDING";
  static constexpr const char* kPrepareClimb = "PREPARE_CLIMB";
  static constexpr const char* kClimb = "CLIMB";
  static constexpr const char* kDock = "DOCK";
  static constexpr const char* kFailed = "FAILED";

  explicit MissionStateMachine(GraspController* controller);

  // 规划自动接近并保存后续攀爬任务。
  ApproachPlan start(const NavigationState& navigation_state,
                     const ClimbCompactConfig& climb_config, int start_stage_index = 0,
                     std::optional<int> end_stage_index = std::nullopt,
                     double approach_timeout_s = 60.0,
                     double approach_position_tolerance_m = 0.01,
                     double approach_yaw_tolerance_rad = 1.0 * M_PI / 180.0,
                     bool climb_hardware_execution = false);

  // 取消自动任务并回到空闲调度状态。
  void cancel(const std::string& reason = "cancelled");

  // 推进当前任务阶段；成功门槛来自各子模式。返回本帧接近计划（如有）。
  std::optional<ApproachPlan> update(const JointAngles& q_cur,
                                     const NavigationState* navigation_state);

  std::string state = kIdle;
  std::string reason;
  std::optional<ClimbCompactConfig> climb_config;
  int climb_start_stage_index = 0;
  std::optional<int> climb_end_stage_index;
  bool climb_hardware_execution = false;
  double approach_elapsed_s = 0.0;
  double approach_timeout_s = 60.0;
  double prepare_elapsed_s = 0.0;
  double prepare_settle_s = 0.0;
  double last_prepare_joint_error_rad = std::numeric_limits<double>::infinity();
  double last_prepare_world_foot_error_m = std::numeric_limits<double>::infinity();
  double last_prepare_position_error_m = std::numeric_limits<double>::infinity();
  double last_prepare_yaw_error_rad = std::numeric_limits<double>::infinity();
  std::optional<Matrix4d> approach_target_pose_pv;
  double approach_position_tolerance_m = 0.01;
  double approach_yaw_tolerance_rad = 1.0 * M_PI / 180.0;
  int prepare_retry_count = 0;
  int max_prepare_retries = 3;
  bool posture_prepared = false;
  std::optional<Matrix4d> prepare_previous_pose_pv;
  double prepare_previous_stamp = 0.0;
  double last_prepare_linear_speed_m_s = std::numeric_limits<double>::infinity();
  double last_prepare_angular_speed_rad_s = std::numeric_limits<double>::infinity();
  double prepare_linear_speed_limit_m_s = 0.01;
  double prepare_angular_speed_limit_rad_s = 1.0 * M_PI / 180.0;
  double final_position_error_m = std::numeric_limits<double>::infinity();
  double final_orientation_error_rad = std::numeric_limits<double>::infinity();
  double final_world_foot_error_m = std::numeric_limits<double>::infinity();
  double final_position_tolerance_m = 0.03;
  double final_orientation_tolerance_rad = 5.0 * M_PI / 180.0;
  double final_world_foot_tolerance_m = 0.03;
  std::string pending_failure_reason;

 private:
  // 用 RTK/IMU 位姿与关节 FK 检查 C30 几何终态。
  bool climbTerminalReady(const JointAngles& q_cur, const NavigationState* navigation_state);

  GraspController* controller_;
};

class GraspController {
 public:
  static constexpr const char* kApproach = "approach";
  static constexpr const char* kClimb = "climb";
  static constexpr const char* kDock = "dock";

  GraspController(double dt, bool enable_link_collision_check,
                  bool climb_timeout_uses_wall_time, const ControlPaths& paths);

  const ControlPaths& paths() const { return paths_; }

  // ---------------------------------------------------------------------------
  // 公有状态（与 Python 属性一一对应；模式与壳层直接读写）。
  // ---------------------------------------------------------------------------
  double dt;
  bool enable_link_collision_check;
  bool climb_timeout_uses_wall_time;
  GraspKinematic kinematic;
  WorkspaceBoundary workspace_boundary;

  JointAngles q_init{};
  JointAngles q_des{};
  FootPositions foot_init_hip{};
  FootPositions foot_init_base{};
  FootPositions foot_current_hip{};
  FootPositions foot_desired_base{};
  FootPositions foot_desired_base_prev{};  // 前馈差分用
  // 普通行走每帧只保留廉价的足端工作空间/间距门。
  bool enable_workspace_check = true;
  FootPositions foot_desired_hip{};

  double base_height_at_stand = 0.0;

  // A 使能和 B 复位都用五次曲线平滑回到标准站姿。
  double reset_duration = 2.0;
  double reset_time = 0.0;
  bool reset_active = false;
  JointAngles reset_start_q{};

  std::array<bool, 6> last_link_collision_free{};
  // Observation-only per-update telemetry; it never feeds control.
  int last_update_velocity_limit_clip_count = 0;
  int last_update_collision_guard_hold_count = 0;

  ApproachMode approach_mode;
  ClimbMode climb_mode;
  // Dock 入口的默认姿态来自 compact 配置的明确末帧快照。
  // B 复位和 ClimbMode 状态清理都不会清除该缓存。
  JointAngles climb_terminal_q{};
  DockMode* dock_mode = nullptr;  // 由壳层附接并持有
  MissionStateMachine mission;
  std::string mode = kApproach;
  std::optional<ApproachPlan> last_mode_result;

  // ---------------------------------------------------------------------------
  // 安全执行层。
  // ---------------------------------------------------------------------------
  // 判断六个候选足端是否位于离线生成的安全工作空间。
  std::array<bool, 6> workspaceFeasible(const FootPositions& foot_positions_base) const;
  // 把候选足端连续投影到安全工作空间边界内。
  FootPositions projectWorkspace(const FootPositions& foot_positions_base) const;
  // 规划层足端球碰撞检查。
  std::array<bool, 6> footCollisionFree(const FootPositions& foot_positions_base) const;
  // 用 URDF 碰撞盒等效胶囊检查整机连杆自碰撞。
  std::array<bool, 6> linkCollisionFree(const JointAngles& joint_angles) const;
  // 普通行走跳过整机扫描，其他模式按配置检查连杆。
  JointAngles collisionGuard(const JointAngles& q_candidate, const JointAngles& q_current);
  // 用当前反馈同步模式交接的足端参考。
  FootPositions syncActualFeet(const JointAngles& q_cur);
  // 连续投影并提交足端候选，碰撞时才保持旧目标。
  bool commitWorkspaceCandidate(const FootPositions& candidate_base);

  // ---------------------------------------------------------------------------
  // 模式入口。
  // ---------------------------------------------------------------------------
  // 从当前关节角平滑回到标准站姿（B 全局恢复动作）。
  void resetToStand(const JointAngles& q_cur);
  // 从当前关节角平滑整理到 compact 要求的标准站姿。
  void prepareClimbEntry(const JointAngles& q_cur);
  // 在 APPROACH 模式下选择安全接近点并启动固定路径。
  ApproachPlan startAutonomousApproach(
      const NavigationState& navigation_state,
      const std::optional<std::string>& target_side = std::nullopt);
  ApproachPlan startFullMission(
      const NavigationState& navigation_state, const ClimbCompactConfig& climb_config,
      int start_stage_index = 0, std::optional<int> end_stage_index = std::nullopt,
      double approach_timeout_s = 60.0,
      double approach_position_tolerance_m = 0.01,
      double approach_yaw_tolerance_rad = 1.0 * M_PI / 180.0,
      bool climb_hardware_execution = false);
  // 切换唯一活动模式。
  void setMode(const std::string& mode);
  // 通过唯一入口进入 compact 攀爬。
  void enterClimb(const JointAngles& q_cur,
                  const std::optional<ClimbCompactConfig>& config = std::nullopt,
                  int start_stage_index = 0,
                  std::optional<int> end_stage_index = std::nullopt,
                  bool hardware_execution = false);
  // 附接已由 ROS 入口配置的 DockMode。
  void attachDockMode(DockMode* dock_mode);
  // 保存攀爬末姿态并把 DockMode 设为唯一活动模式。
  void enterDock(const JointAngles& q_cur);
  // 结束 DockMode 并以实际足端作为下一模式入口。
  void exitDock(const JointAngles& q_cur);
  // 用同一 DLS 控制链连续回放 compact 前缀并返回关节快照。
  JointAngles replayClimbPrefix(const JointAngles& q_cur,
                                const ClimbCompactConfig& config, int end_stage_index,
                                int max_ticks);
  void holdClimb();
  void resumeClimb();
  // 停止当前攀爬会话；B 复位后只能重新通过入口开始。
  void abortClimb();

  // 执行当前任务模式并输出本周期关节目标。
  JointAngles update(const JointAngles& q_cur, const Eigen::Vector4d& command,
                     const NavigationState* navigation_state = nullptr,
                     const DockRobotState* dock_robot_state = nullptr);

  // 根据足端目标计算下一周期关节目标（30Hz 控制热路径）。
  JointAngles calJointPoses(const JointAngles& q_cur);

 private:
  std::array<bool, 6> sameLegCollisionFree(
      const std::array<std::array<Vector3d, 5>, 6>& collision_points) const;

  ControlPaths paths_;
};

}  // namespace grasp_hexapod_control_cpp
