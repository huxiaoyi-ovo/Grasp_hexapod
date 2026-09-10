// Simulation-only compact 攀爬轨迹执行器（对应 Python climb_mode.py::ClimbMode）。
//
// 只播放固定阶段并更新机身/足端参考；不把时间或运动学误差解释为真实接触、
// 承载或稳定性，也不授权实机攀爬。
#pragma once

#include "grasp_hexapod_control_cpp/climb_compact.h"
#include "grasp_hexapod_control_cpp/types.h"

#include <chrono>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace grasp_hexapod_control_cpp {

class GraspController;

// 仅供回放归因：每个 RUNNING 控制周期只能属于一个类别，
// 不参与相位推进、门限或任何实机控制决策。
struct ClimbStageTiming {
  double planned_motion_s = 0.0;
  double effective_settle_required_s = 0.0;
  double actual_stage_elapsed_s = 0.0;
  double motion_progress_s = 0.0;
  double internal_checkpoint_feedback_hold_s = 0.0;
  double terminal_persistence_wait_s = 0.0;
  double terminal_feedback_hold_s = 0.0;
  double collision_guard_hold_s = 0.0;
  double stage_transition_control_cycle_s = 0.0;
};

enum class ClimbState { Idle, Running, Hold, Done, Failed };

class ClimbMode {
 public:
  explicit ClimbMode(GraspController* controller);

  // 读取 compact 攀爬配置（每次进入攀爬重新读盘，与 Python 语义一致）。
  ClimbCompactConfig loadConfig() const;

  // 返回配置中明确保存的攀爬末帧关节姿态。
  JointAngles terminalJoints(const std::optional<ClimbCompactConfig>& config) const;

  void validateConfig(const ClimbCompactConfig& config) const;

  // 进入 compact 攀爬预览；参数与 Python enter() 一一对应。
  void enter(const JointAngles& q_current,
             const std::optional<ClimbCompactConfig>& config, int start_stage_index,
             std::optional<int> end_stage_index, bool hardware_execution);

  void hold();
  void resume();

  // 推进当前攀爬预览阶段；command 不参与攀爬插值。
  void update(const Eigen::Vector4d& command, const JointAngles& q_current);

  // 诊断摘要（实机暂停/失败时直接定位反馈误差）。
  std::string trackingDiagnosticSummary() const;
  std::string activeLegDiagnosticSummary() const;

  // enter() 之后保存的当前 compact 配置（对应 Python 的 self.config）。
  const ClimbCompactConfig& currentConfig() const { return config_; }

  ClimbState state = ClimbState::Idle;
  std::string phase;          // 当前阶段名
  double phase_time = 0.0;
  double stage_elapsed_time = 0.0;
  double settle_time = 0.0;
  std::string failure_reason;
  std::optional<std::array<Vector3d, 6>> anchors_world;
  std::optional<std::array<double, 5>> base_pose;
  std::optional<int> stage_index;
  std::optional<int> end_stage_index;
  std::vector<std::string> stage_names;

  // 跟踪诊断（只读观测，不参与控制）。
  double last_tracking_error_rad = 0.0;
  double last_foot_target_error_m = 0.0;
  std::array<double, 6> last_foot_errors_m{};
  FootPositions last_actual_foot_base_m{};
  FootPositions last_desired_foot_base_m{};
  std::array<Vector3d, 6> last_foot_error_xyz_m{};
  double last_diagnostic_phase_time_s = 0.0;
  double last_diagnostic_stage_elapsed_s = 0.0;
  std::string last_diagnostic_stage_name;
  double last_diagnostic_stage_duration_s = 0.0;
  std::vector<int> last_diagnostic_active_legs;
  std::string last_worst_foot_leg;
  double last_worst_foot_error_m = 0.0;
  std::vector<std::pair<std::string, double>> last_feet_over_gate;
  JointAngles last_joint_tracking_errors_rad{};
  std::string last_worst_joint_leg;
  std::string last_worst_joint_name;
  double last_worst_joint_error_rad = 0.0;
  double last_worst_joint_q_current_rad = 0.0;
  double last_worst_joint_q_des_rad = 0.0;
  std::vector<std::pair<std::string, double>> last_joints_over_tracking_gate;
  bool last_settled = false;
  bool last_phase_hold = false;
  bool last_collision_guard_hold = false;
  std::map<int, ClimbStageTiming> stage_timing_by_index;
  bool hardware_execution = false;

 private:
  static double smoothstep(double value);

  void updateHardwareStageElapsedTime();
  void applyReference(const std::array<double, 5>& base,
                      const std::array<Vector3d, 6>& anchors,
                      bool sync_previous = false);
  // 返回 {pose, anchors, total}。
  std::tuple<std::array<double, 5>, std::array<Vector3d, 6>, double>
  stageReference() const;
  std::array<Vector3d, 6> relativeBaseHighStep(const CompactStage& stage,
                                               const std::array<double, 5>& pose,
                                               double pose_weight, double phase) const;
  std::array<Vector3d, 6> piecewiseBase(const CompactStage& stage,
                                        const std::array<double, 5>& pose) const;
  static std::array<Vector3d, 6> piecewiseAnchors(
      const std::vector<std::array<Vector3d, 6>>& knots,
      const std::vector<double>& durations, double elapsed);
  static std::vector<Vector3d> piecewiseSlots(
      const std::vector<std::vector<Vector3d>>& knots,
      const std::vector<double>& durations,
      const std::vector<std::vector<Vector3d>>* velocities, double elapsed);
  void advanceStage();
  void startStageTiming();
  void recordStageTiming(const std::string& category);
  double effectiveSettleRequired(const CompactStage& stage) const;
  void updateTrackingDiagnostics(const JointAngles& q_current);
  std::string trackingFailureReason() const;

  GraspController* controller_;
  ClimbCompactConfig config_;
  std::optional<std::chrono::steady_clock::time_point>
      hardware_stage_last_monotonic_;
};

}  // namespace grasp_hexapod_control_cpp
