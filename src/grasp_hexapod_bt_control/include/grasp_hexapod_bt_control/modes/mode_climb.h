// climb 模式：紧凑攀爬（compact climb）。
// 迁移自 bt_control_node.cpp 的 startRealClimb、攀爬相对运动监控
// （realClimbObservation/monitorRealClimb）、速度诊断聚合
// （record/flushRealClimbSpeedDiagnostic）与 updateControl 的攀爬帧分支。
// IMU/导航起点缓存与诊断状态为本模式私有。
#pragma once

#include "grasp_hexapod_bt_control/mode_base.h"

#include <array>
#include <memory>
#include <string>

namespace grasp_hexapod_bt_control {
namespace modes {

class ClimbMode : public ModeBase {
 public:
  ClimbMode(ModeContext& ctx, const ModeTopics& topics);

  bool start(const JointAngles& q_cur, std::string& reason) override;
  std::optional<JointAngles> step(const ModeStepInput& input) override;
  std::optional<ModeOutcome> checkTerminal() override;

  // HOLD 租约暂停时冻结攀爬推进（保留内部阶段状态），恢复时继续。
  void pause(const std::string& reason) override;
  void resume() override;
  // 节点级安全保持（反馈丢失等）：冲掉速度诊断后交回通用路径。
  bool onNodeHold(const std::string& reason) override;

 private:
  // 实机攀爬速度诊断聚合（observation-only，从不参与门控）。
  struct SpeedDiagnostic {
    std::string stage;
    JointAngles previous_q_cur{};
    JointAngles previous_q_des{};
    std::array<double, 6> previous_feedback_stamp{};
    double previous_sample_time = 0.0;
    double peak_command_speed_rad_s = 0.0;
    double peak_measured_speed_rad_s = 0.0;
    double peak_tracking_error_rad = 0.0;
    double ratio_sum = 0.0;
    int ratio_count = 0;
    int velocity_limit_clip_count = 0;
    int collision_guard_hold_count = 0;
  };

  std::pair<bool, std::string> realClimbObservation();
  void monitorRealClimb();
  void resetSpeedDiagnostic() { speed_diagnostic_.reset(); }
  void flushSpeedDiagnostic(const std::string& reason);
  void recordSpeedDiagnostic(const std::string& stage,
                             const JointAngles& q_cur,
                             const JointAngles& q_des,
                             const std::array<double, 6>& feedback_stamp,
                             double sample_time);
  void warnHardwareClimbPhaseHold();
  void infoHardwareClimbActiveTrace();

  // ── ~ 参数（原 loadParameters 的攀爬段）。──
  bool enable_real_climb_ = false;
  std::string climb_side_ = "left";
  double climb_foot_gate_m_ = 0.02;
  int persistence_frames_ = 3;
  double max_position_error_ = 0.05;
  double max_orientation_error_ = 10.0 * M_PI / 180.0;
  double max_angular_speed_ = 30.0 * M_PI / 180.0;

  // ── 攀爬起点参考与监控状态。──
  NavigationState climb_start_navigation_;
  bool have_climb_start_navigation_ = false;
  Matrix3d climb_start_imu_rotation_ = Matrix3d::Identity();
  bool have_climb_start_imu_ = false;
  Matrix4d climb_start_planned_pose_ = Matrix4d::Identity();
  bool have_climb_start_planned_ = false;
  bool monitor_active_ = false;
  int bad_frames_ = 0;
  int good_frames_ = 0;
  std::unique_ptr<SpeedDiagnostic> speed_diagnostic_;
};

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
