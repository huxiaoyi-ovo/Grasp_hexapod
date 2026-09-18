// dock 模式：圆环视觉对接 + 末端夹爪闭合。
// 迁移自 bt_control_node.cpp 的 ensureDockMode/startRealDock/dockLockConfirmed
// 与 updateControl 的 dock 帧分支、finishBtModeIfTerminal 的 dock 终态。
// lock_confirmed 订阅（BoolInput）、RingPerceptionRos（进程内圆环识别，
// 订阅 /dock_camera/image_raw）与圆环状态机 grasp_hexapod_ring::DockMode
// 实例均由本模式自持；首次 start 时惰性创建，接线方式与 run_real_ring_cpp
// 的 ensureDockMode 一致（经 RingController::attachDockMode 附接）。
#pragma once

#include "grasp_hexapod_bt_control/io_inputs.h"
#include "grasp_hexapod_bt_control/mode_base.h"

#include "grasp_hexapod_control_cpp/ring_dock_mode.h"
#include "grasp_hexapod_control_cpp/ring_perception_ros.h"

#include <memory>
#include <string>

namespace grasp_hexapod_bt_control {
namespace modes {

class DockMode : public ModeBase {
 public:
  DockMode(ModeContext& ctx, const ModeTopics& topics);
  ~DockMode() override;

  bool start(const JointAngles& q_cur, std::string& reason) override;
  std::optional<JointAngles> step(const ModeStepInput& input) override;
  std::optional<ModeOutcome> checkTerminal() override;
  // 节点级安全保持：对接中直接 failExecution 进 HOLD（不走通用路径）。
  bool onNodeHold(const std::string& reason) override;

 private:
  void ensureRingDock();
  std::optional<bool> dockLockConfirmed();

  // ── ~ 参数（原 loadParameters/ensureDockMode 的 dock 段）。──
  bool enable_real_dock_ = true;
  std::string dock_system_config_;
  bool require_real_calibrated_ = true;
  bool allow_uncalibrated_ = false;
  double lock_confirmation_max_age_ = 0.5;

  double session_started_at_ = 0.0;
  bool clamped_ = false;  // 本次调用是否已执行末端 clamp（原 dock_clamped）
  std::unique_ptr<BoolInput> lock_confirmation_;
  std::unique_ptr<grasp_hexapod_ring::RingPerceptionRos> perception_;
  std::unique_ptr<grasp_hexapod_ring::DockMode> ring_dock_;
};

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
