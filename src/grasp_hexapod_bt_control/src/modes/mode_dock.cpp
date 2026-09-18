// dock 模式实现：入口门控（实机标定校验）、对接推进、成功后夹爪闭合。
#include "grasp_hexapod_bt_control/modes/mode_dock.h"
#include "grasp_hexapod_bt_control/mode_registry.h"

#include "grasp_hexapod_control_cpp/config_io.h"

#include <exception>

namespace grasp_hexapod_bt_control {
namespace modes {

using grasp_hexapod_control_cpp::DockRobotState;
using grasp_hexapod_control_cpp::DockSystem;
using grasp_hexapod_control_cpp::loadDockSystem;

DockMode::DockMode(ModeContext& ctx, const ModeTopics& topics)
    : ModeBase("dock", ctx, topics) {
  ros::NodeHandle& pnh = ctx.privateNodeHandle();
  enable_real_dock_ = pnh.param("enable_real_dock", true);
  dock_system_config_ = pnh.param<std::string>(
      "dock_system_config",
      ctx.controller().paths().workspace_bounds_csv.substr(
          0, ctx.controller().paths().workspace_bounds_csv.find_last_of('/')) +
          "/dock_system.yaml");
  require_real_calibrated_ = pnh.param("dock_require_real_calibrated", true);
  allow_uncalibrated_ = pnh.param("dock_allow_uncalibrated", false);
  lock_confirmation_max_age_ =
      pnh.param("dock_lock_confirmation_max_age_s", 0.5);
  // 订阅由模式内部完成（话题名来自 mode_topics.yaml dock 节）。
  lock_confirmation_ = std::make_unique<BoolInput>(
      ctx.nodeHandle(), topics.get("lock_confirmed",
                                   "/grasp_hexapod/dock/lock_confirmed"));
}

DockMode::~DockMode() = default;

void DockMode::ensureCoreDock() {
  // 按显式实机请求创建核心 DockMode，不给普通控制器增加 ROS 资源。
  if (core_dock_ != nullptr) {
    return;
  }
  ros::NodeHandle& pnh = ctx().privateNodeHandle();
  perception_ = std::make_unique<grasp_hexapod_control_cpp::DockPerceptionRos>(
      pnh.param("dock_max_perception_age", 0.35),
      pnh.param<std::string>("dock_lock_frame", "dock_lock_center"),
      pnh.param<std::string>("dock_pin_frame_prefix", "dock_pin_from_tag_"),
      dock_system_config_);
  core_dock_ = std::make_unique<grasp_hexapod_control_cpp::DockMode>(
      &ctx().controller(), perception_.get(),
      pnh.param("dock_require_lock_confirmation", false),
      pnh.param("dock_linear_speed_m_s", 0.05),
      pnh.param("dock_update_rate_hz", 30.0),
      pnh.param("dock_perception_rate_hz", 10.0),
      pnh.param("dock_leg_lift_speed_m_s", 0.05),
      pnh.param("dock_sit_settle_duration_s", 0.5),
      pnh.param("dock_leg_lift_height_m", 0.06),
      pnh.param("dock_leg_lift_level_tolerance_m", 0.003),
      [](bool is_warn, const std::string& message) {
        if (is_warn) {
          ROS_WARN("%s", message.c_str());
        } else {
          ROS_INFO("%s", message.c_str());
        }
      });
  ctx().controller().attachDockMode(core_dock_.get());
}

bool DockMode::start(const JointAngles& q_cur, std::string& reason) {
  reason = "dock entry rejected";
  if (!enable_real_dock_) {
    ROS_WARN_THROTTLE(2.0, "dock rejected: enable_real_dock is false");
    return false;
  }
  if (ctx().machineState() != MachineState::Hold) {
    ROS_WARN_THROTTLE(
        2.0, "dock rejected: reset must finish and controls must be fresh");
    return false;
  }
  try {
    if (require_real_calibrated_ && !allow_uncalibrated_) {
      const DockSystem dock_system = loadDockSystem(dock_system_config_);
      if (!dock_system.real_calibrated) {
        ROS_WARN_THROTTLE(2.0,
                           "dock rejected: dock_system.yaml is not real-calibrated");
        return false;
      }
    }
    ensureCoreDock();
    ctx().controller().enterDock(q_cur);
  } catch (const std::exception& error) {
    ROS_WARN("dock rejected: DockMode entry failed: %s", error.what());
    return false;
  }
  clamped_ = false;
  session_started_at_ = nowSeconds();
  ctx().setMachineState(MachineState::Running);
  ROS_INFO("dock accepted: entering climb terminal posture, then starting visual docking");
  return true;
}

std::optional<bool> DockMode::dockLockConfirmed() {
  // 只接受本次 dock 进入后、未过期的锁紧确认。
  const auto [lock_confirmed, received_at] = lock_confirmation_->snapshotWithTime();
  const double now = nowSeconds();
  const double age = now - received_at;
  if (received_at < session_started_at_ || age < 0.0 ||
      age > lock_confirmation_max_age_) {
    return std::nullopt;
  }
  return lock_confirmed;
}

bool DockMode::onNodeHold(const std::string& reason) {
  // 对接中的节点级保持：failExecution 进 HOLD，不走通用取消路径。
  if (ctx().controller().mode != GraspController::kDock ||
      core_dock_ == nullptr) {
    return false;
  }
  if (core_dock_->active) {
    core_dock_->failExecution("DockMode stopped: " + reason);
  }
  ctx().setMachineState(MachineState::Hold);
  ROS_WARN("DockMode failed into HOLD: %s", reason.c_str());
  return true;
}

std::optional<JointAngles> DockMode::step(const ModeStepInput& input) {
  GraspController& controller = ctx().controller();
  DockRobotState dock_state;
  dock_state.joints = input.q_cur;
  dock_state.lock_confirmed = dockLockConfirmed();
  const JointAngles q_des =
      controller.update(input.q_cur, Eigen::Vector4d::Zero(), nullptr,
                        &dock_state);
  if (controller.mode == GraspController::kDock && core_dock_ != nullptr &&
      grasp_hexapod_control_cpp::DockMode::isTerminalState(core_dock_->state) &&
      ctx().machineState() == MachineState::Running) {
    ctx().setMachineState(MachineState::Hold);
    ROS_INFO("DockMode terminal HOLD: %s", core_dock_->reason.c_str());
  }
  return q_des;
}

std::optional<ModeOutcome> DockMode::checkTerminal() {
  const grasp_hexapod_control_cpp::DockMode* dock = core_dock_.get();
  if (dock == nullptr) {
    return std::nullopt;
  }
  if (dock->state == grasp_hexapod_control_cpp::DockMode::kSuccess &&
      ctx().machineState() == MachineState::Hold) {
    if (!clamped_) {
      clamped_ = true;
      // dock 末端夹持：对接成功后闭合夹爪，结果即本次调用的最终结果。
      const auto [ok, message] = ctx().callGripper("clamp");
      if (!ok) {
        ROS_ERROR("Dock clamp failed: %s", message.c_str());
      } else {
        ROS_INFO("Dock clamp done: %s", message.c_str());
      }
      return ModeOutcome{ok ? ModeResult::Success : ModeResult::Failure,
                         message};
    }
    return std::nullopt;
  }
  if (dock->state == grasp_hexapod_control_cpp::DockMode::kFailed) {
    return ModeOutcome{ModeResult::Failure, dock->reason};
  }
  return std::nullopt;
}

REGISTER_MODE("dock", DockMode)

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
