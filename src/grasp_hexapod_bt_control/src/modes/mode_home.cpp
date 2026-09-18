// home 模式实现：松夹爪 → 中止当前运动 → Resetting 平滑回站立。
// Resetting 期间的轨迹推进由调度器统一执行（框架职责，非本模式职责）。
#include "grasp_hexapod_bt_control/modes/mode_home.h"
#include "grasp_hexapod_bt_control/mode_registry.h"

namespace grasp_hexapod_bt_control {
namespace modes {

bool HomeMode::start(const JointAngles& q_cur, std::string& reason) {
  // ① 先松开夹爪（结果记入最终响应；失败不阻断回正——到站姿永远优先）。
  gripper_result_ = ctx().callGripper("open");
  if (!gripper_result_.first) {
    ROS_ERROR("home gripper open failed: %s (continue to reset)",
              gripper_result_.second.c_str());
  }
  // ② 中止当前运动并平滑回正（mission.cancel + abortClimb + exitDock +
  // reset_active=false + 置 Resetting）。
  ctx().abortActiveMotion("BT home");
  reason.clear();
  return true;
}

std::optional<JointAngles> HomeMode::step(const ModeStepInput& input) {
  // Resetting 阶段由调度器推进；本模式无额外每帧动作。
  (void)input;
  return ctx().controller().q_des;
}

std::optional<ModeOutcome> HomeMode::checkTerminal() {
  // 回正完成（Hold 且复位结束）后返回；夹爪结果即最终结果。
  if (ctx().machineState() != MachineState::Hold ||
      ctx().controller().reset_active) {
    return std::nullopt;
  }
  if (gripper_result_.first) {
    return ModeOutcome{ModeResult::Success, gripper_result_.second};
  }
  return ModeOutcome{ModeResult::Failure, gripper_result_.second};
}

REGISTER_MODE("home", HomeMode)

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
