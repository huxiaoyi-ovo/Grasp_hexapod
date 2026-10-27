// release 模式实现：gripper_act open 的同步服务化封装（仅松开夹爪）。
#include "grasp_hexapod_bt_control/modes/mode_release.h"
#include "grasp_hexapod_bt_control/mode_registry.h"

namespace grasp_hexapod_bt_control {
namespace modes {

bool ReleaseMode::start(const JointAngles& q_cur, std::string& reason) {
  (void)q_cur;
  // 夹爪结果即本次调用的最终结果；不改变机器状态、不推进步态。
  const auto [ok, message] = ctx().callGripper("open");
  if (!ok) {
    reason = message;
  }
  finish(ok ? ModeResult::Success : ModeResult::Failure, message);
  return true;
}

std::optional<JointAngles> ReleaseMode::step(const ModeStepInput& input) {
  // 无运动；保持当前目标。
  (void)input;
  return ctx().controller().q_des;
}

std::optional<ModeOutcome> ReleaseMode::checkTerminal() {
  // start 内已 finish；这里不再重复给出终态。
  return std::nullopt;
}

REGISTER_MODE("release", ReleaseMode)

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
