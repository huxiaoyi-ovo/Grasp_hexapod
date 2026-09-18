// home 模式：先松开夹爪，再平滑回正到初始站姿。
// 可从任意状态（含 WAIT_B/运行中）进入；夹爪失败仍完成回正（安全优先），
// 但夹爪结果作为最终响应返回（失败透传给行为树）。
#pragma once

#include "grasp_hexapod_bt_control/mode_base.h"

namespace grasp_hexapod_bt_control {
namespace modes {

class HomeMode : public ModeBase {
 public:
  HomeMode(ModeContext& ctx, const ModeTopics& topics)
      : ModeBase("home", ctx, topics) {}

  // home 是全局模式：允许从任意机器状态直接进入。
  bool startFromAnyState() const override { return true; }

  bool start(const JointAngles& q_cur, std::string& reason) override;
  std::optional<JointAngles> step(const ModeStepInput& input) override;
  std::optional<ModeOutcome> checkTerminal() override;

 private:
  std::pair<bool, std::string> gripper_result_{false, "not executed"};
};

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
