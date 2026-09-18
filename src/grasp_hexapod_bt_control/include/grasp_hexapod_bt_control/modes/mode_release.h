// release 模式：仅松开夹爪（无运动、免门控）。
// 可从任意机器状态（含 WAIT_B）直接调用；与其他活动模式互斥由服务层
// 抢占语义保证（旧调用返回 preempted by release）。
#pragma once

#include "grasp_hexapod_bt_control/mode_base.h"

namespace grasp_hexapod_bt_control {
namespace modes {

class ReleaseMode : public ModeBase {
 public:
  ReleaseMode(ModeContext& ctx, const ModeTopics& topics)
      : ModeBase("release", ctx, topics) {}

  // 免门控：任意状态直接执行（松夹爪对安全无前置要求）。
  bool startFromAnyState() const override { return true; }

  bool start(const JointAngles& q_cur, std::string& reason) override;
  std::optional<JointAngles> step(const ModeStepInput& input) override;
  std::optional<ModeOutcome> checkTerminal() override;
};

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
