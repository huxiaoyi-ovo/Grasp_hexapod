// approach 模式实现：假实现（定时成功）与真实自动接近的双分支。
#include "grasp_hexapod_bt_control/modes/mode_approach.h"
#include "grasp_hexapod_bt_control/mode_registry.h"

namespace grasp_hexapod_bt_control {
namespace modes {

using grasp_hexapod_control_cpp::ApproachPlan;

ApproachMode::ApproachMode(ModeContext& ctx, const ModeTopics& topics)
    : ModeBase("approach", ctx, topics),
      fake_(ctx.privateNodeHandle().param("approach_fake", true)),
      fake_duration_s_(
          ctx.privateNodeHandle().param("approach_fake_duration_s", 5.0)),
      control_source_(
          ctx.privateNodeHandle().param<std::string>("control_source",
                                                     "teleop")) {}

bool ApproachMode::start(const JointAngles& q_cur, std::string& reason) {
  if (fake_) {
    // 假实现：不动机器、不看导航，保持当前目标定时返回成功。
    fake_started_at_ = nowSeconds();
    ctx().setMachineState(MachineState::Running);
    reason.clear();
    return true;
  }

  // ── 真实实现（~approach_fake:=false）──
  if (control_source_ != "navigation") {
    reason = "approach requires ~control_source:=navigation";
    return false;
  }
  GraspController& controller = ctx().controller();
  controller.mission.cancel("BT approach");
  controller.abortClimb();
  controller.exitDock(q_cur);
  controller.setMode(GraspController::kApproach);
  const ApproachPlan result =
      controller.startAutonomousApproach(ctx().inputs().navigationSnapshot());
  if (result.failed) {
    reason = result.reason;
    return false;
  }
  ctx().setMachineState(MachineState::Running);
  reason.clear();
  return true;
}

std::optional<JointAngles> ApproachMode::step(const ModeStepInput& input) {
  if (fake_) {
    // 假实现：零速保持（不下发任何步态推进）。
    return ctx().controller().update(input.q_cur, Eigen::Vector4d::Zero());
  }
  // 真实实现：每帧取导航快照（含边界新鲜度要求），由控制器内部完成
  // 足端规划与 IK。
  NavigationState navigation_state = ctx().inputs().navigationSnapshot();
  return ctx().controller().update(
      input.q_cur, Eigen::Vector4d::Zero(), &navigation_state, nullptr);
}

std::optional<ModeOutcome> ApproachMode::checkTerminal() {
  if (fake_) {
    const double elapsed = nowSeconds() - fake_started_at_;
    if (elapsed >= fake_duration_s_) {
      ctx().setMachineState(MachineState::Hold);
      return ModeOutcome{
          ModeResult::Success,
          "fake approach completed after " +
              std::to_string(static_cast<int>(elapsed)) + "s"};
    }
    return std::nullopt;
  }

  const ApproachPlan& plan = ctx().controller().approach_mode.approach_plan;
  if (plan.failed) {
    return ModeOutcome{ModeResult::Failure, plan.reason};
  }
  if (plan.ready_for_climb) {
    ctx().setMachineState(MachineState::Hold);
    return ModeOutcome{ModeResult::Success, "ready for climb"};
  }
  return std::nullopt;
}

REGISTER_MODE("approach", ApproachMode)

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
