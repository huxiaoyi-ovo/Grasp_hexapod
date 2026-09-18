// spin_search 模式实现：假实现——Running 保持 spin_search_fake_duration_s
// 后返回成功（真实圆周搜索落地时替换三个函数内容）。
#include "grasp_hexapod_bt_control/modes/mode_spin_search.h"
#include "grasp_hexapod_bt_control/mode_registry.h"

namespace grasp_hexapod_bt_control {
namespace modes {

SpinSearchMode::SpinSearchMode(ModeContext& ctx, const ModeTopics& topics)
    : ModeBase("spin_search", ctx, topics),
      fake_duration_s_(ctx.privateNodeHandle().param(
          "spin_search_fake_duration_s", 5.0)) {}

bool SpinSearchMode::start(const JointAngles& q_cur, std::string& reason) {
  (void)q_cur;
  fake_started_at_ = nowSeconds();
  ctx().setMachineState(MachineState::Running);
  reason.clear();
  return true;
}

std::optional<JointAngles> SpinSearchMode::step(const ModeStepInput& input) {
  // 假实现：零速保持，不动机器。
  return ctx().controller().update(input.q_cur, Eigen::Vector4d::Zero());
}

std::optional<ModeOutcome> SpinSearchMode::checkTerminal() {
  const double elapsed = nowSeconds() - fake_started_at_;
  if (elapsed >= fake_duration_s_) {
    ctx().setMachineState(MachineState::Hold);
    return ModeOutcome{
        ModeResult::Success,
        "fake spin_search completed after " +
            std::to_string(static_cast<int>(elapsed)) + "s"};
  }
  return std::nullopt;
}

REGISTER_MODE("spin_search", SpinSearchMode)

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
