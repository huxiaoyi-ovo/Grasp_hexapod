// 模式开发模板实现（不参与编译）。占位符替换说明见 mode_template.h。
#include "grasp_hexapod_bt_control/modes/mode_<name>.h"
#include "grasp_hexapod_bt_control/mode_registry.h"

namespace grasp_hexapod_bt_control {
namespace modes {

NameMode::NameMode(ModeContext& ctx, const ModeTopics& topics)
    : ModeBase("__NAME__", ctx, topics) {
  // 读自己的参数（私有命名空间，其他模式不可见）：
  //   max_speed_ = ctx.privateNodeHandle().param("my_max_speed", 0.1);
  // 创建自己的订阅（话题名来自 mode_topics.yaml 的 __NAME__ 节）：
  //   subscriber_ = ctx.nodeHandle().subscribe<MyMsg>(
  //       topics.get("my_input"), 1,
  //       [this](const MyMsg::ConstPtr& m) { myCallback(m); });
  // 回调里只做"加锁缓存最新消息"，计算放 step()。
}

bool NameMode::start(const JointAngles& q_cur, std::string& reason) {
  // 入口门控：不满足返回 false + reason（reason 会原样回给行为树）。
  // if (!condition) { reason = "why rejected"; return false; }
  // 进入模式：调用控制核心原语、记录起点、置 Running：
  // ctx().controller().setMode(...);
  ctx().setMachineState(MachineState::Running);
  (void)q_cur;
  reason.clear();
  return true;
}

std::optional<JointAngles> NameMode::step(const ModeStepInput& input) {
  // 每帧推进（120Hz 控制线程，绝不阻塞）：
  // 1. 取自己的缓存输入；2. 计算；3. 调 controller().update(...) 得关节目标。
  // 需要导航/IMU 用 ctx().inputs().navigationSnapshot()/imu()。
  return ctx().controller().update(input.q_cur, Eigen::Vector4d::Zero());
}

std::optional<ModeOutcome> NameMode::checkTerminal() {
  // 到达终态：先做收尾副作用（夹爪、切 Hold…），再返回终态。
  // if (done) {
  //   ctx().setMachineState(MachineState::Hold);
  //   return ModeOutcome{ModeResult::Success, "describe result"};
  // }
  // 也可以在 start/step 内直接 finish(...)，这里返回 nullopt。
  return std::nullopt;
}

// 文件末尾一行完成自注册：调度器启动时自动发现，无需改任何其他文件。
REGISTER_MODE("__NAME__", NameMode)

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
