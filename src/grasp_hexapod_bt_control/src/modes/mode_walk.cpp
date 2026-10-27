// walk 模式实现：/cmd_vel 订阅、限幅缓存、超时安全停步。
#include "grasp_hexapod_bt_control/modes/mode_walk.h"
#include "grasp_hexapod_bt_control/mode_registry.h"

namespace grasp_hexapod_bt_control {
namespace modes {

using grasp_hexapod_control_cpp::kLegCount;

WalkMode::WalkMode(ModeContext& ctx, const ModeTopics& topics)
    : ModeBase("walk", ctx, topics) {
  ros::NodeHandle& pnh = ctx.privateNodeHandle();
  max_linear_speed_ = pnh.param("max_linear_speed", 0.20);
  double foot_radius_sum = 0.0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    foot_radius_sum +=
        ctx.controller().foot_init_base[leg].head<2>().norm();
  }
  const double foot_radius = foot_radius_sum / kLegCount;
  max_yaw_rate_ =
      pnh.param("max_yaw_rate", max_linear_speed_ / foot_radius);
  max_cmd_vel_age_ = pnh.param("max_cmd_vel_age", 0.2);

  // 订阅由模式内部完成；tcpNoDelay 与原节点一致。
  subscriber_ = ctx.nodeHandle().subscribe<geometry_msgs::Twist>(
      topics.get("cmd_vel", "/cmd_vel"), 1,
      [this](const geometry_msgs::Twist::ConstPtr& message) {
        cmdVelCallback(message);
      },
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
}

void WalkMode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& message) {
  // 回调线程只做限幅+缓存；控制帧内消费。
  const Eigen::Vector4d clamped =
      clampCommand(message->linear.y, message->linear.x,
                   message->angular.z, max_linear_speed_, max_yaw_rate_);
  std::lock_guard<std::mutex> guard(cmd_lock_);
  cmd_ = clamped;
  cmd_stamp_ = nowSeconds();
}

bool WalkMode::start(const JointAngles& q_cur, std::string& reason) {
  // 原 startBtRequest walk 分支：清任务/攀爬/对接，切 approach 步态手动模式。
  GraspController& controller = ctx().controller();
  controller.mission.cancel("BT walk");
  controller.abortClimb();
  controller.exitDock(q_cur);
  controller.setMode(GraspController::kApproach);
  controller.approach_mode.cancelAutonomousApproach("BT walk");
  ctx().setMachineState(MachineState::Running);
  reason.clear();
  return true;
}

void WalkMode::onPreFrame(const ModeStepInput& input) {
  // /cmd_vel 过期：安全停步并失败退出（保持先于 HOLD 租约判断的原时序）。
  Eigen::Vector4d cmd;
  double stamp;
  {
    std::lock_guard<std::mutex> guard(cmd_lock_);
    cmd = cmd_;
    stamp = cmd_stamp_;
  }
  const bool fresh = stamp > 0.0 && 0.0 <= input.now - stamp &&
                     input.now - stamp <= max_cmd_vel_age_;
  if (fresh) {
    return;
  }
  GraspController& controller = ctx().controller();
  controller.approach_mode.cancelAutonomousApproach("cmd_vel lost");
  controller.holdClimb();
  ctx().setMachineState(MachineState::Hold);
  ROS_INFO("Motion paused: cmd_vel lost");
  finish(ModeResult::Failure, "cmd_vel lost");
}

std::optional<JointAngles> WalkMode::step(const ModeStepInput& input) {
  Eigen::Vector4d cmd;
  {
    std::lock_guard<std::mutex> guard(cmd_lock_);
    cmd = cmd_;
  }
  return ctx().controller().update(input.q_cur, cmd);
}

std::optional<ModeOutcome> WalkMode::checkTerminal() {
  // walk 不自动结束：持续运行直到被新模式抢占或 /cmd_vel 失效
  // （onPreFrame 中 finish）。
  return std::nullopt;
}

REGISTER_MODE("walk", WalkMode)

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
