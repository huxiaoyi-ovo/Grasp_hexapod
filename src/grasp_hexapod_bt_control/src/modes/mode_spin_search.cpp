// spin_search keeps its legacy timed fake by default.  The opt-in visual
// branch rotates from XiaolanTarget observations and reports only a coarse
// visual lock; it never authorizes climbing.
#include "grasp_hexapod_bt_control/modes/mode_spin_search.h"
#include "grasp_hexapod_bt_control/mode_registry.h"
#include <xiaolan_yolo_detection/XiaolanTarget.h>
#include <xiaolan_yolo_detection/PerceptionStatus.h>

namespace grasp_hexapod_bt_control {
namespace modes {

SpinSearchMode::SpinSearchMode(ModeContext& ctx, const ModeTopics& topics)
    : ModeBase("spin_search", ctx, topics),
      fake_duration_s_(ctx.privateNodeHandle().param(
          "spin_search_fake_duration_s", 5.0)),
      visual_enabled_(
          ctx.privateNodeHandle().param("visual_guidance_enabled", false)),
      visual_(loadVisualGuidanceConfig(ctx.privateNodeHandle())) {
  if (visual_enabled_) {
    target_subscriber_ = ctx.nodeHandle().subscribe<xiaolan_yolo_detection::XiaolanTarget>(
        topics.get("target", "/xiaolan/target"), 1,
        [this](const xiaolan_yolo_detection::XiaolanTarget::ConstPtr& message) {
          visual_.cache({static_cast<bool>(message->detected),
                         message->x_target,
                         message->z_target,
                         message->header.stamp.toSec(),
                         message->header.frame_id});
        });
    health_subscriber_ = ctx.nodeHandle().subscribe<xiaolan_yolo_detection::PerceptionStatus>(
        topics.get("perception_status", "/xiaolan/perception_status"), 1,
        [this](const xiaolan_yolo_detection::PerceptionStatus::ConstPtr& message) {
          VisualHealth health;
          health.state = message->state;
          health.reason = message->reason;
          health.frame = message->header.frame_id;
          health.stamp = message->header.stamp.toSec();
          health.rgb_stamp = message->rgb_stamp.toSec();
          health.depth_stamp = message->depth_stamp.toSec();
          health.detection_hz = message->detection_hz;
          health.depth_hz = message->depth_hz;
          health.target_hz = message->target_hz;
          health.inference_ms = message->inference_ms;
          visual_.cacheHealth(health);
        });
  }
}

bool SpinSearchMode::start(const JointAngles& q_cur, std::string& reason) {
  if (visual_enabled_) {
    GraspController& controller = ctx().controller();
    controller.mission.cancel("BT visual spin search");
    controller.abortClimb();
    controller.exitDock(q_cur);
    controller.setMode(GraspController::kApproach);
    controller.approach_mode.cancelAutonomousApproach("BT visual spin search");
    controller.approach_mode.approach_plan.ready_for_climb = false;
    failure_started_at_ = 0.0;
    visual_.begin(VisualState::Searching, nowSeconds());
    ROS_INFO("visual spin_search waiting for fresh perception status");
    ctx().setMachineState(MachineState::Running);
    reason.clear();
    return true;
  }
  (void)q_cur;
  fake_started_at_ = nowSeconds();
  ctx().setMachineState(MachineState::Running);
  reason.clear();
  return true;
}

std::optional<JointAngles> SpinSearchMode::step(const ModeStepInput& input) {
  if (visual_enabled_) {
    const auto& approach = ctx().controller().approach_mode;
    const bool gait_stopped =
        !approach.gait_started && !approach.transfer_active_;
    const Eigen::Vector4d command = visual_.update(input.now, gait_stopped);
    visual_.report(input.now, name(), command, gait_stopped);
    return ctx().controller().update(input.q_cur, command);
  }
  // 假实现：零速保持，不动机器。
  return ctx().controller().update(input.q_cur, Eigen::Vector4d::Zero());
}

std::optional<ModeOutcome> SpinSearchMode::checkTerminal() {
  if (visual_enabled_) {
    const auto& approach = ctx().controller().approach_mode;
    const bool drained = !approach.gait_started && !approach.transfer_active_;
    if (visual_.state() == VisualState::Failed) {
      if (!failure_started_at_) {
        failure_started_at_ = nowSeconds();
      }
      if (drained || nowSeconds() - failure_started_at_ >= 3.0) {
        ctx().setMachineState(MachineState::Hold);
        return ModeOutcome{
            ModeResult::Failure,
            visual_.reason() + (drained ? "" : " (zero-drain timeout)")};
      }
    } else if (visual_.state() == VisualState::Success && drained) {
      ctx().setMachineState(MachineState::Hold);
      return ModeOutcome{ModeResult::Success,
                         "visual search lock completed; coarse approach pending"};
    }
    return std::nullopt;
  }
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

void SpinSearchMode::resume() {
  ModeBase::resume();
  if (visual_enabled_) {
    visual_.resume(nowSeconds());
  }
}

bool SpinSearchMode::onNodeHold(const std::string& reason) {
  if (!visual_enabled_) {
    return false;
  }
  ROS_WARN("visual spin_search stopped: %s", reason.c_str());
  finish(ModeResult::Failure, "visual spin search stopped: " + reason);
  return false;
}

REGISTER_MODE("spin_search", SpinSearchMode)

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
