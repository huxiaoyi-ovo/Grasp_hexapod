// approach keeps its legacy fake/RTK branches by default.  Its opt-in visual
// branch is a camera-guided coarse approach; without ~tag_guidance_enabled it
// never sets ready_for_climb.  With it, an AprilTag handoff (tag_guidance.h)
// takes over and ready_for_climb is set only after tag alignment completes.
#include "grasp_hexapod_bt_control/modes/mode_approach.h"
#include "grasp_hexapod_bt_control/mode_registry.h"
#include <xiaolan_yolo_detection/XiaolanTarget.h>
#include <xiaolan_yolo_detection/PerceptionStatus.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

namespace grasp_hexapod_bt_control {
namespace modes {

using grasp_hexapod_control_cpp::ApproachPlan;

namespace {
// Only validated when tag guidance is enabled, so a disabled run never fails
// on tag parameters.  Reference pose defaults are placeholders: teach them.
TagGuidanceConfig loadTagGuidanceConfig(ros::NodeHandle& p, bool enabled) {
  TagGuidanceConfig c;
  if (!enabled) return c;
  c.tag_id = p.param("tag_guidance_id", 0);
  c.confirm_frames = p.param("tag_handoff_confirm_frames", 5);
  c.max_age_s = p.param("tag_max_age_s", .25);
  c.expected_frame = p.param<std::string>("visual_expected_target_frame", "camera_color_optical_frame");
  c.ref_x_m = p.param("tag_ref_x_m", 0.0);
  c.ref_z_m = p.param("tag_ref_z_m", 0.15);
  c.ref_yaw_rad = p.param("tag_ref_yaw_rad", 0.0);
  c.lateral_gain = p.param("tag_lateral_gain", .5);
  c.forward_gain = p.param("tag_forward_gain", .5);
  c.yaw_gain = p.param("tag_yaw_gain", .8);
  c.max_lateral_speed_m_s = p.param("tag_max_lateral_speed_m_s", .02);
  c.max_forward_speed_m_s = p.param("tag_max_forward_speed_m_s", .02);
  c.max_yaw_rate = p.param("tag_max_yaw_rate", .10);
  c.position_tolerance_m = p.param("tag_position_tolerance_m", .01);
  c.yaw_tolerance_rad = p.param("tag_yaw_tolerance_rad", .035);
  c.settle_s = p.param("tag_settle_s", .5);
  c.loss_timeout_s = p.param("tag_loss_timeout_s", 1.);
  c.align_timeout_s = p.param("tag_align_timeout_s", 60.);
  return c;
}
}  // namespace

ApproachMode::ApproachMode(ModeContext& ctx, const ModeTopics& topics)
    : ModeBase("approach", ctx, topics),
      fake_(ctx.privateNodeHandle().param("approach_fake", true)),
      fake_duration_s_(
          ctx.privateNodeHandle().param("approach_fake_duration_s", 5.0)),
      control_source_(
          ctx.privateNodeHandle().param<std::string>("control_source",
                                                     "teleop")),
      visual_enabled_(
          ctx.privateNodeHandle().param("visual_guidance_enabled", false)),
      visual_(loadVisualGuidanceConfig(ctx.privateNodeHandle())),
      tag_enabled_(visual_enabled_ &&
                   ctx.privateNodeHandle().param("tag_guidance_enabled", false)),
      tag_(loadTagGuidanceConfig(ctx.privateNodeHandle(), tag_enabled_)) {
  if (tag_enabled_) {
    // Callback only caches the detection matching the configured id.
    tag_subscriber_ = ctx.nodeHandle().subscribe<apriltag_ros::AprilTagDetectionArray>(
        topics.get("tag_detections", "/xiaolan/tag_detections"), 1,
        [this](const apriltag_ros::AprilTagDetectionArray::ConstPtr& message) {
          for (const auto& detection : message->detections) {
            if (detection.id.size() != 1 || detection.id[0] != tag_.tagId()) {
              continue;
            }
            const auto& pose = detection.pose.pose.pose;
            const bool own_header = !detection.pose.header.stamp.isZero();
            const auto& header = own_header ? detection.pose.header : message->header;
            tag_.cache({tag_.tagId(), pose.position.x, pose.position.y,
                        pose.position.z, pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w,
                        header.stamp.toSec(), header.frame_id});
            return;
          }
        });
  }
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

bool ApproachMode::start(const JointAngles& q_cur, std::string& reason) {
  if (visual_enabled_) {
    GraspController& controller = ctx().controller();
    controller.mission.cancel("BT visual approach");
    controller.abortClimb();
    controller.exitDock(q_cur);
    controller.setMode(GraspController::kApproach);
    controller.approach_mode.cancelAutonomousApproach("BT visual approach");
    controller.approach_mode.approach_plan.ready_for_climb = false;
    failure_started_at_ = 0.0;
    visual_.begin(VisualState::Approaching, nowSeconds());
    if (tag_enabled_) {
      tag_.begin(nowSeconds());
    }
    ROS_INFO("visual approach waiting for fresh perception status");
    ctx().setMachineState(MachineState::Running);
    reason.clear();
    return true;
  }
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
  if (visual_enabled_) {
    const auto& approach = ctx().controller().approach_mode;
    const bool gait_stopped =
        !approach.gait_started && !approach.transfer_active_;
    if (tag_enabled_) {
      const bool was_handed_off = tag_.handedOff();
      const Eigen::Vector4d tag_command = tag_.update(input.now, gait_stopped);
      if (tag_.handedOff()) {
        // After handoff YOLO observations are ignored for control.
        if (!was_handed_off) {
          ROS_INFO("approach: AprilTag handoff (id %d), coarse YOLO guidance ignored",
                   tag_.tagId());
        }
        const Eigen::Vector3d error = tag_.lastError();
        ROS_INFO_THROTTLE(2.0,
                          "tag mode=%s state=%d ex=%.3f ez=%.3f eyaw=%.3f cmd_right=%.3f cmd_forward=%.3f cmd_yaw=%.3f gait_stopped=%d",
                          name().c_str(), static_cast<int>(tag_.state()), error[0],
                          error[1], error[2], tag_command[0], tag_command[1],
                          tag_command[3], gait_stopped ? 1 : 0);
        return ctx().controller().update(input.q_cur, tag_command);
      }
    }
    const Eigen::Vector4d command = visual_.update(input.now, gait_stopped);
    visual_.report(input.now, name(), command, gait_stopped);
    return ctx().controller().update(input.q_cur, command);
  }
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
  if (visual_enabled_) {
    const auto& approach = ctx().controller().approach_mode;
    const bool drained = !approach.gait_started && !approach.transfer_active_;
    if (tag_enabled_ && tag_.handedOff()) {
      if (tag_.state() == TagState::Failed) {
        if (!failure_started_at_) {
          failure_started_at_ = nowSeconds();
        }
        if (drained || nowSeconds() - failure_started_at_ >= 3.0) {
          ctx().setMachineState(MachineState::Hold);
          return ModeOutcome{
              ModeResult::Failure,
              tag_.reason() + (drained ? "" : " (zero-drain timeout)")};
        }
      } else if (tag_.state() == TagState::Success && drained) {
        ctx().controller().approach_mode.approach_plan.ready_for_climb = true;
        ctx().setMachineState(MachineState::Hold);
        return ModeOutcome{ModeResult::Success,
                           "tag alignment complete: ready for climb"};
      }
      return std::nullopt;
    }
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
      if (tag_enabled_) {
        return ModeOutcome{ModeResult::Failure,
                           "coarse stop reached without tag handoff"};
      }
      return ModeOutcome{ModeResult::Success,
                         "coarse approach completed (AprilTag alignment pending)"};
    }
    return std::nullopt;
  }
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

void ApproachMode::resume() {
  ModeBase::resume();
  if (visual_enabled_) {
    visual_.resume(nowSeconds());
    if (tag_enabled_) {
      tag_.resume(nowSeconds());
    }
  }
}

bool ApproachMode::onNodeHold(const std::string& reason) {
  if (!visual_enabled_) {
    return false;
  }
  ROS_WARN("visual approach stopped: %s", reason.c_str());
  finish(ModeResult::Failure, "visual approach stopped: " + reason);
  return false;
}

REGISTER_MODE("approach", ApproachMode)

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
