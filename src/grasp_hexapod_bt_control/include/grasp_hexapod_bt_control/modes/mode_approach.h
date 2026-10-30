// approach defaults to legacy fake/RTK navigation; opt-in visual guidance is
// a coarse YOLO approach.  With ~tag_guidance_enabled it hands off to AprilTag
// servo (tag_guidance.h) and sets ready_for_climb only after tag alignment;
// without it the visual branch never sets ready_for_climb.
//
// 双分支（~approach_fake 开关，默认 true）：
// - 假实现（导航栈未就绪时用）：跳过 control_source 门控，Running 保持
//   ~approach_fake_duration_s（默认 5.0s）后返回成功——用于行为树整链联调。
// - 真实实现（~approach_fake:=false）：导航快照驱动的自动接近，
//   ready_for_climb 即成功。导航位姿输入经共享 SharedInputs。
#pragma once

#include "grasp_hexapod_bt_control/mode_base.h"
#include "grasp_hexapod_bt_control/tag_guidance.h"
#include "grasp_hexapod_bt_control/visual_guidance.h"

#include <ros/ros.h>

#include <string>

namespace grasp_hexapod_bt_control {
namespace modes {

class ApproachMode : public ModeBase {
 public:
  ApproachMode(ModeContext& ctx, const ModeTopics& topics);

  bool start(const JointAngles& q_cur, std::string& reason) override;
  std::optional<JointAngles> step(const ModeStepInput& input) override;
  std::optional<ModeOutcome> checkTerminal() override;
  void resume() override;
  bool onNodeHold(const std::string& reason) override;

 private:
  bool fake_ = true;           // ~approach_fake（默认 true：假实现）
  double fake_duration_s_ = 5.0;
  double fake_started_at_ = 0.0;
  std::string control_source_;  // ~control_source（真实分支用）
  bool visual_enabled_ = false;
  VisualGuidance visual_{VisualGuidanceConfig()};
  bool tag_enabled_ = false;   // ~tag_guidance_enabled（仅在视觉分支内有效）
  TagGuidance tag_{TagGuidanceConfig()};
  ros::Subscriber tag_subscriber_;
  ros::Subscriber target_subscriber_;
  ros::Subscriber health_subscriber_;
  double failure_started_at_ = 0.0;
};

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
