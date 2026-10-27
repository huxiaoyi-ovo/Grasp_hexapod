// spin_search 模式：圆周搜索小蓝——当前为假实现（定时成功）。
//
// 真实实现（原地自转 + 感知搜索）落地时：替换本类的 start/step/checkTerminal
// 内容即可，注册宏与 mode_topics.yaml 节不用改。假实现参数：
//   ~spin_search_fake_duration_s（默认 5.0，到时返回成功）
// 行为树回收链（RTK 门控 → spin_search → approach → climb → dock）在
// 导航/感知栈未就绪时借此整链联调。
#pragma once

#include "grasp_hexapod_bt_control/mode_base.h"

namespace grasp_hexapod_bt_control {
namespace modes {

class SpinSearchMode : public ModeBase {
 public:
  SpinSearchMode(ModeContext& ctx, const ModeTopics& topics);

  bool start(const JointAngles& q_cur, std::string& reason) override;
  std::optional<JointAngles> step(const ModeStepInput& input) override;
  std::optional<ModeOutcome> checkTerminal() override;

 private:
  double fake_duration_s_ = 5.0;
  double fake_started_at_ = 0.0;
};

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
