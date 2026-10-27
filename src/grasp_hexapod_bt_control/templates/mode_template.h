// =============================================================================
// 模式开发模板 —— 新增模式时把本目录两个文件复制为
//   include/grasp_hexapod_bt_control/modes/mode_<name>.h
//   src/modes/mode_<name>.cpp
// 并全局替换 <name>/<Name>/__NAME__ 三种占位符（见 docs/MODE_DEV_GUIDE.md）。
//
// 接口契约（冻结）：mode_base.h；上下文：mode_context.h；线程规则见两者注释。
// 命名空间：统一放在 grasp_hexapod_bt_control::modes（避免与核心库
// ClimbMode/DockMode/ApproachMode 撞名）。
// 本模板不参与编译（templates/ 不在 CMakeLists 中）。
// =============================================================================
#pragma once

#include "grasp_hexapod_bt_control/mode_base.h"

#include <string>

namespace grasp_hexapod_bt_control {
namespace modes {

// 类名建议：<Name>Mode（如 SpinSearchMode），与文件名 mode_<name> 对应。
class NameMode : public ModeBase {
 public:
  // 构造：读取本模式 ~ 参数、创建本模式话题订阅（名字来自
  // mode_topics.yaml 的 <name> 节）。构造发生在节点启动期，参数错误
  // 抛 ConfigError 会在启动日志直接暴露。绝不在此做重活。
  NameMode(ModeContext& ctx, const ModeTopics& topics)
      : ModeBase("__NAME__", ctx, topics) {}

  // ── 必须实现的三个 ──
  bool start(const JointAngles& q_cur, std::string& reason) override;
  std::optional<JointAngles> step(const ModeStepInput& input) override;
  std::optional<ModeOutcome> checkTerminal() override;

  // ── 可选覆盖（按需打开注释）──
  // void pause(const std::string& reason) override;   // 原地冻结 + 自己的附加动作
  // void resume() override;
  // void stop(bool smooth) override;                  // 显式停止清理
  // bool onNodeHold(const std::string& reason) override;  // 反馈丢失看门狗
  // void onPreFrame(const ModeStepInput& input) override; // 输入健康监视
  // bool startFromAnyState() const override;           // 仅全局模式（如 home/release）

 private:
  // 本模式私有状态（起点缓存、计数器、订阅缓存等）。
  // 订阅回调只写缓存（加锁），计算一律放 step()。
};

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
