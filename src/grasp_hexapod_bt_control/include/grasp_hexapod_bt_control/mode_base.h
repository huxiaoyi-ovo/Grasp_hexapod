// 模式统一接口契约（ModeBase）——每个模式一对 mode_<name>.h/.cpp，
// 由不同作者独立编写；本文件是冻结契约，修改需全队同意。
//
// ─────────────────────────── 线程契约（必须遵守） ───────────────────────────
// 1. start / step / checkTerminal / pause / resume / stop / onNodeHold：
//    全部在 120Hz 控制线程调用，绝不阻塞（不 sleep、不等服务、不加长锁）。
//    需要慢操作（如夹爪服务）时用 ctx().callGripper（与原实现一致的短同步）。
// 2. 订阅回调在 AsyncSpinner 线程：只做"加锁缓存最新消息"，不做计算。
// 3. wait / status 在服务线程调用；框架保证其同步正确性，模式不用关心。
// 4. 控制核心 GraspController 的调用只允许发生在 1 中的控制线程上下文里。
// ─────────────────────────────────────────────────────────────────────────────
//
// 生命周期：框架 allocateRun()（清 outcome）→ start()（入口门控+进入）→
// 每帧 step()（推进+返回关节目标）+ checkTerminal()（终态检测）→ 终态后
// 服务线程 wait() 返回。暂停 = pause()/resume() 原地冻结；停止 = stop()。
#pragma once

#include "grasp_hexapod_bt_control/mode_context.h"
#include "grasp_hexapod_bt_control/mode_topic_config.h"

#include <array>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace grasp_hexapod_bt_control {

// 模式执行结果。
enum class ModeResult { Running, Success, Failure, Preempted };

struct ModeOutcome {
  ModeResult result = ModeResult::Running;
  std::string message;
};

// step() 每帧输入（控制线程提供）。
struct ModeStepInput {
  JointAngles q_cur{};   // 本帧六腿关节反馈
  double now = 0.0;      // 当前时间（秒）
  // 六腿反馈时间戳（速度诊断等需要；与 q_cur 同帧）。
  const std::array<double, 6>* feedback_stamp = nullptr;
};

// 模式状态机核心（无 ROS 依赖，便于单测）：run 代次 + 终态 + 等待 + 暂停标志。
// 每次 switch_mode 调用对应一个 run；同模式重复调用/抢占通过 run 代次区分，
// 旧 wait 不会读到新 run 的结果（等价于旧 BtRequest 请求对象替换语义）。
class ModeLifecycle {
 public:
  virtual ~ModeLifecycle() = default;

  // ── 服务线程（框架/调用方使用）──
  // 分配新 run：run 代次 +1、清 outcome、清暂停标志；返回新 run 号。
  // 服务线程在注册新调用时调用（唤醒旧 run 的等待者→superseded）。
  uint64_t allocateRun();
  // 阻塞等待指定 run 的终态：timeout_s<0 无限等；超时返回 Running；
  // run 被更新一代调用取代时返回 Preempted("superseded")。
  ModeOutcome waitForRun(uint64_t run, double timeout_s) const;
  // 非阻塞状态快照。
  ModeOutcome status() const;
  // 当前 run 代次。
  uint64_t currentRun() const;
  // 抢占标记（服务线程）：为当前 run 写入 Preempted 结果并唤醒等待者。
  void markPreempted(const std::string& message);
  bool isPaused() const;

  // ── 模式/框架（控制线程）──
  // 报告终态（幂等：首个终态生效）。可在 start/step/checkTerminal 内调用。
  void finish(ModeResult result, const std::string& message);

 protected:
  void setPaused(bool paused);

 private:
  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  uint64_t run_ = 0;
  std::optional<ModeOutcome> outcome_;
  bool paused_ = false;
};

// 模式统一接口基类：模式作者继承它并实现三个纯虚函数。
class ModeBase : public ModeLifecycle {
 public:
  // name：switch_mode 使用的模式名；ctx：调度器注入的统一上下文；
  // topics：本模式在 mode_topics.yaml 中的节（构造订阅时使用）。
  ModeBase(std::string name, ModeContext& ctx, ModeTopics topics)
      : name_(std::move(name)), ctx_(ctx), topics_(std::move(topics)) {}
  ~ModeBase() override = default;

  // ───────────── 模式作者必须实现的三个函数 ─────────────

  // 启动（非阻塞，120Hz 控制线程）：执行入口门控并进入模式，完成全部后续
  // 动作的"开球"。入口条件不满足时返回 false 并给出 reason（框架会把
  // reason 作为本次调用的失败结果返回给行为树）。
  // 返回 true 表示模式已接管（通常应将机器状态置 Running）。
  virtual bool start(const JointAngles& q_cur, std::string& reason) = 0;

  // 每帧推进（120Hz 控制线程，绝不阻塞）：推进模式并返回本帧 18 关节目标；
  // 返回 nullopt 表示本帧不发布。暂停期间框架不会调用 step（冻结在上一帧
  // 目标），各模式无需自己实现冻结。
  virtual std::optional<JointAngles> step(const ModeStepInput& input) = 0;

  // 每帧终态检测（120Hz 控制线程）：到达终态时先完成收尾副作用（如夹爪、
  // 状态切 Hold），再返回终态；未到终态返回 nullopt。
  // 框架把返回的终态写为本次调用的结果。模式也可在 start/step 中直接
  // finish()，此时 checkTerminal 返回 nullopt 即可。
  virtual std::optional<ModeOutcome> checkTerminal() = 0;

  // ───────────── 可选覆盖（有通用默认实现）───────────────

  // 每帧前置检查（started 后每帧、暂停冻结判断之前调用；保持原节点
  // "cmd_vel 过期检查先于 HOLD 租约" 的时序）。需要输入健康监视的模式
  // （如 walk 的 /cmd_vel 超时）在此自行 finish()。默认无操作。
  virtual void onPreFrame(const ModeStepInput& input) { (void)input; }

  // 默认 false：模式只能从 Hold 进入（WAIT_B/Running 会先走全局门控/回正）。
  // 仅 home 这类"可从任意状态直接进入"的全局模式覆写为 true。
  virtual bool startFromAnyState() const { return false; }

  // 暂停：原地冻结语义。默认实现只置暂停标志（框架冻结目标发布、不再调用
  // step）。需要额外动作的模式（如 climb 的 holdClimb）先调 ModeBase::pause
  // 再做自己的事。
  virtual void pause(const std::string& reason);
  // 恢复：从冻结点继续。默认实现清暂停标志。
  virtual void resume();
  // 停止：终止模式。smooth=true 走平滑回正；抢占路径传 false（中止序列由
  // 调度器统一执行）。默认无操作，需要清理的模式覆写。
  virtual void stop(bool smooth);

  // 节点级安全保持（反馈丢失等看门狗触发，控制线程）：返回 true 表示本模式
  // 已完全处理（如 dock 的 failExecution + Hold）；返回 false 表示框架继续
  // 执行通用保持路径（取消 approach + holdClimb + Hold）。默认 false。
  virtual bool onNodeHold(const std::string& reason) {
    (void)reason;
    return false;
  }

  // ───────────── 框架提供，作者不要覆盖 ─────────────
  const std::string& name() const { return name_; }
  ModeContext& ctx() { return ctx_; }
  const ModeTopics& topics() const { return topics_; }

  // 便捷等待当前 run 到终态（服务线程/进程内调用方使用；调度器内部用
  // waitForRun(run, …) 绑定具体调用代次）。
  ModeOutcome wait(double timeout_s) {
    return waitForRun(currentRun(), timeout_s);
  }

  // 框架每帧调用：先看已有 outcome，再问 checkTerminal 并落盘终态。
  // 返回非 Running 表示本次调用结束。
  ModeOutcome pollTerminal();

 private:
  std::string name_;
  ModeContext& ctx_;
  ModeTopics topics_;
};

}  // namespace grasp_hexapod_bt_control
