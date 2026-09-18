// ModeLifecycle 状态机核心实现（无 ROS 调用，可独立单测）。
#include "grasp_hexapod_bt_control/mode_base.h"

namespace grasp_hexapod_bt_control {

namespace {

ModeOutcome runningOutcome() { return {ModeResult::Running, ""}; }

}  // namespace

uint64_t ModeLifecycle::allocateRun() {
  std::lock_guard<std::mutex> guard(mutex_);
  ++run_;
  outcome_.reset();
  paused_ = false;
  cv_.notify_all();  // 唤醒旧 run 的等待者：它们会看到 superseded。
  return run_;
}

ModeOutcome ModeLifecycle::waitForRun(uint64_t run,
                                      double timeout_s) const {
  std::unique_lock<std::mutex> lock(mutex_);
  if (timeout_s < 0.0) {
    timeout_s = 1e9;  // 实际无限等（调用方按 0.1s 分片自行检查退出条件）。
  }
  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(timeout_s));
  while (true) {
    if (outcome_.has_value()) {
      return *outcome_;
    }
    if (run_ != run) {
      return {ModeResult::Preempted, "superseded"};
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      return runningOutcome();
    }
    cv_.wait_until(lock, deadline, [this, run] {
      return outcome_.has_value() || run_ != run;
    });
  }
}

ModeOutcome ModeLifecycle::status() const {
  std::lock_guard<std::mutex> guard(mutex_);
  return outcome_.value_or(runningOutcome());
}

uint64_t ModeLifecycle::currentRun() const {
  std::lock_guard<std::mutex> guard(mutex_);
  return run_;
}

void ModeLifecycle::markPreempted(const std::string& message) {
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!outcome_.has_value()) {
      outcome_ = ModeOutcome{ModeResult::Preempted, message};
      paused_ = false;
    }
  }
  cv_.notify_all();
}

bool ModeLifecycle::isPaused() const {
  std::lock_guard<std::mutex> guard(mutex_);
  return paused_;
}

void ModeLifecycle::finish(ModeResult result, const std::string& message) {
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!outcome_.has_value()) {
      outcome_ = ModeOutcome{result, message};
      paused_ = false;
    }
  }
  cv_.notify_all();
}

void ModeLifecycle::setPaused(bool paused) {
  std::lock_guard<std::mutex> guard(mutex_);
  paused_ = paused;
}

// ---------------------------------------------------------------------------
// ModeBase 默认实现。
// ---------------------------------------------------------------------------

void ModeBase::pause(const std::string& reason) {
  ROS_INFO("Mode %s paused: %s", name_.c_str(), reason.c_str());
  setPaused(true);
}

void ModeBase::resume() {
  ROS_INFO("Mode %s resumed", name_.c_str());
  setPaused(false);
}

void ModeBase::stop(bool smooth) {
  ROS_INFO("Mode %s stopped (smooth=%d)", name_.c_str(),
           smooth ? 1 : 0);
}

ModeOutcome ModeBase::pollTerminal() {
  // start/step 内直接 finish() 的情况优先返回。
  const ModeOutcome current = status();
  if (current.result != ModeResult::Running) {
    return current;
  }
  const std::optional<ModeOutcome> terminal = checkTerminal();
  if (terminal.has_value()) {
    finish(terminal->result, terminal->message);  // 幂等：首个终态生效。
    return *terminal;
  }
  return runningOutcome();
}

}  // namespace grasp_hexapod_bt_control
