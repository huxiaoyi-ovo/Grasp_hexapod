// 夹爪舵机管理器实现：启动自检、open/clamp 服务、位置状态机。
#include "grasp_hexapod_servo_cpp/gripper_manager.h"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace grasp_hexapod_servo_cpp {

GripperManager::GripperManager(ros::NodeHandle& nh,
                               ros::NodeHandle& nh_private,
                               HiwonderServoController* control,
                               int gripper_id)
    : control_(control), gripper_id_(gripper_id) {
  loadParams(nh_private);
  service_ = nh.advertiseService("gripper_command",
                                 &GripperManager::handleCommand, this);
  ROS_INFO("Gripper manager ready: id=%d open_pulse=%d clamp_pulse=%d "
           "tolerance=%d fail_deviation=%d max_checks=%d max_polls=%d "
           "poll_hz=%.1f duration_ms=%d",
           gripper_id_, open_pulse_, clamp_pulse_, tolerance_,
           fail_deviation_, max_checks_, max_total_polls_, poll_hz_,
           command_duration_ms_);
}

void GripperManager::loadParams(ros::NodeHandle& nh_private) {
  open_pulse_ = nh_private.param("gripper_open_pulse", 683);
  clamp_pulse_ = nh_private.param("gripper_clamp_pulse", 840);
  tolerance_ = nh_private.param("gripper_tolerance", 20);
  fail_deviation_ = nh_private.param("gripper_fail_deviation", 50);
  max_checks_ = nh_private.param("gripper_max_checks", 4);
  max_total_polls_ = nh_private.param("gripper_max_total_polls", 8);
  poll_hz_ = nh_private.param("gripper_poll_hz", 2.0);
  command_duration_ms_ = nh_private.param("gripper_command_duration_ms", 400);

  if (poll_hz_ <= 0.0) {
    throw std::runtime_error("~gripper_poll_hz must be positive");
  }
  if (max_checks_ < 1 || max_total_polls_ < 1) {
    throw std::runtime_error(
        "~gripper_max_checks and ~gripper_max_total_polls must be >= 1");
  }
  if (tolerance_ < 0 || fail_deviation_ <= tolerance_) {
    throw std::runtime_error(
        "~gripper_tolerance must be >= 0 and < ~gripper_fail_deviation");
  }
  if (command_duration_ms_ < 0 || command_duration_ms_ > 30000) {
    throw std::runtime_error("~gripper_command_duration_ms must be in [0, 30000]");
  }
  if (open_pulse_ < 0 || open_pulse_ > 1000 || clamp_pulse_ < 0 ||
      clamp_pulse_ > 1000 || open_pulse_ >= clamp_pulse_) {
    throw std::runtime_error(
        "~gripper_open_pulse/clamp_pulse must satisfy 0 <= open < clamp <= 1000");
  }
}

void GripperManager::init() {
  busy_ = true;
  last_seen_pulse_.reset();
  ensureLoaded();

  // 一次读取即判定在线：无位置反馈 = 不在线。
  std::optional<int> pulse = readPulse();
  if (!pulse) {
    setState(GripperState::kOffline);
    busy_ = false;
    ROS_WARN("Gripper init: offline (id=%d, no position feedback)",
             gripper_id_);
    return;
  }
  if (classifyPulse(*pulse, open_pulse_, clamp_pulse_, tolerance_) ==
      GripperPositionClass::kOpen) {
    setState(GripperState::kOpen);
    busy_ = false;
    publishSync();
    ROS_INFO("Gripper init: online and open at pulse %d", *pulse);
    return;
  }

  // 未打开则打开，等待移动完成后复核。
  ROS_INFO("Gripper init: online at pulse %d, opening to %d", *pulse,
           open_pulse_);
  control_->setServoPosition(gripper_id_, open_pulse_, command_duration_ms_);
  sleepFor(command_duration_ms_ / 1000.0);
  std::optional<int> verified = readPulse();
  if (verified && classifyPulse(*verified, open_pulse_, clamp_pulse_,
                                tolerance_) ==
                     GripperPositionClass::kOpen) {
    setState(GripperState::kOpen);
    ROS_INFO("Gripper init: opened at pulse %d", *verified);
  } else {
    setState(GripperState::kUnknown);
    ROS_WARN("Gripper init: open verification failed (pulse %s)",
             verified ? std::to_string(*verified).c_str() : "no feedback");
  }
  busy_ = false;
  publishSync();
}

bool GripperManager::isBusy() const { return busy_; }

GripperState GripperManager::state() const { return state_; }

GripperSync GripperManager::lastSync() const {
  std::lock_guard<std::mutex> lock(sync_mutex_);
  return sync_;
}

void GripperManager::publishSync() {
  if (!last_seen_pulse_) {
    return;  // 本次命令从未读到位置（离线），无位置可同步。
  }
  std::lock_guard<std::mutex> lock(sync_mutex_);
  sync_.pulse = *last_seen_pulse_;
  sync_.generation++;
}

bool GripperManager::handleCommand(GripperCommand::Request& request,
                                   GripperCommand::Response& response) {
  // 并发命令立即拒绝，避免两次开合的移动/验证互相干扰。
  std::unique_lock<std::mutex> lock(operation_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    response.success = false;
    response.message = "busy: another gripper command in progress";
    return true;
  }
  last_seen_pulse_.reset();
  busy_ = true;
  if (request.command == "open") {
    doOpen(response);
  } else if (request.command == "clamp") {
    doClamp(response);
  } else {
    response.success = false;
    response.message = "unknown command '" + request.command + "' (open|clamp)";
  }
  busy_ = false;
  publishSync();
  return true;
}

bool GripperManager::doOpen(GripperCommand::Response& response) {
  ensureLoaded();
  std::optional<int> pulse = readPulse();
  if (!pulse) {
    setState(GripperState::kOffline);
    response.success = false;
    response.message = "offline: no position feedback";
    ROS_WARN("Gripper open failed: offline");
    return true;
  }
  if (classifyPulse(*pulse, open_pulse_, clamp_pulse_, tolerance_) ==
      GripperPositionClass::kOpen) {
    // open 成功即复位受限状态。
    setState(GripperState::kOpen);
    response.success = true;
    response.message = "already open at pulse " + std::to_string(*pulse);
    ROS_INFO("Gripper open: already open at pulse %d", *pulse);
    return true;
  }

  ROS_INFO("Gripper open: moving from pulse %d to %d", *pulse, open_pulse_);
  control_->setServoPosition(gripper_id_, open_pulse_, command_duration_ms_);
  sleepFor(command_duration_ms_ / 1000.0);
  for (int poll = 0; poll < max_total_polls_; ++poll) {
    sleepFor(1.0 / poll_hz_);
    if (!ros::ok()) {
      response.success = false;
      response.message = "shutdown during open";
      return true;
    }
    pulse = readPulse();
    if (!pulse) {
      setState(GripperState::kOffline);
      response.success = false;
      response.message = "lost position feedback during open";
      ROS_WARN("Gripper open failed: lost feedback");
      return true;
    }
    if (classifyPulse(*pulse, open_pulse_, clamp_pulse_, tolerance_) ==
        GripperPositionClass::kOpen) {
      setState(GripperState::kOpen);
      response.success = true;
      response.message = "open at pulse " + std::to_string(*pulse);
      ROS_INFO("Gripper open: open at pulse %d", *pulse);
      return true;
    }
  }

  // 打开超时：原受限则保持受限（须成功打开才复位），否则置未知。
  if (state_ != GripperState::kRestricted) {
    setState(GripperState::kUnknown);
  }
  response.success = false;
  response.message = "open timeout at pulse " + std::to_string(*pulse);
  ROS_WARN("Gripper open timeout: last pulse %d", *pulse);
  return true;
}

bool GripperManager::doClamp(GripperCommand::Response& response) {
  if (!clampAllowed(state_)) {
    response.success = false;
    response.message = "restricted: clamp failed previously, send open first";
    ROS_WARN("Gripper clamp rejected: restricted state");
    return true;
  }
  ensureLoaded();
  std::optional<int> pulse = readPulse();
  if (!pulse) {
    setState(GripperState::kOffline);
    response.success = false;
    response.message = "offline: no position feedback";
    ROS_WARN("Gripper clamp failed: offline");
    return true;
  }
  if (classifyPulse(*pulse, open_pulse_, clamp_pulse_, tolerance_) ==
      GripperPositionClass::kClamped) {
    setState(GripperState::kClamped);
    response.success = true;
    response.message = "already clamped at pulse " + std::to_string(*pulse);
    ROS_INFO("Gripper clamp: already clamped at pulse %d", *pulse);
    return true;
  }

  ROS_INFO("Gripper clamp: moving from pulse %d to %d", *pulse, clamp_pulse_);
  control_->setServoPosition(gripper_id_, clamp_pulse_, command_duration_ms_);
  sleepFor(command_duration_ms_ / 1000.0);

  // 2Hz 轮询验证到位：小偏差算夹紧；大偏差连续 max_checks 次算失败受限。
  int consecutive_large = 0;
  int last_deviation = -1;
  for (int poll = 0; poll < max_total_polls_; ++poll) {
    sleepFor(1.0 / poll_hz_);
    if (!ros::ok()) {
      response.success = false;
      response.message = "shutdown during clamp";
      return true;
    }
    pulse = readPulse();
    if (!pulse) {
      setState(GripperState::kOffline);
      response.success = false;
      response.message = "lost position feedback during clamp";
      ROS_WARN("Gripper clamp failed: lost feedback");
      return true;
    }
    last_deviation = std::abs(*pulse - clamp_pulse_);
    const ClampVerdict verdict =
        evaluateClampDeviation(last_deviation, tolerance_, fail_deviation_);
    if (verdict == ClampVerdict::kClampedOk) {
      setState(GripperState::kClamped);
      response.success = true;
      response.message = "clamped at pulse " + std::to_string(*pulse) +
                         " (deviation " + std::to_string(last_deviation) + ")";
      ROS_INFO("Gripper clamp: clamped at pulse %d (deviation %d)", *pulse,
               last_deviation);
      return true;
    }
    if (verdict == ClampVerdict::kLargeDeviation) {
      ++consecutive_large;
      if (consecutive_large >= max_checks_) {
        setState(GripperState::kRestricted);
        response.success = false;
        response.message = "clamp failed: deviation " +
                           std::to_string(last_deviation) + " >= " +
                           std::to_string(fail_deviation_) + " for " +
                           std::to_string(consecutive_large) + " checks";
        ROS_WARN("Gripper clamp failed: restricted (deviation %d x%d checks)",
                 last_deviation, consecutive_large);
        return true;
      }
    } else {
      consecutive_large = 0;
    }
  }

  response.success = false;
  response.message = "clamp timeout: deviation " +
                     std::to_string(last_deviation) + " after " +
                     std::to_string(max_total_polls_) + " polls";
  ROS_WARN("Gripper clamp timeout: deviation %d after %d polls",
           last_deviation, max_total_polls_);
  return true;
}

std::optional<int> GripperManager::readPulse() {
  std::optional<uint16_t> pulse = control_->getServoPosition(gripper_id_);
  if (!pulse) {
    // 失败即时重试一次，仍失败视为无反馈。
    pulse = control_->getServoPosition(gripper_id_);
  }
  if (!pulse) {
    return std::nullopt;
  }
  last_seen_pulse_ = *pulse;  // 供命令完成后向盲控路径同步位置。
  return *pulse;
}

void GripperManager::ensureLoaded() {
  control_->unloadServo(gripper_id_, 1);
}

void GripperManager::sleepFor(double seconds) const {
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

void GripperManager::setState(GripperState state) { state_ = state; }

// ---- 纯静态判定 ----

GripperPositionClass GripperManager::classifyPulse(int pulse, int open_pulse,
                                                   int clamp_pulse,
                                                   int tolerance) {
  if (std::abs(pulse - open_pulse) <= tolerance) {
    return GripperPositionClass::kOpen;
  }
  if (std::abs(pulse - clamp_pulse) <= tolerance) {
    return GripperPositionClass::kClamped;
  }
  return GripperPositionClass::kIntermediate;
}

ClampVerdict GripperManager::evaluateClampDeviation(int deviation,
                                                    int tolerance,
                                                    int fail_deviation) {
  if (deviation <= tolerance) {
    return ClampVerdict::kClampedOk;
  }
  if (deviation >= fail_deviation) {
    return ClampVerdict::kLargeDeviation;
  }
  return ClampVerdict::kInTransit;
}

bool GripperManager::clampAllowed(GripperState state) {
  return state != GripperState::kRestricted;
}

}  // namespace grasp_hexapod_servo_cpp
