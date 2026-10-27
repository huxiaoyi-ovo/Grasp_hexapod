#include "grasp_hexapod_bt_control/remote_control_node.h"

#include <ros/ros.h>

#include <Eigen/Core>

#include <atomic>
#include <cmath>

namespace grasp_hexapod_bt_control {

namespace {

double nowSeconds() {
  return ros::Time::now().toSec();
}

double readAxis(const std::vector<double>& values, int index) {
  if (index < 0 || index >= static_cast<int>(values.size())) {
    return 0.0;
  }
  return values[static_cast<std::size_t>(index)];
}

}  // namespace

RemoteControlNode::RemoteControlNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  joy_topic_ = pnh.param<std::string>("joy_topic", "/joy");
  cmd_vel_topic_ = pnh.param<std::string>("cmd_vel_topic", "/cmd_vel");
  switch_mode_service_ = pnh.param<std::string>(
      "switch_mode_service", "/grasp_hexapod/switch_mode");
  gripper_service_ =
      pnh.param<std::string>("gripper_service", "/grasp_hexapod/gripper_act");
  rate_hz_ = pnh.param("rate_hz", 30.0);
  max_joy_age_ = pnh.param("max_joy_age", 0.2);
  button_a_ = pnh.param("button_a", 0);
  button_b_ = pnh.param("button_b", 1);
  button_x_ = pnh.param("button_x", 2);
  button_y_ = pnh.param("button_y", 3);
  axis_right_ = pnh.param("axis_right", 0);
  axis_forward_ = pnh.param("axis_forward", 1);
  axis_yaw_ = pnh.param("axis_yaw", 3);
  axis_gripper_ = pnh.param("axis_gripper", 6);  // 方向键轴：+1=张开, -1=闭合
  axis_right_scale_ = pnh.param("axis_right_scale", -1.0);
  axis_forward_scale_ = pnh.param("axis_forward_scale", 1.0);
  axis_yaw_scale_ = pnh.param("axis_yaw_scale", 1.0);
  max_linear_speed_ = pnh.param("max_linear_speed", 0.20);
  max_yaw_rate_ = pnh.param("max_yaw_rate", 0.50);

  cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
  switch_mode_client_ = nh.serviceClient<grasp_hexapod_msgs::SwitchMode>(
      switch_mode_service_);
  gripper_client_ =
      nh.serviceClient<grasp_hexapod_msgs::GripperAct>(gripper_service_);
  joy_subscriber_ = nh.subscribe<sensor_msgs::Joy>(
      joy_topic_, 1,
      [this](const sensor_msgs::Joy::ConstPtr& message) {
        joyCallback(message);
      },
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

  ROS_INFO(
      "remote control ready: %s → %s + %s; A=walk B=home X=climb Y=dock; "
      "dpad left=clamp right=open",
      joy_topic_.c_str(), cmd_vel_topic_.c_str(), switch_mode_service_.c_str());
}

void RemoteControlNode::joyCallback(const sensor_msgs::Joy::ConstPtr& message) {
  std::vector<double> axes(message->axes.begin(), message->axes.end());
  std::vector<int> buttons(message->buttons.begin(), message->buttons.end());
  std::lock_guard<std::mutex> guard(lock_);
  std::vector<int> previous(buttons.size(), 0);
  const int common =
      static_cast<int>(std::min(previous.size(), buttons_.size()));
  for (int i = 0; i < common; ++i) {
    previous[static_cast<std::size_t>(i)] =
        buttons_[static_cast<std::size_t>(i)];
  }
  if (button_press_latch_.size() != buttons.size()) {
    button_press_latch_.assign(buttons.size(), 0);
  }
  for (int i = 0; i < static_cast<int>(buttons.size()); ++i) {
    if (buttons[static_cast<std::size_t>(i)] != 0 &&
        previous[static_cast<std::size_t>(i)] == 0) {
      button_press_latch_[static_cast<std::size_t>(i)] = 1;  // 按下沿锁存
    }
  }
  axes_ = std::move(axes);
  buttons_ = std::move(buttons);
  joy_stamp_ = nowSeconds();
}

void RemoteControlNode::handleButtonPress(int index) {
  if (index == button_a_) {
    callSwitchMode("walk");
  } else if (index == button_b_) {
    callSwitchMode("home");
  } else if (index == button_x_) {
    callSwitchMode("climb");
  } else if (index == button_y_) {
    callSwitchMode("dock");
  }
}

void RemoteControlNode::callSwitchMode(const std::string& mode) {
  // 服务阻塞至模式结束（切换由服务端抢占语义处理），必须放在独立线程，
  // 否则 walk 这类长驻模式会卡死后续按键与 /cmd_vel 发布。
  if (inflight_switch_calls_.load(std::memory_order_relaxed) >= 4) {
    ROS_WARN_THROTTLE(1.0, "switch_mode %s skipped: too many inflight calls",
                      mode.c_str());
    return;
  }
  inflight_switch_calls_.fetch_add(1, std::memory_order_relaxed);
  std::thread([this, mode]() {
    grasp_hexapod_msgs::SwitchMode service;
    service.request.target_mode = mode;
    if (!switch_mode_client_.waitForExistence(ros::Duration(1.0))) {
      ROS_WARN("switch_mode %s failed: service unavailable", mode.c_str());
    } else if (!switch_mode_client_.call(service)) {
      ROS_WARN("switch_mode %s failed: service call error", mode.c_str());
    } else if (service.response.success) {
      ROS_INFO("switch_mode %s: %s", mode.c_str(),
               service.response.message.c_str());
    } else {
      ROS_WARN("switch_mode %s rejected: %s", mode.c_str(),
               service.response.message.c_str());
    }
    inflight_switch_calls_.fetch_sub(1, std::memory_order_relaxed);
  }).detach();
}

void RemoteControlNode::runGripper(const std::string& action) {
  // 执行方向键触发的夹爪服务并记录结果；结束（含失败）释放占用。
  grasp_hexapod_msgs::GripperAct service;
  service.request.action = action;
  bool ok = false;
  std::string message;
  if (!gripper_client_.waitForExistence(ros::Duration(0.2))) {
    message = "gripper_act service is unavailable";
  } else if (!gripper_client_.call(service)) {
    message = "gripper_act " + action + " failed: service call error";
  } else {
    ok = service.response.success;
    message = service.response.message;
  }
  gripper_service_lock_.unlock();
  if (ok) {
    ROS_INFO("Gripper %s done: %s", action.c_str(), message.c_str());
  } else {
    ROS_WARN("Gripper %s failed: %s", action.c_str(), message.c_str());
  }
}

void RemoteControlNode::maybeTriggerGripper(double dpad) {
  // 方向键位置选择夹爪目标（+1 张开/-1 闭合）；方向回中后允许重复触发。
  const std::string action =
      dpad > 0.5 ? "open" : (dpad < -0.5 ? "clamp" : "");
  if (action.empty()) {
    if (!gripper_last_cmd_.empty()) {
      gripper_last_cmd_.clear();
    }
    return;
  }
  if (gripper_last_cmd_ == action) {
    return;
  }
  if (!gripper_service_lock_.try_lock()) {
    ROS_WARN_THROTTLE(1.0, "Gripper %s skipped: previous call still running",
                      action.c_str());
    return;
  }
  gripper_last_cmd_ = action;
  if (gripper_thread_.joinable()) {
    gripper_thread_.join();
  }
  gripper_thread_ = std::thread([this, action]() { runGripper(action); });
}

geometry_msgs::Twist RemoteControlNode::makeTwist(bool* fresh) const {
  std::lock_guard<std::mutex> guard(lock_);
  const bool joy_fresh = joy_stamp_ > 0.0 &&
                         0.0 <= nowSeconds() - joy_stamp_ &&
                         nowSeconds() - joy_stamp_ <= max_joy_age_;
  if (fresh != nullptr) {
    *fresh = joy_fresh;
  }
  geometry_msgs::Twist twist;
  if (!joy_fresh) {
    return twist;  // 全零：手柄失效安全停步
  }
  // 平面速度：向量模归一后按 max_linear_speed 缩放（与原 makeCommand 一致）。
  Eigen::Vector2d planar(axis_right_scale_ * readAxis(axes_, axis_right_),
                         axis_forward_scale_ * readAxis(axes_, axis_forward_));
  const double norm = planar.norm();
  if (norm > 1.0) {
    planar /= norm;
  }
  twist.linear.y = max_linear_speed_ * planar.x();   // 右为正（与控制端约定一致）
  twist.linear.x = max_linear_speed_ * planar.y();   // 前为正
  twist.angular.z =
      max_yaw_rate_ * axis_yaw_scale_ * readAxis(axes_, axis_yaw_);
  return twist;
}

void RemoteControlNode::spin() {
  ros::Rate rate(rate_hz_);
  while (ros::ok()) {
    // 消费按下沿；B/home 与其余模式同等处理（服务端抢占语义保证安全切换）。
    std::vector<int> presses;
    {
      std::lock_guard<std::mutex> guard(lock_);
      presses = button_press_latch_;
      std::fill(button_press_latch_.begin(), button_press_latch_.end(), 0);
    }
    for (int index = 0; index < static_cast<int>(presses.size()); ++index) {
      if (presses[static_cast<std::size_t>(index)] != 0) {
        handleButtonPress(index);
      }
    }

    double dpad = 0.0;
    {
      std::lock_guard<std::mutex> guard(lock_);
      dpad = readAxis(axes_, axis_gripper_);
    }
    maybeTriggerGripper(dpad);

    cmd_vel_publisher_.publish(makeTwist(nullptr));
    rate.sleep();
  }
  if (gripper_thread_.joinable()) {
    gripper_thread_.join();
  }
}

}  // namespace grasp_hexapod_bt_control
