// 单块九舵机驱动板的 ROS 节点实现。
#include "grasp_hexapod_servo_cpp/servo_side_node.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <boost/bind.hpp>

#include "grasp_hexapod_servo_cpp/servo_utils.h"

namespace grasp_hexapod_servo_cpp {

namespace {

double secondsSince(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                       start)
      .count();
}

}  // namespace

std::optional<ServoSideNode::SideConfig> ServoSideNode::getSideConfig(
    const std::string& side) {
  if (side == "left") {
    return SideConfig{"/dev/ttyTHS0",
                      {"lf", "lm", "lb"},
                      {{"lf", {1, 2, 3}}, {"lm", {4, 5, 6}}, {"lb", {7, 8, 9}}},
                      {1, 1, 1, 1, 1, 1, 1, 1, 1}};
  }
  if (side == "right") {
    return SideConfig{"/dev/ttyACM0",
                      {"rf", "rm", "rb"},
                      {{"rf", {10, 11, 12}},
                       {"rm", {13, 14, 15}},
                       {"rb", {16, 17, 18}}},
                      {1, -1, -1, 1, -1, -1, 1, -1, -1}};
  }
  return std::nullopt;
}

std::vector<int> ServoSideNode::loadDirectionsParam(
    ros::NodeHandle& nh_private) {
  // 未配置时使用板默认方向。
  if (!nh_private.hasParam("directions")) {
    return std::vector<int>(getSideConfig(side_)->directions.begin(),
                            getSideConfig(side_)->directions.end());
  }
  // 优先数组形式（如 <rosparam>）；其次字符串形式（如 launch <param>）。
  std::vector<int> directions;
  if (nh_private.getParam("directions", directions)) {
    return directions;
  }
  std::string text;
  if (nh_private.getParam("directions", text)) {
    std::optional<std::vector<int>> parsed = parseDirectionsString(text);
    if (!parsed) {
      throw std::runtime_error("~directions string could not be parsed: " +
                               text);
    }
    return *parsed;
  }
  throw std::runtime_error("~directions must be an int array or a string");
}

void ServoSideNode::loadParams(ros::NodeHandle& nh_private) {
  side_ = nh_private.param<std::string>("side", "left");
  std::transform(side_.begin(), side_.end(), side_.begin(),
                 [](unsigned char c) { return std::tolower(c); });

  std::optional<SideConfig> config = getSideConfig(side_);
  if (!config) {
    throw std::runtime_error("~side must be one of: left, right");
  }
  legs_ = config->legs;
  id_map_ = config->id_map;
  for (const std::string& leg : legs_) {
    for (int servo_id : id_map_[leg]) {
      servo_ids_.push_back(servo_id);
    }
  }

  port_ = nh_private.param<std::string>("port", config->port);
  baudrate_ = nh_private.param("baudrate", 115200);
  servo_rate_hz_ = nh_private.param("servo_rate_hz", 30.0);
  command_duration_ms_ = nh_private.param("command_duration_ms", 33);
  enable_diagnostics_ = nh_private.param("enable_diagnostics", true);
  voltage_report_interval_s_ =
      nh_private.param("voltage_report_interval_s", 2.0);
  if (voltage_report_interval_s_ <= 0.0) {
    throw std::runtime_error("~voltage_report_interval_s must be positive");
  }

  std::vector<int> directions = loadDirectionsParam(nh_private);
  if (!validateDirections(directions, servo_ids_.size())) {
    throw std::runtime_error(
        "~directions length must match servo count and values must be 1 or -1");
  }
  for (size_t i = 0; i < servo_ids_.size(); ++i) {
    directions_[servo_ids_[i]] = directions[i];
  }

  // 夹爪参数（仅左板支持）。
  if (side_ == "left") {
    const int default_gripper_id = 99;
    if (nh_private.hasParam("gripper_id")) {
      gripper_id_ = nh_private.param("gripper_id", default_gripper_id);
      has_gripper_ = (gripper_id_ > 0);
    } else {
      gripper_id_ = default_gripper_id;
      has_gripper_ = true;
    }
    if (has_gripper_) {
      gripper_direction_ = nh_private.param("gripper_direction", -1);
      gripper_command_duration_ms_ =
          nh_private.param("gripper_command_duration_ms", 400);
      gripper_pulse_min_ = nh_private.param("gripper_pulse_min", 280);
      gripper_pulse_max_ = nh_private.param("gripper_pulse_max", 750);
    }
  }
}

ServoSideNode::ServoSideNode(ros::NodeHandle& nh,
                             ros::NodeHandle& nh_private) {
  loadParams(nh_private);

  // 打开串口；失败抛 SerialOpenException，由 main 统一处理（对应 Python raise）。
  control_.reset(new HiwonderServoController(port_, baudrate_));

  // 启动时整块板全部卸力，此时仍可读取舵机位置。
  setBoardPower(false);

  // 夹爪卸力。
  if (has_gripper_) {
    control_->unloadServo(gripper_id_, 0);
  }

  for (const std::string& leg : legs_) {
    des_subs_[leg] = nh.subscribe<std_msgs::Float64MultiArray>(
        "/" + leg + "_des", 1,
        boost::bind(&ServoSideNode::onDesired, this, boost::placeholders::_1,
                    leg));
    pos_pubs_[leg] =
        nh.advertise<sensor_msgs::JointState>("/" + leg + "_pos", 1);
  }

  // 夹爪订阅/发布。
  if (has_gripper_) {
    gripper_des_sub_ = nh.subscribe<std_msgs::Float64MultiArray>(
        "/gripper_des", 1, &ServoSideNode::onGripperDesired, this);
    gripper_pos_pub_ =
        nh.advertise<sensor_msgs::JointState>("/gripper_pos", 1);
  }

  // 电压标签：leg_joint，顺序与 id_map 一致。
  for (const std::string& leg : legs_) {
    const std::array<int, kJointsPerLeg>& ids = id_map_[leg];
    for (int j = 0; j < kJointsPerLeg; ++j) {
      voltage_labels_[ids[j]] = std::string(leg) + "_" + kJointNames[j];
    }
  }

  // 时序窗口从定时器启用前开始，不把启动卸力和 pub/sub 创建计入。
  timing_.window_start = std::chrono::steady_clock::now();
  voltage_next_report_at_ =
      timing_.window_start +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(voltage_report_interval_s_));

  timer_ = nh.createTimer(ros::Duration(1.0 / servo_rate_hz_),
                          &ServoSideNode::controlLoop, this);

  std::ostringstream legs_text;
  std::ostringstream ids_text;
  ids_text << "(";
  for (size_t i = 0; i < legs_.size(); ++i) {
    if (i > 0) {
      legs_text << ",";
    }
    legs_text << legs_[i];
  }
  for (size_t i = 0; i < servo_ids_.size(); ++i) {
    if (i > 0) {
      ids_text << ", ";
    }
    ids_text << servo_ids_[i];
  }
  ids_text << ")";
  ROS_INFO("Servo board ready: side=%s port=%s legs=%s ids=%s rate=%.1fHz"
           " gripper_id=%s",
           side_.c_str(), port_.c_str(), legs_text.str().c_str(),
           ids_text.str().c_str(), servo_rate_hz_,
           has_gripper_ ? std::to_string(gripper_id_).c_str() : "none");
}

ServoSideNode::~ServoSideNode() = default;

void ServoSideNode::setBoardPower(bool enabled) {
  const uint8_t status = enabled ? 1 : 0;
  for (int servo_id : servo_ids_) {
    control_->unloadServo(servo_id, status);
  }
  power_on_ = enabled;
  ROS_INFO("Servo board %s power: %s", side_.c_str(),
           enabled ? "ON" : "OFF");
}

void ServoSideNode::onDesired(
    const std_msgs::Float64MultiArray::ConstPtr& message,
    const std::string& leg) {
  const std::vector<double>& data = message->data;

  // 接口固定为 10 个元素，避免上下游对数组含义理解不同。
  if (data.size() != kDesiredMessageSize) {
    ROS_WARN_THROTTLE(1.0, "/%s_des must contain 10 values, got %zu",
                      leg.c_str(), data.size());
    return;
  }

  const double power = data[0];
  const std::array<double, kJointsPerLeg> target = {data[1], data[2], data[3]};

  if (power != 0.0 && power != 1.0) {
    ROS_WARN_THROTTLE(1.0, "/%s_des power must be 0 or 1", leg.c_str());
    return;
  }
  if (!std::isfinite(target[0]) || !std::isfinite(target[1]) ||
      !std::isfinite(target[2])) {
    ROS_WARN_THROTTLE(1.0, "/%s_des contains non-finite position",
                      leg.c_str());
    return;
  }

  bool first_message = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    first_message = !leg_targets_[leg].received;
    leg_targets_[leg].pos = target;
    leg_targets_[leg].power_request = (power != 0.0);
    leg_targets_[leg].received = true;
  }
  if (first_message) {
    ROS_INFO("Received first target for %s", leg.c_str());
  }

  // data[4:7] 是预留的目标速度；LX-15D 当前通过 command_duration_ms 控制
  // 运动时间，第一版实机控制不使用速度字段。
}

void ServoSideNode::onGripperDesired(
    const std_msgs::Float64MultiArray::ConstPtr& message) {
  const std::vector<double>& data = message->data;
  if (data.size() != 2) {
    ROS_WARN_THROTTLE(1.0, "/gripper_des must contain 2 values [power, pos]");
    return;
  }
  const double power = data[0];
  const double position = data[1];
  if (power != 0.0 && power != 1.0) {
    ROS_WARN_THROTTLE(1.0, "/gripper_des power must be 0 or 1");
    return;
  }
  if (!std::isfinite(position)) {
    ROS_WARN_THROTTLE(1.0, "/gripper_des contains non-finite position");
    return;
  }
  bool first_message = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    first_message = !gripper_received_;
    gripper_des_pos_ = position;
    gripper_power_request_ = (power != 0.0);
    gripper_received_ = true;
  }
  if (first_message) {
    ROS_INFO("Received first target for gripper");
  }
}

std::optional<int> ServoSideNode::readPositionWithRetry(int servo_id) {
  // 读取失败只追加一次即时重试，仍失败则跳过本腿本周期反馈。
  std::optional<uint16_t> position = control_->getServoPosition(servo_id);
  if (!position) {
    timing_.read_retries[servo_id]++;
    position = control_->getServoPosition(servo_id);
    if (!position) {
      timing_.read_failures[servo_id]++;
    }
  }
  return position;
}

void ServoSideNode::publishLeg(const std::string& leg,
                               const std::vector<double>& positions) {
  sensor_msgs::JointState message;
  message.header.stamp = ros::Time::now();
  message.name = {leg + "_thigh_joint", leg + "_knee_joint",
                  leg + "_ankle_joint"};
  message.position = positions;
  pos_pubs_[leg].publish(message);
}

void ServoSideNode::controlLoop(const ros::TimerEvent&) {
  const std::chrono::steady_clock::time_point started_at =
      std::chrono::steady_clock::now();

  // 回调只更新缓存；串口操作期间不持有缓存锁。
  std::map<std::string, LegTarget> snapshot;
  bool gripper_snapshot_valid = false;
  bool gripper_snapshot_power = false;
  double gripper_snapshot_pos = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot = leg_targets_;
    if (has_gripper_) {
      gripper_snapshot_valid = gripper_received_;
      gripper_snapshot_power = gripper_power_request_;
      gripper_snapshot_pos = gripper_des_pos_;
    }
  }

  std::vector<bool> received;
  std::vector<bool> power_request;
  for (const std::string& leg : legs_) {
    received.push_back(snapshot[leg].received);
    power_request.push_back(snapshot[leg].power_request);
  }
  const bool requested_on = computeRequestedOn(received, power_request);

  try {
    if (requested_on != power_on_) {
      setBoardPower(requested_on);
    }

    // 一、优先写入最新目标，避免反馈读取占用本周期命令延迟。
    if (power_on_) {
      for (const std::string& leg : legs_) {
        const std::array<int, kJointsPerLeg>& ids = id_map_[leg];
        const std::array<double, kJointsPerLeg>& target = snapshot[leg].pos;
        for (int j = 0; j < kJointsPerLeg; ++j) {
          const int pulse = radToServo(target[j], directions_[ids[j]]);
          control_->setServoPosition(ids[j], pulse, command_duration_ms_);
        }
      }
    }

    // 夹爪写目标：独立于行走舵机加卸力，到位后每秒补发保持力矩。
    if (has_gripper_ && gripper_snapshot_valid) {
      const bool gripper_requested_on = gripper_snapshot_power;
      if (gripper_requested_on != gripper_power_on_) {
        control_->unloadServo(gripper_id_, gripper_requested_on ? 1 : 0);
        gripper_power_on_ = gripper_requested_on;
        gripper_last_sent_pulse_ = -1;  // 卸力/加载后重置
        ROS_INFO("Gripper power: %s", gripper_requested_on ? "ON" : "OFF");
      }
      if (gripper_power_on_) {
        int pulse = radToServo(gripper_snapshot_pos, gripper_direction_);
        pulse = std::max(gripper_pulse_min_, std::min(gripper_pulse_max_, pulse));
        const auto now = std::chrono::steady_clock::now();
        bool need_send = false;
        bool is_refresh = false;
        if (pulse != gripper_last_sent_pulse_) {
          need_send = true;
        } else if (std::chrono::duration<double>(
                       now - gripper_last_sent_time_).count() > 0.2) {
          need_send = true;
          is_refresh = true;
        }
        if (need_send) {
          control_->setServoPosition(gripper_id_, pulse,
                                     is_refresh ? 0 : gripper_command_duration_ms_);
          gripper_last_sent_pulse_ = pulse;
          gripper_last_sent_time_ = now;
        }
      }
    }

    // 二、一条腿三个位置都有效时才发布一帧带时间戳的反馈。
    for (const std::string& leg : legs_) {
      std::vector<double> read_positions;
      const std::array<int, kJointsPerLeg>& ids = id_map_[leg];
      for (int j = 0; j < kJointsPerLeg; ++j) {
        std::optional<int> raw = readPositionWithRetry(ids[j]);
        if (!raw) {
          read_positions.clear();
          break;
        }
        read_positions.push_back(servoToRad(*raw, directions_[ids[j]]));
      }
      if (read_positions.empty()) {
        ROS_WARN_THROTTLE(1.0, "Servo feedback unavailable: %s",
                          leg.c_str());
        continue;
      }
      publishLeg(leg, read_positions);
    }

    // 夹爪反馈。
    if (has_gripper_) {
      std::optional<uint16_t> raw = control_->getServoPosition(gripper_id_);
      if (raw) {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name = {"gripper_joint"};
        msg.position = {servoToRad(*raw, gripper_direction_)};
        gripper_pos_pub_.publish(msg);
      }
    }
  } catch (const serial::SerialException& e) {
    ROS_ERROR_THROTTLE(1.0, "Serial error in control loop: %s", e.what());
  } catch (const serial::IOException& e) {
    ROS_ERROR_THROTTLE(1.0, "Serial error in control loop: %s", e.what());
  }

  if (enable_diagnostics_) {
    updateVoltageDiagnostics();
    recordTimingDiagnostics(started_at);
  }
}

void ServoSideNode::updateVoltageDiagnostics() {
  const std::chrono::steady_clock::time_point now =
      std::chrono::steady_clock::now();
  if (voltage_pending_ids_.empty()) {
    if (now < voltage_next_report_at_) {
      return;
    }
    for (int servo_id : servo_ids_) {
      voltage_pending_ids_.push_back(servo_id);
    }
    voltage_samples_mv_.clear();
    voltage_next_report_at_ =
        now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>(voltage_report_interval_s_));
  }

  // 每个控制周期最多读一个电压，避免一次连读九个占用总线。
  const int servo_id = voltage_pending_ids_.front();
  voltage_pending_ids_.pop_front();
  std::optional<uint16_t> voltage_mv = control_->getServoVoltage(servo_id);
  if (voltage_mv) {
    voltage_samples_mv_[servo_id] = *voltage_mv;
  }

  if (!voltage_pending_ids_.empty()) {
    return;
  }

  // 九路全部读取完成，统一输出。
  std::ostringstream values;
  for (size_t i = 0; i < servo_ids_.size(); ++i) {
    if (i > 0) {
      values << " ";
    }
    const int sample_id = servo_ids_[i];
    auto it = voltage_samples_mv_.find(sample_id);
    if (it == voltage_samples_mv_.end()) {
      values << voltage_labels_[sample_id] << "(ID" << sample_id << ")=N/A";
    } else {
      values << voltage_labels_[sample_id] << "(ID" << sample_id << ")="
             << std::fixed << std::setprecision(2) << (it->second / 1000.0)
             << "V";
    }
  }
  ROS_INFO("Servo voltage: side=%s %s", side_.c_str(),
           values.str().c_str());
}

std::string ServoSideNode::timingCountsText(
    const std::map<int, long>& counts) const {
  std::ostringstream out;
  bool first = true;
  for (const auto& entry : counts) {
    if (!entry.second) {
      continue;
    }
    if (!first) {
      out << ",";
    }
    out << entry.first << "=" << entry.second;
    first = false;
  }
  return first ? std::string("none") : out.str();
}

void ServoSideNode::recordTimingDiagnostics(
    std::chrono::steady_clock::time_point started_at) {
  const double elapsed = secondsSince(started_at);
  timing_.callbacks++;
  timing_.max_loop_s = std::max(timing_.max_loop_s, elapsed);
  if (elapsed > 1.0 / servo_rate_hz_) {
    timing_.overruns++;
  }

  const double window_elapsed = secondsSince(timing_.window_start);
  if (window_elapsed < 1.0) {
    return;
  }

  ROS_INFO("Servo timing: side=%s actual_hz=%.2f max_loop_ms=%.3f "
           "overruns=%ld retries=%s failures=%s",
           side_.c_str(), timing_.callbacks / window_elapsed,
           timing_.max_loop_s * 1000.0, timing_.overruns,
           timingCountsText(timing_.read_retries).c_str(),
           timingCountsText(timing_.read_failures).c_str());

  timing_.window_start = std::chrono::steady_clock::now();
  timing_.callbacks = 0;
  timing_.max_loop_s = 0.0;
  timing_.overruns = 0;
  timing_.read_retries.clear();
  timing_.read_failures.clear();
}

}  // namespace grasp_hexapod_servo_cpp
