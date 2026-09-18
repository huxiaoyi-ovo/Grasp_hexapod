// 面向行为树的高层控制节点实现——模式调度器。
// 行为契约（与原实现一致）：回调只写缓存、120Hz 轮询 + 双板新帧门控、
// switch_mode 阻塞服务（带抢占）、HOLD 租约 → pause/resume、WAIT_B 只接受
// home。各模式的入口/推进/终态逻辑在 src/modes/*.cpp 中独立实现。
#include "grasp_hexapod_bt_control/bt_control_node.h"

#include "grasp_hexapod_bt_control/modes/mode_walk.h"

#include <array>

#include <grasp_hexapod_msgs/GripperAct.h>
#include <ros/package.h>

#include <cctype>
#include <cmath>
#include <cstdio>

namespace grasp_hexapod_bt_control {

using namespace grasp_hexapod_control_cpp;

namespace {

std::string formatMessage(const char* format, ...) {
  char buffer[1024];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return buffer;
}

// 双板分组：left=lf/lm/lb, right=rf/rm/rb。
struct BoardLegs {
  const char* board;
  std::array<const char*, 3> legs;
};
constexpr BoardLegs kServoBoardLegs[2] = {
    {"left", {"lf", "lm", "lb"}},
    {"right", {"rf", "rm", "rb"}}};

}  // namespace

// ---------------------------------------------------------------------------
// feedbackFrameState / feedbackIssue / clampCommand（静态，供单测）。
// ---------------------------------------------------------------------------

std::pair<bool, bool> BtControlNode::feedbackFrameState(
    const JointAngles& q_cur, const std::array<double, 6>& feedback_stamp,
    const std::array<double, 6>& last_control_feedback_stamp, double now,
    double max_feedback_age, double max_feedback_skew) {
  double min_stamp = feedback_stamp[0];
  double max_stamp = feedback_stamp[0];
  bool all_finite = true;
  bool all_positive = true;
  bool all_fresh = true;
  for (int leg = 0; leg < kLegCount; ++leg) {
    for (int joint = 0; joint < kJointCount; ++joint) {
      all_finite = all_finite && std::isfinite(q_cur[leg][joint]);
    }
    all_positive = all_positive && feedback_stamp[leg] > 0.0;
    const double age = now - feedback_stamp[leg];
    all_fresh = all_fresh && age >= 0.0 && age <= max_feedback_age;
    min_stamp = std::min(min_stamp, feedback_stamp[leg]);
    max_stamp = std::max(max_stamp, feedback_stamp[leg]);
  }
  const double snapshot_skew = max_stamp - min_stamp;
  const bool feedback_ready =
      all_finite && all_positive && all_fresh && snapshot_skew <= max_feedback_skew;
  bool all_new = true;
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!(feedback_stamp[leg] > last_control_feedback_stamp[leg])) {
      all_new = false;
      break;
    }
  }
  return {feedback_ready, feedback_ready && all_new};
}

std::string BtControlNode::feedbackIssue(
    const JointAngles& q_cur, const std::array<double, 6>& feedback_stamp,
    double now, double max_feedback_age) {
  std::string board_issues;
  for (const BoardLegs& board : kServoBoardLegs) {
    std::string leg_issues;
    for (const char* leg_name : board.legs) {
      const int index = legIndexFromName(leg_name);
      const double stamp = feedback_stamp[index];
      bool finite = true;
      for (int joint = 0; joint < kJointCount; ++joint) {
        finite = finite && std::isfinite(q_cur[index][joint]);
      }
      if (!finite || stamp <= 0.0) {
        leg_issues += formatMessage("%s=missing, ", leg_name);
        continue;
      }
      const double age = now - stamp;
      if (age < 0.0) {
        leg_issues += formatMessage("%s=future(%.3fs), ", leg_name, -age);
      } else if (age > max_feedback_age) {
        leg_issues += formatMessage("%s=stale(%.3fs), ", leg_name, age);
      }
    }
    if (!leg_issues.empty()) {
      // 去掉结尾 ", "。
      leg_issues.resize(leg_issues.size() - 2);
      board_issues += formatMessage("%s[%s]; ", board.board, leg_issues.c_str());
    }
  }
  double min_stamp = std::numeric_limits<double>::infinity();
  double max_stamp = -std::numeric_limits<double>::infinity();
  int valid_count = 0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (feedback_stamp[leg] > 0.0) {
      min_stamp = std::min(min_stamp, feedback_stamp[leg]);
      max_stamp = std::max(max_stamp, feedback_stamp[leg]);
      ++valid_count;
    }
  }
  const double skew =
      valid_count >= 2 ? max_stamp - min_stamp
                       : std::numeric_limits<double>::quiet_NaN();
  return board_issues + formatMessage("snapshot_skew=%.3fs", skew);
}

Eigen::Vector4d BtControlNode::clampCommand(double vx_right, double vy_forward,
                                            double yaw_rate,
                                            double max_linear_speed,
                                            double max_yaw_rate) {
  return modes::WalkMode::clampCommand(vx_right, vy_forward, yaw_rate,
                                       max_linear_speed, max_yaw_rate);
}

// ---------------------------------------------------------------------------
// 参数加载与接口注册。
// ---------------------------------------------------------------------------

void BtControlNode::loadParameters(ros::NodeHandle& pnh) {
  rate_hz_ = pnh.param("controller_rate_hz", 30.0);
  poll_rate_hz_ = rate_hz_ * 4.0;
  // 控制器仍只在一帧完整的新六腿反馈上按 30 Hz 数学步长推进。
  enable_link_collision_check_ = pnh.param("enable_link_collision_check", true);
  max_feedback_age_ = pnh.param("max_feedback_age", 0.30);
  max_feedback_skew_ = pnh.param("max_feedback_skew", 0.20);
  if (max_feedback_skew_ <= 0.0 || max_feedback_skew_ > max_feedback_age_) {
    throw ConfigError(
        "~max_feedback_skew must be positive and no greater than "
        "~max_feedback_age");
  }
  control_source_ = pnh.param<std::string>("control_source", "teleop");
  if (control_source_ != "teleop" && control_source_ != "navigation") {
    throw ConfigError("~control_source must be teleop or navigation");
  }
  bt_hold_lease_s_ = pnh.param("bt_hold_lease_s", 0.15);
  if (!std::isfinite(bt_hold_lease_s_) ||
      bt_hold_lease_s_ <= 1.0 / 30.0) {
    // HOLD 租约必须覆盖至少一个默认行为树 tick 间隔。
    throw ConfigError(
        "~bt_hold_lease_s must exceed one 30 Hz BT tick interval");
  }
  max_linear_speed_ = pnh.param("max_linear_speed", 0.20);
  double foot_radius_sum = 0.0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    foot_radius_sum += controller_->foot_init_base[leg].head<2>().norm();
  }
  const double foot_radius = foot_radius_sum / kLegCount;
  max_yaw_rate_ = pnh.param("max_yaw_rate", max_linear_speed_ / foot_radius);
}

void BtControlNode::configureNavigationApproach(ros::NodeHandle& pnh) {
  if (control_source_ != "navigation") {
    return;
  }
  ClimbCompactConfig compact = controller_->climb_mode.loadConfig();
  validateClimbCompact(compact);
  const std::string mesh_dir = paths_.climb_compact_json.substr(
      0, paths_.climb_compact_json.find("/grasp_hexapod_control/"));
  const CompactApproachGeometry approach_geometry =
      deriveCompactApproachGeometry(
          compact, mesh_dir + "/grasp_hexapod_description/meshes/xiaolan/"
                           "base_link_xiaolan.STL");
  const double boundary_margin = pnh.param("navigation_boundary_margin_m", 0.03);
  const double body_clearance =
      pnh.param("navigation_xiaolan_body_clearance_m", 0.13);
  if (std::min(boundary_margin, body_clearance) <= 0.0) {
    throw ConfigError("navigation safety margins must be positive");
  }
  for (const std::string target_side : {"left", "right"}) {
    controller_->approach_mode.configureFixedApproach(
        approach_geometry.targets.at(target_side), target_side, boundary_margin,
        pnh.param("navigation_linear_speed", max_linear_speed_),
        pnh.param("navigation_yaw_rate", max_yaw_rate_),
        approach_geometry.xiaolan_keepout_polygon_xy_m, body_clearance);
  }
}

void BtControlNode::createInterfaces(ros::NodeHandle& nh,
                                     ros::NodeHandle& pnh) {
  const double max_pose_age = pnh.param("max_pose_age", 0.5);
  const double max_pose_skew = pnh.param("max_pose_skew", 0.2);
  const double max_boundary_age = pnh.param("max_boundary_age", 1.0);
  if (std::min({max_pose_age, max_pose_skew, max_boundary_age}) <= 0.0) {
    throw ConfigError("navigation age and skew limits must be positive");
  }
  navigation_ = std::make_unique<NavigationInput>(
      nh, shared_topics_, max_pose_age, max_pose_skew, max_boundary_age);
  imu_ = std::make_unique<ImuInput>(
      nh, shared_topics_.get("imu", "/grasp_hexapod/imu"),
      pnh.param("real_climb_max_imu_age", 0.2));

  // 目标发布与反馈订阅：话题名来自 shared 节，顺序必须是腿索引 0..5。
  const std::vector<std::string> joint_command =
      topics_config_.sectionList("shared", "joint_command");
  const std::vector<std::string> joint_feedback =
      topics_config_.sectionList("shared", "joint_feedback");
  if (joint_command.size() != kLegCount || joint_feedback.size() != kLegCount) {
    throw ConfigError(
        "mode_topics.yaml[shared.joint_feedback/joint_command] must list 6 "
        "topics in lf,lm,lb,rf,rm,rb order");
  }
  for (int leg = 0; leg < kLegCount; ++leg) {
    des_publishers_.push_back(nh.advertise<std_msgs::Float64MultiArray>(
        joint_command[static_cast<std::size_t>(leg)], 1));
  }
  for (int leg = 0; leg < kLegCount; ++leg) {
    const std::string leg_name = kLegNames[leg];
    subscribers_.push_back(nh.subscribe<sensor_msgs::JointState>(
        joint_feedback[static_cast<std::size_t>(leg)], 1,
        [this, leg, leg_name](const sensor_msgs::JointState::ConstPtr& message) {
          feedbackCallback(leg, leg_name, message);
        },
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
  }

  // BT 标准接口（实机与 Isaac ROS 联调共用）：话题/服务名来自 YAML。
  const ModeTopics services = topics_config_.section("services");
  gripper_client_ = nh.serviceClient<grasp_hexapod_msgs::GripperAct>(
      services.get("gripper_act", "/grasp_hexapod/gripper_act"));
  switch_mode_server_ = nh.advertiseService(
      services.get("switch_mode", "/grasp_hexapod/switch_mode"),
      &BtControlNode::switchModeService, this);
  mode_status_pub_ = nh.advertise<grasp_hexapod_msgs::ModeStatus>(
      topics_config_.section("bt").get(
          "mode_status", "/grasp_hexapod/mode_status"),
      5);
  subscribers_.push_back(nh.subscribe<std_msgs::String>(
      topics_config_.section("bt").get("hold_motion",
                                       "/grasp_hexapod/hold_motion"),
      5,
      [this](const std_msgs::String::ConstPtr& message) {
        btHoldCallback(message);
      }));

  if (!enable_link_collision_check_) {
    ROS_WARN(
        "LINK COLLISION CHECK DISABLED: joint limits, workspace projection, "
        "and inter-foot clearance checks remain enabled");
  }
}

void BtControlNode::createModes() {
  // 启动时按注册表创建全部已实现模式（各自完成参数读取与话题订阅），
  // 参数错误在启动期暴露，而不是首次调用时。
  for (const std::string& name : ModeRegistry::instance().names()) {
    if (!ModeRegistry::instance().implemented(name)) {
      ROS_INFO("mode %s registered (executor not implemented)", name.c_str());
      continue;
    }
    getOrCreateMode(name);
  }
  std::string listing;
  for (const auto& item : mode_cache_) {
    listing += item.first + " ";
  }
  ROS_INFO(
      "BT control node ready: modes=[%s]; call %s (home first); control "
      "source=%s",
      listing.c_str(),
      topics_config_.section("services")
          .get("switch_mode", "/grasp_hexapod/switch_mode")
          .c_str(),
      control_source_.c_str());
}

ModeBase* BtControlNode::getOrCreateMode(const std::string& name) {
  std::lock_guard<std::mutex> guard(bt_mutex_);
  const auto found = mode_cache_.find(name);
  if (found != mode_cache_.end()) {
    return found->second.get();
  }
  std::unique_ptr<ModeBase> mode =
      ModeRegistry::instance().create(name, *this, topics_config_.section(name));
  if (mode == nullptr) {
    return nullptr;
  }
  mode_cache_[name] = std::move(mode);
  return mode_cache_[name].get();
}

BtControlNode::BtControlNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  // 共享 grasp_hexapod_control 的 config/（climb_compact.json 等）。
  std::string config_dir = pnh.param<std::string>("control_config_dir", "");
  if (config_dir.empty()) {
    config_dir = ros::package::getPath("grasp_hexapod_control") + "/config";
  }
  paths_.climb_compact_json = config_dir + "/climb_compact.json";
  paths_.workspace_bounds_csv = config_dir + "/workspace_bounds.csv";

  controller_ = std::make_unique<GraspController>(
      1.0 / 30.0, /*enable_link_collision_check=*/false,
      /*climb_timeout_uses_wall_time=*/true, paths_);
  loadParameters(pnh);
  // rate 参数在 loadParameters 后才确定；控制器 dt 与之一致。
  controller_->dt = 1.0 / rate_hz_;

  // 话题名统一从 mode_topics.yaml 加载（~topics_config 可指定其他文件）。
  topics_config_ = ModeTopicConfig::load(
      pnh.param<std::string>(
          "topics_config",
          ros::package::getPath("grasp_hexapod_bt_control") +
              "/config/mode_topics.yaml"));
  shared_topics_ = topics_config_.section("shared");

  configureNavigationApproach(pnh);
  createInterfaces(nh, pnh);
  createModes();
  tick_stats_window_start_ = ros::Time::now();
}

// ---------------------------------------------------------------------------
// 共享输入快照（SharedInputs 实现）。
// ---------------------------------------------------------------------------

JointAngles BtControlNode::qCur() const {
  std::lock_guard<std::mutex> guard(lock_);
  return q_cur_;
}

std::array<double, 6> BtControlNode::feedbackStamps() const {
  std::lock_guard<std::mutex> guard(lock_);
  return feedback_stamp_;
}

ImuSnapshot BtControlNode::imu() const { return imu_->snapshot(); }

NavigationState BtControlNode::navigationSnapshot() const {
  return navigation_->snapshot();
}

NavigationState BtControlNode::navigationMotionSnapshot() const {
  return navigation_->motionSnapshot();
}

// ---------------------------------------------------------------------------
// ModeContext 实现。
// ---------------------------------------------------------------------------

std::pair<bool, std::string> BtControlNode::callGripper(
    const std::string& action) {
  // 经标准 GripperAct 服务执行并原样返回到位结果。
  if (!gripper_client_.waitForExistence(ros::Duration(0.2))) {
    return {false, "gripper_act service is unavailable"};
  }
  grasp_hexapod_msgs::GripperAct service;
  service.request.action = action;
  if (!gripper_client_.call(service)) {
    return {false, "gripper_act " + action + " failed: service call error"};
  }
  return {service.response.success, service.response.message};
}

void BtControlNode::abortActiveMotion(const std::string& reason) {
  // home 与抢占共用的中止序列：取消任务/攀爬/对接后平滑回正。
  controller_->mission.cancel(reason);
  controller_->abortClimb();
  if (controller_->dock_mode != nullptr && controller_->dock_mode->active) {
    controller_->dock_mode->exit();
  }
  controller_->reset_active = false;
  state_ = MachineState::Resetting;
}

// ---------------------------------------------------------------------------
// 输入回调。
// ---------------------------------------------------------------------------

void BtControlNode::feedbackCallback(int leg_index, const std::string& leg_name,
                                     const sensor_msgs::JointState::ConstPtr& message) {
  // 把一条腿的 thigh、knee、ankle 反馈写入 q_cur 对应行。
  if (message->position.size() != 3) {
    ROS_WARN_THROTTLE(1.0, "/%s_pos must contain 3 joint positions",
                      leg_name.c_str());
    return;
  }
  const double stamp = message->header.stamp.toSec();
  if (stamp <= 0.0) {
    ROS_WARN_THROTTLE(1.0, "/%s_pos must contain a valid timestamp",
                      leg_name.c_str());
    return;
  }
  std::lock_guard<std::mutex> guard(lock_);
  for (int joint = 0; joint < kJointCount; ++joint) {
    q_cur_[leg_index][joint] = message->position[static_cast<std::size_t>(joint)];
  }
  feedback_stamp_[leg_index] = stamp;
}

// ---------------------------------------------------------------------------
// switch_mode 服务与 hold_motion 租约。
// ---------------------------------------------------------------------------

void BtControlNode::btHoldCallback(const std_msgs::String::ConstPtr&) {
  // 续约 BT 专用 HOLD 租约（只在有活动调用时）。
  std::lock_guard<std::mutex> guard(bt_mutex_);
  if (active_call_ != nullptr &&
      active_call_->mode->status().result == ModeResult::Running &&
      active_call_->run == active_call_->mode->currentRun()) {
    bt_hold_deadline_ = nowSeconds() + bt_hold_lease_s_;
  }
}

std::shared_ptr<BtControlNode::ActiveCall> BtControlNode::activeCallSnapshot()
    const {
  std::lock_guard<std::mutex> guard(bt_mutex_);
  if (active_call_ == nullptr ||
      active_call_->mode->status().result != ModeResult::Running ||
      active_call_->run != active_call_->mode->currentRun()) {
    return nullptr;
  }
  return active_call_;
}

bool BtControlNode::switchModeService(
    grasp_hexapod_msgs::SwitchModeRequest& request,
    grasp_hexapod_msgs::SwitchModeResponse& response) {
  // 阻塞式 BT 服务：注册调用后等待模式终态，不执行运动。
  // 抢占语义：进行中的不同模式调用会被立即终结（success=false），新模式由
  // 控制循环先平滑回正到 HOLD 再进入；单调用方（行为树）感知不到抢占。
  std::string mode = request.target_mode;
  for (char& c : mode) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  if (!ModeRegistry::instance().has(mode)) {
    response.success = false;
    response.message = "unknown mode: " + mode;
    return true;
  }
  if (!ModeRegistry::instance().implemented(mode)) {
    response.success = false;
    response.message = "executor not implemented: " + mode;
    return true;
  }
  ModeBase* mode_ptr = getOrCreateMode(mode);

  std::shared_ptr<ActiveCall> call;
  {
    std::lock_guard<std::mutex> guard(bt_mutex_);
    const bool active_in_flight =
        active_call_ != nullptr &&
        active_call_->mode->status().result == ModeResult::Running &&
        active_call_->run == active_call_->mode->currentRun();
    if (active_in_flight && active_call_->mode == mode_ptr) {
      // 同模式重复调用：合并为等待者。
      active_call_->waiters += 1;
      call = active_call_;
    } else {
      if (active_in_flight) {
        ROS_INFO("BT mode %s preempted by %s",
                 active_call_->mode->name().c_str(), mode.c_str());
        active_call_->mode->markPreempted("preempted by " + mode);
      }
      call = std::make_shared<ActiveCall>();
      call->mode = mode_ptr;
      call->run = mode_ptr->allocateRun();
      call->started = false;
      call->waiters = 1;
      active_call_ = call;
      bt_hold_deadline_ = 0.0;
      bt_hold_active_ = false;
    }
  }

  // 0.1s 分片等待终态（与原实现一致，期间持续检查 ros::ok）。
  ModeOutcome outcome = call->mode->waitForRun(call->run, 0.1);
  while (outcome.result == ModeResult::Running && ros::ok()) {
    outcome = call->mode->waitForRun(call->run, 0.1);
  }
  if (outcome.result == ModeResult::Running) {
    call->mode->finish(ModeResult::Failure, "ROS shutdown");
    outcome = call->mode->status();
  }

  {
    std::lock_guard<std::mutex> guard(bt_mutex_);
    call->waiters -= 1;
    if (call->waiters == 0 && active_call_ == call) {
      active_call_ = nullptr;
      // 记录最近一次完成调用，供空闲期模式状态上报。
      last_finished_mode_ = call->mode->name();
      last_finished_outcome_ = outcome;
      last_finished_run_ = call->run;
    }
  }
  response.success = outcome.result == ModeResult::Success;
  response.message = outcome.message;
  return true;
}

void BtControlNode::tryStartCall(ActiveCall& call, const JointAngles& q_cur) {
  // 在控制帧内一次性进入模式；抢占发生时先回正再进入。
  if (call.started) {
    return;
  }
  ModeBase* mode = call.mode;
  if (mode->startFromAnyState()) {
    // home 允许从任意状态直接进入（原 B 键全局抢占的服务化等价）。
    call.started = true;
    std::string reason;
    if (!mode->start(q_cur, reason)) {
      mode->finish(ModeResult::Failure, reason);
    }
    return;
  }
  if (state_ == MachineState::WaitB) {
    // 安全门：首次上电（舵机未上力）只允许 home。
    mode->finish(ModeResult::Failure, "call home first");
    return;
  }
  if (state_ == MachineState::Running) {
    // 抢占路径：与 home 相同的中止序列，平滑回正到 HOLD 后再进入目标模式。
    abortActiveMotion("BT preempt: " + mode->name());
    return;  // started 保持 false；回正完成后本函数被再次调用。
  }
  if (state_ == MachineState::Resetting) {
    // 等回正完成（updateControl 的 Resetting 分支推进）。
    return;
  }
  call.started = true;
  std::string reason;
  if (!mode->start(q_cur, reason)) {
    mode->finish(ModeResult::Failure, reason);
  }
}

bool BtControlNode::btHoldBridge(double now, const ActiveCall& call) {
  // HOLD 租约只冻结现有模式的推进：不取消模式、服务继续等待。
  // 上升沿 pause()、下降沿 resume()（climb 经覆写保留内部阶段状态）。
  bool active;
  {
    std::lock_guard<std::mutex> guard(bt_mutex_);
    active = now < bt_hold_deadline_;
  }
  if (active && !bt_hold_active_) {
    bt_hold_active_ = true;
    call.mode->pause("bt hold lease");
  } else if (!active && bt_hold_active_) {
    bt_hold_active_ = false;
    call.mode->resume();
  }
  return active;
}

// ---------------------------------------------------------------------------
// 安全保持与每帧主逻辑。
// ---------------------------------------------------------------------------

void BtControlNode::holdMotion(const std::string& reason, bool log) {
  // 停止推进步态并保持上力；需要新的 switch_mode 请求才能恢复。
  if (state_ != MachineState::Running) {
    return;
  }
  const std::shared_ptr<ActiveCall> call = activeCallSnapshot();
  if (call != nullptr && call->mode->onNodeHold(reason)) {
    // 模式自行处理（如 dock 的 failExecution 进 HOLD）。
    return;
  }
  controller_->approach_mode.cancelAutonomousApproach(reason);
  controller_->holdClimb();
  state_ = MachineState::Hold;
  if (log) {
    ROS_INFO("Motion paused: %s", reason.c_str());
  }
}

void BtControlNode::publishModeStatus() {
  // 变化即发 + 空闲 1Hz 心跳；只有诊断价值，不影响控制。
  std::string mode_name;
  std::string status = "idle";
  std::string message;
  uint64_t run = 0;
  if (const std::shared_ptr<ActiveCall> call = activeCallSnapshot()) {
    mode_name = call->mode->name();
    run = call->run;
    status = call->mode->isPaused() ? "paused" : "running";
  } else if (!last_finished_mode_.empty()) {
    // 无活动调用：上报最近一次调用的终态。
    mode_name = last_finished_mode_;
    switch (last_finished_outcome_.result) {
      case ModeResult::Success:
        status = "success";
        break;
      case ModeResult::Preempted:
        status = "preempted";
        break;
      default:
        status = "failed";
        break;
    }
    message = last_finished_outcome_.message;
    run = last_finished_run_;
  }
  const std::string key = mode_name + "|" + status + "|" + message;
  const double now = nowSeconds();
  if (key == last_status_key_ && now - last_status_publish_ < 1.0) {
    return;  // 未变化且心跳未到期。
  }
  grasp_hexapod_msgs::ModeStatus msg;
  msg.header.stamp = ros::Time::now();
  msg.mode = mode_name;
  msg.status = status;
  msg.run = run;
  msg.message = message;
  mode_status_pub_.publish(msg);
  last_status_key_ = key;
  last_status_publish_ = now;
}

std::optional<JointAngles> BtControlNode::updateControl(
    const JointAngles& q_cur, double now,
    const std::array<double, 6>* feedback_stamp) {
  // 用一帧完整反馈处理模式调度，返回 18 关节目标。
  std::shared_ptr<ActiveCall> call = activeCallSnapshot();
  if (call != nullptr && !call->started) {
    tryStartCall(*call, q_cur);
    call = activeCallSnapshot();
  }
  if (call != nullptr) {
    // 输入健康前置检查（保持原节点"/cmd_vel 过期先于 HOLD 租约"的时序）。
    call->mode->onPreFrame(ModeStepInput{q_cur, now, feedback_stamp});
    call = activeCallSnapshot();
  }

  // 第一次 home 以前不发布目标，Servo 保持卸力并只读反馈。
  if (state_ == MachineState::WaitB) {
    return std::nullopt;
  }

  if (call != nullptr && btHoldBridge(now, *call)) {
    // 冻结在上一帧目标：模式不推进、控制环不更新。
    return controller_->q_des;
  }

  std::optional<JointAngles> q_des;
  if (state_ == MachineState::Resetting) {
    if (!controller_->reset_active) {
      controller_->resetToStand(q_cur);
    }
    q_des = controller_->update(q_cur, Eigen::Vector4d::Zero());
    if (!controller_->reset_active) {
      state_ = MachineState::Hold;
      ROS_INFO(
          "Stand initialization complete; call switch_mode walk/approach/"
          "climb/dock");
    }
  } else if (state_ == MachineState::Running) {
    if (call != nullptr) {
      q_des = call->mode->step(ModeStepInput{q_cur, now, feedback_stamp});
    } else {
      // RUNNING 且无活动模式（理论上不可达）：安全停步。
      holdMotion("no active mode");
      q_des = controller_->update(q_cur, Eigen::Vector4d::Zero());
    }
  } else {
    // HOLD：让当前摆动腿先落地再停止（零速推进一帧）。
    q_des = controller_->update(q_cur, Eigen::Vector4d::Zero());
  }

  if (call != nullptr && call->started) {
    // 终态落盘（写入模式对象并唤醒服务等待者）。
    call->mode->pollTerminal();
  }
  return q_des;
}

void BtControlNode::publishTargets(const JointAngles& q_des) {
  // 把控制器 6×3 关节目标拆成六个固定 10 元素消息。
  for (int leg = 0; leg < kLegCount; ++leg) {
    std_msgs::Float64MultiArray message;
    message.data.resize(10);
    message.data[0] = 1.0;  // power=1.0 表示上力
    for (int joint = 0; joint < kJointCount; ++joint) {
      message.data[1 + joint] = q_des[leg][joint];
    }
    des_publishers_[static_cast<std::size_t>(leg)].publish(message);
  }
}

void BtControlNode::step() {
  // 实机循环：读取完整反馈，计算一次目标并发布给两块 Servo 板。
  const auto tick_start = ros::WallTime::now();

  JointAngles q_cur_snapshot;
  std::array<double, 6> feedback_stamp_snapshot{};
  bool feedback_ready = false;
  bool complete_new_frame = false;

  {
    std::lock_guard<std::mutex> guard(lock_);
    q_cur_snapshot = q_cur_;
    feedback_stamp_snapshot = feedback_stamp_;
    // 在反馈快照之后取时钟，避免回调刚写入的新时间戳落到 now 之后。
    const double now_snapshot = nowSeconds();
    std::tie(feedback_ready, complete_new_frame) = feedbackFrameState(
        q_cur_snapshot, feedback_stamp_snapshot, last_control_feedback_stamp_,
        now_snapshot, max_feedback_age_, max_feedback_skew_);
    if (complete_new_frame) {
      last_control_feedback_stamp_ = feedback_stamp_snapshot;
    }
  }

  const double now = nowSeconds();
  if (!feedback_ready) {
    holdMotion("joint feedback lost");
    ROS_WARN_THROTTLE(
        1.0, "Waiting for valid 18-DOF feedback: %s",
        feedbackIssue(q_cur_snapshot, feedback_stamp_snapshot, now,
                      max_feedback_age_)
            .c_str());
    return;
  }

  if (!complete_new_frame) {
    // 两块板独立计时，等待较慢板的三条腿；不重复推进控制器状态。
    return;
  }

  const std::optional<JointAngles> q_des = updateControl(
      q_cur_snapshot, now, &feedback_stamp_snapshot);
  publishModeStatus();
  if (q_des.has_value()) {
    for (int leg = 0; leg < kLegCount; ++leg) {
      if (!(*q_des)[leg].allFinite()) {
        // 产生非有限目标意味着控制器内部错误，直接终止节点。
        ROS_FATAL("Controller produced non-finite q_des");
        ros::shutdown();
        return;
      }
    }
    publishTargets(*q_des);
  }

  // 每 tick 计算耗时统计（1Hz 汇总输出）。
  const double elapsed = (ros::WallTime::now() - tick_start).toSec();
  last_tick_compute_seconds_ = elapsed;
  tick_compute_sum_ += elapsed;
  tick_compute_max_ = std::max(tick_compute_max_, elapsed);
  ++tick_compute_count_;
  const double window = (ros::Time::now() - tick_stats_window_start_).toSec();
  if (window >= 1.0 && tick_compute_count_ > 0) {
    ROS_INFO_THROTTLE(
        1.0, "control tick: n=%d mean=%.3fms max=%.3fms last=%.3fms",
        tick_compute_count_, 1000.0 * tick_compute_sum_ / tick_compute_count_,
        1000.0 * tick_compute_max_, 1000.0 * last_tick_compute_seconds_);
    tick_compute_sum_ = 0.0;
    tick_compute_max_ = 0.0;
    tick_compute_count_ = 0;
    tick_stats_window_start_ = ros::Time::now();
  }
}

}  // namespace grasp_hexapod_bt_control
