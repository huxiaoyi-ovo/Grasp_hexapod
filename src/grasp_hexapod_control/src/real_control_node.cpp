// 实机高层控制节点实现。逐条对齐 run_real.py::RosControlNode 的行为契约：
// 回调只写缓存、120Hz 轮询 + 双板新帧门控、B 全局抢占、BT 阻塞服务与 HOLD
// 租约、夹爪后台线程、WAIT_B 不发布。
#include "grasp_hexapod_control_cpp/real_control_node.h"

#include "grasp_hexapod_control_cpp/dock_perception_ros.h"

#include <grasp_hexapod_msgs/GripperAct.h>
#include <ros/package.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace grasp_hexapod_control_cpp {

namespace {

double nowSeconds() {
  return ros::Time::now().toSec();
}

std::string formatMessage(const char* format, ...) {
  char buffer[1024];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return buffer;
}

// 双板分组（run_real.SERVO_BOARD_LEGS）：left=lf/lm/lb, right=rf/rm/rb。
struct BoardLegs {
  const char* board;
  std::array<const char*, 3> legs;
};
constexpr BoardLegs kServoBoardLegs[2] = {
    {"left", {"lf", "lm", "lb"}},
    {"right", {"rf", "rm", "rb"}}};

}  // namespace

const char* RealControlNode::machineStateName(MachineState state) {
  switch (state) {
    case MachineState::WaitB:
      return "WAIT_B";
    case MachineState::Resetting:
      return "RESETTING";
    case MachineState::Hold:
      return "HOLD";
    case MachineState::Running:
      return "RUNNING";
  }
  return "?";
}

// ---------------------------------------------------------------------------
// feedbackFrameState / feedbackIssue（静态，供单测）。
// ---------------------------------------------------------------------------

std::pair<bool, bool> RealControlNode::feedbackFrameState(
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

std::string RealControlNode::feedbackIssue(
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

// ---------------------------------------------------------------------------
// NavigationInput。
// ---------------------------------------------------------------------------

RealControlNode::NavigationInput::NavigationInput(ros::NodeHandle& pnh) {
  max_age_ = pnh.param("max_pose_age", 0.5);
  max_pose_skew_ = pnh.param("max_pose_skew", 0.2);
  max_boundary_age_ = pnh.param("max_boundary_age", 1.0);
  if (std::min({max_age_, max_pose_skew_, max_boundary_age_}) <= 0.0) {
    throw ConfigError("navigation age and skew limits must be positive");
  }
  ros::NodeHandle nh;
  const std::string base_topic = pnh.param<std::string>(
      "base_pose_topic", "/grasp_hexapod/navigation/base_pose");
  const std::string xiaolan_topic = pnh.param<std::string>(
      "xiaolan_pose_topic", "/grasp_hexapod/navigation/xiaolan_pose");
  const std::string boundary_topic = pnh.param<std::string>(
      "pv_boundary_topic", "/grasp_hexapod/navigation/pv_boundary");
  const std::string landing_topic =
      pnh.param<std::string>("landing_topic", "/grasp_hexapod/landing_confirmed");
  subscribers_.push_back(nh.subscribe<geometry_msgs::PoseStamped>(
      base_topic, 1,
      [this](const geometry_msgs::PoseStamped::ConstPtr& message) {
        baseCallback(message);
      }));
  subscribers_.push_back(nh.subscribe<geometry_msgs::PoseStamped>(
      xiaolan_topic, 1,
      [this](const geometry_msgs::PoseStamped::ConstPtr& message) {
        xiaolanCallback(message);
      }));
  subscribers_.push_back(nh.subscribe<geometry_msgs::PolygonStamped>(
      boundary_topic, 1,
      [this](const geometry_msgs::PolygonStamped::ConstPtr& message) {
        boundaryCallback(message);
      }));
  subscribers_.push_back(nh.subscribe<std_msgs::Bool>(
      landing_topic, 1,
      [this](const std_msgs::Bool::ConstPtr& message) {
        landingCallback(message);
      }));
}

bool RealControlNode::NavigationInput::validFrameImpl(
    const std::string& frame_id) const {
  if (frame_id.empty() ||
      (frame_id[0] == '/' ? frame_id.substr(1) : frame_id) == "pv_map") {
    return true;
  }
  ROS_WARN_THROTTLE(2.0, "Navigation frame must be pv_map");
  return false;
}

void RealControlNode::NavigationInput::baseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& message) {
  if (!validFrame(*message)) {
    return;
  }
  Matrix4d transform;
  const geometry_msgs::Point& p = message->pose.position;
  const geometry_msgs::Quaternion& q = message->pose.orientation;
  if (!poseToTransform(Eigen::Quaterniond(q.w, q.x, q.y, q.z),
                       Vector3d(p.x, p.y, p.z), transform)) {
    return;
  }
  std::lock_guard<std::mutex> guard(lock_);
  const double stamp = message->header.stamp.toSec();
  if (stamp < base_stamp_) {
    return;
  }
  pv_from_base_ = transform;
  base_stamp_ = stamp;
  have_base_ = true;
}

void RealControlNode::NavigationInput::xiaolanCallback(
    const geometry_msgs::PoseStamped::ConstPtr& message) {
  if (!validFrame(*message)) {
    return;
  }
  Matrix4d transform;
  const geometry_msgs::Point& p = message->pose.position;
  const geometry_msgs::Quaternion& q = message->pose.orientation;
  if (!poseToTransform(Eigen::Quaterniond(q.w, q.x, q.y, q.z),
                       Vector3d(p.x, p.y, p.z), transform)) {
    return;
  }
  std::lock_guard<std::mutex> guard(lock_);
  const double stamp = message->header.stamp.toSec();
  if (stamp < xiaolan_stamp_) {
    return;
  }
  pv_from_xiaolan_ = transform;
  xiaolan_stamp_ = stamp;
  have_xiaolan_ = true;
}

void RealControlNode::NavigationInput::boundaryCallback(
    const geometry_msgs::PolygonStamped::ConstPtr& message) {
  if (!validFrame(*message)) {
    return;
  }
  std::vector<Vector2d> boundary;
  boundary.reserve(message->polygon.points.size());
  bool finite = true;
  for (const geometry_msgs::Point32& point : message->polygon.points) {
    boundary.emplace_back(point.x, point.y);
    finite = finite && std::isfinite(point.x) && std::isfinite(point.y);
  }
  if (!finite) {
    return;
  }
  std::lock_guard<std::mutex> guard(lock_);
  const double stamp = message->header.stamp.toSec();
  if (stamp < boundary_stamp_) {
    return;
  }
  pv_boundary_ = std::move(boundary);
  boundary_stamp_ = stamp;
}

void RealControlNode::NavigationInput::landingCallback(
    const std_msgs::Bool::ConstPtr& message) {
  std::lock_guard<std::mutex> guard(lock_);
  landing_confirmed_ = message->data;
}

NavigationState RealControlNode::NavigationInput::snapshot() const {
  const double now = nowSeconds();
  std::lock_guard<std::mutex> guard(lock_);
  NavigationState state;
  state.stamp = std::min(base_stamp_, xiaolan_stamp_);
  const double pose_skew = std::abs(base_stamp_ - xiaolan_stamp_);
  state.valid = have_base_ && have_xiaolan_ && pv_boundary_.size() >= 3 &&
                state.stamp > 0.0 && 0.0 <= now - state.stamp &&
                now - state.stamp <= max_age_ && pose_skew <= max_pose_skew_ &&
                boundary_stamp_ > 0.0 &&
                0.0 <= now - boundary_stamp_ &&
                now - boundary_stamp_ <= max_boundary_age_;
  state.landing_confirmed = landing_confirmed_;
  state.pv_from_base = pv_from_base_;
  state.pv_from_xiaolan = pv_from_xiaolan_;
  state.pv_boundary = pv_boundary_;
  return state;
}

NavigationState RealControlNode::NavigationInput::motionSnapshot() const {
  const double now = nowSeconds();
  std::lock_guard<std::mutex> guard(lock_);
  NavigationState state;
  state.stamp = std::min(base_stamp_, xiaolan_stamp_);
  const double pose_skew = std::abs(base_stamp_ - xiaolan_stamp_);
  state.valid = have_base_ && have_xiaolan_ && state.stamp > 0.0 &&
                0.0 <= now - state.stamp && now - state.stamp <= max_age_ &&
                pose_skew <= max_pose_skew_;
  state.landing_confirmed = landing_confirmed_;
  state.pv_from_base = pv_from_base_;
  state.pv_from_xiaolan = pv_from_xiaolan_;
  state.pv_boundary = pv_boundary_;
  return state;
}

// ---------------------------------------------------------------------------
// ImuInput。
// ---------------------------------------------------------------------------

RealControlNode::ImuInput::ImuInput(ros::NodeHandle& pnh) {
  max_age_ = pnh.param("real_climb_max_imu_age", 0.2);
  ros::NodeHandle nh;
  const std::string topic = pnh.param<std::string>("imu_topic", "/grasp_hexapod/imu");
  subscriber_ = nh.subscribe<sensor_msgs::Imu>(
      topic, 1,
      [this](const sensor_msgs::Imu::ConstPtr& message) { callback(message); });
}

void RealControlNode::ImuInput::callback(const sensor_msgs::Imu::ConstPtr& message) {
  const double stamp = message->header.stamp.toSec();
  const geometry_msgs::Quaternion& q = message->orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  Vector3d angular_velocity(message->angular_velocity.x,
                            message->angular_velocity.y,
                            message->angular_velocity.z);
  if (norm <= 0.0 || stamp <= 0.0 || !angular_velocity.allFinite()) {
    return;
  }
  const double x = q.x / norm, y = q.y / norm, z = q.z / norm, w = q.w / norm;
  Matrix3d rotation;
  rotation << 1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
      2.0 * (x * z + y * w), 2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
      2.0 * (y * z - x * w), 2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
      1.0 - 2.0 * (x * x + y * y);
  if (!rotation.allFinite()) {
    return;
  }
  std::lock_guard<std::mutex> guard(lock_);
  rotation_ = rotation;
  angular_velocity_ = angular_velocity;
  stamp_ = stamp;
  have_rotation_ = true;
}

RealControlNode::ImuInput::Snapshot RealControlNode::ImuInput::snapshot() const {
  const double now = nowSeconds();
  std::lock_guard<std::mutex> guard(lock_);
  Snapshot snap;
  snap.valid = have_rotation_ && stamp_ > 0.0 && 0.0 <= now - stamp_ &&
               now - stamp_ <= max_age_;
  snap.rotation = rotation_;
  snap.angular_velocity = angular_velocity_;
  return snap;
}

// ---------------------------------------------------------------------------
// BoolInput。
// ---------------------------------------------------------------------------

RealControlNode::BoolInput::BoolInput(ros::NodeHandle& nh,
                                      const std::string& topic) {
  subscriber_ = nh.subscribe<std_msgs::Bool>(
      topic, 1,
      [this](const std_msgs::Bool::ConstPtr& message) {
        callback(message);
      });
}

void RealControlNode::BoolInput::callback(const std_msgs::Bool::ConstPtr& message) {
  std::lock_guard<std::mutex> guard(lock_);
  value_ = message->data;
  received_at_ = nowSeconds();
}

std::optional<bool> RealControlNode::BoolInput::snapshot() const {
  std::lock_guard<std::mutex> guard(lock_);
  return value_;
}

std::pair<std::optional<bool>, double> RealControlNode::BoolInput::snapshotWithTime()
    const {
  std::lock_guard<std::mutex> guard(lock_);
  return {value_, received_at_};
}

}  // namespace grasp_hexapod_control_cpp

namespace grasp_hexapod_control_cpp {

// ---------------------------------------------------------------------------
// 参数加载与接口注册。
// ---------------------------------------------------------------------------

void RealControlNode::loadParameters(ros::NodeHandle& pnh) {
  rate_hz_ = pnh.param("controller_rate_hz", 30.0);
  poll_rate_hz_ = rate_hz_ * 4.0;
  // 控制器仍只在一帧完整的新六腿反馈上按 30 Hz 数学步长推进。
  // 提高轮询频率仅缩短两块独立 Servo 板反馈错相时的等待。
  enable_link_collision_check_ = pnh.param("enable_link_collision_check", true);
  max_feedback_age_ = pnh.param("max_feedback_age", 0.30);
  max_feedback_skew_ = pnh.param("max_feedback_skew", 0.20);
  if (max_feedback_skew_ <= 0.0 || max_feedback_skew_ > max_feedback_age_) {
    throw ConfigError(
        "~max_feedback_skew must be positive and no greater than "
        "~max_feedback_age");
  }
  max_joy_age_ = pnh.param("max_joy_age", 0.2);
  enable_real_climb_ = pnh.param("enable_real_climb", false);
  climb_side_ = pnh.param<std::string>("climb_side", "left");
  validateCompactClimbSide(climb_side_);
  climb_foot_gate_m_ = pnh.param("climb_foot_gate_m", 0.02);
  if (!std::isfinite(climb_foot_gate_m_) || climb_foot_gate_m_ <= 0.0 ||
      climb_foot_gate_m_ > 0.10) {
    throw ConfigError("~climb_foot_gate_m must be finite and in (0, 0.10]");
  }
  enable_real_dock_ = pnh.param("enable_real_dock", true);
  dock_system_config_ = pnh.param<std::string>(
      "dock_system_config",
      paths_.workspace_bounds_csv.substr(
          0, paths_.workspace_bounds_csv.find_last_of('/')) + "/dock_system.yaml");
  dock_require_real_calibrated_ = pnh.param("dock_require_real_calibrated", true);
  dock_allow_uncalibrated_ = pnh.param("dock_allow_uncalibrated", false);
  dock_lock_confirmation_max_age_ =
      pnh.param("dock_lock_confirmation_max_age_s", 0.5);
  if (dock_lock_confirmation_max_age_ <= 0.0) {
    throw ConfigError("~dock_lock_confirmation_max_age_s must be positive");
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

  button_a_ = pnh.param("button_a", 0);
  button_b_ = pnh.param("button_b", 1);
  button_x_ = pnh.param("button_x", 2);
  button_y_ = pnh.param("button_y", 3);
  axis_gripper_ = pnh.param("axis_gripper", 6);  // 方向键轴：+1=张开, -1=闭合
  axis_right_ = pnh.param("axis_right", 0);
  axis_forward_ = pnh.param("axis_forward", 1);
  axis_yaw_ = pnh.param("axis_yaw", 3);
  axis_body_down_ = pnh.param("axis_body_down", 4);
  axis_body_up_ = pnh.param("axis_body_up", 5);
  axis_right_scale_ = pnh.param("axis_right_scale", -1.0);
  axis_forward_scale_ = pnh.param("axis_forward_scale", 1.0);
  axis_yaw_scale_ = pnh.param("axis_yaw_scale", 1.0);
  axis_body_scale_ = pnh.param("axis_body_scale", -1.0);

  max_linear_speed_ = pnh.param("max_linear_speed", 0.20);
  max_vertical_speed_ = pnh.param("max_vertical_speed", 0.02);
  double foot_radius_sum = 0.0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    foot_radius_sum += controller_->foot_init_base[leg].head<2>().norm();
  }
  const double foot_radius = foot_radius_sum / kLegCount;
  max_yaw_rate_ = pnh.param("max_yaw_rate", max_linear_speed_ / foot_radius);

  real_climb_persistence_frames_ = pnh.param("real_climb_persistence_frames", 3);
  real_climb_max_position_error_ =
      pnh.param("real_climb_max_position_error_m", 0.05);
  real_climb_max_orientation_error_ =
      pnh.param("real_climb_max_orientation_error_deg", 10.0) * M_PI / 180.0;
  real_climb_max_angular_speed_ =
      pnh.param("real_climb_max_angular_speed_deg_s", 30.0) * M_PI / 180.0;
}

void RealControlNode::configureNavigationApproach(ros::NodeHandle& pnh) {
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

void RealControlNode::createInterfaces(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  navigation_ = std::make_unique<NavigationInput>(pnh);
  imu_ = std::make_unique<ImuInput>(pnh);
  lock_confirmation_ = std::make_unique<BoolInput>(
      nh, pnh.param<std::string>("lock_confirmed_topic",
                                 "/grasp_hexapod/dock/lock_confirmed"));

  // 目标发布：六个固定 10 元素 Float64MultiArray（与 servo 约定一致）。
  for (int leg = 0; leg < kLegCount; ++leg) {
    des_publishers_[kLegNames[leg]] =
        nh.advertise<std_msgs::Float64MultiArray>(
            std::string("/") + kLegNames[leg] + "_des", 1);
  }

  // 摇杆是连续控制量，只消费最新值；按键按下沿由回调锁存。
  subscribers_.push_back(nh.subscribe<sensor_msgs::Joy>(
      "/joy", 1,
      [this](const sensor_msgs::Joy::ConstPtr& message) { joyCallback(message); },
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));

  // BT 标准接口（实机与 Isaac ROS 联调共用）。
  gripper_client_ =
      nh.serviceClient<grasp_hexapod_msgs::GripperAct>("/grasp_hexapod/gripper_act");
  switch_mode_server_ = nh.advertiseService(
      "/grasp_hexapod/switch_mode", &RealControlNode::switchModeService, this);
  subscribers_.push_back(nh.subscribe<grasp_hexapod_msgs::RemoteCmd>(
      "/grasp_hexapod/remote_cmd", 5,
      [this](const grasp_hexapod_msgs::RemoteCmd::ConstPtr& message) {
        remoteCmdCallback(message);
      }));
  subscribers_.push_back(nh.subscribe<std_msgs::String>(
      "/grasp_hexapod/hold_motion", 5,
      [this](const std_msgs::String::ConstPtr& message) { btHoldCallback(message); }));

  // 六腿关节反馈。
  for (int leg = 0; leg < kLegCount; ++leg) {
    const std::string leg_name = kLegNames[leg];
    subscribers_.push_back(nh.subscribe<sensor_msgs::JointState>(
        "/" + leg_name + "_pos", 1,
        [this, leg, leg_name](const sensor_msgs::JointState::ConstPtr& message) {
          feedbackCallback(leg, leg_name, message);
        },
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
  }

  if (!enable_link_collision_check_) {
    ROS_WARN(
        "LINK COLLISION CHECK DISABLED: joint limits, workspace projection, "
        "and inter-foot clearance checks remain enabled");
  }
  ROS_INFO(
      "Control source=%s; climb source=hardware_feedback foot_gate_m=%.3f; "
      "persistence/timeout come from compact config; waiting for feedback; "
      "press B before A",
      control_source_.c_str(), climb_foot_gate_m_);
}

RealControlNode::RealControlNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  // 共享 Python 控制包的 config/（climb_compact.json 等），与
  // utils.package_config_path 的 rospkg 回退一致。
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
  configureNavigationApproach(pnh);
  createInterfaces(nh, pnh);
  tick_stats_window_start_ = ros::Time::now();
}

// ---------------------------------------------------------------------------
// 手柄。
// ---------------------------------------------------------------------------

void RealControlNode::joyCallback(const sensor_msgs::Joy::ConstPtr& message) {
  Eigen::VectorXd axes(message->axes.size());
  for (int i = 0; i < axes.size(); ++i) {
    axes[i] = message->axes[static_cast<std::size_t>(i)];
  }
  Eigen::VectorXi buttons(message->buttons.size());
  for (int i = 0; i < buttons.size(); ++i) {
    buttons[i] = message->buttons[static_cast<std::size_t>(i)];
  }
  std::lock_guard<std::mutex> guard(lock_);
  Eigen::VectorXi previous = Eigen::VectorXi::Zero(buttons.size());
  const int common = std::min<int>(previous.size(), buttons_.size());
  if (common > 0) {
    previous.head(common) = buttons_.head(common);
  }
  if (button_press_latch_.size() != buttons.size()) {
    button_press_latch_ = Eigen::VectorXi::Zero(buttons.size());
  }
  for (int i = 0; i < buttons.size(); ++i) {
    if (buttons[i] != 0 && previous[i] == 0) {
      button_press_latch_[i] = 1;
    }
  }
  axes_ = axes;
  buttons_ = buttons;
  joy_stamp_ = nowSeconds();
}

double RealControlNode::readAxis(const Eigen::VectorXd& values, int index) {
  if (index < 0 || index >= values.size()) {
    return 0.0;
  }
  return values[index];
}

double RealControlNode::bodyAxis(const Eigen::VectorXd& axes) const {
  // 把 RT/LT 两个同基准扳机合成为 [-1,1] 升降指令。
  return 0.5 * axis_body_scale_ *
         (readAxis(axes, axis_body_up_) - readAxis(axes, axis_body_down_));
}

Eigen::Vector4d RealControlNode::makeCommand(const Eigen::VectorXd& axes) const {
  // 把归一化摇杆值转换成 [vx_right, vy_forward, vz, yaw_rate]。
  Eigen::Vector2d planar(axis_right_scale_ * readAxis(axes, axis_right_),
                         axis_forward_scale_ * readAxis(axes, axis_forward_));
  const double planar_norm = planar.norm();
  if (planar_norm > 1.0) {
    planar /= planar_norm;
  }
  return Eigen::Vector4d(
      max_linear_speed_ * planar.x(), max_linear_speed_ * planar.y(),
      max_vertical_speed_ * bodyAxis(axes),
      max_yaw_rate_ * axis_yaw_scale_ * readAxis(axes, axis_yaw_));
}

bool RealControlNode::manualCommandActive(const Eigen::VectorXd& axes) const {
  // 升降、平移或转向任一有效即接管导航。
  return std::max({std::abs(readAxis(axes, axis_right_)),
                   std::abs(readAxis(axes, axis_forward_)),
                   std::abs(bodyAxis(axes)),
                   std::abs(readAxis(axes, axis_yaw_))}) > 0.1;
}

// ---------------------------------------------------------------------------
// BT 接口。
// ---------------------------------------------------------------------------

void RealControlNode::remoteCmdCallback(
    const grasp_hexapod_msgs::RemoteCmd::ConstPtr& message) {
  // 缓存行为树遥控测试链的模式结束信号；不直接驱动控制器。
  std::lock_guard<std::mutex> guard(bt_mutex_);
  bt_remote_ = *message;
  bt_remote_valid_ = true;
}

void RealControlNode::btHoldCallback(const std_msgs::String::ConstPtr&) {
  // 续约 BT 专用 HOLD；没有活动 BT 请求时不影响人工控制。
  std::lock_guard<std::mutex> guard(bt_mutex_);
  if (bt_request_ != nullptr && !bt_request_->final_result.has_value()) {
    bt_hold_deadline_ = nowSeconds() + bt_hold_lease_s_;
  }
}

std::shared_ptr<RealControlNode::BtRequest> RealControlNode::btRequestSnapshot()
    const {
  std::lock_guard<std::mutex> guard(bt_mutex_);
  if (bt_request_ == nullptr || bt_request_->final_result.has_value()) {
    return nullptr;
  }
  return bt_request_;
}

void RealControlNode::finishBtRequest(const std::shared_ptr<BtRequest>& request,
                                      bool success, const std::string& message) {
  {
    std::lock_guard<std::mutex> guard(bt_mutex_);
    if (bt_request_ == request && !request->final_result.has_value()) {
      request->final_result = std::make_pair(success, message);
      bt_hold_deadline_ = 0.0;
      bt_hold_active_ = false;
    }
  }
  bt_condition_.notify_all();
}

void RealControlNode::abortBtRequest(const std::string& reason) {
  const std::shared_ptr<BtRequest> request = btRequestSnapshot();
  if (request != nullptr) {
    finishBtRequest(request, false, reason);
  }
}

bool RealControlNode::switchModeService(
    grasp_hexapod_msgs::SwitchModeRequest& request,
    grasp_hexapod_msgs::SwitchModeResponse& response) {
  // 阻塞式 BT 服务：只提交请求/等待控制循环终态，不执行运动。
  static const std::map<std::string, std::string> bt_mode_labels = {
      {"home", "return to stand"},   {"walk", "manual approach gait"},
      {"approach", "navigation approach"}, {"climb", "compact climb"},
      {"dock", "dock and clamp"},    {"release", "open gripper"},
      {"spin_search", "not implemented"}, {"tag_nav", "not implemented"}};
  static const std::array<const char*, 2> bt_unsupported = {"spin_search", "tag_nav"};

  std::string mode = request.target_mode;
  for (char& c : mode) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  const auto label = bt_mode_labels.find(mode);
  if (label == bt_mode_labels.end()) {
    response.success = false;
    response.message = "unknown mode: " + mode;
    return true;
  }
  if (std::find(bt_unsupported.begin(), bt_unsupported.end(), mode) !=
      bt_unsupported.end()) {
    response.success = false;
    response.message = "executor not implemented: " + mode;
    return true;
  }

  std::shared_ptr<BtRequest> active;
  {
    std::lock_guard<std::mutex> guard(bt_mutex_);
    active = bt_request_;
    if (active != nullptr && !active->final_result.has_value()) {
      if (active->mode != mode) {
        response.success = false;
        response.message = "busy: " + active->mode + " is running";
        return true;
      }
      active->waiters += 1;
    } else {
      active = std::make_shared<BtRequest>();
      active->mode = mode;
      active->waiters = 1;
      bt_request_ = active;
      bt_hold_deadline_ = 0.0;
      bt_hold_active_ = false;
    }
  }
  while (true) {
    {
      std::lock_guard<std::mutex> guard(bt_mutex_);
      if (active->final_result.has_value()) {
        break;
      }
    }
    if (!ros::ok()) {
      std::lock_guard<std::mutex> guard(bt_mutex_);
      if (!active->final_result.has_value()) {
        active->final_result = std::make_pair(false, "ROS shutdown");
      }
      break;
    }
    std::unique_lock<std::mutex> lock(bt_mutex_);
    bt_condition_.wait_for(lock, std::chrono::duration<double>(0.1));
  }
  const auto final_result = active->final_result.value_or(
      std::make_pair(false, std::string("ROS shutdown")));
  {
    std::lock_guard<std::mutex> guard(bt_mutex_);
    active->waiters -= 1;
    if (active->waiters == 0 && bt_request_ == active) {
      bt_request_ = nullptr;
    }
  }
  response.success = final_result.first;
  response.message = final_result.second;
  return true;
}

std::pair<bool, std::string> RealControlNode::actuateGripper(
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

void RealControlNode::runManualGripper(const std::string& action) {
  // 执行手柄触发的夹爪服务并记录结果；结束（含失败）释放占用。
  const auto [ok, message] = actuateGripper(action);
  gripper_service_lock_.unlock();
  gripper_thread_active_ = false;
  if (ok) {
    ROS_INFO("Gripper %s done: %s", action.c_str(), message.c_str());
  } else {
    ROS_WARN("Gripper %s failed: %s", action.c_str(), message.c_str());
  }
}

void RealControlNode::maybeTriggerManualGripper(double dpad) {
  // 方向键位置选择夹爪目标（+1 张开/-1 闭合），仅指令变化时触发。
  // 服务同步阻塞且并发请求会被拒：非阻塞锁保证同时只有一个调用，忙碌时
  // 跳过（方向键保持时按控制帧自动重试），后台线程执行，避免最长约 2.5s
  // 的服务验证卡住 30Hz 控制循环。
  const std::string action =
      dpad > 0.5 ? "open" : (dpad < -0.5 ? "clamp" : "");
  if (action.empty() || gripper_last_cmd_ == action) {
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
  gripper_thread_ = std::thread([this, action]() { runManualGripper(action); });
}




// ---------------------------------------------------------------------------
// 模式入口。
// ---------------------------------------------------------------------------

void RealControlNode::startRealClimb(const JointAngles& q_cur,
                                     bool controls_ready) {
  // 在回站和关节反馈门限通过后进入诊断回放 C1。
  if (!enable_real_climb_) {
    ROS_WARN_THROTTLE(2.0, "X ignored: enable_real_climb is false");
    return;
  }
  if (!controls_ready || state_ != MachineState::Hold ||
      controller_->mode != GraspController::kApproach) {
    ROS_WARN_THROTTLE(2.0,
                      "X ignored: reset must finish and controls must be fresh");
    return;
  }
  const NavigationState navigation = navigation_->motionSnapshot();
  const ImuInput::Snapshot imu = imu_->snapshot();
  try {
    ClimbCompactConfig config = controller_->climb_mode.loadConfig();
    config = selectCompactClimbSide(config, climb_side_);
    config.settle_gate.max_foot_target_error_m = climb_foot_gate_m_;
    controller_->enterClimb(q_cur, config, 0, std::nullopt, true);
  } catch (const ConfigError& error) {
    ROS_WARN("X ignored: compact entry gate failed: %s", error.what());
    return;
  }
  ROS_INFO("Compact climb side selected: %s", climb_side_.c_str());
  real_climb_monitor_active_ = navigation.valid && imu.valid;
  if (real_climb_monitor_active_) {
    climb_start_navigation_ = navigation;
    have_climb_start_navigation_ = true;
    climb_start_imu_rotation_ = imu.rotation;
    have_climb_start_imu_ = true;
    climb_start_planned_pose_ = worldFromBase(*controller_->climb_mode.base_pose);
    have_climb_start_planned_ = true;
    ROS_INFO(
        "Diagnostic replay: optional IMU/RTK relative-motion monitoring enabled");
  } else {
    have_climb_start_navigation_ = false;
    have_climb_start_imu_ = false;
    have_climb_start_planned_ = false;
    ROS_WARN(
        "Diagnostic replay: IMU or RTK/LoRa unavailable at start; "
        "optional relative-motion monitoring disabled for this replay");
  }
  climb_bad_frames_ = 0;
  climb_good_frames_ = 0;
  resetRealClimbSpeedDiagnostic();
  command_.setZero();
  local_climb_armed_ = false;
  local_climb_entry_q_.reset();
  state_ = MachineState::Running;
  ROS_INFO(
      "X accepted: diagnostic replay C1-C%d started with source=%s "
      "foot_gate_m=%.3f; joint-feedback gates are not contact/load evidence",
      static_cast<int>(controller_->climb_mode.stage_names.size()),
      "hardware_feedback", climb_foot_gate_m_);
}

void RealControlNode::ensureDockMode() {
  // 按显式实机请求创建 DockMode，不给普通控制器增加 ROS 资源。
  if (dock_mode_instance_ != nullptr) {
    return;
  }
  ros::NodeHandle pnh("~");
  dock_perception_ = std::make_unique<DockPerceptionRos>(
      pnh.param("dock_max_perception_age", 0.35),
      pnh.param<std::string>("dock_lock_frame", "dock_lock_center"),
      pnh.param<std::string>("dock_pin_frame_prefix", "dock_pin_from_tag_"),
      dock_system_config_);
  dock_mode_instance_ = std::make_unique<DockMode>(
      controller_.get(), dock_perception_.get(),
      pnh.param("dock_require_lock_confirmation", false),
      pnh.param("dock_linear_speed_m_s", 0.05),
      pnh.param("dock_update_rate_hz", 30.0),
      pnh.param("dock_perception_rate_hz", 10.0),
      pnh.param("dock_leg_lift_speed_m_s", 0.05),
      pnh.param("dock_sit_settle_duration_s", 0.5),
      pnh.param("dock_leg_lift_height_m", 0.06),
      pnh.param("dock_leg_lift_level_tolerance_m", 0.003),
      [](bool is_warn, const std::string& message) {
        if (is_warn) {
          ROS_WARN("%s", message.c_str());
        } else {
          ROS_INFO("%s", message.c_str());
        }
      });
  controller_->attachDockMode(dock_mode_instance_.get());
}

void RealControlNode::startRealDock(const JointAngles& q_cur,
                                    bool controls_ready) {
  // 在实机输入新鲜时进入 DockMode 并恢复缓存的攀爬末姿态。
  if (!enable_real_dock_) {
    ROS_WARN_THROTTLE(2.0, "Y ignored: enable_real_dock is false");
    return;
  }
  if (!controls_ready || state_ != MachineState::Hold) {
    ROS_WARN_THROTTLE(2.0,
                      "Y ignored: reset must finish and controls must be fresh");
    return;
  }
  try {
    if (dock_require_real_calibrated_ && !dock_allow_uncalibrated_) {
      const DockSystem dock_system = loadDockSystem(dock_system_config_);
      if (!dock_system.real_calibrated) {
        ROS_WARN_THROTTLE(2.0, "Y ignored: dock_system.yaml is not real-calibrated");
        return;
      }
    }
    ensureDockMode();
    controller_->enterDock(q_cur);
  } catch (const std::exception& error) {
    ROS_WARN("Y ignored: DockMode entry failed: %s", error.what());
    return;
  }
  command_.setZero();
  dock_session_started_at_ = nowSeconds();
  state_ = MachineState::Running;
  ROS_INFO("Y accepted: entering climb terminal posture, then starting visual docking");
}

std::optional<bool> RealControlNode::dockLockConfirmed() {
  // 只接受本次 Y 后、未过期的锁紧确认。
  const auto [lock_confirmed, received_at] = lock_confirmation_->snapshotWithTime();
  const double now = nowSeconds();
  const double age = now - received_at;
  if (received_at < dock_session_started_at_ || age < 0.0 ||
      age > dock_lock_confirmation_max_age_) {
    return std::nullopt;
  }
  return lock_confirmed;
}

void RealControlNode::holdMotion(const std::string& reason, bool log) {
  // 停止推进步态并保持上力；再次按 A 才能恢复。
  if (state_ != MachineState::Running) {
    return;
  }
  if (controller_->mode == GraspController::kDock) {
    if (dock_mode_instance_ != nullptr) {
      dock_mode_instance_->failExecution("DockMode stopped: " + reason);
    }
    state_ = MachineState::Hold;
    command_.setZero();
    if (log) {
      ROS_WARN("DockMode failed into HOLD: %s", reason.c_str());
    }
    return;
  }
  controller_->approach_mode.cancelAutonomousApproach(reason);
  controller_->holdClimb();
  flushRealClimbSpeedDiagnostic("hold: " + reason);
  state_ = MachineState::Hold;
  command_.setZero();
  if (log) {
    ROS_INFO("Motion paused: %s", reason.c_str());
  }
}

// ---------------------------------------------------------------------------
// 按钮仲裁。
// ---------------------------------------------------------------------------

void RealControlNode::processButtons(const Eigen::VectorXi& button_presses,
                                     bool controls_ready,
                                     const JointAngles& q_cur) {
  // 处理一次按钮事件；B 不依赖 Joy 或关节反馈是否有效。
  const bool a_pressed = static_cast<bool>(readAxis(button_presses.cast<double>(), button_a_));
  const bool b_pressed = static_cast<bool>(readAxis(button_presses.cast<double>(), button_b_));
  const bool x_pressed = static_cast<bool>(readAxis(button_presses.cast<double>(), button_x_));
  const bool y_pressed = static_cast<bool>(readAxis(button_presses.cast<double>(), button_y_));
  if (b_pressed) {
    // B 是唯一无条件抢占：无论 BT 服务是否在等待，都先唤醒调用方，
    // 再沿用原有回站路径。
    abortBtRequest("aborted by B");
    local_climb_armed_ = false;
    local_climb_entry_q_.reset();
    flushRealClimbSpeedDiagnostic("reset");
    state_ = MachineState::Resetting;
    controller_->reset_active = false;
    controller_->mission.cancel("reset requested by B");
    controller_->abortClimb();
    if (dock_mode_instance_ != nullptr && dock_mode_instance_->active) {
      dock_mode_instance_->exit();
    }
    manual_override_ = false;
    command_.setZero();
    ROS_INFO("B pressed: returning to stand");
    return;
  }

  if (x_pressed && y_pressed) {
    ROS_WARN_THROTTLE(2.0, "X/Y ignored: climb and dock requests conflict");
    return;
  }
  if (x_pressed) {
    startRealClimb(q_cur, controls_ready);
    return;
  }
  if (y_pressed) {
    startRealDock(q_cur, controls_ready);
    return;
  }
  if (!a_pressed) {
    return;
  }

  if (local_climb_armed_) {
    ROS_WARN_THROTTLE(2.0, "A ignored: Isaac compact C1 is armed; press X or B");
    return;
  }
  if (controller_->mode == GraspController::kDock) {
    ROS_WARN_THROTTLE(2.0, "A ignored while DockMode is active");
    return;
  }
  if (controller_->mode == GraspController::kClimb) {
    if (state_ == MachineState::Hold) {
      if (!real_climb_monitor_active_) {
        controller_->resumeClimb();
        state_ = MachineState::Running;
        ROS_INFO(
            "Diagnostic replay CLIMB resumed without optional motion monitoring");
      } else {
        const auto [okay, reason] = realClimbObservation();
        if (okay && climb_good_frames_ >= real_climb_persistence_frames_) {
          controller_->resumeClimb();
          state_ = MachineState::Running;
          ROS_INFO("Diagnostic replay CLIMB resumed after IMU/RTK persistence");
        } else {
          ROS_WARN("A ignored: CLIMB HOLD persists: %s", reason.c_str());
        }
      }
    } else if (state_ == MachineState::Running) {
      controller_->holdClimb();
      flushRealClimbSpeedDiagnostic("paused by A");
      state_ = MachineState::Hold;
      command_.setZero();
      ROS_INFO("CLIMB paused by A");
    }
    return;
  }

  if (!controls_ready) {
    ROS_WARN("A ignored: waiting for valid control inputs");
  } else if (state_ == MachineState::Hold) {
    state_ = MachineState::Running;
    if (control_source_ == "navigation" && !manual_override_) {
      const ApproachPlan result =
          controller_->startAutonomousApproach(navigation_->snapshot());
      if (result.failed) {
        ROS_WARN("Navigation holding: %s; move joystick to take over",
                 result.reason.c_str());
      } else {
        ROS_INFO("Navigation started: side=%s", result.target_side.c_str());
      }
    } else {
      ROS_INFO("Motion enabled");
    }
  } else if (state_ == MachineState::Running) {
    holdMotion("paused by A");
  } else {
    ROS_WARN("A ignored: press B and wait for stand first");
  }
}

// ---------------------------------------------------------------------------
// 攀爬监控与诊断。
// ---------------------------------------------------------------------------

namespace {

// 6×3 关节矩阵的最大绝对差分。
double maxAbsDiffLocal(const JointAngles& a, const JointAngles& b) {
  double worst = 0.0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    worst = std::max(worst, (a[leg] - b[leg]).cwiseAbs().maxCoeff());
  }
  return worst;
}

}  // namespace

std::pair<bool, std::string> RealControlNode::realClimbObservation() {
  // 比较部署传感器相对运动与当前计划机身运动。
  const NavigationState navigation = navigation_->motionSnapshot();
  const ImuInput::Snapshot imu = imu_->snapshot();
  if (!navigation.valid) {
    return {false, "RTK/LoRa navigation pose is stale or invalid"};
  }
  if (!imu.valid) {
    return {false, "IMU is stale or invalid"};
  }
  if (!have_climb_start_navigation_ || !have_climb_start_imu_ ||
      !have_climb_start_planned_ || !controller_->climb_mode.base_pose.has_value()) {
    return {false, "climb safety reference is missing"};
  }
  const Matrix4d start_xiaolan_from_base =
      navigation.pv_from_xiaolan.inverse() * climb_start_navigation_.pv_from_base;
  const Matrix4d current_xiaolan_from_base =
      navigation.pv_from_xiaolan.inverse() * navigation.pv_from_base;
  const Matrix4d actual_relative =
      start_xiaolan_from_base.inverse() * current_xiaolan_from_base;
  const Matrix4d planned_now =
      worldFromBase(*controller_->climb_mode.base_pose);
  const Matrix4d planned_relative =
      climb_start_planned_pose_.inverse() * planned_now;
  const double position_error =
      (actual_relative.block<3, 1>(0, 3) - planned_relative.block<3, 1>(0, 3))
          .norm();
  const Matrix3d imu_relative =
      climb_start_imu_rotation_.transpose() * imu.rotation;
  const double orientation_error = rotationAngle(
      planned_relative.block<3, 3>(0, 0).transpose() * imu_relative);
  const double angular_speed = imu.angular_velocity.norm();
  if (position_error > real_climb_max_position_error_) {
    return {false, formatMessage("relative RTK position error %.3f m", position_error)};
  }
  if (orientation_error > real_climb_max_orientation_error_) {
    return {false, formatMessage("IMU attitude error %.1f deg",
                                 orientation_error * 180.0 / M_PI)};
  }
  if (angular_speed > real_climb_max_angular_speed_) {
    return {false, formatMessage("IMU angular speed %.1f deg/s",
                                 angular_speed * 180.0 / M_PI)};
  }
  return {true, ""};
}

void RealControlNode::monitorRealClimb() {
  // 持续观察已启用的可选相对运动监控。
  if (!real_climb_monitor_active_ ||
      controller_->mode != GraspController::kClimb) {
    return;
  }
  const auto [okay, reason] = realClimbObservation();
  if (okay) {
    climb_good_frames_ += 1;
    climb_bad_frames_ = 0;
    return;
  }
  climb_good_frames_ = 0;
  if (reason.find("stale") != std::string::npos ||
      reason.find("invalid") != std::string::npos ||
      reason.find("missing") != std::string::npos) {
    climb_bad_frames_ = real_climb_persistence_frames_;
  } else {
    climb_bad_frames_ += 1;
  }
  if (controller_->climb_mode.state == ClimbState::Running &&
      climb_bad_frames_ >= real_climb_persistence_frames_) {
    controller_->holdClimb();
    flushRealClimbSpeedDiagnostic("safety hold");
    state_ = MachineState::Hold;
    command_.setZero();
    ROS_WARN(
        "Diagnostic replay CLIMB HOLD: %s; joint-feedback gates are not "
        "contact/load evidence",
        reason.c_str());
  }
}

void RealControlNode::resetRealClimbSpeedDiagnostic() {
  climb_speed_diagnostic_.reset();
}

void RealControlNode::warnHardwareClimbPhaseHold() {
  // 报告实机反馈门冻结，保留完整足端与关节定位。
  if (controller_->mode != GraspController::kClimb) {
    return;
  }
  const ClimbMode& climb = controller_->climb_mode;
  if (climb.state != ClimbState::Running || !climb.hardware_execution ||
      !climb.last_phase_hold) {
    return;
  }
  ROS_WARN_THROTTLE(
      0.5, "CLIMB PHASE HOLD: source=%s stage=%s %s collision_guard_hold=%s",
      "hardware_feedback", climb.phase.c_str(),
      climb.trackingDiagnosticSummary().c_str(),
      climb.last_collision_guard_hold ? "true" : "false");
}

void RealControlNode::infoHardwareClimbActiveTrace() {
  // 周期输出活动腿的 base_link 规划/反馈对照，不推断因果。
  if (controller_->mode != GraspController::kClimb) {
    return;
  }
  const ClimbMode& climb = controller_->climb_mode;
  if (climb.state != ClimbState::Running || !climb.hardware_execution) {
    return;
  }
  ROS_INFO_THROTTLE(
      1.0,
      "CLIMB ACTIVE LEG TRACE: source=%s stage=%s %s; diagnostic only: "
      "desired_z/planned lift low suggests planning, while desired-correct "
      "actual lag with same-leg joint error suggests load/execution/feedback; "
      "not causal proof",
      "hardware_feedback", climb.last_diagnostic_stage_name.c_str(),
      climb.activeLegDiagnosticSummary().c_str());
}

void RealControlNode::flushRealClimbSpeedDiagnostic(const std::string& reason) {
  // 输出并丢弃已完成阶段的聚合，不影响运动。
  const ClimbSpeedDiagnostic* item = climb_speed_diagnostic_.get();
  if (item == nullptr) {
    return;
  }
  const double peak_command = item->peak_command_speed_rad_s;
  const double peak_measured = item->peak_measured_speed_rad_s;
  const double peak_ratio =
      peak_command > 0.0 ? peak_measured / peak_command
                         : std::numeric_limits<double>::quiet_NaN();
  const double mean_ratio =
      item->ratio_count > 0
          ? item->ratio_sum / item->ratio_count
          : std::numeric_limits<double>::quiet_NaN();
  ROS_INFO(
      "CLIMB speed diagnostic-only stage=%s reason=%s peak_cmd=%.3f "
      "peak_meas=%.3f R_v=%.3f mean_meas_cmd=%.3f samples=%d "
      "peak_tracking=%.4f clip_joints=%d guard_holds=%d; mean ratio uses "
      "per-joint |cmd_speed| >= 0.05 rad/s and is not contact/load evidence",
      item->stage.c_str(), reason.c_str(), peak_command, peak_measured,
      peak_ratio, mean_ratio, item->ratio_count, item->peak_tracking_error_rad,
      item->velocity_limit_clip_count, item->collision_guard_hold_count);
  climb_speed_diagnostic_.reset();
}

void RealControlNode::recordRealClimbSpeedDiagnostic(
    const std::string& stage, const JointAngles& q_cur, const JointAngles& q_des,
    const std::array<double, 6>& feedback_stamp, double sample_time) {
  // 聚合实机 CLIMB 反馈；从不门控阶段推进。
  if (controller_->mode != GraspController::kClimb) {
    return;
  }
  ClimbSpeedDiagnostic* item = climb_speed_diagnostic_.get();
  if (item == nullptr || item->stage != stage) {
    flushRealClimbSpeedDiagnostic("stage transition");
    auto fresh = std::make_unique<ClimbSpeedDiagnostic>();
    fresh->stage = stage;
    fresh->previous_q_cur = q_cur;
    fresh->previous_q_des = q_des;
    fresh->previous_feedback_stamp = feedback_stamp;
    fresh->previous_sample_time = sample_time;
    fresh->peak_tracking_error_rad = maxAbsDiffLocal(q_des, q_cur);
    fresh->velocity_limit_clip_count =
        controller_->last_update_velocity_limit_clip_count;
    fresh->collision_guard_hold_count =
        controller_->last_update_collision_guard_hold_count;
    climb_speed_diagnostic_ = std::move(fresh);
    return;
  }
  const double command_dt = sample_time - item->previous_sample_time;
  bool feedback_dt_positive = true;
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (feedback_stamp[leg] - item->previous_feedback_stamp[leg] <= 0.0) {
      feedback_dt_positive = false;
      break;
    }
  }
  if (command_dt <= 0.0 || !feedback_dt_positive) {
    item->previous_q_cur = q_cur;
    item->previous_q_des = q_des;
    item->previous_feedback_stamp = feedback_stamp;
    item->previous_sample_time = sample_time;
    return;
  }
  double peak_command_speed = 0.0;
  double peak_measured_speed = 0.0;
  double ratio_sum = 0.0;
  int ratio_count = 0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    for (int joint = 0; joint < kJointCount; ++joint) {
      const double command_speed =
          std::abs(q_des[leg][joint] - item->previous_q_des[leg][joint]) / command_dt;
      const double measured_speed =
          std::abs(q_cur[leg][joint] - item->previous_q_cur[leg][joint]) /
          (feedback_stamp[leg] - item->previous_feedback_stamp[leg]);
      peak_command_speed = std::max(peak_command_speed, command_speed);
      peak_measured_speed = std::max(peak_measured_speed, measured_speed);
      if (command_speed >= 0.05) {
        ratio_sum += measured_speed / command_speed;
        ++ratio_count;
      }
    }
  }
  item->peak_command_speed_rad_s =
      std::max(item->peak_command_speed_rad_s, peak_command_speed);
  item->peak_measured_speed_rad_s =
      std::max(item->peak_measured_speed_rad_s, peak_measured_speed);
  item->peak_tracking_error_rad =
      std::max(item->peak_tracking_error_rad, maxAbsDiffLocal(q_des, q_cur));
  item->ratio_sum += ratio_sum;
  item->ratio_count += ratio_count;
  item->velocity_limit_clip_count +=
      controller_->last_update_velocity_limit_clip_count;
  item->collision_guard_hold_count +=
      controller_->last_update_collision_guard_hold_count;
  item->previous_q_cur = q_cur;
  item->previous_q_des = q_des;
  item->previous_feedback_stamp = feedback_stamp;
  item->previous_sample_time = sample_time;
}




// ---------------------------------------------------------------------------
// BT 请求启动与终态。
// ---------------------------------------------------------------------------

void RealControlNode::startBtRequest(BtRequest& request, const JointAngles& q_cur) {
  // 在已有反馈帧内一次性进入现有模式；后续只由 update 推进。
  if (request.started) {
    return;
  }
  const std::string mode = request.mode;
  request.started = true;
  command_.setZero();
  manual_override_ = false;
  if (mode == "home") {
    controller_->mission.cancel("BT home");
    controller_->abortClimb();
    if (dock_mode_instance_ != nullptr && dock_mode_instance_->active) {
      dock_mode_instance_->exit();
    }
    controller_->reset_active = false;
    state_ = MachineState::Resetting;
    return;
  }
  if (mode == "walk") {
    controller_->mission.cancel("BT walk");
    controller_->abortClimb();
    controller_->exitDock(q_cur);
    controller_->setMode(GraspController::kApproach);
    controller_->approach_mode.cancelAutonomousApproach("BT walk");
    state_ = MachineState::Running;
    return;
  }
  if (mode == "approach") {
    if (control_source_ != "navigation") {
      finishBtRequest(btRequestSnapshot(), false,
                      "approach requires ~control_source:=navigation");
      return;
    }
    controller_->mission.cancel("BT approach");
    controller_->abortClimb();
    controller_->exitDock(q_cur);
    controller_->setMode(GraspController::kApproach);
    const ApproachPlan result =
        controller_->startAutonomousApproach(navigation_->snapshot());
    if (result.failed) {
      finishBtRequest(btRequestSnapshot(), false, result.reason);
      return;
    }
    state_ = MachineState::Running;
    return;
  }
  if (mode == "climb") {
    state_ = MachineState::Hold;
    const MachineState before = state_;
    startRealClimb(q_cur, true);
    if (state_ != MachineState::Running || before != MachineState::Hold) {
      finishBtRequest(btRequestSnapshot(), false, "climb entry rejected");
    }
    return;
  }
  if (mode == "dock") {
    state_ = MachineState::Hold;
    startRealDock(q_cur, true);
    if (state_ != MachineState::Running) {
      finishBtRequest(btRequestSnapshot(), false, "dock entry rejected");
    }
    return;
  }
  if (mode == "release") {
    const auto [ok, message] = actuateGripper("open");
    finishBtRequest(btRequestSnapshot(), ok, message);
  }
}

void RealControlNode::finishBtModeIfTerminal(
    const std::shared_ptr<BtRequest>& request) {
  if (request == nullptr || request->final_result.has_value()) {
    return;
  }
  const std::string mode = request->mode;
  if (mode == "home" && state_ == MachineState::Hold &&
      !controller_->reset_active) {
    const auto [ok, message] = actuateGripper("open");
    finishBtRequest(request, ok, message);
  } else if (mode == "walk") {
    bool remote_reset = false;
    std::string remote_mode;
    {
      std::lock_guard<std::mutex> guard(bt_mutex_);
      remote_reset = bt_remote_valid_ && bt_remote_.reset_edge;
      remote_mode = bt_remote_.mode;
    }
    if (!bt_remote_valid_ || remote_reset || remote_mode != "walk") {
      command_.setZero();
      finishBtRequest(request, true, "walk stopped by remote");
    }
  } else if (mode == "approach") {
    const ApproachPlan& plan = controller_->approach_mode.approach_plan;
    if (plan.failed) {
      finishBtRequest(request, false, plan.reason);
    } else if (plan.ready_for_climb) {
      state_ = MachineState::Hold;
      command_.setZero();
      finishBtRequest(request, true, "ready for climb");
    }
  } else if (mode == "climb") {
    if (controller_->climb_mode.state == ClimbState::Done) {
      finishBtRequest(request, true, "climb complete");
    } else if (controller_->climb_mode.state == ClimbState::Failed) {
      finishBtRequest(request, false, controller_->climb_mode.failure_reason);
    }
  } else if (mode == "dock") {
    const DockMode* dock = dock_mode_instance_.get();
    if (dock != nullptr && dock->state == DockMode::kSuccess &&
        state_ == MachineState::Hold) {
      if (!request->dock_clamped) {
        request->dock_clamped = true;
        const auto [ok, message] = actuateGripper("clamp");
        finishBtRequest(request, ok, message);
      }
    } else if (dock != nullptr && dock->state == DockMode::kFailed) {
      finishBtRequest(request, false, dock->reason);
    }
  }
}

bool RealControlNode::btHoldIsActive(double now) {
  const std::shared_ptr<BtRequest> request = btRequestSnapshot();
  bool active = request != nullptr && now < bt_hold_deadline_;
  if (active && !bt_hold_active_) {
    if (controller_->mode == GraspController::kClimb) {
      controller_->holdClimb();
    }
    bt_hold_active_ = true;
  } else if (!active && bt_hold_active_) {
    if (controller_->mode == GraspController::kClimb) {
      controller_->resumeClimb();
    }
    bt_hold_active_ = false;
  }
  return active;
}

// ---------------------------------------------------------------------------
// 反馈回调。
// ---------------------------------------------------------------------------

void RealControlNode::feedbackCallback(int leg_index, const std::string& leg_name,
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
// 每帧主逻辑。
// ---------------------------------------------------------------------------

std::optional<JointAngles> RealControlNode::updateControl(
    const JointAngles& q_cur, const Eigen::VectorXd& axes,
    const Eigen::VectorXi& button_presses, double joy_stamp, double now,
    bool feedback_ready, const std::array<double, 6>* feedback_stamp) {
  // 用一帧完整反馈处理 B/A 和运动指令，返回 18 关节目标。
  const bool joy_fresh =
      joy_stamp > 0.0 && 0.0 <= now - joy_stamp && now - joy_stamp <= max_joy_age_;
  std::shared_ptr<BtRequest> bt_request = btRequestSnapshot();
  if (bt_request != nullptr) {
    // BT 活动期间 A/X/Y 不参与仲裁，只有 B 保留全局中止语义。
    const int bt_size = std::max<int>(button_presses.size(), button_b_ + 1);
    Eigen::VectorXi bt_buttons = Eigen::VectorXi::Zero(bt_size);
    if (button_b_ < button_presses.size()) {
      bt_buttons[button_b_] = button_presses[button_b_];
    }
    bool remote_reset = false;
    {
      std::lock_guard<std::mutex> guard(bt_mutex_);
      remote_reset = bt_remote_valid_ && bt_remote_.reset_edge;
    }
    if (remote_reset && button_b_ < bt_buttons.size()) {
      bt_buttons[button_b_] = 1;
    }
    processButtons(bt_buttons, feedback_ready, q_cur);
    bt_request = btRequestSnapshot();
    if (bt_request != nullptr) {
      startBtRequest(*bt_request, q_cur);
    }
  } else {
    processButtons(button_presses, joy_fresh && feedback_ready, q_cur);
  }

  // 夹爪控制：方向键轴 +1=张开, -1=闭合；经 GripperAct 服务执行（open/clamp
  // 带到位验证），行为树活动期间不响应。
  if (joy_fresh && bt_request == nullptr) {
    maybeTriggerManualGripper(readAxis(axes, axis_gripper_));
  }

  const bool bt_walk = bt_request != nullptr && bt_request->mode == "walk";
  if (!joy_fresh && (bt_request == nullptr || bt_walk)) {
    // 自动 BT 模式不依赖 Joy；walk 则仍是原始手柄步态，不能复用一帧已失效
    // 的非零速度。沿既有 HOLD 路径以零命令完成安全停步。
    command_.setZero();
    holdMotion("joystick lost");
    if (bt_walk) {
      finishBtRequest(bt_request, false, "joystick lost");
    }
  }

  // 第一次 B 以前不发布目标，Servo 保持卸力并只读反馈。
  if (state_ == MachineState::WaitB) {
    return std::nullopt;
  }

  if (bt_request != nullptr && btHoldIsActive(now)) {
    // HOLD 只冻结现有模式的推进：Approach/Dock 不走旧的取消/失败路径；
    // Climb 由 hold/resume 保留其内部阶段状态。home 同样暂停其平滑插值。
    command_.setZero();
    return controller_->q_des;
  }

  if (local_climb_armed_ && state_ == MachineState::Hold) {
    return local_climb_entry_q_;
  }

  if (state_ == MachineState::Resetting) {
    if (!controller_->reset_active) {
      controller_->resetToStand(q_cur);
    }
    const JointAngles q_des = controller_->update(q_cur, command_);
    if (!controller_->reset_active) {
      state_ = MachineState::Hold;
      ROS_INFO("Stand initialization complete; press A to move");
    }
    finishBtModeIfTerminal(bt_request);
    return q_des;
  }

  std::optional<NavigationState> navigation_state;
  std::optional<DockRobotState> dock_robot_state;
  std::optional<std::string> climb_stage;
  if (controller_->mode == GraspController::kClimb &&
      controller_->climb_mode.state == ClimbState::Running) {
    climb_stage = controller_->climb_mode.phase;
  }
  if (state_ == MachineState::Running) {
    const std::string bt_mode = bt_request != nullptr ? bt_request->mode : "";
    if (controller_->mode == GraspController::kDock) {
      command_.setZero();
      DockRobotState dock_state;
      dock_state.joints = q_cur;
      dock_state.lock_confirmed = dockLockConfirmed();
      dock_robot_state = std::move(dock_state);
    } else if (bt_mode == "walk") {
      command_ = makeCommand(axes);
    } else if (bt_mode == "approach") {
      command_.setZero();
      navigation_state = navigation_->snapshot();
    } else if (!bt_mode.empty()) {
      command_.setZero();
    } else {
      if (control_source_ == "navigation" && !manual_override_ &&
          manualCommandActive(axes)) {
        manual_override_ = true;
        controller_->approach_mode.cancelAutonomousApproach("joystick takeover");
        ROS_INFO("Joystick took over navigation");
      }
      if (manual_override_ || control_source_ == "teleop") {
        command_ = makeCommand(axes);
      } else {
        command_.setZero();
        navigation_state = navigation_->snapshot();
      }
    }
  } else {
    // 暂停时让当前摆动腿先落地再停止。
    command_.setZero();
  }

  // update() 内部完成足端规划、工作空间检查和 DLS 逆运动学；
  // reset_active 期间则输出五次曲线回站立轨迹。
  const JointAngles q_des = controller_->update(
      q_cur, command_,
      navigation_state.has_value() ? &navigation_state.value() : nullptr,
      dock_robot_state.has_value() ? &dock_robot_state.value() : nullptr);
  warnHardwareClimbPhaseHold();
  infoHardwareClimbActiveTrace();
  if (climb_stage.has_value() && feedback_stamp != nullptr) {
    recordRealClimbSpeedDiagnostic(*climb_stage, q_cur, q_des, *feedback_stamp,
                                   now);
  }
  if (controller_->mode == GraspController::kDock &&
      dock_mode_instance_ != nullptr &&
      DockMode::isTerminalState(dock_mode_instance_->state) &&
      state_ == MachineState::Running) {
    state_ = MachineState::Hold;
    ROS_INFO("DockMode terminal HOLD: %s", dock_mode_instance_->reason.c_str());
  }
  if (controller_->mode == GraspController::kClimb) {
    const bool climb_terminal =
        controller_->climb_mode.state == ClimbState::Done ||
        controller_->climb_mode.state == ClimbState::Failed;
    if (climb_terminal && state_ == MachineState::Running) {
      flushRealClimbSpeedDiagnostic("terminal");
      state_ = MachineState::Hold;
      command_.setZero();
      if (controller_->climb_mode.state == ClimbState::Done) {
        ROS_INFO("CLIMB DONE: HOLD; press Y to request docking");
      } else {
        ROS_WARN("CLIMB FAILED: HOLD: %s",
                 controller_->climb_mode.failure_reason.c_str());
      }
    }
    if (real_climb_monitor_active_) {
      monitorRealClimb();
    }
  }
  finishBtModeIfTerminal(bt_request);
  return q_des;
}

void RealControlNode::publishTargets(const JointAngles& q_des) {
  // 把控制器 6×3 关节目标拆成六个固定 10 元素消息。
  for (int leg = 0; leg < kLegCount; ++leg) {
    std_msgs::Float64MultiArray message;
    message.data.resize(10);
    message.data[0] = 1.0;  // power=1.0 表示上力
    for (int joint = 0; joint < kJointCount; ++joint) {
      message.data[1 + joint] = q_des[leg][joint];
    }
    des_publishers_[kLegNames[leg]].publish(message);
  }
}

void RealControlNode::step() {
  // 实机循环：读取完整反馈，计算一次目标并发布给两块 Servo 板。
  const auto tick_start = ros::WallTime::now();

  JointAngles q_cur_snapshot;
  std::array<double, 6> feedback_stamp_snapshot{};
  Eigen::VectorXd axes_snapshot;
  double joy_stamp = 0.0;
  bool feedback_ready = false;
  bool complete_new_frame = false;
  bool b_pending = false;
  Eigen::VectorXi button_presses;

  {
    std::lock_guard<std::mutex> guard(lock_);
    q_cur_snapshot = q_cur_;
    feedback_stamp_snapshot = feedback_stamp_;
    axes_snapshot = axes_;
    joy_stamp = joy_stamp_;
    // 在反馈快照之后取时钟，避免回调刚写入的新时间戳落到 now 之后。
    const double now = nowSeconds();
    std::tie(feedback_ready, complete_new_frame) = feedbackFrameState(
        q_cur_snapshot, feedback_stamp_snapshot, last_control_feedback_stamp_,
        now, max_feedback_age_, max_feedback_skew_);
    b_pending = button_b_ < button_press_latch_.size() &&
                button_press_latch_[button_b_] != 0;
    if (complete_new_frame || !feedback_ready || b_pending) {
      button_presses = button_press_latch_;
      button_press_latch_.setZero();
    } else {
      // A/X/Y 最多等待另一个板完成本轮反馈；B 仍在上方立即消费。
      button_presses = Eigen::VectorXi::Zero(button_press_latch_.size());
    }
    if (complete_new_frame) {
      last_control_feedback_stamp_ = feedback_stamp_snapshot;
    }
  }

  const double now = nowSeconds();
  if (!feedback_ready) {
    // B 已经被转换为 RESETTING 状态，即使反馈暂时不可用也不会丢失。
    processButtons(button_presses, false, q_cur_snapshot);
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
    if (b_pending) {
      processButtons(button_presses, false, q_cur_snapshot);
    }
    return;
  }

  const std::optional<JointAngles> q_des = updateControl(
      q_cur_snapshot, axes_snapshot, button_presses, joy_stamp, now,
      feedback_ready, &feedback_stamp_snapshot);
  if (q_des.has_value()) {
    for (int leg = 0; leg < kLegCount; ++leg) {
      if (!(*q_des)[leg].allFinite()) {
        // 初始化、暂停和行走都保持舵机上力；暂停不等于卸力。
        // 产生非有限目标意味着控制器内部错误，直接终止节点。
        ROS_FATAL("Controller produced non-finite q_des");
        ros::shutdown();
        return;
      }
    }
    // 初始化、暂停和行走都保持舵机上力；暂停不等于卸力。
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

}  // namespace grasp_hexapod_control_cpp
