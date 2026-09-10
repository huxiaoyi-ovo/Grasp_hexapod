// 实机视觉对接模式（对应 Python dock_mode.py::DockMode）。
//
// DockMode 只负责三件事：接管攀爬结束关节姿态、读取 AprilTag 感知、把闭环
// 机身修正转换为六足 base_link 目标。关节反馈、DLS 逆运动学和舵机下发统一
// 走 GraspController 链路。tf2 感知通过 PerceptionInterface 注入，核心库
// 不依赖 ROS；阶段日志通过 logger 回调输出。
#pragma once

#include "grasp_hexapod_control_cpp/types.h"

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace grasp_hexapod_control_cpp {

class GraspController;

// 感知结果（dock_mode.py::PerceptionResult）。
struct PerceptionResult {
  bool valid = false;
  Matrix4d lock_from_pin = Matrix4d::Identity();
  double stamp = 0.0;
  std::vector<int> decoded_ids;
  std::vector<int> inferred_ids;
  double confidence = 0.0;
  double position_spread = std::numeric_limits<double>::infinity();
  double angle_spread = std::numeric_limits<double>::infinity();
  std::string reason = "no perception result";
};

// DockPerception 的抽象接口；ROS 实现在壳层用 tf2 实现。
class PerceptionInterface {
 public:
  virtual ~PerceptionInterface() = default;
  virtual void reset() = 0;
  virtual PerceptionResult latest() = 0;
};

// 对接执行所需的机器人状态（dock_mode.py::DockRobotState）。
struct DockRobotState {
  std::optional<JointAngles> joints;
  std::optional<Matrix4d> lock_from_pin;
  std::optional<std::vector<int>> decoded_ids;
  std::optional<bool> lock_confirmed;
};

struct DockResult {
  std::optional<FootPositions> foot_positions_base;
  bool active = false;
  bool success = false;
  bool failed = false;
  bool request_lock = false;
  std::string reason;
  std::optional<JointAngles> joint_positions;
  std::string state;
};

class DockMode {
 public:
  // 状态常量与 Python 字符串一一对应。
  static constexpr const char* kIdle = "idle";
  static constexpr const char* kClimbTerminalEntry = "climb_terminal_entry";
  static constexpr const char* kBodyRaise = "body_raise";
  static constexpr const char* kSearchingTag = "searching_tag";
  static constexpr const char* kWaitingTag = "waiting_tag";
  static constexpr const char* kPrealign = "prealign";
  static constexpr const char* kPreDescentSettle = "pre_descent_settle";
  static constexpr const char* kDescent = "descent";
  static constexpr const char* kSitSettle = "sit_settle";
  static constexpr const char* kLegLift = "leg_lift";
  static constexpr const char* kAligned = "aligned";
  static constexpr const char* kSuccess = "success";
  static constexpr const char* kFailed = "failed";

  static bool isTerminalState(const std::string& state) {
    return state == kSuccess || state == kFailed;
  }

  DockMode(GraspController* controller, PerceptionInterface* perception,
           bool require_lock_confirmation, double linear_speed_m_s,
           double update_rate_hz, double perception_rate_hz,
           double leg_lift_speed_m_s, double sit_settle_duration_s,
           std::function<void(bool is_warn, const std::string& message)> logger);

  // 先回到控制器保留的攀爬末关节姿态，再开放视觉伺服。
  void enter(const JointAngles& current_joints,
             const JointAngles* climb_terminal_joints);
  void exit();
  void failExecution(const std::string& reason);

  DockResult update(const DockRobotState& robot_state);
  bool descentHasStarted() const;

  // Python 属性面（公有，供壳层与测试读取）。
  bool active = false;
  std::string state = kIdle;
  std::string reason;
  double update_period = 0.0;
  double perception_period = 0.0;
  double update_elapsed = 0.0;
  double update_dt = 0.0;
  std::optional<DockResult> last_update_result;
  double perception_elapsed = 0.0;
  bool perception_sampled = false;
  std::string last_perception_reason = "no perception result";
  bool using_last_complete_frame = false;
  std::optional<Matrix4d> cached_pose;
  std::vector<int> cached_ids;
  std::optional<FootPositions> last_visual_target;

  static constexpr double kEntryTrackingTolerance = 2.0 * M_PI / 180.0;
  // 只用于从视觉调整切换到机械导向下降，不作为成功或失败限制。
  static constexpr double kPrealignPositionReference = 0.004;
  static constexpr double kLinearSpeed = 0.050;
  static constexpr double kBodyRaiseHeight = 0.040;
  static constexpr double kTagSearchRadius = 0.020;
  static constexpr double kTagSearchSpeed = 0.020;
  static constexpr double kPreDescentSettleDuration = 0.5;
  static constexpr double kLegLiftHeight = 0.060;
  static constexpr double kLegLiftSpeed = 0.050;
  static constexpr double kLegLiftLevelTolerance = 0.003;
  static constexpr double kSitSettleDuration = 0.5;

 private:
  void setState(const std::string& state, const std::string& reason);
  std::optional<std::pair<Matrix4d, std::vector<int>>> perceptionPose(
      const DockRobotState& robot_state, std::string& reason_out);
  FootPositions actualFeet(const JointAngles& joints) const;
  FootPositions syncedFeet(const JointAngles& current) const;
  DockResult makeResult(std::optional<FootPositions> feet,
                        std::optional<JointAngles> joints) const;
  DockResult updateEntry(const JointAngles& current);
  DockResult bodyRaiseStep(const JointAngles& current);
  DockResult tagSearchStep(const JointAngles& current, const std::string& reason);
  FootPositions visualStep(const JointAngles& current, const Matrix4d& pose);
  DockResult preDescentSettleStep(const JointAngles& current);
  DockResult descentStep(const JointAngles& current);
  DockResult sitSettleStep(const JointAngles& current);
  DockResult legLiftStep(const JointAngles& current);
  DockResult updateOnce(const DockRobotState& robot_state);

  GraspController* controller_;
  PerceptionInterface* perception_;
  bool require_lock_confirmation_ = false;
  double linear_speed_m_s_ = 0.050;
  double leg_lift_speed_m_s_ = 0.050;
  double sit_settle_duration_s_ = 0.5;
  std::function<void(bool, const std::string&)> logger_;

  std::optional<JointAngles> entry_start_;
  std::optional<JointAngles> entry_target_;
  double entry_elapsed_ = 0.0;
  double entry_duration_ = 0.0;
  std::optional<FootPositions> body_raise_start_feet_;
  double body_raise_progress_ = 0.0;
  std::optional<FootPositions> search_anchor_feet_;
  Eigen::Vector2d search_body_offset_ = Eigen::Vector2d::Zero();
  double search_angle_ = -0.5 * M_PI;
  bool search_on_circle_ = false;
  double descent_total_ = 0.0;
  double descent_remaining_ = 0.0;
  double descent_duration_ = 0.0;
  double pre_descent_elapsed_ = 0.0;
  std::optional<FootPositions> pre_descent_feet_;
  double sit_settle_elapsed_ = 0.0;
  std::optional<FootPositions> sit_settle_feet_;
  std::optional<FootPositions> leg_lift_start_feet_;
  double leg_lift_progress_ = 0.0;
};

}  // namespace grasp_hexapod_control_cpp
