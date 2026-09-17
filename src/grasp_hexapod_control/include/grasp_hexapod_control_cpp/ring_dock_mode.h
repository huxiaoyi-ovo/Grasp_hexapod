// Independent ring state machine, adapted from the existing C++ DockMode.
// Used only by the independent run_real_ring_cpp chain.
#pragma once

#include "grasp_hexapod_control_cpp/types.h"

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace grasp_hexapod_ring {
using namespace grasp_hexapod_control_cpp;



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
  // 圆环状态机；middle_lift为本链路独立阶段。
  static constexpr const char* kIdle = "idle";
  static constexpr const char* kClimbTerminalEntry = "climb_terminal_entry";
  static constexpr const char* kMiddleLift = "middle_lift";
  static constexpr const char* kBodyRaise = "body_raise";
  static constexpr const char* kSearchingTag = "searching_tag";
  static constexpr const char* kWaitingTag = "waiting_ring";
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

  DockMode(double dt, std::function<FootPositions(const JointAngles&, bool)> feet,
           std::function<PerceptionResult()> latest, std::function<void()> reset,
           bool require_lock_confirmation, double linear_speed_m_s,
           double update_rate_hz, double perception_rate_hz,
           double leg_lift_speed_m_s, double sit_settle_duration_s,
           double leg_lift_height_m, double leg_lift_level_tolerance_m,
           std::function<void(bool is_warn, const std::string& message)> logger,
           std::optional<double> alignment_speed_m_s = std::nullopt);

  // 保持当前入口姿态，再开放视觉伺服。
  void enter(const JointAngles& current_joints,
             const JointAngles* climb_terminal_joints);
  void exit();
  void failExecution(const std::string& reason);

  DockResult update(const DockRobotState& robot_state);
  bool descentHasStarted() const;
  static bool isMiddleLeg(int leg) { return leg == 2 || leg == 5; }
  const std::optional<JointAngles>& middleHold() const { return middle_hold_q_; }
  // 唯一对接输出入口：DLS之后覆盖中腿，检查边界/碰撞并保存已发指令。
  JointAngles acceptCommand(const JointAngles& candidate, const JointAngles& current);

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
  static constexpr double kTagSearchRadius = 0.030;
  static constexpr double kTagSearchSpeed = 0.020;
  static constexpr double kPreDescentSettleDuration = 0.5;
  static constexpr double kLegLiftHeight = 0.040;
  static constexpr double kLegLiftSpeed = 0.050;
  static constexpr double kLegLiftLevelTolerance = 0.003;
  static constexpr double kSitSettleDuration = 0.5;

 public: // State snapshots are exposed for offline equivalence tests.
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

  double dt_;
  std::function<FootPositions(const JointAngles&, bool)> feet_;
  std::function<PerceptionResult()> latest_;
  std::function<void()> reset_;
  std::optional<double> descent_distance_;
  double hole_depth_ = 0.043;
  bool require_lock_confirmation_ = false;
  double linear_speed_m_s_ = 0.050;
  double alignment_speed_m_s_ = 0.050;
  double leg_lift_speed_m_s_ = 0.050;
  double sit_settle_duration_s_ = 0.5;
  double leg_lift_height_m_ = kLegLiftHeight;
  double leg_lift_level_tolerance_m_ = kLegLiftLevelTolerance;
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

 private:
  // 仅圆环对接使用；不改变站姿与攀爬的公共关节边界。
  static constexpr double kLiftKneeLimit = 100.0 * M_PI / 180.0;
  static constexpr double kLiftKneePlanLimit = 98.0 * M_PI / 180.0;
  static constexpr double kLiftAnkleLimit = 118.0 * M_PI / 180.0;
  static constexpr double kLiftJointSpeed = 20.0 * M_PI / 180.0;
  static constexpr double kLiftTrackingTolerance = 2.0 * M_PI / 180.0;
  void resetLegLift();
  bool planLegLift(const JointAngles& current, std::array<double, kLegCount>* lift_heights = nullptr);
  static constexpr double kDescentHorizontalAdjustment = .030;  // 支撑下降期间二维水平调整最大30mm，收腿不受此约束
  bool descentPose(double progress, const JointAngles& previous, JointAngles& target) const;
  bool planDescent(const JointAngles& current);
  struct DescentWaypoint { JointAngles joints; double height; double duration; };
  std::vector<DescentWaypoint> descent_path_;
  size_t descent_index_ = 1, descent_lift_index_ = 1;
  double descent_segment_elapsed_ = 0.;
  std::optional<JointAngles> descent_command_q_;
  FootPositions freezeMiddleFeet(FootPositions feet) const;
  bool trackingHealthy(const JointAngles& current);
  std::optional<JointAngles> middle_hold_q_;
  std::optional<JointAngles> last_command_q_;
  double support_wait_elapsed_ = 0.0;
  double body_raise_elapsed_ = 0.0;
  double descent_elapsed_ = 0.0;
  DockResult stopLegLift(const JointAngles& current, const std::string& message);
  std::optional<JointAngles> lift_start_q_;
  std::optional<JointAngles> lift_target_q_;
  std::optional<JointAngles> lift_command_q_;
  std::optional<JointAngles> lift_last_feedback_q_;
  // 失败终态必须保留撤销后的目标，即使 HOLD 不再注入反馈。
  std::optional<JointAngles> lift_stop_q_;
  double lift_duration_ = 0.0;
  double lift_elapsed_ = 0.0;
  double lift_wall_elapsed_ = 0.0;
  double lift_settled_time_ = 0.0;
};

}  // namespace grasp_hexapod_control_cpp
