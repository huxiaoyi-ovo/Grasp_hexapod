// 实机高层控制节点（对应 Python run_real.py::RosControlNode 的完整移植）。
//
// 职责：订阅手柄/18 关节反馈/导航位姿，执行 B 回正、A 启停、X/Y 模式入口、
// 行为树仲裁与 HOLD 租约，调用 GraspController 并发布 6×3 关节目标。
// 舵机协议仍由 grasp_hexapod_servo(_cpp) 负责；计算核心在
// grasp_hexapod_control_core。
#pragma once

#include "grasp_hexapod_control_cpp/control.h"
#include "grasp_hexapod_control_cpp/dock_mode.h"
#include "grasp_hexapod_control_cpp/dock_perception_ros.h"

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <grasp_hexapod_msgs/RemoteCmd.h>
#include <grasp_hexapod_msgs/SwitchMode.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace grasp_hexapod_control_cpp {

class RealControlNode {
 public:
  RealControlNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  // 实机循环单步：读取完整反馈，计算一次目标并发布给两块 Servo 板。
  void step();

  double pollRateHz() const { return poll_rate_hz_; }

  // ---------------------------------------------------------------------
  // 静态反馈门控（对应 _feedback_frame_state/_feedback_issue；公开便于测试）。
  // ---------------------------------------------------------------------
  static std::pair<bool, bool> feedbackFrameState(
      const JointAngles& q_cur, const std::array<double, 6>& feedback_stamp,
      const std::array<double, 6>& last_control_feedback_stamp, double now,
      double max_feedback_age, double max_feedback_skew);

  static std::string feedbackIssue(const JointAngles& q_cur,
                                   const std::array<double, 6>& feedback_stamp,
                                   double now, double max_feedback_age);

  // 最近一次控制帧的计算耗时（秒）；供性能诊断/基准。
  double lastTickComputeSeconds() const { return last_tick_compute_seconds_; }

 private:
  // ---------------------------------------------------------------------
  // 输入缓存：回调只保存最新消息，控制计算统一放在 step()。
  // ---------------------------------------------------------------------
  class NavigationInput {
   public:
    explicit NavigationInput(ros::NodeHandle& pnh);
    NavigationState snapshot() const;      // 位姿过期时 valid=false
    NavigationState motionSnapshot() const;  // 实机攀爬（无边界要求）

   private:
    template <typename Message>
    bool validFrame(const Message& message) const {
      return validFrameImpl(message.header.frame_id);
    }
    bool validFrameImpl(const std::string& frame_id) const;
    void baseCallback(const geometry_msgs::PoseStamped::ConstPtr& message);
    void xiaolanCallback(const geometry_msgs::PoseStamped::ConstPtr& message);
    void boundaryCallback(const geometry_msgs::PolygonStamped::ConstPtr& message);
    void landingCallback(const std_msgs::Bool::ConstPtr& message);

    double max_age_;
    double max_pose_skew_;
    double max_boundary_age_;
    mutable std::mutex lock_;
    double base_stamp_ = 0.0;
    double xiaolan_stamp_ = 0.0;
    double boundary_stamp_ = 0.0;
    bool have_base_ = false;
    bool have_xiaolan_ = false;
    Matrix4d pv_from_base_ = Matrix4d::Identity();
    Matrix4d pv_from_xiaolan_ = Matrix4d::Identity();
    std::vector<Vector2d> pv_boundary_;
    bool landing_confirmed_ = false;
    std::vector<ros::Subscriber> subscribers_;
  };

  class ImuInput {
   public:
    struct Snapshot {
      bool valid = false;
      Matrix3d rotation = Matrix3d::Identity();
      Vector3d angular_velocity = Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
    };
    explicit ImuInput(ros::NodeHandle& pnh);
    Snapshot snapshot() const;

   private:
    void callback(const sensor_msgs::Imu::ConstPtr& message);

    double max_age_;
    mutable std::mutex lock_;
    double stamp_ = 0.0;
    bool have_rotation_ = false;
    Matrix3d rotation_ = Matrix3d::Identity();
    Vector3d angular_velocity_ = Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    ros::Subscriber subscriber_;
  };

  class BoolInput {
   public:
    BoolInput(ros::NodeHandle& nh, const std::string& topic);
    std::optional<bool> snapshot() const;
    std::pair<std::optional<bool>, double> snapshotWithTime() const;

   private:
    void callback(const std_msgs::Bool::ConstPtr& message);
    mutable std::mutex lock_;
    std::optional<bool> value_;
    double received_at_ = 0.0;
    ros::Subscriber subscriber_;
  };

  // BT 服务请求（对应 _switch_mode_callback 的请求字典）。
  struct BtRequest {
    std::string mode;
    bool started = false;
    std::optional<std::pair<bool, std::string>> final_result;
    int waiters = 1;
    bool dock_clamped = false;
  };

  // 实机攀爬速度诊断聚合（observation-only，从不参与门控）。
  struct ClimbSpeedDiagnostic {
    std::string stage;
    JointAngles previous_q_cur{};
    JointAngles previous_q_des{};
    std::array<double, 6> previous_feedback_stamp{};
    double previous_sample_time = 0.0;
    double peak_command_speed_rad_s = 0.0;
    double peak_measured_speed_rad_s = 0.0;
    double peak_tracking_error_rad = 0.0;
    double ratio_sum = 0.0;
    int ratio_count = 0;
    int velocity_limit_clip_count = 0;
    int collision_guard_hold_count = 0;
  };

  // ---------------------------------------------------------------------
  // 参数与初始化。
  // ---------------------------------------------------------------------
  void loadParameters(ros::NodeHandle& pnh);
  void createInterfaces(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void configureNavigationApproach(ros::NodeHandle& pnh);
  void ensureDockMode();

  // ---------------------------------------------------------------------
  // BT 接口。
  // ---------------------------------------------------------------------
  bool switchModeService(grasp_hexapod_msgs::SwitchModeRequest& request,
                         grasp_hexapod_msgs::SwitchModeResponse& response);
  void remoteCmdCallback(const grasp_hexapod_msgs::RemoteCmd::ConstPtr& message);
  void btHoldCallback(const std_msgs::String::ConstPtr& message);
  std::shared_ptr<BtRequest> btRequestSnapshot() const;
  void finishBtRequest(const std::shared_ptr<BtRequest>& request, bool success,
                       const std::string& message);
  void abortBtRequest(const std::string& reason);
  bool btHoldIsActive(double now);
  void startBtRequest(BtRequest& request, const JointAngles& q_cur);
  void finishBtModeIfTerminal(const std::shared_ptr<BtRequest>& request);
  std::pair<bool, std::string> actuateGripper(const std::string& action);

  // ---------------------------------------------------------------------
  // 手柄。
  // ---------------------------------------------------------------------
  void joyCallback(const sensor_msgs::Joy::ConstPtr& message);
  void maybeTriggerManualGripper(double dpad);
  void runManualGripper(const std::string& action);
  static double readAxis(const Eigen::VectorXd& values, int index);
  Eigen::Vector4d makeCommand(const Eigen::VectorXd& axes) const;
  double bodyAxis(const Eigen::VectorXd& axes) const;
  bool manualCommandActive(const Eigen::VectorXd& axes) const;

  // ---------------------------------------------------------------------
  // 反馈与模式入口。
  // ---------------------------------------------------------------------
  void feedbackCallback(int leg_index, const std::string& leg_name,
                        const sensor_msgs::JointState::ConstPtr& message);
  void processButtons(const Eigen::VectorXi& button_presses, bool controls_ready,
                      const JointAngles& q_cur);
  void startRealClimb(const JointAngles& q_cur, bool controls_ready);
  void startRealDock(const JointAngles& q_cur, bool controls_ready);
  std::optional<bool> dockLockConfirmed();
  void holdMotion(const std::string& reason, bool log = true);

  // 攀爬监控与诊断。
  std::pair<bool, std::string> realClimbObservation();
  void monitorRealClimb();
  void resetRealClimbSpeedDiagnostic();
  void warnHardwareClimbPhaseHold();
  void infoHardwareClimbActiveTrace();
  void flushRealClimbSpeedDiagnostic(const std::string& reason);
  void recordRealClimbSpeedDiagnostic(const std::string& stage,
                                      const JointAngles& q_cur,
                                      const JointAngles& q_des,
                                      const std::array<double, 6>& feedback_stamp,
                                      double sample_time);

  // 每帧主逻辑；返回 nullopt 表示本帧不发布（WAIT_B）。
  std::optional<JointAngles> updateControl(
      const JointAngles& q_cur, const Eigen::VectorXd& axes,
      const Eigen::VectorXi& button_presses, double joy_stamp, double now,
      bool feedback_ready, const std::array<double, 6>* feedback_stamp);
  void publishTargets(const JointAngles& q_des);

  // ---------------------------------------------------------------------
  // 参数（名与默认值与 run_real.py 一致）。
  // ---------------------------------------------------------------------
  bool local_execution_ = false;
  double rate_hz_ = 30.0;
  double poll_rate_hz_ = 120.0;
  bool enable_link_collision_check_ = true;
  double max_feedback_age_ = 0.30;
  double max_feedback_skew_ = 0.20;
  double max_joy_age_ = 0.2;
  bool enable_real_climb_ = false;
  std::string climb_side_ = "left";
  double climb_foot_gate_m_ = 0.02;
  bool enable_real_dock_ = true;
  std::string dock_system_config_;
  bool dock_require_real_calibrated_ = true;
  bool dock_allow_uncalibrated_ = false;
  double dock_lock_confirmation_max_age_ = 0.5;
  double dock_session_started_at_ = 0.0;
  std::string control_source_ = "teleop";

  double bt_hold_lease_s_ = 0.15;

  int button_a_ = 0;
  int button_b_ = 1;
  int button_x_ = 2;
  int button_y_ = 3;
  int axis_gripper_ = 6;
  std::string gripper_last_cmd_;
  std::mutex gripper_service_lock_;
  int axis_right_ = 0;
  int axis_forward_ = 1;
  int axis_yaw_ = 3;
  int axis_body_down_ = 4;
  int axis_body_up_ = 5;
  double axis_right_scale_ = -1.0;
  double axis_forward_scale_ = 1.0;
  double axis_yaw_scale_ = 1.0;
  double axis_body_scale_ = -1.0;

  double max_linear_speed_ = 0.20;
  double max_vertical_speed_ = 0.02;
  double max_yaw_rate_ = 0.0;

  int real_climb_persistence_frames_ = 3;
  double real_climb_max_position_error_ = 0.05;
  double real_climb_max_orientation_error_ = 10.0 * M_PI / 180.0;
  double real_climb_max_angular_speed_ = 30.0 * M_PI / 180.0;

  // ---------------------------------------------------------------------
  // 状态机与控制器。
  // ---------------------------------------------------------------------
  enum class MachineState { WaitB, Resetting, Hold, Running };
  MachineState state_ = MachineState::WaitB;
  static const char* machineStateName(MachineState state);

  ControlPaths paths_;
  std::unique_ptr<GraspController> controller_;
  std::unique_ptr<DockPerceptionRos> dock_perception_;  // DockMode 依赖
  std::unique_ptr<DockMode> dock_mode_instance_;        // attach 到 controller

  // ---------------------------------------------------------------------
  // 共享输入缓存（回调写、step 读）。
  // ---------------------------------------------------------------------
  std::mutex lock_;
  JointAngles q_cur_{};
  std::array<double, 6> feedback_stamp_{};
  std::array<double, 6> last_control_feedback_stamp_{};
  Eigen::VectorXd axes_{};
  Eigen::VectorXi buttons_{};
  Eigen::VectorXi button_press_latch_{};
  double joy_stamp_ = 0.0;

  // BT 仲裁。
  mutable std::mutex bt_mutex_;
  std::condition_variable bt_condition_;
  std::shared_ptr<BtRequest> bt_request_;
  double bt_hold_deadline_ = 0.0;
  bool bt_hold_active_ = false;
  grasp_hexapod_msgs::RemoteCmd bt_remote_;
  bool bt_remote_valid_ = false;

  // 附加状态。
  bool manual_override_ = false;
  Eigen::Vector4d command_ = Eigen::Vector4d::Zero();
  bool local_climb_armed_ = false;
  std::optional<JointAngles> local_climb_entry_q_;
  NavigationState climb_start_navigation_;
  bool have_climb_start_navigation_ = false;
  Matrix3d climb_start_imu_rotation_ = Matrix3d::Identity();
  bool have_climb_start_imu_ = false;
  Matrix4d climb_start_planned_pose_ = Matrix4d::Identity();
  bool have_climb_start_planned_ = false;
  bool real_climb_monitor_active_ = false;
  int climb_bad_frames_ = 0;
  int climb_good_frames_ = 0;
  std::unique_ptr<ClimbSpeedDiagnostic> climb_speed_diagnostic_;

  // 计时统计。
  double last_tick_compute_seconds_ = 0.0;
  double tick_compute_sum_ = 0.0;
  double tick_compute_max_ = 0.0;
  int tick_compute_count_ = 0;
  ros::Time tick_stats_window_start_;

  // ROS I/O。
  std::unique_ptr<NavigationInput> navigation_;
  std::unique_ptr<ImuInput> imu_;
  std::unique_ptr<BoolInput> lock_confirmation_;
  std::map<std::string, ros::Publisher> des_publishers_;
  ros::ServiceClient gripper_client_;
  ros::ServiceServer switch_mode_server_;
  std::vector<ros::Subscriber> subscribers_;
  std::thread gripper_thread_;
  bool gripper_thread_active_ = false;
};

}  // namespace grasp_hexapod_control_cpp
