// 面向行为树的高层控制节点——模式调度器。
//
// 职责（模式本身的逻辑在 src/modes/ 下各自独立实现）：
// - 实现统一上下文 ModeContext 与共享输入 SharedInputs，注入各模式；
// - 机器状态机 WAIT_B/RESETTING/HOLD/RUNNING 与平滑回正序列；
// - switch_mode 阻塞服务：注册表查模式 → 抢占旧调用 → 门控后 start() →
//   服务线程 wait()；同模式重复调用合并等待；
// - hold_motion 心跳租约 → 活动模式 pause()/resume()（原地冻结）；
// - 六腿反馈门控/看门狗、6×3 目标发布、NaN 防护；
// - 话题名统一从 config/mode_topics.yaml（yaml-cpp 直读）加载。
//
// 对外接口与原实现完全一致（/grasp_hexapod/switch_mode + hold_motion），
// 行为树/仪表盘/手柄节点零改动。行为契约：回调只写缓存、120Hz 轮询 +
// 双板新帧门控、WAIT_B 只接受 home、dock 成功必闭合夹爪（在 DockMode 内）。
#pragma once

#include "grasp_hexapod_bt_control/io_inputs.h"
#include "grasp_hexapod_bt_control/mode_base.h"
#include "grasp_hexapod_bt_control/mode_context.h"
#include "grasp_hexapod_bt_control/mode_registry.h"
#include "grasp_hexapod_bt_control/mode_topic_config.h"

#include "grasp_hexapod_control_cpp/control.h"
#include "grasp_hexapod_control_cpp/dock_mode.h"

#include <grasp_hexapod_msgs/ModeStatus.h>
#include <grasp_hexapod_msgs/SwitchMode.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace grasp_hexapod_bt_control {

class BtControlNode : public ModeContext, public SharedInputs {
 public:
  BtControlNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

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

  // /cmd_vel 平面速度命令限幅（委托 WalkMode::clampCommand，保持旧测试签名）。
  static Eigen::Vector4d clampCommand(double vx_right, double vy_forward,
                                      double yaw_rate, double max_linear_speed,
                                      double max_yaw_rate);

 private:
  // ---------------------------------------------------------------------
  // 一次 switch_mode 调用（对应原 BtRequest；结果存于模式对象本身）。
  // ---------------------------------------------------------------------
  struct ActiveCall {
    ModeBase* mode = nullptr;
    uint64_t run = 0;       // 模式 run 代次（区分重复调用/抢占）
    bool started = false;   // 控制环是否已执行 start()
    int waiters = 1;
  };

  // ---------------------------------------------------------------------
  // SharedInputs 实现（模式经 ctx().inputs() 只读访问）。
  // ---------------------------------------------------------------------
  JointAngles qCur() const override;
  std::array<double, 6> feedbackStamps() const override;
  ImuSnapshot imu() const override;
  NavigationState navigationSnapshot() const override;
  NavigationState navigationMotionSnapshot() const override;

  // ---------------------------------------------------------------------
  // ModeContext 实现（调度器即模式的统一上下文）。
  // ---------------------------------------------------------------------
  ros::NodeHandle& nodeHandle() override { return nh_; }
  ros::NodeHandle& privateNodeHandle() override { return pnh_; }
  GraspController& controller() override { return *controller_; }
  const SharedInputs& inputs() const override { return *this; }
  std::pair<bool, std::string> callGripper(const std::string& action) override;
  MachineState machineState() const override { return state_; }
  void setMachineState(MachineState state) override { state_ = state; }
  void abortActiveMotion(const std::string& reason) override;

  // ---------------------------------------------------------------------
  // 参数与初始化。
  // ---------------------------------------------------------------------
  void loadParameters(ros::NodeHandle& pnh);
  void configureNavigationApproach(ros::NodeHandle& pnh);
  void createInterfaces(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void createModes();  // 启动时按注册表创建全部已实现模式实例
  ModeBase* getOrCreateMode(const std::string& name);

  // ---------------------------------------------------------------------
  // switch_mode 服务与 hold_motion 租约。
  // ---------------------------------------------------------------------
  bool switchModeService(grasp_hexapod_msgs::SwitchModeRequest& request,
                         grasp_hexapod_msgs::SwitchModeResponse& response);
  void btHoldCallback(const std_msgs::String::ConstPtr& message);
  std::shared_ptr<ActiveCall> activeCallSnapshot() const;
  void tryStartCall(ActiveCall& call, const JointAngles& q_cur);
  bool btHoldBridge(double now, const ActiveCall& call);

  // ---------------------------------------------------------------------
  // 输入回调、安全保持与每帧主逻辑。
  // ---------------------------------------------------------------------
  void feedbackCallback(int leg_index, const std::string& leg_name,
                        const sensor_msgs::JointState::ConstPtr& message);
  void holdMotion(const std::string& reason, bool log = true);
  // 模式状态反馈：状态变化立即发 + 空闲 1Hz 心跳（每帧调用，内部节流）。
  void publishModeStatus();
  std::optional<JointAngles> updateControl(
      const JointAngles& q_cur, double now,
      const std::array<double, 6>* feedback_stamp);
  void publishTargets(const JointAngles& q_des);

  // ---------------------------------------------------------------------
  // 参数（名与默认值与原实现一致；各模式专属参数由各模式自读）。
  // ---------------------------------------------------------------------
  double rate_hz_ = 30.0;
  double poll_rate_hz_ = 120.0;
  bool enable_link_collision_check_ = true;
  double max_feedback_age_ = 0.30;
  double max_feedback_skew_ = 0.20;
  std::string control_source_ = "teleop";
  double bt_hold_lease_s_ = 0.15;
  double max_linear_speed_ = 0.20;
  double max_yaw_rate_ = 0.0;

  // ---------------------------------------------------------------------
  // 机器状态机与控制器。
  // ---------------------------------------------------------------------
  MachineState state_ = MachineState::WaitB;

  ControlPaths paths_;
  std::unique_ptr<GraspController> controller_;
  std::unique_ptr<NavigationInput> navigation_;
  std::unique_ptr<ImuInput> imu_;

  // ---------------------------------------------------------------------
  // 共享输入缓存（回调写、step 读；qCur()/feedbackStamps() 为 const 读）。
  // ---------------------------------------------------------------------
  mutable std::mutex lock_;
  JointAngles q_cur_{};
  std::array<double, 6> feedback_stamp_{};
  std::array<double, 6> last_control_feedback_stamp_{};

  // switch_mode 仲裁（active_call_ 与租约；等待同步在各模式对象内）。
  mutable std::mutex bt_mutex_;
  std::shared_ptr<ActiveCall> active_call_;
  double bt_hold_deadline_ = 0.0;
  bool bt_hold_active_ = false;

  // 模式实例缓存（启动时创建完毕，此后只读）。
  std::map<std::string, std::unique_ptr<ModeBase>> mode_cache_;
  ModeTopicConfig topics_config_;
  ModeTopics shared_topics_;

  // 计时统计。
  double last_tick_compute_seconds_ = 0.0;
  double tick_compute_sum_ = 0.0;
  double tick_compute_max_ = 0.0;
  int tick_compute_count_ = 0;
  ros::Time tick_stats_window_start_;

  // ROS I/O。
  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;
  std::vector<ros::Publisher> des_publishers_;
  ros::Publisher mode_status_pub_;
  ros::ServiceClient gripper_client_;
  ros::ServiceServer switch_mode_server_;
  std::vector<ros::Subscriber> subscribers_;

  // 模式状态发布节流（变化即发 + 空闲 1Hz 心跳）。
  std::string last_status_key_ = "__init__";
  double last_status_publish_ = 0.0;
  // 最近一次完成调用的终态（服务线程在最后一个等待者离开时记录）。
  std::string last_finished_mode_;
  ModeOutcome last_finished_outcome_;
  uint64_t last_finished_run_ = 0;
};

}  // namespace grasp_hexapod_bt_control
