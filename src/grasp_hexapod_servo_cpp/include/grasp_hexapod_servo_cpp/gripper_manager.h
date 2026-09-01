// 夹爪舵机（ID 99，仅左板）管理器：服务化开合控制 + 位置状态机。
// 设计要点（与行走舵机隔离，尽可能少影响其他舵机的控制）：
//   - 空闲时对 ID99 无任何串口读写；仅在启动自检与 open/clamp 服务处理期间
//     以 gripper_poll_hz（默认 2Hz）读取位置验证；
//   - 与 /gripper_des 话题盲控路径并存：服务处理期间（busy）话题写入暂停；
//   - 服务为同步阻塞：响应即最终结果（成功/失败 + 原因）。
#ifndef GRASP_HEXAPOD_SERVO_CPP_GRIPPER_MANAGER_H_
#define GRASP_HEXAPOD_SERVO_CPP_GRIPPER_MANAGER_H_

#include <atomic>
#include <mutex>
#include <optional>
#include <string>

#include <ros/ros.h>

#include "grasp_hexapod_servo_cpp/GripperCommand.h"
#include "grasp_hexapod_servo_cpp/hiwonder_servo_controller.h"

namespace grasp_hexapod_servo_cpp {

// 夹爪状态机状态。
enum class GripperState {
  kUnknown,     // 未完成自检或自检/打开验证失败。
  kOffline,     // 无位置反馈（舵机不在线）。
  kOpen,        // 已打开（≈ gripper_open_pulse）。
  kClamped,     // 已夹紧（≈ gripper_clamp_pulse）。
  kRestricted,  // 夹紧失败受限：后续 clamp 直接失败，须先 open 复位。
};

// 脉冲位置分类（启动自检与开/合快路径判定）。
enum class GripperPositionClass {
  kOpen,          // ≈ 打开脉冲。
  kClamped,       // ≈ 夹紧脉冲。
  kIntermediate,  // 中间位置（在途）。
};

// 单次夹紧轮询判定。
enum class ClampVerdict {
  kClampedOk,       // 偏差 ≤ tolerance：算作夹紧。
  kLargeDeviation,  // 偏差 ≥ fail_deviation：连续多次则夹紧失败 → 受限。
  kInTransit,       // 中间偏差：继续轮询。
};

class GripperManager {
 public:
  // control 与腿部循环共享（控制器内部按操作互斥）；nh 用于全局服务
  // /gripper_command，nh_private 用于 ~gripper_* 参数。
  GripperManager(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                 HiwonderServoController* control, int gripper_id);
  ~GripperManager() = default;

  // 启动自检：加载扭矩 → 读位置判定在线/开合 → 未打开则打开并复核。
  // 同步阻塞（最坏 ~2.5s），须在节点构造期（腿部定时器启动前）调用。
  void init();

  // 服务处理中（移动/验证）：话题写入路径应暂停夹爪写入。
  bool isBusy() const;

  GripperState state() const;

  // ---- 纯静态判定（可单测，不依赖串口） ----
  static GripperPositionClass classifyPulse(int pulse, int open_pulse,
                                            int clamp_pulse, int tolerance);
  static ClampVerdict evaluateClampDeviation(int deviation, int tolerance,
                                             int fail_deviation);
  // 受限状态下拒绝 clamp，须先 open 复位。
  static bool clampAllowed(GripperState state);

 private:
  void loadParams(ros::NodeHandle& nh_private);
  bool handleCommand(GripperCommand::Request& request,
                     GripperCommand::Response& response);
  bool doOpen(GripperCommand::Response& response);
  bool doClamp(GripperCommand::Response& response);
  // 一次位置读取 + 一次即时重试；nullopt = 无反馈（不在线）。
  std::optional<int> readPulse();
  // 幂等加载夹爪扭矩（服务移动前与启动自检时调用）。
  void ensureLoaded();
  void sleepFor(double seconds) const;
  void setState(GripperState state);

  HiwonderServoController* control_;
  int gripper_id_;

  // ---- 参数（~gripper_*）----
  int open_pulse_ = 683;
  int clamp_pulse_ = 840;
  int tolerance_ = 20;
  int fail_deviation_ = 50;
  int max_checks_ = 4;
  int max_total_polls_ = 8;
  double poll_hz_ = 2.0;
  int command_duration_ms_ = 400;

  // ---- 运行时状态 ----
  // operation_mutex_ 串行化 open/clamp（并发命令立即拒绝）；state_/busy_ 用
  // 原子量，供 30Hz 循环线程无锁查询，不阻塞腿部控制。
  std::mutex operation_mutex_;
  std::atomic<GripperState> state_{GripperState::kUnknown};
  std::atomic<bool> busy_{false};
  ros::ServiceServer service_;
};

}  // namespace grasp_hexapod_servo_cpp

#endif  // GRASP_HEXAPOD_SERVO_CPP_GRIPPER_MANAGER_H_
