// 模式统一上下文（ModeContext）——模式作者可见的全部世界。
//
// 模式作者只 include 本文件与 mode_base.h，永不 include bt_control_node.h。
// ModeContext 由调度器（BtControlNode）实现并注入每个模式；对模式作者它是
// 只读接口：拿 NodeHandle 建自己的订阅/发布、直接访问运动控制核心
// RingController（GraspController 的圆环扩展）、读节点级安全输入快照、
// 调用夹爪服务、驱动机器状态机。
#pragma once

#include "grasp_hexapod_bt_control/io_inputs.h"

#include "grasp_hexapod_control_cpp/control.h"
#include "grasp_hexapod_control_cpp/ring_controller.h"

#include <ros/ros.h>

#include <string>
#include <utility>

namespace grasp_hexapod_bt_control {

using grasp_hexapod_control_cpp::ConfigError;
using grasp_hexapod_control_cpp::ControlPaths;
using grasp_hexapod_control_cpp::GraspController;
using grasp_hexapod_control_cpp::JointAngles;
using grasp_hexapod_control_cpp::kJointCount;
using grasp_hexapod_control_cpp::kLegCount;
using grasp_hexapod_ring::RingController;

// 机器状态机（调度器持有；模式经 ModeContext 读取/驱动）。
// WaitB     上电安全门：不发布目标，只接受 home。
// Resetting 平滑回正中（五次曲线回站立）；完成后进入 Hold。
// Hold      已上力、无步态推进；模式从这里 start()。
// Running   某个模式正在执行。
enum class MachineState { WaitB, Resetting, Hold, Running };

inline const char* machineStateName(MachineState state) {
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

// 节点级安全输入的只读快照接口（六腿关节反馈 + IMU + 导航位姿）。
// 这些订阅由调度器常驻持有——安全看门狗在无模式运行时也依赖它们；
// approach 与 climb 共用导航输入。线程安全性由实现保证。
class SharedInputs {
 public:
  virtual ~SharedInputs() = default;
  virtual JointAngles qCur() const = 0;
  virtual std::array<double, 6> feedbackStamps() const = 0;
  virtual ImuSnapshot imu() const = 0;
  // 含边界新鲜度要求的导航快照（approach 巡航用）。
  virtual NavigationState navigationSnapshot() const = 0;
  // 只要求位姿新鲜的导航快照（climb 相对运动监控用）。
  virtual NavigationState navigationMotionSnapshot() const = 0;
};

// 模式上下文：调度器实现，模式构造/运行期只读使用。
class ModeContext {
 public:
  virtual ~ModeContext() = default;

  // 公共命名空间 NodeHandle：模式在这里订阅/发布自己的话题。
  virtual ros::NodeHandle& nodeHandle() = 0;
  // 私有命名空间 NodeHandle：模式读自己的 ~ 参数（各模式参数互不干扰）。
  virtual ros::NodeHandle& privateNodeHandle() = 0;

  // 运动控制核心（步态/运动学/爬坡/圆环对接原语），直接调用，不做白名单封装。
  // RingController 是 GraspController 的圆环扩展：非 dock 路径行为与基类一致，
  // kDock 走圆环状态机（ring_dock_mode）。可用范围与线程规则见 docs/MODE_DEV_GUIDE.md。
  virtual RingController& controller() = 0;

  // 节点级安全输入只读快照。
  virtual const SharedInputs& inputs() const = 0;

  // 夹爪服务（gripper_act）同步调用：返回 <成功, 消息>。
  // 注意：与原实现一致，等待服务存在最多 0.2s，仅在控制帧内短时使用。
  virtual std::pair<bool, std::string> callGripper(
      const std::string& action) = 0;

  // 机器状态机（只能在控制线程的 start/step/checkTerminal 内调用）。
  virtual MachineState machineState() const = 0;
  virtual void setMachineState(MachineState state) = 0;
  // 中止当前运动并进入平滑回正：mission.cancel(reason) + abortClimb +
  // exitDock（若对接中）+ reset_active=false + 置 Resetting。
  // home 与抢占路径共用。
  virtual void abortActiveMotion(const std::string& reason) = 0;
};

}  // namespace grasp_hexapod_bt_control
