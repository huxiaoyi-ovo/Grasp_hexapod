// 遥控器节点：把手柄 /joy 映射为服务调用与 /cmd_vel（原控制节点内 /joy 直连
// 链路的替代层，使控制节点完全不依赖手柄话题）。
//
// - 左摇杆/右摇杆轴 → /cmd_vel（平面三自由度 linear.x/linear.y/angular.z，
//   无 z 轴线速度；30Hz 持续发布，手柄失效发零速）。
// - 按键（按下沿）→ 阻塞服务调用（后台线程，不阻塞发布循环）：
//   A=switch_mode(walk)  B=switch_mode(home)  X=switch_mode(climb)  Y=switch_mode(dock)
//   切换在服务端带抢占语义：进行中的模式会被新模式取代。
// - 方向键（轴）→ /grasp_hexapod/gripper_act：左=clamp 右=open。
#pragma once

#include <geometry_msgs/Twist.h>
#include <grasp_hexapod_msgs/GripperAct.h>
#include <grasp_hexapod_msgs/SwitchMode.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace grasp_hexapod_bt_control {

class RemoteControlNode {
 public:
  RemoteControlNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  // 30Hz 发布 /cmd_vel（含手柄失效零速），直到 ros::shutdown。
  void spin();

 private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& message);
  void handleButtonPress(int index);
  void callSwitchMode(const std::string& mode);
  void maybeTriggerGripper(double dpad);
  void runGripper(const std::string& action);
  geometry_msgs::Twist makeTwist(bool* fresh) const;

  // ---------------------------------------------------------------------
  // 参数（沿用原控制节点手柄映射命名）。
  // ---------------------------------------------------------------------
  std::string joy_topic_ = "/joy";
  std::string cmd_vel_topic_ = "/cmd_vel";
  std::string switch_mode_service_ = "/grasp_hexapod/switch_mode";
  std::string gripper_service_ = "/grasp_hexapod/gripper_act";
  double rate_hz_ = 30.0;
  double max_joy_age_ = 0.2;
  int button_a_ = 0;
  int button_b_ = 1;
  int button_x_ = 2;
  int button_y_ = 3;
  int axis_right_ = 0;
  int axis_forward_ = 1;
  int axis_yaw_ = 3;
  int axis_gripper_ = 6;
  double axis_right_scale_ = -1.0;
  double axis_forward_scale_ = 1.0;
  double axis_yaw_scale_ = 1.0;
  double max_linear_speed_ = 0.20;
  double max_yaw_rate_ = 0.50;

  ros::Publisher cmd_vel_publisher_;
  ros::ServiceClient switch_mode_client_;
  ros::ServiceClient gripper_client_;
  ros::Subscriber joy_subscriber_;

  // ---------------------------------------------------------------------
  // 共享输入缓存（回调写、spin 读）。
  // ---------------------------------------------------------------------
  mutable std::mutex lock_;
  std::vector<double> axes_;
  std::vector<int> buttons_;
  std::vector<int> button_press_latch_;
  double joy_stamp_ = 0.0;

  // 夹爪单飞：服务串口忙时跳过新请求（与原手动夹爪行为一致）。
  std::mutex gripper_service_lock_;
  std::string gripper_last_cmd_;
  std::thread gripper_thread_;
  std::atomic<int> inflight_switch_calls_{0};
};

}  // namespace grasp_hexapod_bt_control
