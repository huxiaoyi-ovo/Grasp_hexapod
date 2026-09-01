// 单板舵机驱动节点入口。
// 每实例管理一块 LX-15D 驱动板（9 个舵机、3 条腿）；两块板各起一个实例。
// 对应 Python 版 servo.py 的入口（rospy.init_node("servo_side_node")）。
#include <memory>

#include <ros/ros.h>

#include "grasp_hexapod_servo_cpp/servo_side_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "servo_side_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  try {
    auto node =
        std::make_unique<grasp_hexapod_servo_cpp::ServoSideNode>(nh, nh_private);
    // 三个线程：定时控制循环、订阅回调、夹爪服务处理（同步阻塞式 open/clamp
    // 最多占用一个线程数秒，腿部 30Hz 循环在其余线程不受影响）。
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception& e) {
    ROS_FATAL("servo_side_node failed: %s", e.what());
    return 1;
  }
  return 0;
}
