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
    // 两个线程：订阅回调与定时控制循环并发（对应 rospy 的线程模型）。
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception& e) {
    ROS_FATAL("servo_side_node failed: %s", e.what());
    return 1;
  }
  return 0;
}
