// 遥控器节点入口：手柄 → 服务调用 + /cmd_vel。
#include "grasp_hexapod_bt_control/remote_control_node.h"

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "grasp_hexapod_remote_control");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  grasp_hexapod_bt_control::RemoteControlNode node(nh, pnh);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  node.spin();
  spinner.stop();
  return 0;
}
