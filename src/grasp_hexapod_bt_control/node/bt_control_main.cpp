// 行为树控制节点入口。
//
// 节点名 grasp_hexapod_bt_control（与旧控制栈互斥：同一时刻只能有一个
// /grasp_hexapod/switch_mode 服务提供者）；AsyncSpinner 保证阻塞式
// switch_mode 服务不饿死 120Hz 控制循环。
#include "grasp_hexapod_bt_control/bt_control_node.h"

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "grasp_hexapod_bt_control");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  grasp_hexapod_bt_control::BtControlNode node(nh, pnh);
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::WallRate rate(node.pollRateHz());

  while (ros::ok()) {
    node.step();
    rate.sleep();
  }
  spinner.stop();
  return 0;
}
