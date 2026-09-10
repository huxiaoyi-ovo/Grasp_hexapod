// 实机高层控制节点入口（对应 run_real.py::main）。
//
// 节点名与 Python 版一致（grasp_hexapod_control，同一时刻只运行一个实例）；
// AsyncSpinner 保证阻塞式 switch_mode 服务不饿死 120Hz 控制循环。
#include "grasp_hexapod_control_cpp/real_control_node.h"

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "grasp_hexapod_control");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  grasp_hexapod_control_cpp::RealControlNode node(nh, pnh);
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
