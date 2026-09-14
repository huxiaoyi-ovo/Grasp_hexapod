// 实机高层控制节点入口（对应 run_real.py::main）。
//
// 节点名与 Python 版一致（grasp_hexapod_control，同一时刻只运行一个实例）；
// AsyncSpinner 保证阻塞式 switch_mode 服务不饿死 120Hz 控制循环。
#include "grasp_hexapod_control_cpp/ring_control_node.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <algorithm>
#include <string>

int main(int argc, char** argv) {
  bool perception_only=false;
  for(int i=1;i<argc;++i) if(std::string(argv[i])=="--perception-only") perception_only=true;
  ros::init(argc, argv, perception_only ? "grasp_hexapod_ring_probe" : "grasp_hexapod_control");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  if(perception_only) {
    const auto config=pnh.param<std::string>("dock_system_config",
        ros::package::getPath("grasp_hexapod_control")+"/config/dock_system.yaml");
    grasp_hexapod_ring::RingPerceptionRos perception(pnh,config);
    ros::AsyncSpinner spinner(2);spinner.start();
    ros::WallRate rate(10);int valid=0,samples=0;
    for(int i=0;i<100 && ros::ok();++i) {
      const auto result=perception.latest();++samples;valid+=result.valid;
      rate.sleep();
    }
    ROS_INFO("Ring perception-only probe: %d/%d valid samples; no controller or servo publisher created",valid,samples);
    spinner.stop();return valid>0 ? 0 : 2;
  }
  grasp_hexapod_ring::RingControlNode node(nh, pnh);
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
