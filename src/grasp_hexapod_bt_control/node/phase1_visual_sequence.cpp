#include <ros/ros.h>
#include <grasp_hexapod_msgs/SwitchMode.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "phase1_visual_sequence");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  const std::string controller_node = pnh.param<std::string>(
      "controller_node", "/grasp_hexapod_bt_control");
  const std::string service_name = "/grasp_hexapod/switch_mode";
  auto client = nh.serviceClient<grasp_hexapod_msgs::SwitchMode>(
      service_name);
  if (!client.waitForExistence(ros::Duration(5))) {
    ROS_ERROR("switch_mode unavailable at %s", service_name.c_str());
    return 2;
  }
  bool visual_guidance_enabled = false;
  bool enable_real_climb = true;
  bool enable_real_dock = true;
  if (!ros::param::get(controller_node + "/visual_guidance_enabled",
                       visual_guidance_enabled) ||
      !ros::param::get(controller_node + "/enable_real_climb",
                       enable_real_climb) ||
      !ros::param::get(controller_node + "/enable_real_dock",
                       enable_real_dock) ||
      !visual_guidance_enabled || enable_real_climb || enable_real_dock) {
    ROS_ERROR(
        "refusing phase1 sequence: %s must set visual_guidance_enabled=true "
        "and enable_real_climb=enable_real_dock=false",
        controller_node.c_str());
    return 3;
  }
  for (const char* mode : {"home", "spin_search", "approach"}) {
    grasp_hexapod_msgs::SwitchMode call;
    call.request.target_mode = mode;
    if (!client.call(call) || !call.response.success) {
      ROS_ERROR("%s failed: %s", mode, call.response.message.c_str());
      return 1;
    }
    ROS_INFO("%s: %s", mode, call.response.message.c_str());
  }
  ROS_INFO("phase1 coarse approach complete; HOLD, no climb or dock requested");
  return 0;
}
