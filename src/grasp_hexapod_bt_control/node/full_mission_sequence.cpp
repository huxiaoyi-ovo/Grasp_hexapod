#include <ros/ros.h>
#include <grasp_hexapod_msgs/SwitchMode.h>

#include <algorithm>
#include <string>
#include <vector>

// Full-mission sequencer: home -> spin_search -> approach -> climb -> [home] -> dock.
// Each mode is one blocking switch_mode call. On any failure the controller
// holds, so this node stops immediately and never retries or calls another mode.
int main(int argc, char** argv) {
  ros::init(argc, argv, "full_mission_sequence");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  const std::string controller_node = pnh.param<std::string>(
      "controller_node", "/grasp_hexapod_bt_control");
  const bool home_before_dock = pnh.param("home_before_dock", false);
  const std::string stop_after = pnh.param<std::string>("stop_after", "");

  std::vector<std::string> sequence = {"home", "spin_search", "approach",
                                       "climb"};
  if (home_before_dock) {
    sequence.push_back("home");
  }
  sequence.push_back("dock");

  if (!stop_after.empty()) {
    const auto stop_it =
        std::find(sequence.begin(), sequence.end(), stop_after);
    if (stop_it == sequence.end()) {
      ROS_ERROR(
          "invalid ~stop_after '%s': use empty or one of home, spin_search, "
          "approach, climb, dock",
          stop_after.c_str());
      return 4;
    }
    sequence.erase(stop_it + 1, sequence.end());
  }

  const std::string service_name = "/grasp_hexapod/switch_mode";
  auto client = nh.serviceClient<grasp_hexapod_msgs::SwitchMode>(
      service_name);
  if (!client.waitForExistence(ros::Duration(5))) {
    ROS_ERROR("switch_mode unavailable at %s", service_name.c_str());
    return 2;
  }

  bool preflight_ok = true;
  auto require_true = [&](const std::string& param) {
    bool value = false;
    if (!ros::param::get(controller_node + "/" + param, value)) {
      ROS_ERROR("refusing full mission: %s/%s is missing (must be true)",
                controller_node.c_str(), param.c_str());
      preflight_ok = false;
    } else if (!value) {
      ROS_ERROR("refusing full mission: %s/%s must be true",
                controller_node.c_str(), param.c_str());
      preflight_ok = false;
    }
  };
  require_true("visual_guidance_enabled");
  require_true("tag_guidance_enabled");
  const auto includes = [&](const char* mode) {
    return std::find(sequence.begin(), sequence.end(), mode) !=
           sequence.end();
  };
  if (includes("climb")) {
    require_true("enable_real_climb");
  }
  if (includes("dock")) {
    require_true("enable_real_dock");
  }
  if (!preflight_ok) {
    return 3;
  }

  for (const std::string& mode : sequence) {
    ROS_INFO("%s: starting", mode.c_str());
    grasp_hexapod_msgs::SwitchMode call;
    call.request.target_mode = mode;
    const bool called = client.call(call);
    if (!called || !call.response.success) {
      const std::string message =
          called ? call.response.message : std::string("service call failed");
      ROS_ERROR(
          "%s failed: %s; robot left in HOLD, operator takeover required",
          mode.c_str(), message.c_str());
      return 1;
    }
    ROS_INFO("%s: %s", mode.c_str(), call.response.message.c_str());
  }

  if (stop_after.empty()) {
    ROS_INFO("full mission sequence complete");
  } else {
    ROS_INFO("full mission sequence stopped after %s", stop_after.c_str());
  }
  return 0;
}
