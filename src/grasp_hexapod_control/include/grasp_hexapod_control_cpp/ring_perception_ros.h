#pragma once
#include "grasp_hexapod_control_cpp/ring_dock_mode.h"
#include "grasp_hexapod_control_cpp/config_io.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <condition_variable>
#include <mutex>
#include <thread>
namespace grasp_hexapod_ring {
class RingPerceptionRos : public PerceptionInterface {
 public:
  RingPerceptionRos(ros::NodeHandle& pnh,const std::string& config);
  ~RingPerceptionRos() override;
  void reset() override;
  PerceptionResult latest() override;
 private:
  void work();
  ros::Subscriber images_,infos_;
  std::mutex mutex_;
  std::condition_variable ready_;
  bool stop_=false,rectified_=false;
  uint64_t generation_=0;
  sensor_msgs::ImageConstPtr pending_;
  sensor_msgs::CameraInfoConstPtr info_;
  PerceptionResult result_;
  Matrix4d lock_from_camera_;
  double max_age_;
  std::thread worker_;
};
}
