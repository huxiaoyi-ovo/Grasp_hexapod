#include "grasp_hexapod_control_cpp/ring_perception_ros.h"
#include "grasp_hexapod_control_cpp/ring_detection.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sstream>
namespace grasp_hexapod_ring {
RingPerceptionRos::RingPerceptionRos(ros::NodeHandle& pnh,const std::string& config)
 :lock_from_camera_(loadDockSystem(config).lock_from_camera) {
  max_age_=pnh.param("dock_max_perception_age",1.0);
  if(!std::isfinite(max_age_) || max_age_<=0) throw ConfigError("dock_max_perception_age must be finite and positive");
  const auto topic=pnh.param<std::string>("dock_image_topic","/dock_camera/image_raw");
  rectified_=pnh.param("dock_image_is_rectified",topic.find("image_rect")!=std::string::npos);
  images_=pnh.subscribe<sensor_msgs::Image>(topic,1,[this](const sensor_msgs::ImageConstPtr& msg){
    std::lock_guard<std::mutex> lock(mutex_);pending_=msg;ready_.notify_one();
  },ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
  infos_=pnh.subscribe<sensor_msgs::CameraInfo>(pnh.param<std::string>("dock_camera_info_topic","/dock_camera/camera_info"),1,
    [this](const sensor_msgs::CameraInfoConstPtr& msg){std::lock_guard<std::mutex> lock(mutex_);info_=msg;});
  worker_=std::thread(&RingPerceptionRos::work,this);
  ROS_INFO("C++ ring perception: %s, maximum capture age %.2fs",topic.c_str(),max_age_);
}
RingPerceptionRos::~RingPerceptionRos() {
  images_.shutdown();infos_.shutdown();
  {std::lock_guard<std::mutex> lock(mutex_);stop_=true;ready_.notify_one();}
  if(worker_.joinable())worker_.join();
}
void RingPerceptionRos::reset() {
  std::lock_guard<std::mutex> lock(mutex_);++generation_;pending_.reset();result_=PerceptionResult{};
}
PerceptionResult RingPerceptionRos::latest() {
  std::lock_guard<std::mutex> lock(mutex_);auto result=result_;
  if(result.stamp==0)return result;
  const double age=ros::Time::now().toSec()-result.stamp;
  if(age<0 || age>max_age_) {
    result.valid=false;std::ostringstream reason;
    reason<<"ring image is stale: age="<<age<<"s, limit="<<max_age_<<"s; "<<result.reason;
    result.reason=reason.str();
  }
  return result;
}
void RingPerceptionRos::work() {
  cv::Mat map_x,map_y;
  sensor_msgs::CameraInfoConstPtr mapped_info;
  while(true) {
    sensor_msgs::ImageConstPtr message;sensor_msgs::CameraInfoConstPtr info;uint64_t generation;
    {
      std::unique_lock<std::mutex> lock(mutex_);ready_.wait(lock,[this]{return stop_ || bool(pending_);});
      if(stop_)return;
      message=std::move(pending_);info=info_;generation=generation_;
    }
    const auto started=ros::WallTime::now();
    PerceptionResult result;result.stamp=message->header.stamp.toSec();
    try {
      if(!info)throw ConfigError("waiting for valid camera_info");
      if(info->width!=message->width || info->height!=message->height)throw ConfigError("image size does not match camera_info");
      cv::Mat intrinsic(3,3,CV_64F),raw_k(3,3,CV_64F),rotation(3,3,CV_64F);
      for(int i=0;i<3;++i)for(int j=0;j<3;++j) {
        intrinsic.at<double>(i,j)=info->P[4*i+j];raw_k.at<double>(i,j)=info->K[3*i+j];rotation.at<double>(i,j)=info->R[3*i+j];
      }
      if(intrinsic.at<double>(0,0)<=0 || intrinsic.at<double>(1,1)<=0)intrinsic=raw_k.clone();
      if(!cv::checkRange(intrinsic) || intrinsic.at<double>(0,0)<=0 || intrinsic.at<double>(1,1)<=0)
        throw ConfigError("waiting for valid camera_info");
      auto converted=cv_bridge::toCvShare(message,sensor_msgs::image_encodings::MONO8);
      cv::Mat gray=converted->image;
      if(!rectified_) {
        if((info->distortion_model!="plumb_bob" && info->distortion_model!="rational_polynomial") ||
           !cv::checkRange(raw_k) || !cv::checkRange(rotation) || raw_k.at<double>(0,0)<=0 || raw_k.at<double>(1,1)<=0)
          throw ConfigError("raw image requires valid supported camera calibration");
        cv::Mat distortion(info->D,true);
        if(!distortion.empty() && !cv::checkRange(distortion))throw ConfigError("non-finite camera distortion");
        if(!mapped_info || mapped_info->K!=info->K || mapped_info->P!=info->P || mapped_info->R!=info->R ||
           mapped_info->D!=info->D || mapped_info->width!=info->width || mapped_info->height!=info->height) {
          cv::initUndistortRectifyMap(raw_k,distortion,rotation,intrinsic,gray.size(),CV_32FC1,map_x,map_y);mapped_info=info;
        }
        cv::Mat rectified;cv::remap(gray,rectified,map_x,map_y,cv::INTER_LINEAR);gray=rectified;
      }
      const auto detection=detectRings(gray);result.valid=detection.valid;result.reason=detection.reason;
      if(detection.valid) {
        const double fx=intrinsic.at<double>(0,0),fy=intrinsic.at<double>(1,1);
        const double z=fx*.182/detection.diameter;
        Matrix4d camera_pose=Matrix4d::Identity();
        camera_pose(0,3)=(detection.center.x-intrinsic.at<double>(0,2))*z/fx;
        camera_pose(1,3)=(detection.center.y-intrinsic.at<double>(1,2))*z/fy;camera_pose(2,3)=z;
        result.lock_from_pin=lock_from_camera_*camera_pose;
      }
    } catch(const std::exception& error) {result.valid=false;result.reason=error.what();}
    ROS_INFO_THROTTLE(5.0,"C++ ring: compute=%.1fms capture_age=%.1fms valid=%d %s",
      (ros::WallTime::now()-started).toSec()*1000,(ros::Time::now().toSec()-result.stamp)*1000,result.valid,result.reason.c_str());
    std::lock_guard<std::mutex> lock(mutex_);
    if(generation==generation_)result_=std::move(result);
  }
}
}
