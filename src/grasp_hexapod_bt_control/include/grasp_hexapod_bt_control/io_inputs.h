// 节点级/模式级订阅输入封装：回调只写缓存（互斥锁保护），控制线程经
// snapshot() 只读访问。NavigationInput 与 ImuInput 由调度器持有（安全环路
// 与多个模式共用）；BoolInput 供单个模式自持（如 dock 的锁紧确认）。
#pragma once

#include "grasp_hexapod_bt_control/mode_topic_config.h"

#include "grasp_hexapod_control_cpp/control.h"

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

#include <algorithm>
#include <array>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace grasp_hexapod_bt_control {

using grasp_hexapod_control_cpp::JointAngles;
using grasp_hexapod_control_cpp::Matrix3d;
using grasp_hexapod_control_cpp::Matrix4d;
using grasp_hexapod_control_cpp::NavigationState;
using grasp_hexapod_control_cpp::Vector2d;
using grasp_hexapod_control_cpp::Vector3d;
using grasp_hexapod_control_cpp::poseToTransform;

inline double nowSeconds() { return ros::Time::now().toSec(); }

// IMU 只读快照（valid=false 表示过期或未收到）。
struct ImuSnapshot {
  bool valid = false;
  Matrix3d rotation = Matrix3d::Identity();
  Vector3d angular_velocity =
      Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
};

// RTK/LoRa 导航 + 着陆确认四话题输入。
class NavigationInput {
 public:
  NavigationInput(ros::NodeHandle& nh, const ModeTopics& topics,
                  double max_age, double max_pose_skew,
                  double max_boundary_age)
      : max_age_(max_age),
        max_pose_skew_(max_pose_skew),
        max_boundary_age_(max_boundary_age) {
    subscribers_.push_back(nh.subscribe<geometry_msgs::PoseStamped>(
        topics.get("base_pose", "/grasp_hexapod/navigation/base_pose"), 1,
        [this](const geometry_msgs::PoseStamped::ConstPtr& message) {
          baseCallback(message);
        }));
    subscribers_.push_back(nh.subscribe<geometry_msgs::PoseStamped>(
        topics.get("xiaolan_pose", "/grasp_hexapod/navigation/xiaolan_pose"),
        1, [this](const geometry_msgs::PoseStamped::ConstPtr& message) {
          xiaolanCallback(message);
        }));
    subscribers_.push_back(nh.subscribe<geometry_msgs::PolygonStamped>(
        topics.get("pv_boundary", "/grasp_hexapod/navigation/pv_boundary"), 1,
        [this](const geometry_msgs::PolygonStamped::ConstPtr& message) {
          boundaryCallback(message);
        }));
    subscribers_.push_back(nh.subscribe<std_msgs::Bool>(
        topics.get("landing_confirmed", "/grasp_hexapod/landing_confirmed"), 1,
        [this](const std_msgs::Bool::ConstPtr& message) {
          landingCallback(message);
        }));
  }

  // 巡航快照：位姿 + 边界全部新鲜才 valid（approach 用）。
  NavigationState snapshot() const;
  // 运动快照：只要求位姿新鲜，不要求边界（climb 相对运动监控用）。
  NavigationState motionSnapshot() const;

 private:
  bool validFrameImpl(const std::string& frame_id) const;
  void baseCallback(const geometry_msgs::PoseStamped::ConstPtr& message);
  void xiaolanCallback(const geometry_msgs::PoseStamped::ConstPtr& message);
  void boundaryCallback(const geometry_msgs::PolygonStamped::ConstPtr& message);
  void landingCallback(const std_msgs::Bool::ConstPtr& message);

  double max_age_;
  double max_pose_skew_;
  double max_boundary_age_;
  mutable std::mutex lock_;
  double base_stamp_ = 0.0;
  double xiaolan_stamp_ = 0.0;
  double boundary_stamp_ = 0.0;
  bool have_base_ = false;
  bool have_xiaolan_ = false;
  Matrix4d pv_from_base_ = Matrix4d::Identity();
  Matrix4d pv_from_xiaolan_ = Matrix4d::Identity();
  std::vector<Vector2d> pv_boundary_;
  bool landing_confirmed_ = false;
  std::vector<ros::Subscriber> subscribers_;
};

// IMU 姿态/角速度输入（新鲜度按 max_age 判定）。
class ImuInput {
 public:
  ImuInput(ros::NodeHandle& nh, const std::string& topic, double max_age)
      : max_age_(max_age) {
    subscriber_ = nh.subscribe<sensor_msgs::Imu>(
        topic, 1,
        [this](const sensor_msgs::Imu::ConstPtr& message) {
          callback(message);
        });
  }

  ImuSnapshot snapshot() const;

 private:
  void callback(const sensor_msgs::Imu::ConstPtr& message);

  double max_age_;
  mutable std::mutex lock_;
  double stamp_ = 0.0;
  bool have_rotation_ = false;
  Matrix3d rotation_ = Matrix3d::Identity();
  Vector3d angular_velocity_ =
      Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  ros::Subscriber subscriber_;
};

// 单话题 Bool 输入（模式自持；记录收到时间以便会话内新鲜度判断）。
class BoolInput {
 public:
  BoolInput(ros::NodeHandle& nh, const std::string& topic) {
    subscriber_ = nh.subscribe<std_msgs::Bool>(
        topic, 1, [this](const std_msgs::Bool::ConstPtr& message) {
          callback(message);
        });
  }

  std::optional<bool> snapshot() const;
  std::pair<std::optional<bool>, double> snapshotWithTime() const;

 private:
  void callback(const std_msgs::Bool::ConstPtr& message);

  mutable std::mutex lock_;
  std::optional<bool> value_;
  double received_at_ = 0.0;
  ros::Subscriber subscriber_;
};

}  // namespace grasp_hexapod_bt_control
