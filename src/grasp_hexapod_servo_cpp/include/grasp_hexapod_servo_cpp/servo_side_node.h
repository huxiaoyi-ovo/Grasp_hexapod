// 单块九舵机驱动板的 ROS 节点（C++ 版 ServoSideNode）。
// 对应 Python 的 servo.py::ServoSideNode，接口与安全行为逐条对齐：
//   - 订阅 /<leg>_des（Float64MultiArray，固定 10 元素）；
//   - 发布 /<leg>_pos（JointState，thigh/knee/ankle 三关节带时间戳）；
//   - 板级上电安全机制：三腿都收到完整目标且都请求加载才整板加载。
#ifndef GRASP_HEXAPOD_SERVO_CPP_SERVO_SIDE_NODE_H_
#define GRASP_HEXAPOD_SERVO_CPP_SERVO_SIDE_NODE_H_

#include <array>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include "grasp_hexapod_servo_cpp/hiwonder_servo_controller.h"

namespace grasp_hexapod_servo_cpp {

class ServoSideNode {
 public:
  ServoSideNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~ServoSideNode();

 private:
  static constexpr int kLegsPerBoard = 3;
  static constexpr int kJointsPerLeg = 3;
  static constexpr int kServosPerBoard = 9;
  static constexpr int kDesiredMessageSize = 10;
  static constexpr std::array<const char*, kJointsPerLeg> kJointNames = {
      "thigh", "knee", "ankle"};

  struct LegTarget {
    bool received = false;
    bool power_request = false;
    std::array<double, kJointsPerLeg> pos = {0.0, 0.0, 0.0};
  };

  struct SideConfig {
    std::string port;
    std::vector<std::string> legs;
    std::map<std::string, std::array<int, kJointsPerLeg>> id_map;
    std::array<int, kServosPerBoard> directions;
  };

  struct TimingStats {
    std::chrono::steady_clock::time_point window_start;
    long callbacks = 0;
    double max_loop_s = 0.0;
    long overruns = 0;
    std::map<int, long> read_retries;
    std::map<int, long> read_failures;
  };

  // ---- 配置 ----
  std::string side_;
  std::string port_;
  int baudrate_ = 115200;
  double servo_rate_hz_ = 30.0;
  int command_duration_ms_ = 33;
  bool enable_diagnostics_ = true;
  double voltage_report_interval_s_ = 2.0;

  std::vector<std::string> legs_;
  std::map<std::string, std::array<int, kJointsPerLeg>> id_map_;
  std::map<int, int> directions_;  // servo_id -> 方向系数（1 或 -1）。
  std::vector<int> servo_ids_;     // 按腿顺序展开的 9 个舵机 ID。

  // ---- 运行时状态 ----
  std::mutex mutex_;
  std::map<std::string, LegTarget> leg_targets_;
  bool power_on_ = false;
  std::unique_ptr<HiwonderServoController> control_;

  std::map<std::string, ros::Subscriber> des_subs_;
  std::map<std::string, ros::Publisher> pos_pubs_;
  ros::Timer timer_;

  // ---- 诊断 ----
  TimingStats timing_;
  std::map<int, std::string> voltage_labels_;  // servo_id -> leg_joint。
  std::deque<int> voltage_pending_ids_;
  std::map<int, int> voltage_samples_mv_;
  std::chrono::steady_clock::time_point voltage_next_report_at_;

  static std::optional<SideConfig> getSideConfig(const std::string& side);
  void loadParams(ros::NodeHandle& nh_private);
  std::vector<int> loadDirectionsParam(ros::NodeHandle& nh_private);
  void setBoardPower(bool enabled);
  void onDesired(const std_msgs::Float64MultiArray::ConstPtr& message,
                 const std::string& leg);
  void controlLoop(const ros::TimerEvent&);
  std::optional<int> readPositionWithRetry(int servo_id);
  void publishLeg(const std::string& leg,
                  const std::vector<double>& positions);
  void updateVoltageDiagnostics();
  void recordTimingDiagnostics(
      std::chrono::steady_clock::time_point started_at);
  std::string timingCountsText(const std::map<int, long>& counts) const;
};

}  // namespace grasp_hexapod_servo_cpp

#endif  // GRASP_HEXAPOD_SERVO_CPP_SERVO_SIDE_NODE_H_
