// walk 模式：手柄/导航速度驱动的步行接近步态（/cmd_vel 平面三自由度）。
// 订阅 /cmd_vel 由本类自持（话题名来自 mode_topics.yaml 的 walk 节）。
// 迁移自原 bt_control_node 的 cmdVelCallback/clampCommand、startBtRequest
// walk 分支与 "/cmd_vel lost" 失败路径。
#pragma once

#include "grasp_hexapod_bt_control/mode_base.h"

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <mutex>

namespace grasp_hexapod_bt_control {
namespace modes {

class WalkMode : public ModeBase {
 public:
  WalkMode(ModeContext& ctx, const ModeTopics& topics);

  bool start(const JointAngles& q_cur, std::string& reason) override;
  std::optional<JointAngles> step(const ModeStepInput& input) override;
  void onPreFrame(const ModeStepInput& input) override;
  std::optional<ModeOutcome> checkTerminal() override;

  // /cmd_vel 平面速度命令限幅（静态纯函数，头文件内联便于测试与节点复用）。
  // 轴约定：linear.y=右、linear.x=前、angular.z=航向；z 恒为 0。
  static Eigen::Vector4d clampCommand(double vx_right, double vy_forward,
                                      double yaw_rate, double max_linear_speed,
                                      double max_yaw_rate) {
    // 平面速度按向量模限幅（方向保持），航向单独限幅；z 恒为 0。
    Eigen::Vector2d planar(vx_right, vy_forward);
    const double norm = planar.norm();
    if (!std::isfinite(norm) || norm <= 0.0) {
      planar.setZero();
    } else if (norm > max_linear_speed) {
      planar *= max_linear_speed / norm;
    }
    double yaw = std::isfinite(yaw_rate) ? yaw_rate : 0.0;
    yaw = std::max(-max_yaw_rate, std::min(max_yaw_rate, yaw));
    return Eigen::Vector4d(planar.x(), planar.y(), 0.0, yaw);
  }

 private:
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& message);

  double max_linear_speed_;
  double max_yaw_rate_;
  double max_cmd_vel_age_;
  ros::Subscriber subscriber_;
  mutable std::mutex cmd_lock_;
  Eigen::Vector4d cmd_ = Eigen::Vector4d::Zero();  // 已限幅
  double cmd_stamp_ = 0.0;
};

}  // namespace modes
}  // namespace grasp_hexapod_bt_control
