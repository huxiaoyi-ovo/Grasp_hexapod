#include "grasp_hexapod_control_cpp/control.h"
#include <nlohmann/json.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
using namespace grasp_hexapod_control_cpp;
int main(int argc, char** argv) {
  ControlPaths paths{argv[1], argv[2]};
  std::ifstream f(argv[3]);
  nlohmann::json golden;
  f >> golden;
  const nlohmann::json* seq = nullptr;
  for (const auto& item : golden["sequences"]) {
    if (item["name"] == "reset_then_walk") { seq = &item; break; }
  }
  const int frames = static_cast<int>((*seq)["frames"].size());
  const int repeats = 20;
  std::unique_ptr<GraspController> controller;
  JointAngles q_start = makeQStand();
  const double offs[6][3] = {{10,-12,8},{-9,11,-7},{12,-10,9},{-11,9,-8},{9,-13,11},{-10,8,-9}};
  for (int leg = 0; leg < kLegCount; ++leg)
    for (int j = 0; j < kJointCount; ++j)
      q_start[leg][j] += offs[leg][j] * M_PI / 180.0;
  controller = std::make_unique<GraspController>(1.0/30.0, false, false, paths);
  controller->resetToStand(q_start);
  JointAngles q = q_start;
  const auto t0 = std::chrono::steady_clock::now();
  for (int r = 0; r < repeats; ++r) {
    // 每轮重复需要重置内部状态，但计时包含重置；重置成本可忽略（无ROS）。
    controller = std::make_unique<GraspController>(1.0/30.0, false, false, paths);
    controller->resetToStand(q_start);
    q = q_start;
    int idx = 0;
    for (const auto& fr : (*seq)["frames"]) {
      (void)fr;
      Eigen::Vector4d cmd = Eigen::Vector4d::Zero();
      const int reset_frames = 75;
      if (idx >= reset_frames && idx < reset_frames + 300)
        cmd = Eigen::Vector4d(0.05, 0.10, 0.0, 0.15);
      else if (idx >= reset_frames + 300 && idx < reset_frames + 600)
        cmd = Eigen::Vector4d(-0.08, -0.05, 0.008, -0.20);
      q = controller->update(q, cmd);
      ++idx;
    }
  }
  const auto t1 = std::chrono::steady_clock::now();
  const double total_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  std::cout << "cpp_per_frame_us=" << total_us / (repeats * frames) << "\n";
  std::cout << "frames=" << frames << " repeats=" << repeats << "\n";
  return 0;
}
