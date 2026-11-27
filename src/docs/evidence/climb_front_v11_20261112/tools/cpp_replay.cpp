// Offline C++ replay of a compact climb config with an ideal servo (q_cur = previous q_des).
// Usage: cpp_replay <config.json> <config_dir> <hardware 0|1> <out.csv>
#include "grasp_hexapod_control_cpp/control.h"
#include "grasp_hexapod_control_cpp/climb_compact.h"
#include <fstream>
#include <iostream>
using namespace grasp_hexapod_control_cpp;
int main(int argc, char** argv) {
  const std::string cfg_path = argv[1], dir = argv[2];
  const bool hw = std::string(argv[3]) == "1";
  ControlPaths paths{dir + "/climb_compact.json", dir + "/workspace_bounds.csv"};
  GraspController c(1.0 / 30.0, false, false, paths);
  ClimbCompactConfig cfg = loadClimbCompact(cfg_path);
  cfg = selectCompactClimbSide(cfg, "left");
  if (hw) { cfg.allow_front_hardware = true; cfg.settle_gate.max_foot_target_error_m = 0.04; }
  JointAngles q = makeQStand();
  c.enterClimb(q, cfg, 0, std::nullopt, hw);
  std::ofstream out(argv[4]);
  int tick = 0;
  while (c.climb_mode.state == ClimbState::Running && tick < 40000) {
    q = c.update(q, Eigen::Vector4d::Zero());
    out << tick;
    for (int l = 0; l < kLegCount; ++l) for (int j = 0; j < kJointCount; ++j) out << "," << q[l][j];
    out << "\n"; ++tick;
  }
  std::cout << "state=" << static_cast<int>(c.climb_mode.state) << " ticks=" << tick << std::endl;
  return 0;
}
