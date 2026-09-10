#include "grasp_hexapod_control_cpp/control.h"
#include <chrono>
#include <iostream>
using namespace grasp_hexapod_control_cpp;
int main(int argc, char** argv) {
  ControlPaths paths{argv[1], argv[2]};
  auto c = std::make_unique<GraspController>(1.0/30.0, false, false, paths);
  JointAngles q = makeQStand();
  for (int leg = 0; leg < kLegCount; ++leg) q[leg] = q[leg].array() + 0.3;
  for (int i = 0; i < 100; ++i) c->linkCollisionFree(q);
  const auto t0 = std::chrono::steady_clock::now();
  const int N = 100000;
  for (int i = 0; i < N; ++i) c->linkCollisionFree(q);
  const auto t1 = std::chrono::steady_clock::now();
  std::cout << "cpp_link_collision_us="
            << std::chrono::duration<double, std::micro>(t1 - t0).count() / N << "\n";
  return 0;
}
