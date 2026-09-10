// 金标回归测试共用：加载 golden JSON、数组转换与逐帧对比辅助。
#pragma once

#include "grasp_hexapod_control_cpp/control.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <string>

#ifndef GRASP_CONTROL_CONFIG_DIR
#define GRASP_CONTROL_CONFIG_DIR "../grasp_hexapod_control/config"
#endif

namespace grasp_hexapod_control_cpp {
namespace testing {

using nlohmann::json;

inline json loadGolden() {
  std::ifstream file(GRASP_GOLDEN_JSON);
  if (!file) {
    throw std::runtime_error("cannot open golden JSON: " + std::string(GRASP_GOLDEN_JSON));
  }
  json golden;
  file >> golden;
  return golden;
}

inline ControlPaths goldenPaths() {
  const std::string config_dir = GRASP_CONTROL_CONFIG_DIR;
  return ControlPaths{
      config_dir + "/climb_compact.json",
      config_dir + "/workspace_bounds.csv",
  };
}

inline JointAngles toJoints(const std::vector<double>& flat) {
  JointAngles q{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    for (int joint = 0; joint < kJointCount; ++joint) {
      q[leg][joint] = flat[leg * kJointCount + joint];
    }
  }
  return q;
}

inline FootPositions toFeet(const std::vector<double>& flat) {
  FootPositions feet{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    for (int axis = 0; axis < 3; ++axis) {
      feet[leg][axis] = flat[leg * 3 + axis];
    }
  }
  return feet;
}

// JointAngles 与 FootPositions 同为 std::array<Vector3d,6>，共用一个重载。
inline double maxAbsDiff(const std::array<Eigen::Vector3d, kLegCount>& actual,
                         const std::array<Eigen::Vector3d, kLegCount>& expected) {
  double worst = 0.0;
  for (int leg = 0; leg < kLegCount; ++leg) {
    worst = std::max(worst, (actual[leg] - expected[leg]).cwiseAbs().maxCoeff());
  }
  return worst;
}

}  // namespace testing
}  // namespace grasp_hexapod_control_cpp
