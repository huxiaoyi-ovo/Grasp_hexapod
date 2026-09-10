#!/usr/bin/env bash
# 金标测试快速构建脚本（不依赖 ROS/catkin，直接 g++）。
set -euo pipefail
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${1:-/tmp/golden_build}"
GOLDEN="$PKG_DIR/test/golden_control_trajectories.json"
CFG="$PKG_DIR/../grasp_hexapod_control/config"
MESH="$PKG_DIR/../grasp_hexapod_description/meshes"
SRCS=(
  src/math_utils.cpp
  src/kinematics.cpp
  src/config_io.cpp
  src/climb_compact.cpp
  src/climb_mode.cpp
  src/approach_mode.cpp
  src/dock_mode.cpp
  src/control.cpp
)
mkdir -p "$BUILD_DIR"
for test_name in kinematics_golden golden_trajectory golden_config; do
  extra=()
  if [ "$test_name" = "golden_config" ]; then
    extra+=("-DGRASP_DESCRIPTION_MESH_DIR=\"$MESH\"")
  fi
  g++ -std=c++17 -O2 -Wall \
    -I"$PKG_DIR/include" -I"$PKG_DIR/vendor" -I/usr/include/eigen3 \
    $(pkg-config --cflags yaml-cpp) \
    "-DGRASP_GOLDEN_JSON=\"$GOLDEN\"" \
    "-DGRASP_CONTROL_CONFIG_DIR=\"$CFG\"" \
    "${extra[@]}" \
    -o "$BUILD_DIR/test_$test_name" \
    "${SRCS[@]/#/$PKG_DIR/}" "$PKG_DIR/test/test_$test_name.cpp" \
    -lgtest -lgtest_main -lpthread $(pkg-config --libs yaml-cpp)
  echo "built: $BUILD_DIR/test_$test_name"
done
