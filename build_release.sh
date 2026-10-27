#!/usr/bin/env bash
# 工作空间编译脚本（唯一推荐的编译入口）。
#
# 用法:
#   ./build_release.sh                    # 标准编译（Release）
#   ./build_release.sh -j4                # 限制并行度（Eigen 重模板内存紧张时用）
#   ./build_release.sh --pkg grasp_hexapod_control   # 只编译单个包
#   ./build_release.sh --setup-orbbec     # 首次部署 Orbbec 相机：装依赖 + udev 规则后编译
#
# 为什么必须用本脚本（2026-09 实测教训）:
#   直接 `catkin_make` 时 CMAKE_BUILD_TYPE 为空 → C++ 以 -O0 编译。
#   Eigen 表达式模板在 -O0 下性能崩塌，实测攀爬热路径（FK+雅可比+DLS）
#   从 16µs/步 恶化到 1650µs/步（102 倍），攀爬期控制器 CPU 从 16.6%
#   飙到 37.2%，指令周期抖动 std 从 1.03ms 恶化到 2.84ms。
#   本脚本强制 Release 并在编译后校验缓存，防止退回无优化状态。
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

# --- ROS 环境 ---
# ROS 的 setup 脚本内部引用未定义变量，与 set -u 冲突，source 期间临时关闭。
if [ -z "${ROS_DISTRO:-}" ]; then
  if [ -f /opt/ros/noetic/setup.bash ]; then
    set +u
    source /opt/ros/noetic/setup.bash
    set -u
  else
    echo "错误: 未找到 ROS 环境（/opt/ros/*/setup.bash）且 ROS_DISTRO 未设置" >&2
    exit 1
  fi
fi

# --- 编译前检查：缓存里的构建类型不对时，删掉 build/devel 重新配置 ---
# 注意: 对已配置的工作空间传 -D 参数并不保证写回缓存（catkin_make 会比对
# 参数列表决定是否重新 configure），实测缓存被改坏后会照旧以 -O0 全量重编。
# 唯一可靠的做法是删除缓存目录强制全新配置（反正编译标志全换了，本就必须全量重编）。
CACHE="$WS_DIR/build/CMakeCache.txt"
if [ -f "$CACHE" ]; then
  PREV_TYPE="$(grep -m1 '^CMAKE_BUILD_TYPE' "$CACHE" | cut -d= -f2 || true)"
  if [ -z "$PREV_TYPE" ] || [ "$PREV_TYPE" != "Release" ]; then
    echo "警告: 检测到缓存构建类型为 '${PREV_TYPE:-空}'（非 Release），删除 build/ devel/ 强制全新配置"
    rm -rf "$WS_DIR/build" "$WS_DIR/devel"
  fi
fi

# --- Orbbec 相机包（src/reference/OrbbecSDK_ROS1，Gemini 336L）---
# 普通 catkin 包，随下方 catkin_make 一同编译，无单独编译步骤。
# SDK 预编译库只保留 arm64（Jetson）；换 x64 主机需从上游补 SDK/lib/x64。
# 相机是标准 USB 设备，首次使用必须装 udev 规则，否则非 root 下枚举不到设备。
ORBBEC_PKG="$WS_DIR/src/reference/OrbbecSDK_ROS1"

setup_orbbec() {
  if [ ! -f "$ORBBEC_PKG/package.xml" ]; then
    echo "错误: 未找到 Orbbec 相机包: $ORBBEC_PKG" >&2
    exit 1
  fi
  echo "==> 安装 orbbec_camera 依赖"
  # 首选 rosdep（按 package.xml 自动解析）；raw.githubusercontent.com 被墙时
  # 回退到 apt 直装完整映射清单（与 package.xml 一致，幂等）。
  if rosdep install --from-paths "$ORBBEC_PKG" --ignore-src -y; then
    echo "==> rosdep 依赖安装完成"
  else
    echo "警告: rosdep 不可用（GitHub 连接问题），改用 apt 直装依赖清单" >&2
    sudo apt-get install -y \
      ros-noetic-roscpp ros-noetic-sensor-msgs ros-noetic-std-msgs \
      ros-noetic-std-srvs ros-noetic-image-transport \
      ros-noetic-camera-info-manager ros-noetic-cv-bridge \
      ros-noetic-dynamic-reconfigure ros-noetic-image-geometry \
      ros-noetic-message-filters ros-noetic-tf2 ros-noetic-tf2-ros \
      ros-noetic-pluginlib ros-noetic-nodelet \
      ros-noetic-diagnostic-updater ros-noetic-backward-ros \
      libudev-dev libusb-1.0-0-dev libdw-dev
  fi
  echo "==> 安装 Orbbec udev 规则（需要 sudo；脚本内含 udevadm reload+trigger）"
  sudo bash "$ORBBEC_PKG/scripts/install_udev_rules.sh"
  echo "==> udev 就绪：重新插拔相机 USB 后生效，验证: lsusb | grep -i 2bc5"
}

# --- 编译: Release 强制固定，额外参数原样透传（--setup-orbbec 除外）---
JOBS="$(nproc)"
EXTRA_ARGS=()
SETUP_ORBBEC=0
for arg in "$@"; do
  if [ "$arg" = "--setup-orbbec" ]; then
    SETUP_ORBBEC=1
  else
    EXTRA_ARGS+=("$arg")
  fi
done
if [ "$SETUP_ORBBEC" -eq 1 ]; then
  setup_orbbec
fi
if [[ "${EXTRA_ARGS[*]:-}" != *-j* ]]; then
  EXTRA_ARGS=("-j$JOBS" "${EXTRA_ARGS[@]}")
fi

echo "==> catkin_make -DCMAKE_BUILD_TYPE=Release ${EXTRA_ARGS[*]}"
SECONDS=0
catkin_make -DCMAKE_BUILD_TYPE=Release "${EXTRA_ARGS[@]}"
echo "==> 编译完成，耗时 ${SECONDS}s"

# --- 编译后校验: 缓存里的构建类型必须是 Release，否则视为失败 ---
POST_TYPE="$(grep -m1 '^CMAKE_BUILD_TYPE' "$CACHE" 2>/dev/null | cut -d= -f2 || true)"
if [ "$POST_TYPE" != "Release" ]; then
  echo "错误: 编译后 CMAKE_BUILD_TYPE='${POST_TYPE:-空}'，不是 Release！" >&2
  echo "      请删除 build/ devel/ 后重试: rm -rf build devel && ./build_release.sh" >&2
  exit 1
fi
echo "==> 校验通过: CMAKE_BUILD_TYPE=Release"
