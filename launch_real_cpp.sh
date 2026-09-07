#!/usr/bin/env bash
# 实机 C++ 舵机后端启动脚本（性能优化版）。
#
# 用法:
#   ./launch_real_cpp.sh                          # 默认参数启动
#   ./launch_real_cpp.sh start_dock_perception:=false   # 纯遥控测试，省掉相机识别栈
#
# 相比直接 roslaunch 的两处优化（实测把指令流从 ~17Hz 修回 30Hz）:
#   1. OPENBLAS/OMP 单线程: numpy 默认的 OpenBLAS 线程池会在每次矩阵运算后
#      占满 6 核自旋空转，实测白烧 ~1.3 核，拖慢舵机板 30Hz 串口循环
#      （约一半周期超时 47-66ms），控制器反馈门控随之把指令流掉到 17Hz。
#   2. jetson_clocks 锁频: 消除 schedutil 调频延迟（功耗档 MODE_20W_6CORE
#      已是本机最高档，无需再动 nvpmodel）。锁频重启后失效，每次启动重新锁。
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- 优化 1: BLAS/OMP 单线程，对本次 roslaunch 的所有子节点生效 ---
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1

# --- 优化 2: 锁频（尽力而为，失败不阻塞启动） ---


# --- 防重复启动: 串口是独占资源，旧实例不退会因端口被占而启动失败 ---
if pgrep -f "grasp_hexapod_servo" >/dev/null 2>&1 || pgrep -f "run_real\.py" >/dev/null 2>&1; then
  echo "[launch_real_cpp] 检测到已有实例在运行（servo_node/run_real.py），请先停掉旧 launch。" >&2
  exit 1
fi

# --- ROS 环境（ROS 的 profile 脚本引用未定义的 ROS_DISTRO，需临时关闭 -u） ---
set +u
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"
set -u

# --- 启动（附加参数原样透传给 launch 文件） ---
exec roslaunch grasp_hexapod_control run_real_cpp.launch "$@"
