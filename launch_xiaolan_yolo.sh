#!/usr/bin/env bash
# 启动小蓝检测、深度定位和可视化；相机从独立 launch 启动。
set -eo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
YOLO_PYTHON="${YOLO_PYTHON:-$WS_DIR/.venv/jetson-gpu/bin/python3}"
if [[ ! -f "$WS_DIR/devel/setup.bash" || ! -x "$YOLO_PYTHON" ]]; then
  echo "请先按 src/xiaolan_yolo_detection/README.md 完成编译和推理环境安装。" >&2
  exit 1
fi
source "$WS_DIR/devel/setup.bash"
exec roslaunch xiaolan_yolo_detection xiaolan_yolo.launch yolo_python:="$YOLO_PYTHON" "$@"
