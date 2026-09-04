#!/usr/bin/env bash
# =============================================================================
# grasp_hexapod_bt 行为树依赖安装脚本
#
# 本包行为树使用 py_trees 2.2.x（纯 Python 库，无 rosdep 键，
# 见 package.xml 注释），本脚本负责安装并验证。
#
# 用法：
#   ./install_py_trees.sh            # 用户级安装（推荐，无需 sudo）
#   sudo ./install_py_trees.sh --system   # 系统级安装（所有用户可用）
# =============================================================================
set -euo pipefail

PY_TREES_VERSION="2.2.3"   # 2.2.x 最新稳定版，兼容 Python 3.8 / ROS Noetic
INSTALL_MODE="user"
if [[ "${1:-}" == "--system" ]]; then
    INSTALL_MODE="system"
fi

echo "==> [1/4] 检查 python3 与 pip3 ..."
if ! command -v python3 >/dev/null 2>&1; then
    echo "错误：未找到 python3，请先安装：sudo apt install python3 python3-pip" >&2
    exit 1
fi
if ! command -v pip3 >/dev/null 2>&1; then
    echo "错误：未找到 pip3，请先安装：sudo apt install python3-pip" >&2
    exit 1
fi
echo "    python3: $(python3 --version)"
echo "    pip3:    $(pip3 --version)"

echo "==> [2/4] 安装 py_trees==${PY_TREES_VERSION}（${INSTALL_MODE} 模式）..."
if [[ "${INSTALL_MODE}" == "system" ]]; then
    sudo pip3 install "py_trees==${PY_TREES_VERSION}"
else
    # --user 安装到 ~/.local，无需 sudo，不污染系统环境
    pip3 install --user "py_trees==${PY_TREES_VERSION}"
fi

echo "==> [3/4] 验证安装 ..."
python3 - <<'EOF'
# py_trees 模块本身不带 __version__ 属性，从包元数据读取版本
from importlib.metadata import version
import py_trees
v = version("py_trees")
assert v.startswith("2.2"), f"py_trees 版本不符：{v}（需要 2.2.x）"
print(f"    py_trees {v} 导入成功")
print(f"    安装位置: {py_trees.__file__}")
EOF

echo "==> [4/4] 完成。可选自检：py-trees-demo --help 查看自带演示。"
echo "    运行行为树节点前记得先 source 工作空间："
echo "    source ~/Grasp_hexapod/devel/setup.bash"
