# Grasp Hexapod

这是一个基于 ROS Noetic 的六足机器人工作空间。项目使用手写控制器，支持 Isaac Gym 仿真和三块 LX-15D 舵机板实机运行。

## 目录

- `src/grasp_hexapod_control/`：步态、运动学、安全控制和仿真/实机入口。
- `src/grasp_hexapod_description/`：URDF、网格、RViz 和 Gazebo 配置。
- `src/grasp_hexapod_servo/`：LX-15D 三板驱动和串口协议。
- `src/reference/`：参考代码，不参与运行。

仿真和实机共用 `GraspController`：前者在 Isaac 控制帧内运行，后者通过 ROS 读取反馈并发布关节目标。

控制包按职责命名：`run_*.py` 是执行入口，`control.py`、`*_mode.py` 和
`kinematics.py` 是运行时核心，`scripts/utils/` 只放共享库；
`scripts/tools/` 收纳 `analyze_*.py`、`plan_*.py` 和历史 `validate_climb.py`
等离线命令，不进入实时控制链。

## 开发协作

- `test` 是集成分支；从 `test` 创建功能分支，合并目标也是 `test`，不要直接在 `main` 开发。
- 在仓库根目录构建和运行检查。
- URDF 和 CAD 导出文件只做必要的小修改；如果以后重新导出，需同步保留这些手工修改。

## 构建

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

Isaac Gym 需要安装在它支持的独立 Python 环境中。日常仿真可直接使用该环境运行入口脚本。

## 普通仿真

```bash
/home/artrc/miniconda3/envs/grasp_hexapod/bin/python \
  src/grasp_hexapod_control/scripts/run_sim.py
```

使用 ROS 手柄或导航输入时：

```bash
roslaunch grasp_hexapod_control run_sim_ros.launch
```

## 实机部署

三块驱动板的默认映射如下。`left`、`right`、`mid` 是启动参数中的板标识。

| 板 | 腿 | 舵机 ID | 默认串口 |
|---|---|---|---|
| `left` | `lf`、`lm` | 1～6 | `/dev/ttyUSB0` |
| `right` | `rf`、`rm` | 10～15 | `/dev/ttyUSB1` |
| `mid` | `lb`、`rb` | 7～9、16～18 | `/dev/ttyUSB2` |

部署时优先使用稳定的 `/dev/serial/by-id/...` 路径：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  left_port:=/dev/serial/by-id/<left-board> \
  right_port:=/dev/serial/by-id/<right-board> \
  mid_port:=/dev/serial/by-id/<mid-board>
```

首次架空测试可显式降低速度：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  max_linear_speed:=0.01 \
  max_vertical_speed:=0.003
```

启动前检查设备和 ROS 节点：

```bash
ls -l /dev/input/js*
ls -l /dev/serial/by-id/
roslaunch --nodes grasp_hexapod_control run_real.launch
```

启动后检查手柄和一条腿的反馈、目标：

```bash
rostopic echo /joy
rostopic echo /lf_pos
rostopic echo /lf_des
```

## 安全操作

机器人必须架起后，先确认串口、舵机 ID、安装方向和机械零位，再进行行走测试。

```text
B -> 等待回到标准站姿 -> A -> 接受运动指令
```

- B：取消当前行为并平滑回到标准站姿。
- A：站姿初始化完成后启用或暂停运动。
- 第一次按 B 前，高层控制器不发送目标，舵机保持卸力。

## 攀爬预览

攀爬只能在 Isaac Gym 中预览，实机攀爬禁止启动。

```bash
conda run --no-capture-output -n grasp_hexapod python3 \
  src/grasp_hexapod_control/scripts/run_sim.py \
  --climb-start --climb-speed 4 --climb-joint-speed 3
```

可只回放一个闭区间：`C1` 至 `C35` 是 compact 数组顺序的固定用户别名，也可
使用运行时阶段名。中途入口会把 Isaac root、18 个关节和足端锚点同步到该阶段
的规划起点；它仍只是仿真预览。

```bash
conda run --no-capture-output -n grasp_hexapod python3 \
  src/grasp_hexapod_control/scripts/run_sim.py \
  --climb-start --climb-from C13 --climb-to C15 \
  --climb-speed 4 --climb-joint-speed 3 \
  --climb-metrics logs/c13_c15_metrics.json
```

指标 JSON 是 simulation-only 诊断，记录关节跟踪、运动学足端目标误差和关节
限位余量；不证明接触、载荷或稳定性。`--climb-from`、`--climb-to` 和
`--climb-metrics` 只能与 Isaac 的 `--climb-start` 或 `--climb-scene` 一起使用，
不进入 ROS 或实机路径。

## 包内说明

- [启动命令与参数](src/grasp_hexapod_control/launch/README.md)
- [舵机协议、ID 和方向](src/grasp_hexapod_servo/README.md)
