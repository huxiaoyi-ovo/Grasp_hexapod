# Grasp Hexapod

这是一个基于 ROS Noetic 的六足机器人工作空间。项目使用手写控制器，支持 Isaac Gym 仿真和两块 LX-15D 舵机板实机运行。

## 目录

- `src/grasp_hexapod_control/`：步态、运动学、安全控制和仿真/实机入口。
- `src/grasp_hexapod_description/`：URDF、网格、RViz 和 Gazebo 配置。
- `src/grasp_hexapod_servo/`：LX-15D 三板驱动和串口协议。
- `src/reference/`：参考代码，不参与运行。

仿真和实机共用 `GraspController`：前者在 Isaac 控制帧内运行，后者通过 ROS 读取反馈并发布关节目标。

控制包按职责命名：`run_*.py` 是执行入口，`control.py`、`*_mode.py` 和
`kinematics.py` 是运行时核心，`scripts/utils/` 只放共享库；
`scripts/tools/` 保留 `analyze_workspace.py` 和离线候选重建工具；旧规划、trace 和长 validator 在
`src/reference/climb_history/`，不进入实时控制链也不再维护。

## 开发协作

- `test` 是集成分支；从 `test` 创建功能分支，合并目标也是 `test`，不要直接在 `main` 开发。
- 在仓库根目录构建和运行检查。
- URDF 和 CAD 导出文件只做必要的小修改；如果以后重新导出，需同步保留这些手工修改。

### 协作者提交准则

- 最小改动：只实现当前需求，不顺带重构、格式化或修改无关代码。
- 控制文件：优先复用和合并现有文件，避免拆出零散模块；默认修改不超过 3 个文件，确需新增时说明理由。
- 边界清晰：保持现有接口、调用链和配置兼容；若必须改变公共接口或扩大范围，先沟通确认。
- 风格一致：沿用附近代码的命名、结构、注释和错误处理方式，避免过度抽象和非必要依赖。
- 可直接接入：提交前完成相关语法及针对性测试，并说明修改文件、接口影响、验证结果和未覆盖风险；不要提交日志、临时文件或无关生成物。

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

两块驱动板的默认映射如下。`left`、`right` 是启动参数中的板标识。

| 板 | 腿 | 舵机 ID | 默认串口 |
|---|---|---|---|
| `left` | `lf`、`lm`、`lb` | 1～9 | `/dev/ttyTHS0` |
| `right` | `rf`、`rm`、`rb` | 10～18 | `/dev/ttyACM0` |

串口名称变化时可在启动时覆盖：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  left_port:=/dev/ttyTHS0 \
  right_port:=/dev/ttyACM0
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

Isaac Gym 仍是攀爬动作和物理行为的预览权威。`run_real.launch`也提供默认启用的
C1--C35 实机诊断回放入口：完成 B 回站后按 X 启动，A 暂停或恢复，B 可随时中止。
它只按 18 个关节位置反馈的实际关节跟踪与 FK 足端目标误差连续稳定门限推进；这些
关节反馈门限不证明接触、承载或稳定性，也不构成实机攀爬安全授权。

```bash
conda run --no-capture-output -n grasp_hexapod python3 \
  src/grasp_hexapod_control/scripts/run_sim.py \
  --climb-start --climb-speed 1 --climb-joint-speed 1.2
```

可只回放一个闭区间：`C1` 至 `C35` 是 compact 数组顺序的固定用户别名，也可
使用运行时阶段名。中途入口会把 Isaac root、18 个关节和足端锚点同步到该阶段
的规划起点；它仍只是仿真预览。

```bash
conda run --no-capture-output -n grasp_hexapod python3 \
  src/grasp_hexapod_control/scripts/run_sim.py \
  --climb-start --climb-from C13 --climb-to C15 \
  --climb-speed 1 --climb-joint-speed 1.2 \
  --climb-metrics logs/c13_c15_metrics.json
```

指标 JSON 是 simulation-only 诊断，记录关节跟踪、运动学足端目标误差和关节
限位余量；不证明接触、载荷或稳定性。`--climb-from`、`--climb-to` 和
`--climb-metrics` 只能与 Isaac 的 `--climb-start` 或 `--climb-scene` 一起使用，
不进入 ROS 或实机路径。

离线候选先由 `rebuild_climb_preview.py snapshot` 生成模型/profile，再由 `build`
仅重定向显式编辑的阶段终点；它不会自动选落脚点、改变阶段顺序、限位或时序。

## 包内说明

- [启动命令与参数](src/grasp_hexapod_control/launch/README.md)
- [舵机协议、ID 和方向](src/grasp_hexapod_servo/README.md)
