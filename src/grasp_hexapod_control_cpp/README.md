# grasp_hexapod_control_cpp

Grasp Hexapod 高层控制器的 C++ 移植版。逐函数移植自
`grasp_hexapod_control/scripts/run_real.py` 及其全部计算依赖
（`control.py` / `kinematics.py` / `climb_mode.py` / `dock_mode.py` /
`approach_mode.py` / `utils/`），用于消除 Python 解释器、GIL 与
OpenBLAS 线程池开销，稳定 30 Hz 控制循环并释放机载算力。

## 与 Python 版的关系

| 层 | Python 包 | C++ 包（本包） | 切换方式 |
|---|---|---|---|
| 高层控制器 | `grasp_hexapod_control` / `run_real.py` | `grasp_hexapod_control_cpp` / `run_real_cpp` | `control_backend:=python\|cpp` |
| 舵机驱动 | `grasp_hexapod_servo` | `grasp_hexapod_servo_cpp` | `servo_backend:=python\|cpp` |

- ROS 话题、服务、参数名与行为契约完全一致，节点同名（同一时刻只能运行一个）。
- 仿真链（`run_sim.py` 等）保持 Python 不动；本包只覆盖实机链。
- Python 版完整保留，可随时回滚。

### 启动方式

```bash
# 全 C++（高层控制 + 舵机后端）
roslaunch grasp_hexapod_control_cpp run_real_full_cpp.launch

# 只切高层控制器，舵机保持 Python
roslaunch grasp_hexapod_control run_real.launch control_backend:=cpp

# 全 Python（默认，行为与重构前完全一致）
roslaunch grasp_hexapod_control run_real.launch
```

配置文件（`climb_compact.json` / `workspace_bounds.csv` / `dock_system.yaml`）
不复制，C++ 通过 `ros::package::getPath` 读取 `grasp_hexapod_control/config`；
可用私有参数 `~control_config_dir` 覆盖。

## 结构

```
grasp_hexapod_control_core   纯数值库（零 ROS 依赖；Eigen + nlohmann/json + yaml-cpp）
  types/math_utils           几何常量、变换、多边形、线段距离
  kinematics                 FK / 解析 Jacobian / DLS 阻尼逆 / hip↔base
  config_io                  workspace_bounds.csv、dock_system.yaml
  climb_compact              compact JSON 解析/完整校验/右侧镜像/STL 包围盒
  climb_mode                 27 阶段回放（含硬件反馈门控与耗时归因）
  approach_mode              三角步态 + 自动接近
  dock_mode                  12 态对接状态机（感知经 PerceptionInterface 注入）
  control                    GraspController + MissionStateMachine + DLS 热路径

grasp_hexapod_control_ros    ROS 壳层
  real_control_node          run_real.py 全量移植（BT 仲裁/夹爪线程/双板门控）
  dock_perception_ros        tf2 感知 + 一致性/融合/置信度

run_real_cpp                 入口（ros::init + AsyncSpinner(4) + 120Hz 轮询）
```

## 数值等价性（金标回归，上实机硬门槛）

`test/golden_control_trajectories.json` 由 `scripts/tools/export_golden_control.py`
用**原版 Python 控制器**录制（理想伺服确定性回放）：

- `reset_then_walk`：825 帧（扰动 → B 回站 → 前进 → 反向+升降 → 停止）
- `climb_left`：1835 帧完整 27 阶段到 DONE（硬件门控、foot_gate=0.05）
- 运动学快照 5 组 + 右侧镜像摘要 + dock_system 解析 + 接近几何

C++ gtest 逐帧对比，容差 1e-6 rad/m（FK 要求逐位相等，Jacobian/DLS ≤1e-12）：

```bash
# 重录金标（Python 侧变更后必须重跑）
python3 src/grasp_hexapod_control/scripts/tools/export_golden_control.py

# 构建并运行
catkin_make --pkg grasp_hexapod_control_cpp
catkin_make run_tests_grasp_hexapod_control_cpp
```

当前状态：7/7 全部通过。移植中发现并修复的典型差异：`wrap_angle` 的
Python 取模符号语义、`rows @ T.T` 的齐次变换列向量等价形式
（`T @ h` 而非 `T.T @ h`）、`np.interp/np.argmax` 的边界/平局语义。

## 性能实测（Jetson，-O2，同序列同机器对比）

| 指标 | Python | C++ | 提速 |
|---|---|---|---|
| 单帧控制全链（update：步态+工作空间+DLS+守卫） | 915.5 µs | **3.9 µs** | **~235×** |
| 整机连杆胶囊碰撞检查（Python 实机默认因此关闭） | 3689 µs | **2.1 µs** | **~1720×** |
| 节点 30Hz 实跑 tick（含 6 话题发布） | — | 0.05~1.3 ms | 见 tick 日志 |

碰撞检查在 C++ 下可低成本开启（`enable_link_collision_check:=true`），
安全性提升。基准脚本见 `bench/`（`bench_golden_replay.*`、
`bench_link_collision.cpp`、`e2e_smoke.sh` 端到端冒烟）。

## 已知行为差异

- dock 关节/足端目标的 shape 异常路径在 C++ 强类型下不可达（Python 有
  try/except 分支）。
- 错误日志中的数字格式化（`%.9g` vs Python format）可能有微小排版差异，
  诊断摘要字符串的关键字段格式已对齐。
- `switch_mode` 服务响应消息与 Python 一致；`spin_search`/`tag_nav` 仍拒绝。

## 移植中已知限制

- 仅移植实机链路使用的接口；run_sim/离线工具专用函数（COM、奇异值等）
  保留在 Python。
- 金标目前覆盖 walk + climb；dock 状态机由 12 态单元行为对照与 E2E 冒烟
  覆盖，尚未有逐帧金标（可后续扩展导出脚本注入 DockRobotState 序列）。
