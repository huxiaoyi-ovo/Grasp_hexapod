# grasp_hexapod_bt_control — 行为树服务化控制栈

面向行为树（`grasp_hexapod_bt`，现有 Python/py_trees 版，**零改动**）的全链路
C++ 控制栈：控制节点完全不依赖手柄话题，由服务切换状态机；手柄功能整体移到
独立的遥控器节点。与 `src/docs/README.md` 描述的模式契约一致。

> **与旧链路互斥**：`bt_control_node` 与 `grasp_hexapod_control` 的
> `run_real.py`/`run_real_cpp` 提供同一个 `/grasp_hexapod/switch_mode` 服务，
> 同一时刻只能运行一个。旧链路（含 /joy 手动模式）原样保留，按需二选一。

## 架构

```
手柄 → joy_node(/joy) → remote_control_node ─┬─ /cmd_vel（walk 速度，平面三自由度）
                                             ├─ 服务 /grasp_hexapod/switch_mode（抢占式）
                                             └─ 服务 /grasp_hexapod/gripper_act
现有 Python 行为树 run_real_bt.py ── /grasp_hexapod/switch_mode、/hold_motion ──┐
                                                                              ▼
bt_control_node（无 /joy；服务切换状态机；walk 由 /cmd_vel 驱动）
                              ▼ /{leg}_des
        servo_two_boards_cpp（左板含夹爪 ID 99）→ /{leg}_pos → bt_control_node
```

## 节点

### bt_control_node（`node/bt_control_main.cpp`）

原 `real_control_node`（现 `grasp_hexapod_control/src/`）的服务化改造版，
数值核心复用 `grasp_hexapod_control_core` 纯计算库。差异：

- **不订阅** `/joy`、`/grasp_hexapod/remote_cmd`；手柄映射参数全部删除。
- **`/grasp_hexapod/switch_mode` 带抢占语义**：进行中的不同模式请求被立即终结
  （`success=false, "preempted by X"`），新模式先平滑回正到 HOLD 再进入；
  同模式重复调用按等待者合并。单调用方（行为树）感知不到抢占。
- **`/cmd_vel` 驱动 walk**：平面三自由度 `linear.x/linear.y/angular.z`，
  **无 z 轴线速度**（`linear.z` 恒忽略）；按 `max_linear_speed` 向量模限幅、
  `max_yaw_rate` 航向限幅。`~max_cmd_vel_age`（默认 0.2s）内无新速度指令 →
  零命令停步回 HOLD，walk 以 `"cmd_vel lost"` 结束。
- **WAIT_B 安全门**：上电初始态不发布目标（舵机卸力）；只接受 `home`，
  其余模式拒绝 `"call home first"`（原"先按 B"安全语义的服务化等价）。
- **dock 末端夹持必然执行**：dock 只能经服务进入（原手柄 Y 直连路径已删），
  `DockMode` 到 `success` + HOLD 后调用 `gripper_act clamp`，夹爪结果即
  BT 服务的最终结果；失败 `ROS_ERROR` 并透传。`release` → open、
  `home` 完成 → open 同链路。
- 状态机仍为 `WAIT_B/RESETTING/HOLD/RUNNING`；`home/walk/approach/climb/
  dock/release` 可执行，`spin_search/tag_nav` 预留（返回 executor not
  implemented）。
- 保留：`/grasp_hexapod/hold_motion` 租约、六腿反馈双板门控、导航/IMU/
  锁紧确认订阅、反馈丢失 → HOLD、攀爬监控与诊断。

### remote_control_node（`node/remote_control_main.cpp`）

手柄 → 服务的映射层（参数沿用原命名，`~button_*`/`~axis_*`/`~axis_*_scale`/
`~max_linear_speed`/`~max_yaw_rate`/`~rate_hz`）：

| 输入 | 动作 |
|---|---|
| 左摇杆（axis_right/axis_forward） | `/cmd_vel` 平面速度（30Hz；手柄失效发零速） |
| 右摇杆横向（axis_yaw） | `/cmd_vel` 角速度 |
| A(0) / B(1) / X(2) / Y(3) 按下沿 | 后台线程调 `switch_mode(walk/home/climb/dock)` |
| 方向键轴 6：左(-1) / 右(+1) | `gripper_act(clamp/open)`，回中后可重复触发 |

## 启动

```bash
# 全链路（含现有 Python 行为树；remote_test 为空则跑主链，默认 walk 测试链）
roslaunch grasp_hexapod_bt_control full_chain.launch remote_test:=walk

# 仅控制栈 + 遥控器（无行为树，联调/排查用）
roslaunch grasp_hexapod_bt_control control_only.launch
```

服务切换状态机与原按键语义对照：

| 原按键 | 服务调用 | 行为 |
|---|---|---|
| B | `switch_mode("home")` | 无条件抢占回正，完成后张开夹爪 |
| A | `switch_mode("walk")` | 步态行走，速度来自 `/cmd_vel` |
| X | `switch_mode("climb")` | 需 `~enable_real_climb`，且已在 HOLD |
| Y | `switch_mode("dock")` | 视觉对接 → 六腿同高 → **夹爪闭合** |

## 测试

```bash
# gtest（反馈门控 + cmd_vel 限幅，无 ROS master 依赖）
catkin_make run_tests_grasp_hexapod_bt_control_gtest_test_bt_control_routing

# bt_control_node e2e：WAIT_B 门 / home+夹爪 / walk+cmd_vel / 抢占 / release
python3 src/grasp_hexapod_bt_control/test/fake_hardware_e2e.py

# 遥控器 e2e：cmd_vel 映射 / 失效归零 / 按键服务 / 方向键夹爪
python3 src/grasp_hexapod_bt_control/test/remote_control_e2e.py
```

（e2e 需要 roscore；伪舵机板 + 伪夹爪服务由脚本自带，不需真实硬件。）

## 已知边界

- `dock` 全流程依赖真实视觉（AprilTag + `dock_system.yaml` 实标定）与可选
  锁紧确认话题 `/grasp_hexapod/dock/lock_confirmed`；e2e 只验证到夹爪服务
  链路（release 与 dock clamp 走同一 `actuateGripper` 实现）。
- `climb` 需 `~enable_real_climb:=true` 且 IMU/RTK 数据可用（仅影响可选监控）。
- `approach` 需 `~control_source:=navigation`。
