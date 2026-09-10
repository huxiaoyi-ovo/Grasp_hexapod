# 接口实现清单（README）

> 实机接入开发总纲：列出待实现接口（服务 / 话题数据源 / 桥接方法 / 控制栈模式）的
> 语义、返回协议与 TODO 清单。字段级契约见 [`BT_INTERFACES.md`](BT_INTERFACES.md)；
> 本文件只讲"要实现什么、什么含义、还差什么"。

## 1. 概览与文件索引

**架构**：所有运动/任务统一为"模式"（home/walk/climb/dock/spin_search/release/approach/tag_nav）；
行为树一模式一节点 `RunMode(mode)` → `~/switch_mode`（阻塞执行完整流程，**响应即最终结果**）；
夹爪 open/clamp 由 `release`/`dock` 模式内部调 `~/gripper_act`（不在树中；**系统唯一夹爪控制入口**，
话题盲控与调试服务路径已删除）；编码器/传感器状态走话题；主链不含遥控（独立测试链）。

| 文件 | 角色 |
|---|---|
| `src/grasp_hexapod_bt/scripts/hexapod_bt.py` | 主链 + 遥控测试树（py_trees，不 import ROS，桥接靠注入） |
| `src/grasp_hexapod_bt/scripts/bt_mock_world.py` | 联调模拟节点（模式执行器/夹爪/传感器假实现 + `_remote_test`） |
| `src/grasp_hexapod_bt/scripts/run_real_bt.py` | **实机运行入口（纯真实）**：真实桥接订阅/调用标准名话题与服务 |
| `src/grasp_hexapod_bt/scripts/sim_feedback.py` | **唯一仿真来源**（独立进程）：按 `config/real_bt.yaml` 逐接口模拟，补缺真实节点 |
| `src/reference/encoder_driver/scripts/encoder_status_node.py` | 编码器状态节点（`/grasp_hexapod/encoder_state`） |
| `src/grasp_hexapod_bt/scripts/sensor_health_monitor.py` | 传感器健康层（`/grasp_hexapod/sensor_health`） |
| `src/grasp_hexapod_bt/scripts/remote_control.py` | 遥控语义化（`/joy` → `/grasp_hexapod/remote_cmd`，仅测试链） |
| `src/reference/lora/scripts/lora_node.py` | LoRa 串口节点（`/dev/lora`@115200 ↔ `/lora/command`、`/lora/status`） |
| `src/grasp_hexapod_bt/config/real_bt.yaml` | 逐接口 `simulate` 开关 + 模拟时间线/故障注入（LoRa 为话题级仿真，串口不仿真） |
| `src/docs/BT_INTERFACES.md` | 话题/服务字段级契约（唯一契约来源） |
| `src/docs/BT_INTERFACE_GUIDE.md` | 接口速览（对接同事用）：是什么、怎么用、联调命令 |
| `src/grasp_hexapod_bt/behavior_trees/hexapod_mission.xml` | Groot2 可视化稿（与 py_trees 人工同步） |
| `src/grasp_hexapod_bt_control/` | **面向行为树的全链路 C++ 控制栈（新，可选替代旧控制栈）**：`bt_control_node`（服务切换状态机 + 抢占 + walk 由 `/cmd_vel` 驱动 + dock 成功必夹爪闭合）+ `remote_control_node`（手柄 → 服务调用与 `/cmd_vel`）；详见其 README |

## 2. 需要实现的接口

### A. 服务

#### A1. `~/switch_mode`（`SwitchMode.srv`）
- 请求 `string target_mode`（8 模式）；响应 `bool success` + `string message`（模式**最终**结果）。
- 语义：**阻塞执行该模式完整流程**（如 dock = tag 导引到充电桩 ㉙ + 六腿抬起 + 夹爪 clamp +
  结束确认；release = 夹爪 open + 松开确认），完成或失败才应答。
- 幂等：已处于目标模式时返回当前状态，不重复触发（反应式父级逐 tick 复检）。
- 桥接：`switch_mode(mode) → (state, message)`，`state ∈ RUNNING/SUCCESS/FAILED`（RUNNING = 服务尚未返回）。
- 现状：mock 已有；**真实 mode_server 未实现**（TODO-1/2/3/7）。

#### A2. `~/gripper_act`（`GripperAct.srv`）
- 请求 `string action`（`open` 松开 / `clamp` 夹紧）；响应 `bool success`（**到位**）+
  `string message`（受限 / 离线 / 超时 / 忙）。
- 语义：夹爪执行器服务，由 release/dock 模式内部调用，不在树中体现；到位结果折入模式最终结果
  （dock clamp 失败 → dock `FAILED`）。
- 实机：`grasp_hexapod_servo_cpp` 的 `GripperManager` 原生提供；
  手柄方向键与调试 rosservice 共用此唯一入口（并发 busy 拒绝；夹紧失败受限后须先 open 复位）。

### B. 数据源（话题，行为树订阅）

| # | 话题 | 类型 | 语义 | 现状 |
|---|---|---|---|---|
| B1 | `/grasp_hexapod/encoder_state` | `EncoderState` | 编码器三态：landed→True；not_landed→False；normal=false→故障 | `encoder_status_node.py` 已实现（串口直读） |
| B2 | `/grasp_hexapod/sensor_health` | `SensorHealthArray` | IMU/GPS/RTK/舵机/双目/单目 六路频率+新鲜度 | `sensor_health_monitor.py` 已实现 |
| B3 | `/fix` | `sensor_msgs/NavSatFix` | RTK 定位与协方差（对角最大 ≤ 0.04 m²） | 依赖真实 GPS 驱动（TODO-10 实测） |
| B4 | `/lora/command` | `std_msgs/String` | 地面指令净帧：`CMD,HEX,RELEASE/RECOVER/DEPLOY/HOIST_DONE,…` | 真实 `lora_node.py` 已实现 |
| B5 | `/lora/status`（发布） | `std_msgs/String` | 上报净帧：`STA,HEX,<status>,<x>,<y>` | 真实 `lora_node.py` 已实现 |
| B6 | `/grasp_hexapod/remote_cmd` | `RemoteCmd` | 遥控语义命令（仅测试链；主链不用） | `remote_control.py` 已实现 |

### C. BridgeContext 方法（树 ↔ 世界桥接，需真实实现方提供）

统一动作返回协议：**True=完成 / False=进行中 / None=失败**（None 走失败回退）；
C1 除外（三态元组）。

| # | 方法 | 语义 / 返回协议 | 对应节点 | 现状 |
|---|---|---|---|---|
| C1 | `switch_mode(target_mode)` | 服务 A1 桥接，返回 `(state,message)`，state∈RUNNING/SUCCESS/FAILED；幂等 | `RunMode(mode)` | ✅ 真实桥接（run_real_bt.py）；❌ 真实服务端（TODO-1/2/3/7） |
| C2 | `sensor_health()` | 读 B2+B1，返回 `{name:{online,fresh,freq_hz,age_s,reason}}` | `IsSensorDataOk`/`WaitSensorsReady` | ✅ 真实（run_real_bt.py） |
| C3 | `is_landing_confirmed()` | B1：landed→True；not_landed→False；normal=false→None | `IsLandingConfirmed` | ✅ 真实（run_real_bt.py） |
| C4 | `rtk_covariance_ok()` | `/fix` 协方差对角最大 ≤ 阈值（默认 0.04 m²） | `WaitRtkPrecise` | ✅ 真实（run_real_bt.py）；⚠️ 数据源待实测（TODO-10） |
| C5 | `hold_motion(reason)` | 零速保持（RTK 超限停走、等待恢复） | `WaitRtkPrecise` | ✅ 真实（run_real_bt.py 节流告警；停走由模式执行端保证） |
| C6 | `receive_task_command()` | 解析 `/lora/command`：RELEASE→"release"；RECOVER→"recover"；无→None；其他→非法 | `WaitTaskCommand` | ✅ 真实（run_real_bt.py + lora_node.py） |
| C7 | `wait_deployment(dt)` | ⑨ 等 `CMD,HEX,DEPLOY,…`；True=已下放 | `WaitDeployment` | ✅ 真实（run_real_bt.py + lora_node.py） |
| C8 | `wait_winch_hoisted(dt)` | ⑫/㉜ 等 `CMD,HEX,HOIST_DONE,…`；True=已回收 | `WaitWinchHoisted` | ✅ 真实（run_real_bt.py + lora_node.py） |
| C9 | `report_status(status, dt)` | 发布 `/lora/status`：`STA,HEX,<status>,<x>,<y>` | `ReportStatus` | ✅ 真实（run_real_bt.py + lora_node.py） |
| C10 | `read_remote_cmd()` | 读 `/grasp_hexapod/remote_cmd` 返回 dict（仅遥控测试链） | `CheckRemoteCmd` | ✅ 真实（run_real_bt.py + remote_control.py） |

### D. 控制栈模式枚举（需扩展）

行为树共 8 模式；`control.py` 的 `GraspController` 现仅 APPROACH/CLIMB/DOCK，
`MissionStateMachine` 亦不全——需扩展并建立映射（TODO-2/3）。

## 3. 待完成事项（TODO）

- [ ] **TODO-1 真实 mode_server**：提供 `~/switch_mode`，把 8 个模式映射到 `GraspController`
  （`set_mode` / `reset_to_stand` / `mission.start-update`），输出最终 `success/message`。
- [ ] **TODO-2 控制栈模式枚举扩展**：增加 HOME/WALK/SPIN_SEARCH/RELEASE/APPROACH/TAG_NAV；
  `home` = B 复位 + 平滑回站姿 + 使能。
- [ ] **TODO-3 dock/release 内部流程**：dock = tag 导引 + 六腿抬起 + 夹爪 clamp + 结束确认；
  release = 夹爪 open + 松开确认（均不在树中单列）。
- [x] **TODO-5 LoRa 节点**：串口 `CMD,…` 帧（含校验）→ `/lora/command` 净帧（C6/C7/C8 消费）；
  `/lora/status` 净帧（C9 产生）→ 带校验 `STA,…` 帧写串口。
- [x] **TODO-6 真实 BridgeContext（run_real_bt.py）**：C1~C10 接到标准名话题/服务；仿真不混入树，
  缺项由 `sim_feedback.py`（按 `real_bt.yaml`）补齐。实机仍缺 mode_server（TODO-1/2/3/7）。
- [ ] **TODO-7 spin_search 结果源**：感知节点写入"发现小蓝 + `xiaolan_pose` 有效"作为模式最终结果。
- [ ] **TODO-8 双侧同步**：改树须同步 `hexapod_bt.py` 与 `hexapod_mission.xml`；
  回归用 `python3 hexapod_bt.py --selftest` + `bt_mock_world.py`。
- [ ] **TODO-9 实机攀爬授权**：`climb_mode.py` 现为 simulation-only；接入 mode_server 前需补
  C1→C35 步态的接触/承载/稳定性判定与安全边界，否则实机 ㉘ 攀爬节点不可放行。
- [ ] **TODO-10 GPS/RTK 实测**：`/fix`（`reference/gps_ntrip_py`）与 `rtk_covariance_ok` 联调验收：
  协方差对角最大 ≤ `rtk_max_covariance`（0.04 m²）、超限停走（C5）、无 `/fix` 默认放行行为。

## 4. 接口现状总表（验收对照）

| 接口 | 语义要点 | 现状 |
|---|---|---|
| `~/switch_mode` | 执行模式并返回最终结果 | ✅ mock / ❌ 真实（TODO-1/2/3/7） |
| `~/gripper_act` | open/clamp 到位 | ✅ mock / ✅ 真实（`grasp_hexapod_servo_cpp` 原生提供，唯一入口） |
| `/grasp_hexapod/encoder_state` | 编码器三态持续发布 | ✅ 已实现 |
| `/grasp_hexapod/sensor_health` | 六路健康 | ✅ 已实现 |
| `/fix` | RTK 协方差 | ⚠️ 依赖真实 GPS 驱动（TODO-10） |
| `/lora/command`、`/lora/status` | 任务指令 / STA 上报 | ✅ mock / ✅ 真实 `lora_node.py` |
| `/grasp_hexapod/remote_cmd` | 遥控（仅测试链） | ✅ 已实现 |
| BridgeContext C1~C10 | 树 ↔ 世界桥接 | ✅ mock / ✅ 真实（run_real_bt.py，TODO-6） |
| 控制栈模式 8 种 | home/walk/climb/dock/spin_search/release/approach/tag_nav | ⚠️ 需扩展（TODO-2） |
| 实机攀爬 climb | C1→C35 实机步态执行 | ⚠️ simulation-only，不授权实机（TODO-9） |

## 5. 变更记录

- **2026-09-11 行为树服务化控制栈（grasp_hexapod_bt_control，可选新链路）**
  - 新增 `src/grasp_hexapod_bt_control/`：`bt_control_node`（不订阅 `/joy` 与
    `/remote_cmd`，由 `~/switch_mode` 服务切换状态机；`switch_mode` 增加**抢占
    语义**——进行中的不同模式请求被终结并回正后进入新模式；WAIT_B 安全门只接受
    `home`；walk 由 `/cmd_vel` 平面三自由度驱动，`~max_cmd_vel_age` 超时安全停步）
    与 `remote_control_node`（手柄按键 → 服务调用，摇杆 → `/cmd_vel`，方向键 →
    `gripper_act`）。
  - **dock 末端夹持链路修正**：旧链路手柄 Y 键直连进入 dock 时不产生 BT 请求，
    对接成功后不会调用夹爪（实测"没有夹爪闭合"的根因）；新链路 dock 只能经
    服务进入，`success` 后必然 `gripper_act clamp` 且结果透传给调用方。
  - C++ 包合并：`grasp_hexapod_control_cpp` 并入 `grasp_hexapod_control`
    （src/include/node/vendor/bench/test），`grasp_hexapod_servo_cpp` 并入
    `grasp_hexapod_servo`（launch 改名 `servo_two_boards_cpp.launch`）；
    旧链路 Python 代码与 ROS 接口零改动。
  - 新旧控制栈互斥运行（同一 `/grasp_hexapod/switch_mode` 服务，同一时刻
    只能一个提供者）。

- **2026-09-09 夹爪接口统一 + 舵机电压开关**
  - 手柄方向键夹爪控制由 `/gripper_des` 话题盲控改为调用 `/grasp_hexapod/gripper_act`
    （十字键左 = clamp 闭合，右 = open 张开；后台线程执行，不阻塞 30Hz 控制循环；
    行为树活动期间不响应）。
  - 删除 `/gripper_des` 话题路径（含 `/gripper_pos` 反馈发布、`GripperSync` 同步机制）与
    `/gripper_command` 调试服务（`GripperCommand.srv` 一并删除）——夹爪只剩 `gripper_act`
    一个服务入口，实机/仿真同名对接。
  - 舵机节点新增 `~enable_voltage_read` 电压读取开关（**默认关闭**），与时序诊断
    （`~enable_diagnostics`）分离；launch 经 `voltage_read:=true` 开启。
