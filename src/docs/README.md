# 接口实现清单（README）

> 本文件列出**需要实现的接口**（服务 / 话题数据源 / 桥接方法 / 控制栈模式），
> 逐一给出**详细语义**与**返回协议**，并附**待完成事项（TODO）**清单，
> 作为实机接入的开发总纲。
> 字段级契约见 [`BT_INTERFACES.md`](BT_INTERFACES.md)；本文件只讲"要实现什么、什么含义、还差什么"。

## 1. 概览与文件索引

**架构一句话**：所有运动/任务动作统一为"模式"（home/walk/climb/dock/spin_search/release/approach/tag_nav）；
行为树一模式一节点 `RunMode(mode)` → `~/switch_mode`（服务自动执行该模式完整连续性流程，**响应即最终结果**）；
夹爪 open/clamp 由 `release`/`dock` 模式内部调 `~/gripper_act`（不在树中）；
编码器/传感器状态走 topic；主链**不含遥控**（遥控为独立测试链）。

| 文件 | 角色 |
|---|---|
| `src/grasp_hexapod_bt/scripts/hexapod_bt.py` | 主链 + 遥控测试树（py_trees，不 import ROS，桥接靠注入） |
| `src/grasp_hexapod_bt/scripts/bt_mock_world.py` | 联调模拟节点（含模式执行器/夹爪/传感器假实现 + `_remote_test`） |
| `src/grasp_hexapod_bt/scripts/run_real_bt.py` | **实机运行入口（纯真实，无仿真）**：真实桥接 RosBridgeContext，订阅/调用标准名话题与服务 |
| `src/grasp_hexapod_bt/scripts/sim_feedback.py` | **唯一的仿真来源**（独立进程）：按 `config/real_bt.yaml` 逐接口模拟发布/应答，补缺真实节点 |
| `src/reference/encoder_driver/scripts/encoder_status_node.py` | 编码器状态节点（持续发布 `/grasp_hexapod/encoder_state`） |
| `src/grasp_hexapod_bt/scripts/sensor_health_monitor.py` | 传感器健康判断层（发布 `/grasp_hexapod/sensor_health`） |
| `src/grasp_hexapod_bt/scripts/remote_control.py` | 遥控语义化（`/joy` → `/grasp_hexapod/remote_cmd`，仅测试链） |
| `src/reference/lora/scripts/lora_node.py` | LoRa 串口节点（`/dev/lora`@115200 ↔ `/lora/command`/`/lora/status`） |
| `src/grasp_hexapod_bt/config/real_bt.yaml` | 逐接口 `simulate` 开关 + 模拟时间线/故障注入（供 sim_feedback.py 读取；LoRa 为**话题级仿真**，串口读取不仿真） |
| `src/docs/BT_INTERFACES.md` | 话题/服务字段级契约（唯一契约来源） |
| `src/docs/BT_INTERFACE_GUIDE.md` | **接口速览（给对接同事）**：是什么、怎么用、分工与联调命令 |
| `src/grasp_hexapod_bt/behavior_trees/hexapod_mission.xml` | Groot2 可视化稿（与 py_trees 人工同步） |

## 2. 需要实现的接口

### A. 服务

#### A1. `~/switch_mode`（`SwitchMode.srv`）
- 请求：`string target_mode`（home / walk / climb / dock / spin_search / release / approach / tag_nav）
- 响应：`bool success`（**最终成功**）、`string message`（**最终问题/结果说明**）
- 语义：**阻塞式执行目标模式**——接收请求后自动执行该模式内部完整连续性流程
  （例如 dock = tag 导引到充电桩 ㉙ + 六腿抬起 + 夹爪 clamp + 结束确认；release = 夹爪 open + 松开确认），
  执行完成或失败才应答，响应即该模式的最终结果。
- 幂等性：已处于目标模式时返回其当前状态、不重复触发（行为树反应式父级会逐 tick 复检）。
- 桥接包装：`switch_mode(mode) → (state, message)`，`state ∈ RUNNING/SUCCESS/FAILED`
  （RUNNING = 阻塞式服务尚未返回 / 模式执行中）。
- 现状：`bt_mock_world` 已模拟；**真实 mode_server 未实现**（见 TODO-1）。

#### A2. `~/gripper_act`（`GripperAct.srv`）
- 请求：`string action`（`open` 松开 / `clamp` 夹紧）
- 响应：`bool success`（夹紧/松开**到位**）、`string message`（受限 / 离线 / 超时 等）
- 语义：夹爪执行器服务。**由 release / dock 模式内部调用，不在行为树中体现**；
  到位结果折入该模式的最终结果（dock clamp 失败 → dock 最终 `FAILED` + 问题）。


### B. 数据源（话题，行为树订阅）

| # | 话题 | 类型 | 语义 | 现状 |
|---|---|---|---|---|
| B1 | `/grasp_hexapod/encoder_state` | `EncoderState` | 编码器当前状态持续反馈：`normal/landed/not_landed/angle/reason`（landed→True；not_landed→False；normal=false→故障） | `encoder_status_node.py` 已实现（串口直读） |
| B2 | `/grasp_hexapod/sensor_health` | `SensorHealthArray` | IMU/GPS/RTK导航/舵机/双目/单目 六路频率+新鲜度 | `sensor_health_monitor.py` 已实现 |
| B3 | `/fix` | `sensor_msgs/NavSatFix` | RTK 定位与协方差（`rtk_covariance_ok` 用：对角最大 ≤ 0.04 m²） | 依赖真实 GPS 驱动（reference 已有） |
| B4 | `/lora/command` | `std_msgs/String` | 地面指令净帧：`CMD,HEX,RELEASE/RECOVER/DEPLOY/HOIST_DONE,…` | mock 已给；**真实 `lora_node.py` 已实现** |
| B5 | `/lora/status`（发布） | `std_msgs/String` | STA 上报净帧：`STA,HEX,<status>,<x>,<y>` | mock 已给；**真实 `lora_node.py` 已实现** |
| B6 | `/grasp_hexapod/remote_cmd` | `RemoteCmd` | 遥控语义命令（仅测试链；主链不用） | `remote_control.py` 已实现 |

### C. BridgeContext 方法（行为树 ↔ 世界的桥接，需真实实现方提供）

统一动作返回协议：**True=完成 / False=进行中 / None=失败**（None 走失败回退）。
`switch_mode` 除外（见 C1，返回三态元组）。

| # | 方法 | 语义 / 返回协议 | 对应节点 | 现状 |
|---|---|---|---|---|
| C1 | `switch_mode(target_mode)` | 执行模式（服务 A1 的桥接），返回 `(state,message)`，state∈RUNNING/SUCCESS/FAILED；幂等 | `RunMode(mode)` | ✅ 真实桥接（run_real_bt.py，后台线程三态）；❌ 真实服务端（TODO-1/2/3/7） |
| C2 | `sensor_health()` | 读 `/grasp_hexapod/sensor_health` + `/grasp_hexapod/encoder_state`，返回 `{name:{online,fresh,freq_hz,age_s,reason}}` | `IsSensorDataOk`/`WaitSensorsReady` | ✅ 真实（run_real_bt.py） |
| C3 | `is_landing_confirmed()` | 订阅 encoder_state：landed→True；not_landed→False；normal=false→None | `IsLandingConfirmed` | ✅ 真实（run_real_bt.py） |
| C4 | `rtk_covariance_ok()` | `/fix` 协方差对角最大 ≤ 阈值（默认 0.04 m²） | `WaitRtkPrecise` | ✅ 真实（run_real_bt.py）；⚠️ `/fix` 数据源待实测（TODO-10） |
| C5 | `hold_motion(reason)` | 零速保持（RTK 协方差超限停走、等待恢复） | `WaitRtkPrecise` | ✅ 真实（run_real_bt.py 节流告警，实际停走由模式执行端保证） |
| C6 | `receive_task_command()` | 解析 `/lora/command`：`…RELEASE…`→"release"；`…RECOVER…`→"recover"；无→None；其他→非法 | `WaitTaskCommand` | ✅ 真实（run_real_bt.py + lora_node.py） |
| C7 | `wait_deployment(dt)` | ⑨ 等待 `CMD,HEX,DEPLOY,…`；True=已下放 | `WaitDeployment` | ✅ 真实（run_real_bt.py + lora_node.py） |
| C8 | `wait_winch_hoisted(dt)` | ⑫/㉜ 等待 `CMD,HEX,HOIST_DONE,…`；True=已回收 | `WaitWinchHoisted` | ✅ 真实（run_real_bt.py + lora_node.py） |
| C9 | `report_status(status, dt)` | 发布 `/lora/status` `STA,HEX,<status>,<x>,<y>` | `ReportStatus` | ✅ 真实（run_real_bt.py + lora_node.py） |
| C10 | `read_remote_cmd()` | 读 `/grasp_hexapod/remote_cmd`，返回 dict（**仅遥控测试链**） | `CheckRemoteCmd` | ✅ 真实（run_real_bt.py + remote_control.py） |

### D. 控制栈模式枚举（需扩展）

行为树共 8 个模式：`home / walk / climb / dock / spin_search / release / approach / tag_nav`。
当前 `control.py` 的 `GraspController` 仅有 `APPROACH / CLIMB / DOCK`，`MissionStateMachine` 有
`IDLE/APPROACH/PREPARE_CLIMB/CLIMB/DOCK/FAILED`——**需扩展并建立映射**（见 TODO-2/3）。

## 3. 待完成事项（TODO）

- [ ] **TODO-1 真实 mode_server.py**：提供 `~/switch_mode`（阻塞式或内部状态机），把 8 个模式映射到
  `GraspController`（`set_mode` / `reset_to_stand` / `mission.start-update`），并输出最终结果 `success/message`。
- [ ] **TODO-2 控制栈模式枚举扩展**：增加 `HOME / WALK / SPIN_SEARCH / RELEASE / APPROACH / TAG_NAV`
  （现仅 APPROACH/CLIMB/DOCK）；`home` = B复位语义 + 平滑回站姿 + 使能。
- [ ] **TODO-3 dock/release 模式内部流程**：`dock` 内部完成 tag 导引到充电桩 ㉙ + 六腿抬起 + 夹爪 clamp +
  结束确认；`release` 内部完成夹爪 open + 松开确认（均不在树中单列）。
- [x] **TODO-4 GripperAct 真实适配**：已在 `grasp_hexapod_servo_cpp` 的
  `GripperManager` 内原生实现 `~/gripper_act`（`grasp_hexapod_msgs/GripperAct`），
  与 `/gripper_command` 共用执行路径/operation 互斥/受限状态机（open 683 /
  clamp 840；受限先 open 复位；带到位校验）；服务线程独立于 30Hz 腿部循环，
  处理期间盲控写入按 busy 暂停。
- [x] **TODO-5 LoRa 节点**：`lora_node.py` 已实现——串口 `CMD,…` 帧（含校验）→ `/lora/command` 净帧（C6/C7/C8 消费）；`/lora/status` 净帧（C9 产生）→ 带校验 `STA,…` 帧写串口。
- [x] **TODO-6 真实 BridgeContext（run_real_bt.py，纯真实）**：C1~C10 全部接到标准名话题/服务。仿真不混入树——补缺由独立 `sim_feedback.py`（按 `real_bt.yaml`）提供。实机仍缺：真实 `mode_server`（TODO-1/2/3）等控制栈执行端
  注入 `build_hexapod_tree`；遥控测试链用 C10 + `remote_control.py`。
- [ ] **TODO-7 spin_search 结果源**：自转搜索的成功由感知节点写入（发现小蓝 + `xiaolan_pose` 有效），
  作为 `spin_search` 模式最终结果（服务 A1 返回）。
- [ ] **TODO-8 双侧同步**：改树须同步 `hexapod_bt.py` 与 `hexapod_mission.xml`；实机接入后用
  `python3 hexapod_bt.py --selftest` + `bt_mock_world.py` 回归。
- [ ] **TODO-9 实机攀爬授权**：`grasp_hexapod_control/scripts/climb_mode.py` 当前为
  simulation-only（自述"只播放固定阶段…不授权实机攀爬"）；TODO-1/2 把 `climb` 接入
  mode_server 前，需补实机 C1→C35 步态序列的接触/承载/稳定性判定与安全边界，
  否则树中 ㉘ 攀爬节点在实机不可放行。
- [ ] **TODO-10 GPS/RTK 驱动实测**：`/fix` 数据源（`reference/gps_ntrip_py`）未与
  `rtk_covariance_ok` 监护联调验收；接实机后验证协方差对角最大 ≤
  `rtk_max_covariance`（0.04 m²）、超限停走（C5 hold_motion）与无 `/fix` 默认放行行为。

## 4. 接口现状总表（验收对照）

| 接口 | 语义要点 | 现状 |
|---|---|---|
| `~/switch_mode` | 执行模式并返回最终结果 | ✅ mock / ❌ 真实（TODO-1,2,3,7） |
| `~/gripper_act` | open/clamp 到位 | ✅ mock / ✅ 真实（TODO-4，`grasp_hexapod_servo_cpp` 原生提供） |
| `/grasp_hexapod/encoder_state` | 编码器三态持续发布 | ✅ 已实现 |
| `/grasp_hexapod/sensor_health` | 六路健康 | ✅ 已实现 |
| `/fix` | RTK 协方差 | ⚠️ 依赖真实 GPS 驱动（TODO-10 实测验收） |
| `/lora/command`、`/lora/status` | 任务指令 / STA 上报 | ✅ mock / ✅ 真实 `lora_node.py` |
| `/grasp_hexapod/remote_cmd` | 遥控（仅测试链） | ✅ 已实现 |
| BridgeContext C1~C10 | 树 ↔ 世界桥接 | ✅ mock / ✅ 真实（run_real_bt.py，TODO-6） |
| 控制栈模式 8 种 | home/walk/climb/dock/spin_search/release/approach/tag_nav | ⚠️ 需扩展（TODO-2） |
| 实机攀爬 climb | C1→C35 实机步态执行 | ⚠️ simulation-only，不授权实机（TODO-9） |
