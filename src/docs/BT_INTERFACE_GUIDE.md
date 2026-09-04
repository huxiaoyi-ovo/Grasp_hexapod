---

## 1. 你要做什么(分工速查)

| 模块 | 接口 | 你要做的事 | 现状 |
|---|---|---|---|
| **控制栈 / 模式执行** | 服务 `/grasp_hexapod/switch_mode` | **实现这个服务**(8 个模式,见 §4.1) | ❌ 待实现(**重点**) |
| 夹爪驱动 | 服务 `/grasp_hexapod/gripper_act` | 无需开发,直接用 | ✅ `grasp_hexapod_servo_cpp` 已提供 |
| LoRa | `/lora/command`(下行)、`/lora/status`(上行) | 跑 `lora_node.py`,保证串口帧格式 | ✅ 已实现 |
| 编码器 | `/grasp_hexapod/encoder_state` | 跑 `encoder_status_node.py`(串口直读) | ✅ 已实现 |
| 传感器健康 | `/grasp_hexapod/sensor_health` | 跑 `sensor_health_monitor.py` | ✅ 已实现 |
| GPS/RTK | `/fix`(`sensor_msgs/NavSatFix`) | 发布定位与协方差 | ⚠️ 驱动已有,待实测联调 |
| 感知 | 小蓝位姿 `xiaolan_pose` 等 | 检测小蓝(无人机) | ✅ `hexapod_perception` 已有 |
| 遥控(仅测试链) | `/grasp_hexapod/remote_cmd` | 跑 `remote_control.py` | ✅ 已实现 |

---

## 2. 使用规则(先看这个,避免踩坑)

1. **行为树是"调度员",不是"干活的人"**。它每隔一小段时间 tick 一次,检查话题数据、
   调用服务,决定下一步。
2. **`switch_mode` 是阻塞式服务**:收到请求后你把该模式**从头执行到尾**,执行完才
   返回响应。响应即最终结果:
   - `success=true` → 行为树进入下一步;
   - `success=false` → 行为树走失败处理(尽力执行 home,通过 LoRa 上报 `FAILED`);
   - 服务没人应答 → 行为树立即按失败处理。
   - 执行期间**不要**提前返回 true/false;树会一直等。
3. **幂等**:同一个模式已在执行中,又收到同样的请求时,不要重复触发一遍——直接
   返回当前状态即可。
4. **失败要带原因**:`message` 是给人看的,写清楚为什么失败(如 "clamp 失败:偏差超限")。
5. **夹爪不用你(控制栈)另外写驱动**:release 模式内部调 `gripper_act` 的 `open`,
   dock 模式内部调 `clamp`;`success=true` 表示已到位。

---

## 3. 接口一览

```
                 ┌──────────────────────────────────────────┐
  外部模块  ──话题──▶ │  行为树 run_real_bt.py                       │ ──服务调用──▶ 模式执行(控制栈)
  (传感器/LoRa/RTK) │  (每 tick:读话题 → 调服务 → 决定下一步)        │    switch_mode
                 │                                          │    gripper_act(模式内部调)
                 └──────────────────────────────────────────┘
                 ▲ /lora/status(上报)  ▲ /grasp_hexapod/bt_state(可视化)
```

| 方向 | 接口 | 类型 | 用途 |
|---|---|---|---|
| 外部 → 树 | `/grasp_hexapod/sensor_health` | `SensorHealthArray`(~2-5Hz) | 六路传感器在线/新鲜度 |
| 外部 → 树 | `/grasp_hexapod/encoder_state` | `EncoderState`(≥10Hz) | 落地判定(角度 ≥90° 且 ≤180° = 落地) |
| 外部 → 树 | `/fix` | `sensor_msgs/NavSatFix` | RTK 精度监护(协方差对角最大 ≤0.04 m² 放行;完全没数据则默认放行) |
| 外部 → 树 | `/lora/command` | `std_msgs/String` | 地面命令帧 `CMD,HEX,<命令>,…` |
| 树 → 外部 | `/lora/status` | `std_msgs/String` | 上报帧 `STA,HEX,<状态>,<x>,<y>` |
| 树 → 执行端 | `/grasp_hexapod/switch_mode` | `SwitchMode` 服务 | 执行某个模式,响应=最终结果 |
| 模式内部 → 夹爪 | `/grasp_hexapod/gripper_act` | `GripperAct` 服务 | `open` 松开 / `clamp` 夹紧 |
| 树 → 调试 | `/grasp_hexapod/bt_state` | `BtStateArray`(≤5Hz) | 供终端 monitor / Web 看板 |

---

## 4. 重点接口怎么用

### 4.1 `/grasp_hexapod/switch_mode` — 控制栈要实现的服务 ⭐

```text
请求:  string target_mode        # 要执行的模式
响应:  bool   success            # true=模式最终成功 / false=最终失败
       string message            # 成功说明或失败原因(人读)
```

测试命令(服务实现好后):

```bash
rosservice call /grasp_hexapod/switch_mode "target_mode: 'home'"
```

8 个模式各自要干的活、什么叫成功:

| 模式 | 干什么 | 返回 success=true 的条件 |
|---|---|---|
| `home` | 机构复位(释放夹爪) + 回初始站姿 | 姿态到位 |
| `walk` | 遥控速度连续行走(仅测试链,随 remote_cmd) | 连续执行,直到遥控切走 |
| `spin_search` | 原地自转 + 感知搜索小蓝 | 感知发现小蓝 |
| `approach` | RTK 导航粗对准,进入小蓝可视范围 | 目标进入可视区 |
| `tag_nav` | 视觉 tag 伺服到攀爬起始点 | 到达攀爬点 |
| `climb` | 攀爬姿态准备 + C1→C35 步态序列 | 全程爬完并稳定 |
| `dock` | tag 导引到充电桩 → 六腿抬起 → 调夹爪 `clamp` → 结束确认 | 四步全部完成 |
| `release` | 调夹爪 `open` 松开载荷 | 夹爪张到位 |

### 4.2 `/grasp_hexapod/gripper_act` — 夹爪服务(已实现,直接调)

```bash
rosservice call /grasp_hexapod/gripper_act "action: 'open'"    # 松开
rosservice call /grasp_hexapod/gripper_act "action: 'clamp'"   # 夹紧
```

- 响应 `success=true` 表示**已到位**(内部带位置验证),不是"命令已发出"。
- 失败原因示例:受限(上次夹紧失败,须先 open 复位)/ 离线 / 超时 / busy。

### 4.3 `/lora/command` — 地面命令(下行)

帧格式:`CMD,HEX,<命令>,NOW`,如 `CMD,HEX,RECOVER,NOW`。

| 命令 | 含义 | 行为树收到后 |
|---|---|---|
| `RELEASE` | 任务 = 释放小蓝 | 走释放分支 |
| `RECOVER` | 任务 = 抓取回收 | 走回收分支 |
| `DEPLOY` | 无人机下放开始 | 等编码器确认落地 → 上报 `LANDED` |
| `HOIST_DONE` | 拉升回收完成 | 等待 HOME 命令 |
| `HOME` | 恢复初始 | 执行 home 模式 → 上报 `RESET_DONE` |

手动测试(不接 LoRa 硬件时):

```bash
rostopic pub /lora/command std_msgs/String "data: 'CMD,HEX,RECOVER,NOW'"
```

> 注意:`DEPLOY / HOIST_DONE / HOME` 是**读后清除**的——消费一次即失效,不会重复触发。

### 4.4 `/lora/status` — 状态上报(上行)

帧格式:`STA,HEX,<状态>,<x>,<y>`(x/y 为定位坐标,无定位时 0.00)。

| 状态 | 含义 |
|---|---|
| `LANDED` | 已确认落地 |
| `RELEASED` | 已释放小蓝 |
| `CLAMPED` | 已夹紧对接 |
| `RESET_DONE` | 已恢复初始姿态 |
| `DONE` | 整个任务完成 |
| `FAILED` | 任务失败(已尽力回 home) |

---

## 5. 全流程对照(一张表看懂任务)

| 阶段 | 地面命令 | 行为树动作 | 完成后上报 |
|---|---|---|---|
| 任务下发 | `RECOVER` / `RELEASE` | 分流到对应任务分支 | — |
| 初始化 | — | 执行 `home` 模式 | — |
| 下放 | `DEPLOY` | 等待 | — |
| 落地 | —(编码器反馈) | 角度 ∈ [90°,180°] 判定落地 | `LANDED` |
| 接近 | — | `spin_search` → `approach` | — |
| 精准攀爬 | — | `tag_nav` → `climb` | — |
| 对接夹紧 | — | `dock`(内部调夹爪 clamp) | `CLAMPED` |
| 拉升回收 | `HOIST_DONE` | 等待 | — |
| 恢复初始 | `HOME` | 执行 `home` 模式 | `RESET_DONE` |
| 完成 | — | — | `DONE` |
| 任一步失败 | — | 尽力执行 `home` | `FAILED` |

(释放任务支线:`RELEASE` → release 模式 → `RELEASED` → `HOIST_DONE` → `HOME` → `RESET_DONE` → `DONE`)

---

## 6. 联调怎么跑(三条命令)

```bash
# 1. 跑行为树(纯真实,不含仿真)
rosrun grasp_hexapod_bt run_real_bt.py

# 2. 缺哪个真实模块,就开 sim_feedback 模拟哪个(按 config/real_bt.yaml 的 simulate 开关)
rosrun grasp_hexapod_bt sim_feedback.py

# 3. 实时看行为树在干什么
rosrun grasp_hexapod_bt bt_monitor.py        # 终端版
rosrun grasp_hexapod_bt bt_dashboard.py      # Web 看板
```

- 你的真实模块就绪后,把 `config/real_bt.yaml` 里对应接口的 `simulate` 改为 `false`,
  行为树即切换到真实数据。
- 也可以用 `sim_manual.py` 手动注入命令/故障,做单步联调。

### 单独发起服务调用(不跑行为树,手动测)

前置:先确认服务已有人提供(真实环境 = 夹爪服务由 `servo_two_boards.launch` 提供;
联调环境 = 跑一个 `sim_feedback.py` 即有两个模拟服务):

```bash
rosservice list | grep -E "switch_mode|gripper_act"
```

**模式服务**(8 个模式,逐个测):

```bash
rosservice call /grasp_hexapod/switch_mode "target_mode: 'home'"
rosservice call /grasp_hexapod/switch_mode "target_mode: 'walk'"
rosservice call /grasp_hexapod/switch_mode "target_mode: 'spin_search'"
rosservice call /grasp_hexapod/switch_mode "target_mode: 'approach'"
rosservice call /grasp_hexapod/switch_mode "target_mode: 'tag_nav'"
rosservice call /grasp_hexapod/switch_mode "target_mode: 'climb'"
rosservice call /grasp_hexapod/switch_mode "target_mode: 'dock'"
rosservice call /grasp_hexapod/switch_mode "target_mode: 'release'"
```

**夹爪服务**:

```bash
rosservice call /grasp_hexapod/gripper_act "action: 'open'"    # 松开
rosservice call /grasp_hexapod/gripper_act "action: 'clamp'"   # 夹紧
```

注意事项:

- **阻塞式**:命令会一直等到该模式执行完才返回。`climb`/`dock` 等长模式可能要等
  几分钟;怕卡住可加超时,例如 `timeout 10 rosservice call …`(超时只断开客户端,
  不中断模式执行)。
- **不要连发**:上一次调用还没返回时再发,会得到 `success: False`、
  `message: "busy: …"`——这是正常的并发保护。
- **返回解读**:`success: True` = 该模式/动作最终成功;`success: False` + `message`
  = 失败及原因(受限 / 离线 / 超时 / busy 等)。
- `walk`/`spin_search` 等模式的结束条件不同(见 §4.1 表),单独调用时会等到模式自身
  完成才返回,不是发完指令就结束。

---

## 7. 常见问题

- **行为树卡住不动?** 多数在等数据:传感器没全部 online、没收到命令、或 `switch_mode`
  服务还没人提供(会一直等或失败回退)。用 `bt_monitor.py` 看当前卡在哪个节点。
- **`/fix` 没有会怎样?** RTK 监护默认放行,不阻塞无 RTK 场景;有数据但协方差超限时
  行为树会请求停走等待,超时算失败。
- **编码器没数据会怎样?** 编码器节点只在收到有效串口帧时发布;没数据时行为树保持
  上次状态继续等,落地确认外层超时 120 秒算失败。
- **switch_mode 调用失败/没响应?** 服务不可用 = 立即失败回退,所以联调时先确认
  服务提供方已启动:`rosservice list | grep switch_mode`。
