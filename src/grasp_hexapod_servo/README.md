# grasp_hexapod_servo — 幻尔 LX-15D 舵机 ROS 驱动包

本包负责把 ROS 话题中的关节目标转换为幻尔总线舵机（LX-15D）的串口指令，并把读取到的当前位置发布回 ROS。六足机器人共有 18 个舵机，分布在 **left / right / mid** 三块驱动板上，各接一条独立串口。

## 节点：`servo.py`（`ServoSideNode`）

每个节点实例只管理**一块**驱动板。通常同时启动三个实例：

| 实例 | side | 默认串口 | 舵机 ID | 控制的腿 |
|------|------|----------|---------|----------|
| `servo_left_node`  | left  | `/dev/ttyUSB0` | 1~6   | lf, lm |
| `servo_right_node` | right | `/dev/ttyUSB1` | 10~15 | rf, rm |
| `servo_mid_node`   | mid   | `/dev/ttyUSB2` | 7~9, 16~18 | lb, rb |

> 腿的命名约定：lf（左前）、lm（左中）、lb（左后）、rf（右前）、rm（右中）、rb（右后）。每条腿含 thigh、knee、ankle 三个关节，对应话题中的三个元素。

---

## 订阅话题

节点为它所控制的**每一条腿**独立订阅一个话题。

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/<leg>_des` | `std_msgs/Float64MultiArray` | 第 `<leg>` 条腿的关节目标与使能标志 |

`<leg>` 根据 `~side` 参数自动确定：
- **left** → `/lf_des`, `/lm_des`
- **right** → `/rf_des`, `/rm_des`
- **mid** → `/lb_des`, `/rb_des`

### `/<leg>_des` 数据格式

数组长度至少为 9，各字段含义如下：

| 索引 | 字段 | 说明 |
|------|------|------|
| 0 | `power_status` | 使能标志：`1` 为该腿舵机上电（加载扭矩），`0` 为卸力 |
| 1 | `thigh_pos` | thigh 关节目标位置，单位 **rad** |
| 2 | `knee_pos` | knee 关节目标位置，单位 **rad** |
| 3 | `ankle_pos` | ankle 关节目标位置，单位 **rad** |
| 4 | `thigh_vel` | thigh 关节目标速度，单位 **rad/s**（当前未使用，保留字段） |
| 5 | `knee_vel` | knee 关节目标速度，单位 **rad/s**（当前未使用，保留字段） |
| 6 | `ankle_vel` | ankle 关节目标速度，单位 **rad/s**（当前未使用，保留字段） |
| 7 | `reserved_0` | 保留，填 `0` |
| 8 | `reserved_1` | 保留，填 `0` |
| 10 | `reserved_2` | 保留，填 `0` |

### 安全机制：话题触发的写入

节点启动后**仅读取并发布当前位置**，**不会向舵机写入任何目标**。只有当某条腿**首次收到**对应的 `/<leg>_des` 话题数据后，该腿才进入“允许写入”状态，后续周期才会把目标下发到舵机。这是为了防止程序启动瞬间的零值或噪声导致舵机意外运动。

---

## 发布话题

节点为它所控制的**每一条腿**独立发布一个反馈话题。

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/<leg>_pos` | `std_msgs/Float64MultiArray` | 第 `<leg>` 条腿三个关节的实时位置反馈 |

### `/<leg>_pos` 数据格式

数组固定长度为 3：

| 索引 | 字段 | 说明 |
|------|------|------|
| 0 | `thigh_pos` | thigh 关节当前位置，单位 **rad** |
| 1 | `knee_pos` | knee 关节当前位置，单位 **rad** |
| 2 | `ankle_pos` | ankle 关节当前位置，单位 **rad** |

> 位置由舵机原始脉冲值（0~1000）经中位 500 换算而来，并乘以对应 `~directions` 方向系数。若某舵机读数失败，对应元素为 `nan`。

---

## 坐标与方向约定

- 角度单位：**rad**（ROS 侧）↔ **度**（舵机总线侧）。
- 舵机原始量程：0~1000 脉冲，对应 0°~240°，中位 500。
- 分辨率：`1000 / 240 ≈ 4.167` 脉冲/度。
- 方向系数：通过 `~directions` 参数配置（`1` 为正方向，`-1` 为反方向），用于抵消机械安装反向。

---

## ROS 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `~side` | `string` | `left` | 驱动板标识：`left`、`right` 或 `mid` |
| `~port` | `string` | 见上表 | 串口设备路径，按 `side` 自动选择默认值 |
| `~baudrate` | `int` | `115200` | 串口波特率 |
| `~control_rate_hz` | `float` | `50.0` | 控制循环频率（Hz），决定读/写舵机的周期 |
| `~command_duration_ms` | `int` | `20` | 舵机单次转动指令的目标耗时（ms），传给 LX-15D 的 `MOVE_TIME_WRITE` |
| `~directions` | `list<int>` | 全 `1` | 各舵机方向列表（`1` 正 / `-1` 反），顺序与该板舵机 ID 顺序一致 |

### `~directions` 配置示例

launch 文件中可按板分别指定（见 `launch/servo_dual_side.launch`）：

```xml
<arg name="left_directions"  default="[1, 1, 1, 1, 1, 1]"/>
<arg name="right_directions" default="[1, -1, -1, 1, -1, -1]"/>
<arg name="mid_directions"   default="[1, 1, 1, 1, -1, -1]"/>
```

- **left**：ID 1,2,3,4,5,6
- **right**：ID 10,11,12,13,14,15
- **mid**：ID 7,8,9,16,17,18

---

## 启动方式

### 同时启动三块板（推荐）

```bash
roslaunch grasp_hexapod_servo servo_dual_side.launch
```

可在 launch 中按实机接线修改串口：

```bash
roslaunch grasp_hexapod_servo servo_dual_side.launch left_port:=/dev/ttyUSB0 right_port:=/dev/ttyUSB1 mid_port:=/dev/ttyUSB2
```

### 单独启动一块板

```bash
rosrun grasp_hexapod_servo servo.py _side:=left _port:=/dev/ttyUSB0 _control_rate_hz:=50.0
```

---

## 依赖

- `rospy`
- `std_msgs`
- `pyserial`

---

## 文件结构

```
grasp_hexapod_servo/
├── launch/
│   └── servo_dual_side.launch    # 三块板同时启动
├── scripts/
│   ├── servo.py                   # 主节点：ServoSideNode
│   ├── hiwonder_servo_controller.py   # LX-15D 串口协议封装
│   └── hiwonder_servo_cmd.py      # 协议命令常量
└── package.xml
```

---

## 注意事项

1. **必须先收到 `/<leg>_des` 话题数据**，对应腿才会开始写入舵机；否则节点只读位置、不执行动作。
2. `power_status`（`/<leg>_des.data[0]`）控制舵机是否加载扭矩；置 `0` 时卸力，机械腿可手动摆动。
3. 若某舵机读数超时，对应 `/<leg>_pos` 元素为 `nan`，不影响其他舵机。
4. 串口设备名（`/dev/ttyUSB*`）可能因插拔顺序变化，建议在系统层用 `udev` 规则固定别名。
