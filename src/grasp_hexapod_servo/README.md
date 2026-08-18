# grasp_hexapod_servo — 幻尔 LX-15D 舵机 ROS 驱动包

本包负责把 ROS 话题中的关节目标转换为幻尔总线舵机（LX-15D）的串口指令，并把读取到的当前位置发布回 ROS。六足机器人共有 18 个舵机，分布在 **left / right** 两块驱动板上，各接一条独立串口。

## 节点：`servo.py`（`ServoSideNode`）

每个节点实例只管理**一块**驱动板。通常同时启动两个实例：

| 实例 | side | 默认串口 | 舵机 ID | 控制的腿 |
|------|------|----------|---------|----------|
| `servo_left_node`  | left  | `/dev/ttyTHS0` | 1~9   | lf, lm, lb |
| `servo_right_node` | right | `/dev/ttyACM0` | 10~18 | rf, rm, rb |

> 腿的命名约定：lf（左前）、lm（左中）、lb（左后）、rf（右前）、rm（右中）、rb（右后）。每条腿含 thigh、knee、ankle 三个关节，对应话题中的三个元素。

---

## 订阅话题

节点为它所控制的**每一条腿**独立订阅一个话题。

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/<leg>_des` | `std_msgs/Float64MultiArray` | 第 `<leg>` 条腿的关节目标与使能标志 |

`<leg>` 根据 `~side` 参数自动确定：
- **left** → `/lf_des`, `/lm_des`, `/lb_des`
- **right** → `/rf_des`, `/rm_des`, `/rb_des`

### `/<leg>_des` 数据格式

数组长度固定为 10，各字段含义如下：

| 索引 | 字段 | 说明 |
|------|------|------|
| 0 | `power_status` | 板级使能请求：`1` 为加载扭矩，`0` 为卸力 |
| 1 | `thigh_pos` | thigh 关节目标位置，单位 **rad** |
| 2 | `knee_pos` | knee 关节目标位置，单位 **rad** |
| 3 | `ankle_pos` | ankle 关节目标位置，单位 **rad** |
| 4 | `thigh_vel` | thigh 关节目标速度，单位 **rad/s**（当前未使用，保留字段） |
| 5 | `knee_vel` | knee 关节目标速度，单位 **rad/s**（当前未使用，保留字段） |
| 6 | `ankle_vel` | ankle 关节目标速度，单位 **rad/s**（当前未使用，保留字段） |
| 7 | `reserved_0` | 保留，填 `0` |
| 8 | `reserved_1` | 保留，填 `0` |
| 9 | `reserved_2` | 保留，填 `0` |

### 安全机制：话题触发的写入

节点启动后先向本板九个舵机发送卸力命令，随后只读取并发布当前位置，
不会写入位置目标。只有当本板三条腿都收到完整的`/<leg>_des`消息且都请求
`power_status=1`时，节点才统一加载九个舵机并写入三条腿的位置目标。

---

## 发布话题

节点为它所控制的**每一条腿**独立发布一个反馈话题。

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/<leg>_pos` | `sensor_msgs/JointState` | 第 `<leg>` 条腿三个关节的带时间戳反馈 |

### `/<leg>_pos` 数据格式

```text
header.stamp: 本腿三个关节读取完成时间
name: [<leg>_thigh_joint, <leg>_knee_joint, <leg>_ankle_joint]
position: [thigh_pos, knee_pos, ankle_pos]
```

位置由舵机原始脉冲值（0~1000）经中位500换算而来，并乘以对应
`~directions`方向系数。一条腿三个关节全部读取成功时才发布新反馈。

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
| `~side` | `string` | `left` | 驱动板标识：`left` 或 `right` |
| `~port` | `string` | 见上表 | 串口设备路径，按 `side` 自动选择默认值 |
| `~baudrate` | `int` | `115200` | 串口波特率 |
| `~servo_rate_hz` | `float` | `30.0` | Servo串口读写频率（Hz） |
| `~command_duration_ms` | `int` | `33` | 舵机单次转动指令的目标耗时（ms），传给 LX-15D 的 `MOVE_TIME_WRITE` |
| `~directions` | `list<int>` | 按板配置 | 可选覆盖值；顺序与该板舵机 ID 顺序一致 |

### `~directions` 配置示例

默认方向只有一份，位于`servo.py::SIDE_CONFIG`：

```xml
left:  [1, 1, 1, 1, 1, 1, 1, 1, 1]
right: [1, -1, -1, 1, -1, -1, -1, -1, -1]
```

- **left**：ID 1,2,3,4,5,6,7,8,9
- **right**：ID 10,11,12,13,14,15,16,17,18

---

## 启动方式

### 同时启动两块板（推荐）

```bash
roslaunch grasp_hexapod_servo servo_two_boards.launch
```

可在 launch 中按实机接线修改串口：

```bash
roslaunch grasp_hexapod_servo servo_two_boards.launch left_port:=/dev/ttyTHS0 right_port:=/dev/ttyACM0
```

### 单独启动一块板

```bash
rosrun grasp_hexapod_servo servo.py _side:=left _port:=/dev/ttyTHS0
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
│   └── servo_two_boards.launch # 两块板同时启动
├── scripts/
│   ├── servo.py                   # 主节点：ServoSideNode
│   ├── hiwonder_servo_controller.py   # LX-15D 串口协议封装
│   └── hiwonder_servo_cmd.py      # 协议命令常量
└── package.xml
```

---

## 注意事项

1. 本板三条腿必须都收到完整目标且都请求加载，整块板才开始写入位置。
2. 任一腿的`power_status=0`都会卸载该板全部九个舵机。
3. 每周期先写最新目标，再读取反馈；旧目标不会排队补发。
4. 舵机读取失败最多即时重试一次，仍失败时跳过本腿本周期反馈。
5. `/dev/ttyTHS0` 是 Jetson 板载串口、`/dev/ttyACM0` 是 USB 串口，名称相对稳定；必要时仍可在 launch 层覆盖。
