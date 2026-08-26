# grasp_hexapod_servo_cpp — 幻尔 LX-15D 舵机 ROS 驱动包（C++ 版）

本包是 `grasp_hexapod_servo`（Python/rospy 版）的 **C++/roscpp 重写**，用于降低
资源占用、提高运行速度。ROS 接口（话题、消息、参数、节点名、launch 参数）与原包
**完全一致**，可作为独立替代品直接启动；两块驱动板各起一个节点实例。

| 对比项 | Python 版（grasp_hexapod_servo） | C++ 版（本包） |
|--------|----------------------------------|----------------|
| 语言/运行时 | Python 3 + rospy（解释器、GIL、动态分配） | C++17 + roscpp（原生、零 GC） |
| 协议层 | pyserial | `serial`（ros-noetic-serial） |
| 实时性 | rospy 线程池 + 定时器 | `ros::AsyncSpinner(2)` + `ros::Timer` |
| 单位换算 | `round()` 银行家舍入 | `std::nearbyint`（同语义） |

## 节点：`servo_node`（`ServoSideNode`）

每个节点实例只管理**一块**驱动板，通常同时启动两个实例：

| 实例 | side | 默认串口 | 舵机 ID | 控制的腿 |
|------|------|----------|---------|----------|
| `servo_left_node`  | left  | `/dev/ttyTHS0` | 1~9   | lf, lm, lb |
| `servo_right_node` | right | `/dev/ttyACM0` | 10~18 | rf, rm, rb |

> 腿的命名约定：lf（左前）、lm（左中）、lb（左后）、rf（右前）、rm（右中）、
> rb（右后）。每条腿含 thigh、knee、ankle 三个关节，对应话题中的三个元素。

## 订阅话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/<leg>_des` | `std_msgs/Float64MultiArray` | 第 `<leg>` 条腿的关节目标与使能标志 |

`<leg>` 由 `~side` 参数确定：**left** → `/lf_des`、`/lm_des`、`/lb_des`；
**right** → `/rf_des`、`/rm_des`、`/rb_des`。

### `/<leg>_des` 数据格式（固定 10 个元素）

| 索引 | 字段 | 说明 |
|------|------|------|
| 0 | `power_status` | 板级使能请求：`1` 加载扭矩，`0` 卸力 |
| 1~3 | `thigh/knee/ankle_pos` | 关节目标位置，单位 **rad** |
| 4~6 | `thigh/knee/ankle_vel` | 目标速度（保留字段，当前未使用） |
| 7~9 | `reserved_*` | 保留，填 `0` |

### 安全机制：话题触发的写入

启动后先向本板九个舵机发送卸力命令，随后只读取并发布当前位置，不会写入位置目标。
只有当本板三条腿都收到完整的 `/<leg>_des` 消息且都请求 `power_status=1` 时，节点
才统一加载九个舵机并写入三条腿的位置目标；任一腿 `power_status=0` 立即卸载整板。

## 发布话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/<leg>_pos` | `sensor_msgs/JointState` | 第 `<leg>` 条腿三个关节的带时间戳反馈 |

位置由舵机原始脉冲值（0~1000）经中位 500 换算而来，并乘以对应 `~directions`
方向系数。一条腿三个关节全部读取成功时才发布新反馈；读取失败最多即时重试一次，
仍失败则跳过本腿本周期反馈。

## 坐标与方向约定

- 角度单位：**rad**（ROS 侧）↔ **度**（舵机总线侧）。
- 舵机原始量程：0~1000 脉冲，对应 0°~240°，中位 500；分辨率 `1000/240` 脉冲/度。
- 方向系数：`~directions` 参数（`1` 正方向，`-1` 反方向），抵消机械安装反向。
- 取整使用 round-half-to-even（`std::nearbyint`），与 Python `round()` 一致。

## ROS 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `~side` | `string` | `left` | 驱动板标识：`left` 或 `right` |
| `~port` | `string` | 见上表 | 串口设备路径，按 `side` 自动选择默认值 |
| `~baudrate` | `int` | `115200` | 串口波特率 |
| `~servo_rate_hz` | `float` | `30.0` | Servo 串口读写频率（Hz） |
| `~command_duration_ms` | `int` | `33` | 舵机单次转动指令的目标耗时（ms），传给 `MOVE_TIME_WRITE` |
| `~directions` | `list<int>` 或字符串 | 按板配置 | 可选覆盖；顺序与该板舵机 ID 顺序一致 |
| `~enable_diagnostics` | `bool` | `true` | 输出时序与供电电压诊断 |
| `~voltage_report_interval_s` | `float` | `2.0` | 电压轮询汇总间隔（秒），必须为正 |

默认方向（与 Python 版 `servo.py::SIDE_CONFIG` 相同）：

```text
left:  [1, 1, 1, 1, 1, 1, 1, 1, 1]      # ID 1~9
right: [1, -1, -1, 1, -1, -1, 1, -1, -1]  # ID 10~18
```

## 启动方式

### 同时启动两块板

```bash
roslaunch grasp_hexapod_servo_cpp servo_two_boards.launch
```

按实机接线覆盖串口：

```bash
roslaunch grasp_hexapod_servo_cpp servo_two_boards.launch \
  left_port:=/dev/ttyTHS0 right_port:=/dev/ttyACM0
```

### 单独启动一块板

```bash
rosrun grasp_hexapod_servo_cpp servo_node _side:=left _port:=/dev/ttyTHS0
```

> launch 参数名与 Python 版 `grasp_hexapod_servo` 完全相同；若要将
> `grasp_hexapod_control/run_real.launch` 切到本包，只需把其中的
> `$(find grasp_hexapod_servo)/launch/servo_two_boards.launch` 改为
> `$(find grasp_hexapod_servo_cpp)/launch/servo_two_boards.launch`。

## 依赖

- `roscpp`
- `sensor_msgs`、`std_msgs`
- `serial`（`ros-noetic-serial`，串口库；安装：`sudo apt install ros-noetic-serial`）

## 文件结构

```
grasp_hexapod_servo_cpp/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/grasp_hexapod_servo_cpp/
│   ├── hiwonder_servo_cmd.h        # 协议命令常量（对应 hiwonder_servo_cmd.py）
│   ├── hiwonder_servo_controller.h # LX-15D 串口协议控制器
│   ├── servo_side_node.h           # 单板节点类
│   └── servo_utils.h               # 纯函数：换算/方向解析/上电决策
├── src/
│   ├── hiwonder_servo_controller.cpp
│   ├── servo_side_node.cpp
│   └── servo_node.cpp              # 节点入口
├── launch/
│   └── servo_two_boards.launch
└── test/
    └── test_servo_protocol.cpp     # catkin gtest：协议/换算/方向/决策
```

## 测试与构建

```bash
source /opt/ros/noetic/setup.bash
catkin_make
catkin_make run_tests_grasp_hexapod_servo_cpp
catkin_test_results build/test_results
```

单元测试全部为纯 CPU 回归（不依赖真实串口）：协议封包与校验和、响应帧解析、
rad↔脉冲换算（含银行家舍入与钳位）、方向参数解析、板级上电决策。

## 与原 Python 包的关系

- 两个包并存；本包是原包的 C++ 替代实现，接口一致。
- 离线回放工具 `servo_trace_replay.py` 留在 Python 包，它只向 `/<leg>_des`
  发目标，与两种驱动都兼容。
- 原包保持不动，便于 AB 对比与回退。

## 注意事项

1. 本板三条腿必须都收到完整目标且都请求加载，整块板才开始写入位置。
2. 任一腿的 `power_status=0` 都会卸载该板全部九个舵机。
3. 每周期先写最新目标，再读取反馈；旧目标不会排队补发。
4. 串口读写失败只影响当周期：位置读取失败跳过本腿反馈，串口异常按节流日志记录。
5. `/dev/ttyTHS0` 是 Jetson 板载串口、`/dev/ttyACM0` 是 USB 串口，名称相对
   稳定；必要时仍在 launch 层覆盖。
