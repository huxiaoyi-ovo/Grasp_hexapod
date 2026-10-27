# encoder_driver

`src/reference/encoder_driver/`：串口绝对编码器驱动。两个实现：

- **`scripts/encoder_status_node.py`（Python，行为树运行链路使用）**：读角度并
  在节点内做落地判断，持续发布 `/grasp_hexapod/encoder_state`
  （`grasp_hexapod_msgs/EncoderState`），供行为树 `IsLandingConfirmed` 消费。
- C++ 参考实现（roscpp）：只解析帧、发布角度/原始值，不做落地判断，不参与
  运行链路（见下文）。

## 编码器状态节点（落地判断，运行链路）

复用与 C++ 版相同的串口协议（Modbus RTU 读保持寄存器），按**角度范围两态**
判定并在每收到有效帧时发布状态：

**落地判据**：最近有效角度 ∈ **[90, 180] deg**（边界含）→ `landed=true`（已落地）；
越界 → `landed=false`（未落地）。无数据时不发布（订阅方保持上次状态）。

```bash
rosrun encoder_driver encoder_status_node.py _port:=/dev/ttyUSB0
python3 scripts/encoder_status_node.py --selftest   # 离线自检（假串口）
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `~port` | `/dev/ttyUSB0` | 串口设备 |
| `~baud` | `115200` | 波特率 |
| `~slave_id` | `0` | 模块地址 |
| `~start_reg` / `~reg_count` | `0` / `2` | 读保持寄存器（4 字节 → 18 位值） |
| `~poll_hz` | `10` | 轮询频率 |
| `~landing_min` / `~landing_max` | `90` / `180` | 落地角度范围（deg，闭区间） |
| `~state_topic` | `/grasp_hexapod/encoder_state` | 状态话题 |

发布：`EncoderState{ header, bool landed, float64 angle, string reason }`——
`landed` 由角度范围决定，`reason` 为人读说明（含具体角度）。
消费方：行为树落地确认（超时 120s 走失败回退）。详见
`src/docs/BT_MODE_INTERFACES.md` §1.2。

## C++ 参考实现（不参与运行链路）

## 数据帧格式

响应帧遵循 `ID CMD Len Data ... CRCH CRCL`：

| 字段 | 说明 |
|---|---|
| `ID` | 模块地址，如 `0x00` |
| `CMD` | 读命令 `0x03` |
| `Len` | 随后的数据字节数 |
| `Data` | 编码器原始值，按大端组合 |
| `CRC` | Modbus CRC16，发送时低字节在前 |

示例：

```text
00 03 04 00 01 5F CF C2 97
```

- 原始数据 `00015FCF`(hex) = `90063`(dec)
- 角度 `Angle = 90063 / 262144 * 360 = 123.682708`（度）

其中 `262144 = 2**18`，即 18 位绝对编码器。

## 目录结构（C++ 参考）

```text
include/encoder_driver/encoder_frame.hpp   协议头（不依赖 ROS）
src/encoder_frame.cpp                      协议实现 + 离线自检
src/encoder_node.cpp                       roscpp 节点
launch/encoder.launch                      启动文件
scripts/encoder_status_node.py             落地判断节点（运行链路，见上）
```

## 构建

```bash
source /opt/ros/noetic/setup.bash
catkin_make            # 或 catkin_make --only-pkg-with-deps encoder_driver
source devel/setup.bash
```

## 运行自检

协议层不依赖 ROS 主节点，可直接离线校验 CRC、帧解析和角度换算：

```bash
rosrun encoder_driver encoder_node --selftest
```

会依次核对示例 1（`90063 -> 123.682708 deg`）、示例 2（`00 03 02 00 00 85 84`）、
夹带/分段字节流的帧提取，以及读请求帧自身的 CRC。

## 使用

```bash
source /opt/ros/noetic/setup.bash
roslaunch encoder_driver encoder.launch
```

默认参数（可在 launch 中覆盖）：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `port` | `/dev/ttyUSB0` | 串口设备 |
| `baudrate` | `115200` | 波特率 |
| `slave_id` | `0` | 模块地址 |
| `start_reg` | `0` | 读取起始寄存器 |
| `reg_count` | `2` | 读取寄存器个数（对应 4 字节 -> 18 位值） |
| `rate_hz` | `50` | 采样频率 |
| `units` | `deg` | 角度单位 `deg` 或 `rad` |
| `angle_topic` | `encoder_angle` | 角度话题 |
| `raw_topic` | `encoder_raw` | 原始话题 |

节点发布：

- `encoder_angle`：`std_msgs/Float64`，编码器角度（默认度，`units:=rad` 时为弧度）。
- `encoder_raw`：`std_msgs/Int32`，编码器原始计数值。

多个编码器时可启多个节点，并用 `slave_id`/命名空间区分。

## 说明

- `start_reg` 需按编码器寄存器的实际定义配置；本包只实现通用的
  Modbus-RTU 读保持寄存器请求与响应解析。
- 单位为度时角度范围与 18 位分辨率对应；`units:=rad` 时按弧度换算。
- 串口使用 POSIX termios 直接配置为 8N1 无流控，无额外第三方依赖。
