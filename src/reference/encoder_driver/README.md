# encoder_driver

`src/reference/encoder_driver/` 下的参考功能包（C++ / roscpp）：读取串口绝对编码器的
角度数据，解析响应帧并发布编码器角度。属于参考代码，不参与 `run_real.launch`
等运行链路。

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

## 目录结构

```text
include/encoder_driver/encoder_frame.hpp   协议头（不依赖 ROS）
src/encoder_frame.cpp                      协议实现 + 离线自检
src/encoder_node.cpp                       roscpp 节点
launch/encoder.launch                      启动文件
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
