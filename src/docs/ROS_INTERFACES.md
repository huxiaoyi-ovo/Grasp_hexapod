# ROS控制接口

## 控制链路

实机使用ROS连接高层控制器和两块Servo板：

```text
/joy
  -> /grasp_hexapod_control
  -> 六个 /<leg>_des
  -> Servo三板
  -> 六个 /<leg>_pos
  -> /grasp_hexapod_control
```

ROS仿真中，ROS只传递Joy、导航和模式输入，`GraspController`在Isaac的
60 Hz控制帧内同步执行：

```text
/joy/导航
  -> grasp_hexapod_sim中的GraspController
  -> Isaac Gym
  -> 下一控制帧反馈
```

仿真仍发布六个`/<leg>_des`和`/<leg>_pos`用于观察、记录和接口检查，
但这些话题不参与仿真闭环，避免两个同频异步节点造成目标重复和关节卡顿。

腿顺序固定为：

```text
lb, lf, lm, rb, rf, rm
```

每条腿内部顺序固定为：

```text
thigh, knee, ankle
```

位置单位为rad，速度单位为rad/s。`base_link`中`+x`向右、`+y`向前、
`+z`向上，偏航正方向绕`+z`。仿真和实机默认使用相同的满杆速度：
平移0.20 m/s，升降0.02 m/s。

## 关节话题

`/<leg>_pos`使用`sensor_msgs/JointState`：

```text
header.stamp: 本腿反馈采样完成时间
name: [<leg>_thigh_joint, <leg>_knee_joint, <leg>_ankle_joint]
position: [thigh_pos, knee_pos, ankle_pos]
```

一条腿三个舵机全部读取成功时才发布新反馈。读取失败不会发布`nan`，
控制器通过`header.stamp`判断反馈是否仍然有效。

`/<leg>_des`使用`std_msgs/Float64MultiArray`，固定十个元素：

```text
[power,
 thigh_pos, knee_pos, ankle_pos,
 thigh_vel, knee_vel, ankle_vel,
 0, 0, 0]
```

当前LX-15D只执行位置和`power`字段。速度及末尾三个字段必须填0。
`power=1`加载整块板的六个舵机，`power=0`卸载整块板。

## 手柄状态机

默认按钮和轴：

| 输入 | 索引 | 语义 |
|---|---:|---|
| A | 0 | 初始化完成后启用/暂停 |
| B | 1 | 最高优先级，停止当前行为并平滑返回站姿 |
| X | 2 | 预留CLIMB；当前只暂停并提示未实现 |
| Y | 3 | 预留DOCK；当前只暂停并提示未实现 |
| 左摇杆水平 | 0 | 向右速度 |
| 左摇杆竖直 | 1 | 向前速度 |
| 右摇杆水平 | 3 | 偏航速度 |
| LT / RT | 4 / 5 | LT下降、RT上升 |

不同Linux手柄驱动的轴符号可能不同。四个`axis_*_scale`参数分别控制横移、
前后、偏航和升降方向；默认值为`-1, 1, 1, -1`。按钮索引和轴倍率在实机、
ROS仿真两个launch入口中使用相同参数。

运动使能顺序固定为：

```text
B -> 完成站姿初始化 -> A -> 接受运动指令
```

第一次B以前控制节点不发布关节目标，实机Servo保持卸力。

整机只使用四个状态：

```text
WAIT_B -> RESETTING -> HOLD <-> RUNNING
```

B在反馈暂时无效时也会锁存为`RESETTING`，反馈恢复后再从真实关节角回站。
平地三角步态内部的支撑组、摆动组和换相状态不属于这四个整机状态，
仍由`ApproachMode`维护。

## 手柄与导航启动

手柄模式（默认）：

```bash
roslaunch grasp_hexapod_control run_real.launch
roslaunch grasp_hexapod_control run_sim_ros.launch
```

导航模式：

```bash
roslaunch grasp_hexapod_control run_real.launch mode:=navigation
roslaunch grasp_hexapod_control run_sim_ros.launch mode:=navigation
```

导航模式仍启动`joy_node`。任一平移、升降或偏航轴超过0.1时，当前导航计划
立即取消，并锁定为手柄控制；松开摇杆不会自动恢复导航。按B重新初始化后，
再按A才会重新规划导航。

两个入口复用同一个`RosControlNode`状态机和`GraspController`。实机由
`control_stack.launch`启动独立控制节点；仿真在Isaac控制帧中同步调用状态机。
`mode`只决定初始命令源，不改变手柄的最高接管权限。

## 导航输入

导航几何统一使用`frame_id=pv_map`：

| 话题 | 类型 |
|---|---|
| `/grasp_hexapod/navigation/base_pose` | `geometry_msgs/PoseStamped` |
| `/grasp_hexapod/navigation/xiaolan_pose` | `geometry_msgs/PoseStamped` |
| `/grasp_hexapod/navigation/pv_boundary` | `geometry_msgs/PolygonStamped` |
| `/grasp_hexapod/landing_confirmed` | `std_msgs/Bool` |

自动接近还需要两个经过实测的4×4变换：

```text
xiaolan_from_left_base
xiaolan_from_right_base
```

未配置这两个变换、导航数据过期、没有落板确认或路径不满足边界约束时，
按A不会启动导航，机器人保持不动；此时推动摇杆仍可接管为手柄控制。
项目不提供猜测默认值。

## 两块舵机板

| 板 | 腿 | ID |
|---|---|---|
| left | lf, lm, lb | 1,2,3 / 4,5,6 / 7,8,9 |
| right | rf, rm, rb | 10,11,12 / 13,14,15 / 16,17,18 |

ID和默认方向的唯一来源是
`grasp_hexapod_servo/scripts/servo.py::SIDE_CONFIG`。

实机控制器和Joy以60 Hz运行，两个Servo节点以30 Hz运行。Servo每周期
先执行最新目标，再读取反馈；单次读取失败最多即时重试一次，仍失败时
本周期不发布对应腿的反馈。
