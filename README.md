# Grasp Hexapod

<p align="center">
  <a href="https://github.com/huxiaoyi-ovo/Grasp_hexapod/stargazers"><img src="https://img.shields.io/github/stars/huxiaoyi-ovo/Grasp_hexapod?style=flat-square&logo=github" alt="GitHub stars" /></a>
  <a href="https://github.com/huxiaoyi-ovo/Grasp_hexapod/forks"><img src="https://img.shields.io/github/forks/huxiaoyi-ovo/Grasp_hexapod?style=flat-square&logo=github" alt="GitHub forks" /></a>
  <a href="https://github.com/huxiaoyi-ovo/Grasp_hexapod/issues"><img src="https://img.shields.io/github/issues/huxiaoyi-ovo/Grasp_hexapod?style=flat-square" alt="GitHub issues" /></a>
  <img src="https://img.shields.io/github/last-commit/huxiaoyi-ovo/Grasp_hexapod?style=flat-square" alt="Last commit" />
</p>

ROS Noetic六足机器人工作空间。项目使用手写运动控制器，同时支持Isaac Gym
仿真和三块LX-15D舵机板实机执行。

## 控制架构

仿真和实机共用同一个`GraspController`和B/A状态机，但按执行后端采用
不同的调度方式：

```text
ROS仿真：/joy/导航 -> Isaac内同步GraspController -> 关节目标
实机：   /joy/导航 -> ROS控制节点 -> Servo三板 -> 关节反馈
```

仿真的六腿目标和反馈话题只用于监控，不参与闭环。

固定约定：

- 腿顺序：`lb, lf, lm, rb, rf, rm`
- 关节顺序：`thigh, knee, ankle`
- 角度单位：rad
- `base_link`：`+x`向右、`+y`向前、`+z`向上
- 仿真和实机控制器：60 Hz；实机Servo链路：30 Hz
- 仿真和实机满杆速度：平移0.20 m/s，升降0.02 m/s

主要功能包：

```text
src/grasp_hexapod_control/      步态、运动学、安全状态机、ROS和Isaac入口
src/grasp_hexapod_description/  URDF、mesh、RViz和Gazebo
src/grasp_hexapod_servo/        三板LX-15D驱动和验证过的串口协议
src/reference/                  参考控制器和SDK，不参与运行
```

## 构建

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

主要依赖：

```bash
sudo apt install \
  ros-noetic-joy \
  python3-numpy \
  python3-rospkg \
  python3-serial
```

Isaac Gym需要单独安装在其支持的Python环境中。

## 运行仿真

保留原来的Python直接控制链：

```bash
/home/artrc/miniconda3/envs/grasp_hexapod/bin/python \
  src/grasp_hexapod_control/scripts/run_sim.py
```

ROS手柄仿真：

```bash
roslaunch grasp_hexapod_control run_sim_ros.launch
```

导航模式或无界面运行：

```bash
roslaunch grasp_hexapod_control run_sim_ros.launch \
  mode:=navigation \
  headless:=true
```

## 运行实机

三块板分别控制两条腿：

| 板 | 腿 | 舵机ID | 默认串口 |
|---|---|---|---|
| left | lf、lm | 1～6 | `/dev/ttyUSB0` |
| right | rf、rm | 10～15 | `/dev/ttyUSB1` |
| mid | lb、rb | 7～9、16～18 | `/dev/ttyUSB2` |

开发机可直接覆盖临时串口：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  left_port:=/dev/ttyUSB0 \
  right_port:=/dev/ttyUSB1 \
  mid_port:=/dev/ttyUSB2
```

仿真和实机默认使用同一速度。首次架空测试如需临时降速，可以显式覆盖：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  max_linear_speed:=0.01 \
  max_vertical_speed:=0.003
```

工控机算力不足导致控制卡顿时，可在架空且周围无障碍的测试中临时绕过
整机连杆自碰撞检查：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  enable_link_collision_check:=false
```

该开关默认启用。关闭后仍保留关节限位、足端工作空间投影和足端间距检查，
但连杆之间及连杆与机身之间的自碰撞不再受保护。

工控机部署时应把三个串口参数替换为稳定的
`/dev/serial/by-id/...`路径。

## 安全操作顺序

```text
B -> 等待回到标准站姿 -> A -> 接受运动指令
```

- B：最高优先级，取消当前行为并平滑返回标准站姿。
- A：站姿初始化完成后启用或暂停运动。
- X：预留攀爬模式，当前只暂停。
- Y：预留对接模式，当前只暂停。
- 第一次按B前，高层控制器不发送目标，实机舵机保持卸力。
- 导航模式下推动运动摇杆会立即取消导航并锁存为手柄控制。

整机状态固定为：

```text
WAIT_B -> RESETTING -> HOLD <-> RUNNING
```

支撑组、摆动组和三角步态换相仍由`ApproachMode`独立管理。

## 实机启动前检查

```bash
ls -l /dev/input/js*
ls -l /dev/serial/by-id/
roslaunch --nodes grasp_hexapod_control run_real.launch
```

启动后可检查：

```bash
rostopic echo /joy
rostopic echo /lf_pos
rostopic echo /lf_des
```

`/lf_pos`是带采样时间的`sensor_msgs/JointState`；只有该腿三个舵机
都读取成功时才发布新反馈。

必须在机器人架起的状态下确认三块板串口、舵机ID、安装方向和机械零位，
再进行平地行走。

## 文档

- [启动命令与参数](src/grasp_hexapod_control/launch/README.md)
- [ROS消息、状态机和导航接口](src/docs/ROS_INTERFACES.md)
- [Servo协议、ID和方向](src/grasp_hexapod_servo/README.md)
- [实机延迟测试](src/docs/SERVO_LATENCY_TEST.md)

## Contributors

<a href="https://github.com/huxiaoyi-ovo/Grasp_hexapod/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=huxiaoyi-ovo/Grasp_hexapod" alt="Contributors" />
</a>
