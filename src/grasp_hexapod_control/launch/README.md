# Launch 使用说明

## 使用前准备

在工作空间根目录执行：

```bash
catkin_make
source devel/setup.bash
```

## 实机

手柄模式是默认模式：

```bash
roslaunch grasp_hexapod_control run_real.launch
```

导航模式仍会启动手柄，摇杆可以随时接管：

```bash
roslaunch grasp_hexapod_control run_real.launch mode:=navigation
```

开发机上的`ttyUSB`编号可能变化，可以在启动时覆盖：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  joy_dev:=/dev/input/js0 \
  left_port:=/dev/ttyUSB0 \
  right_port:=/dev/ttyUSB1 \
  mid_port:=/dev/ttyUSB2
```

工控机部署时再用稳定的`/dev/serial/by-id/...`路径替换三个串口参数。

仿真和实机默认都使用`0.20 m/s`平移、`0.02 m/s`升降。
实机控制器与Servo串口均以30 Hz运行，并使用33 ms位置插值。
第一次架空测试如需临时降速，可以显式覆盖：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  max_linear_speed:=0.01 \
  max_vertical_speed:=0.003
```

当前工控机上三块板依次为`left=/dev/ttyUSB1`、`right=/dev/ttyUSB2`、
`mid=/dev/ttyUSB0`。如果无法在30 Hz内完成整机连杆碰撞计算，使用下面的
完整指令临时关闭这一项：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  joy_dev:=/dev/input/js0 \
  left_port:=/dev/ttyUSB1 \
  right_port:=/dev/ttyUSB2 \
  mid_port:=/dev/ttyUSB0 \
  enable_link_collision_check:=false
```

该参数默认是`true`。关闭后仍保留关节限位、足端工作空间投影和足端间距
检查，但不再阻止连杆之间或连杆与机身之间的自碰撞。只能在机器人架空、
周围无障碍且已确认运动方向正确时使用。需要恢复连杆碰撞检查时，删除
`enable_link_collision_check:=false`或将其改成`true`。

## ROS仿真

带界面的手柄模式：

```bash
roslaunch grasp_hexapod_control run_sim_ros.launch
```

带界面的导航模式：

```bash
roslaunch grasp_hexapod_control run_sim_ros.launch mode:=navigation
```

无界面运行：

```bash
roslaunch grasp_hexapod_control run_sim_ros.launch headless:=true
```

Isaac Gym不在默认位置时：

```bash
roslaunch grasp_hexapod_control run_sim_ros.launch \
  isaacgym_python_path:=/path/to/isaacgym/python
```

ROS仿真默认使用`/usr/bin/python3`，因为它同时具备ROS的`rospy/rospkg`
和Isaac Gym所需的Python 3.8 ABI。原Python直启链路才使用conda环境。
ROS仿真默认使用60 Hz控制器和60 Hz Isaac执行节拍，与原Python仿真一致。
实机舵机节点仍按30 Hz发送带33 ms运动时间的指令。若只想观察30 Hz
零阶保持对Isaac位置驱动的影响，可以显式运行：

```bash
roslaunch grasp_hexapod_control run_sim_ros.launch actuator_rate_hz:=30.0
```

原来的非ROS仿真链路仍然保留：

```bash
python src/grasp_hexapod_control/scripts/run_sim.py
```

## Isaac 攀爬区间回放

仅 Python/Isaac 的 compact 攀爬预览支持闭区间参数，`C1` 到 `C35` 是固定数组
别名，亦可传运行时阶段名：

```bash
python src/grasp_hexapod_control/scripts/run_sim.py \
  --climb-scene --climb-from C13 --climb-to C15 \
  --climb-speed 4 --climb-joint-speed 3
```

场景会在 C13 精确入口等待 X；等待期间保持该入口快照，不运行普通 APPROACH。
`--climb-metrics path.json` 可输出逐阶段 simulation-only 指标。这些指标不参与
阶段门限，且不构成接触、承载或稳定性证明。上述三个参数不属于 ROS launch 或
实机攀爬接口。

## 导航模式目标

导航模式固定使用`config/approach_fixed.json`中的左侧 P0 相对位姿；它由当前
`climb_compact.json`的 P0 导出，仅是仿真基线，尚未构成实机安全验证。运行时由
小蓝和机器人实时`pv_map`位姿生成“横向投影到左侧中心线 -> 沿中心线到 P0”的两段
路线，先原地对齐目标偏航。路线、转向和每个标准足印都会检查光伏板边界；每个拐点
必须在完整步态落地、停稳和复测后才会继续。没有新鲜导航位姿、落板确认或安全余量时，
A不会启动导航，机器人保持不动；推动摇杆仍可接管。

## 安全操作顺序

```text
B -> 等待回到标准站姿 -> A -> 接受运动指令
```

- B：最高优先级，停止当前行为并回到站姿。
- A：站姿初始化完成后启用或暂停运动。
- X（默认索引2）：默认禁用。只有显式传入`enable_real_climb:=true`、已完成B回站、18关节反馈和手柄新鲜、IMU及RTK/LoRa位姿新鲜，并通过compact入口关节门限时，才会从C1进入全C1--C53。实机阶段只依赖实际关节跟踪与FK足端目标误差的连续稳定门限推进；IMU/RTK相对运动偏差、角速度或失鲜会进入`CLIMB HOLD`。这些观察不是接触、承载或稳定性证明。
- Y（默认索引3）：默认禁用。只有显式传入`enable_real_dock:=true`、已完成B回站、反馈/手柄/IMU新鲜且没有运行中的攀爬时，才进入DockMode。DockMode等待稳定的完整AprilTag输入；它只把18关节候选交给同一个`run_real.py -> GraspController -> /<leg>_des`链路，未另行发布舵机或`JointTrajectory`命令。完成或失败都进入HOLD，B可随时中止。
- 导航运行时推动任一运动摇杆会取消导航并锁存为手柄控制。
- 松开摇杆不会恢复导航；按B重新初始化，再按A才会重新规划。

整机状态为`WAIT_B -> RESETTING -> HOLD <-> RUNNING`。`ApproachMode`
内部的摆动腿、支撑腿和换相状态始终保留。

实机攀爬和对接的传感器话题都必须在启动时明确提供，不会猜测相机设备号、内参或标定。常用参数包括`imu_topic`、`base_pose_topic`、`xiaolan_pose_topic`、`lock_confirmed_topic`、`dock_detections_topic`、`dock_image_topic`和`dock_camera_info_topic`；AprilTag推断与锁紧确认分别由`dock_allow_inference`和`dock_require_lock_confirmation`显式控制，默认均为`false`。例如仅在架空和已完成部署传感器验收后才显式开启：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  enable_real_climb:=true \
  imu_topic:=/your_imu \
  base_pose_topic:=/your_rtk/base_pose \
  xiaolan_pose_topic:=/your_lora/xiaolan_pose
```

## 常用检查

查看启动后会创建的节点，不实际运行：

```bash
roslaunch --nodes src/grasp_hexapod_control/launch/run_real.launch
roslaunch --nodes src/grasp_hexapod_control/launch/run_sim_ros.launch
```

检查手柄：

```bash
ls -l /dev/input/js*
rostopic echo /joy
```

如果LT/RT不是默认的4/5轴，按`/joy.axes`实测结果覆盖：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  axis_body_down:=<LT索引> \
  axis_body_up:=<RT索引>
```

不同手柄的按钮编号和轴符号可能不同。用`rostopic echo /joy`确认后，可在
实机和ROS仿真入口中使用同一组覆盖参数：

```bash
roslaunch grasp_hexapod_control run_sim_ros.launch \
  button_a:=<A索引> \
  button_b:=<B索引> \
  axis_forward_scale:=-1.0
```

四个符号参数为`axis_right_scale`、`axis_forward_scale`、
`axis_yaw_scale`和`axis_body_scale`，通常使用`1.0`或`-1.0`。
摇杆默认死区为`0.20`；死区外会重新映射为从0到最大速度的线性输出。
需要调整时，两个入口都可传入`joy_deadzone:=<0到1之间的值>`。

检查六腿反馈和目标：

```bash
rostopic echo /lf_pos
rostopic echo /lf_des
```

查看控制模式和串口参数：

```bash
rosparam get /grasp_hexapod_control/control_source
rosparam get /servo_left_node/port
rosparam get /servo_right_node/port
rosparam get /servo_mid_node/port
```

仅启动三块舵机板：

```bash
roslaunch grasp_hexapod_servo servo_three_boards.launch
```

## 文件职责

- `run_real.launch`：公共控制链加三块Servo板。
- `run_sim_ros.launch`：ROS输入加Isaac内部同步控制循环。
- `control_stack.launch`：实机复用的`joy_node`和高层控制节点，不作为日常入口。
