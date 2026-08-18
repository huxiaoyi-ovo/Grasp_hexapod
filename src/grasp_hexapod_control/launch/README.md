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

串口名称可在启动时覆盖：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  joy_dev:=/dev/input/js0 \
  left_port:=/dev/ttyTHS0 \
  right_port:=/dev/ttyACM0
```

默认即为 Jetson 板载串口`/dev/ttyTHS0`（left）和 USB 串口`/dev/ttyACM0`（right），
串口名称变化时再显式覆盖。

仿真和实机默认都使用`0.20 m/s`平移、`0.02 m/s`升降。
实机控制器与Servo串口均以30 Hz运行，并使用33 ms位置插值。
第一次架空测试如需临时降速，可以显式覆盖：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  max_linear_speed:=0.01 \
  max_vertical_speed:=0.003
```

当前两块板依次为`left=/dev/ttyTHS0`、`right=/dev/ttyACM0`。如果无法在
30 Hz内完成整机连杆碰撞计算，使用下面的完整指令临时关闭这一项：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  joy_dev:=/dev/input/js0 \
  left_port:=/dev/ttyTHS0 \
  right_port:=/dev/ttyACM0 \
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
ROS仿真默认使用30 Hz控制器、30 Hz执行器写入和240 Hz物理步进，与实机目标
节拍一致。若需要60 Hz诊断，必须同时覆盖控制器和执行器频率，并保持物理频率可整除：

```bash
roslaunch grasp_hexapod_control run_sim_ros.launch \
  controller_rate_hz:=60.0 actuator_rate_hz:=60.0 physics_rate_hz:=240.0
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
- X（默认索引2）：`run_real.launch`默认启用。完成B回站、Joy和18关节反馈新鲜并通过compact入口关节门限后，按X从C1启动完整C1--C35诊断回放；A暂停或恢复，B可随时中止。阶段只按实际关节跟踪与FK足端目标误差的连续稳定门限推进。IMU和RTK/LoRa不要求；若启动瞬间两者恰好有效，才启用可选相对运动监控。关节反馈门限不证明接触、承载或稳定性；可用`enable_real_climb:=false`显式关闭该入口。
- Y（默认索引3）：`run_real.launch`默认启用。完成B回站、Joy和18关节反馈新鲜且没有运行中的攀爬时，按Y进入DockMode；不依赖IMU或RTK。DockMode等待稳定的完整AprilTag输入；它只把18关节候选交给同一个`run_real.py -> GraspController -> /<leg>_des`链路，未另行发布舵机或`JointTrajectory`命令。完成或失败都进入HOLD，B可随时中止。
- 导航运行时推动任一运动摇杆会取消导航并锁存为手柄控制。
- 松开摇杆不会恢复导航；按B重新初始化，再按A才会重新规划。

整机状态为`WAIT_B -> RESETTING -> HOLD <-> RUNNING`。`ApproachMode`
内部的摆动腿、支撑腿和换相状态始终保留。

IMU和RTK/LoRa话题保留为攀爬诊断回放的可选相对运动监控接口，不是启动或推进条件。DOCK使用独立底部USB链：`/dock_camera/image_raw -> image_proc -> /dock_camera/image_rect_color -> /dock/tag_detections`；它与顶部Orbbec及`xiaolan_tag_system.launch`严格分离。

先安装相机与标定工具：

```bash
sudo apt install ros-noetic-usb-cam ros-noetic-image-proc ros-noetic-apriltag-ros \
  ros-noetic-nodelet ros-noetic-camera-calibration python3-opencv python3-yaml
```

必须以最终实机分辨率和像素格式完成标定。下面的棋盘格`8x6`、格长`0.025 m`只是示例，必须替换为实际棋盘参数：

```bash
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 \
  image:=/dock_camera/image_raw camera:=/dock_camera
```

保存内参 YAML 后，使用绝对路径让USB相机读取它；相机外参必须以相机光学帧到卡紧机构为实测值。Tag size 是黑白边界的有效边长，不是标签纸张外沿。推荐把`dock_system.yaml`复制为部署文件，写入实测外参与`real_calibrated: true`，并在同一条启动命令中把该绝对路径同时交给 detector 与控制器：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  dock_camera_info_url:=file:///absolute/path/dock_camera.yaml \
  dock_system_config:=/absolute/path/dock_system.yaml
```

仅在机器人架空状态下检查节点、频率和检测输入：

```bash
roslaunch --nodes src/grasp_hexapod_control/launch/dock_tag_system.launch
rostopic hz /dock_camera/image_rect_color
rostopic echo -n1 /dock_camera/camera_info
rostopic echo -n1 /dock/tag_detections
```

`run_real.launch`默认启动该链，可用`start_dock_perception:=false`关闭。默认未标定时Y会被`dock_require_real_calibrated:=true`阻止；仅在架空调试时可显式传入`dock_allow_uncalibrated:=true`。视觉检测、对准或锁紧话题通过都不证明物理锁紧、接触、承载或整机安全。`dock_allow_inference`和`dock_require_lock_confirmation`仍默认`false`；未要求锁紧确认时，`SUCCESS`只表示对准流程完成，不证明物理锁紧。锁紧确认只接受本次Y启动后、`dock_lock_confirmation_max_age_s`（默认0.5 s）内的新消息。若要关闭默认攀爬诊断回放入口：

```bash
roslaunch grasp_hexapod_control run_real.launch enable_real_climb:=false
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
```

仅启动两块舵机板：

```bash
roslaunch grasp_hexapod_servo servo_two_boards.launch
```

## 文件职责

- `run_real.launch`：公共控制链加两块Servo板。
- `run_sim_ros.launch`：ROS输入加Isaac内部同步控制循环。
- `control_stack.launch`：实机复用的`joy_node`和高层控制节点，不作为日常入口。
