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
实机控制器以60 Hz运行，Servo串口以30 Hz运行并使用33 ms位置插值。
第一次架空测试如需临时降速，可以显式覆盖：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  max_linear_speed:=0.01 \
  max_vertical_speed:=0.003
```

当前工控机上三块板依次为`left=/dev/ttyUSB1`、`right=/dev/ttyUSB2`、
`mid=/dev/ttyUSB0`。如果无法在60 Hz内完成整机连杆碰撞计算，使用下面的
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

## 导航模式参数

导航模式需要经过实测的左右接近位姿，每个位姿都是按行展开的4×4矩阵：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  mode:=navigation \
  xiaolan_from_left_base:='[r00,r01,r02,tx,r10,r11,r12,ty,r20,r21,r22,tz,0,0,0,1]' \
  xiaolan_from_right_base:='[r00,r01,r02,tx,r10,r11,r12,ty,r20,r21,r22,tz,0,0,0,1]'
```

没有配置外参、导航位姿过期、没有落板确认或路径越界时，A不会启动导航，
机器人保持不动；推动摇杆仍可接管。

## 安全操作顺序

```text
B -> 等待回到标准站姿 -> A -> 接受运动指令
```

- B：最高优先级，停止当前行为并回到站姿。
- A：站姿初始化完成后启用或暂停运动。
- X：仅在 Isaac `--climb-scene` 中启动 compact 攀爬预览；`--climb-start` 会自动启动。ROS与实机仍禁止攀爬启动。
- 导航运行时推动任一运动摇杆会取消导航并锁存为手柄控制。
- 松开摇杆不会恢复导航；按B重新初始化，再按A才会重新规划。

整机状态为`WAIT_B -> RESETTING -> HOLD <-> RUNNING`。`ApproachMode`
内部的摆动腿、支撑腿和换相状态始终保留。

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
