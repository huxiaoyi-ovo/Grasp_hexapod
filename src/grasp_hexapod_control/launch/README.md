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

第一次实机行走建议显式使用低速：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  max_linear_speed:=0.01 \
  max_vertical_speed:=0.003
```

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
  python_executable:=/path/to/conda/env/bin/python \
  isaacgym_python_path:=/path/to/isaacgym/python
```

原来的非ROS仿真链路仍然保留：

```bash
python src/grasp_hexapod_control/scripts/run_sim.py
```

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
- X：预留攀爬模式，当前只暂停。
- Y：预留对接模式，当前只暂停。
- 导航运行时推动任一运动摇杆会取消导航并锁存为手柄控制。
- 松开摇杆不会恢复导航；按B重新初始化，再按A才会重新规划。

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
- `run_sim_ros.launch`：公共控制链加Isaac Gym执行后端。
- `control_stack.launch`：内部复用的`joy_node`和高层控制节点，不作为日常入口。
