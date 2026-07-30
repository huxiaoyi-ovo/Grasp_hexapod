# 舵机行走轨迹延迟测试

## 数据内容

PhysX固定以240 Hz运行、每个物理帧使用2个子步。控制器以30 Hz或60 Hz
运行并输出对应CSV：

1. 标准站姿保持1.0 s。
2. 沿`base_link +y`前进若干完整步态周期，约5 s。
3. 六足落地停稳。
4. 沿`base_link +x`右移若干完整步态周期，约5 s。
5. 六足落地停稳。
6. 绕`base_link +z`正向旋转若干完整步态周期，约5 s。
7. 六足落地停稳。

每行是一帧，`frame`决定严格先后顺序，`time_s`是计划发送时间。只把
`target_*`发送给实机；`actual_*`是Isaac Gym同帧反馈，只用于仿真对照。
角度单位均为rad。

CSV中的18关节顺序固定为：

```text
lb: thigh knee ankle
lf: thigh knee ankle
lm: thigh knee ankle
rb: thigh knee ankle
rf: thigh knee ankle
rm: thigh knee ankle
```

其中`lf/lm/lb`分别表示左前、左中、左后，`rf/rm/rb`分别表示右前、
右中、右后。注意CSV不是按“左前到右后”的直观顺序排列，而是：

```text
lb, lf, lm, rb, rf, rm
```

回放脚本通过列名和ROS话题名取数据，不依赖CSV列号，因此不能删改
`target_<leg>_<joint>_joint`这些列名。

## 实机映射必须先确认

CSV保存的是URDF/控制器关节坐标`q`，不是LX-15D的物理ID或原始
`0~1000`位置。现有回放接口的语义如下：

| CSV腿名 | 机身位置 | ROS目标话题 | 消息中的关节顺序 | 当前`servo.py`假定的ID顺序 |
|---|---|---|---|---|
| `lb` | 左后 | `/lb_des` | thigh, knee, ankle | 9, 8, 7 |
| `lf` | 左前 | `/lf_des` | thigh, knee, ankle | 3, 2, 1 |
| `lm` | 左中 | `/lm_des` | thigh, knee, ankle | 6, 5, 4 |
| `rb` | 右后 | `/rb_des` | thigh, knee, ankle | 18, 17, 16 |
| `rf` | 右前 | `/rf_des` | thigh, knee, ankle | 12, 11, 10 |
| `rm` | 右中 | `/rm_des` | thigh, knee, ankle | 15, 14, 13 |

表中ID只是本项目当前驱动代码的假定值，不能直接套用到另一台实机。
协作者必须为自己的18个舵机逐个记录：

```text
控制器关节名 -> ROS话题和字段 -> 实机ID -> raw零位 -> 转向sign -> raw限位
```

`/<leg>_des`的消息格式为：

```text
[enable, thigh_pos, knee_pos, ankle_pos,
         thigh_vel, knee_vel, ankle_vel, 0, 0, 0]
```

位置单位为rad，速度单位为rad/s。当前驱动只执行位置字段，速度字段暂未参与
控制。腿的发布先后顺序不重要，腿名和每条腿内部的三个字段顺序才重要。

当前`servo.py`使用：

```text
raw = 500 + q * 750 / pi
```

它假设所有舵机`q=0`都对应raw 500，且所有舵机的raw增大方向都与控制器
关节正方向相同，没有逐关节零位偏置和方向参数。若协作者实机安装不同，应改为：

```text
raw = raw_zero + servo_direction * q * 750 / pi
q = servo_direction * (raw - raw_zero) * pi / 750
```

`servo_direction`必须通过逐关节小角度测试确定，不能用舵机ID、左右腿位置或
URDF中的`JOINT_AXIS_SIGNS`猜测。CSV已经是控制器关节坐标，不能再盲目乘一次
运动学符号。

固定240 Hz物理仿真生成的正式轨迹中，ankle最大绝对目标约为117.4°，已经
非常接近LX-15D标称±120°范围。任何实际零位偏差都可能使raw目标超过
`0~1000`；当前驱动会直接截断，截断后轨迹不再等价，也会污染延迟结果。
因此必须先检查全部目标经过实机映射后的raw最小值和最大值。

## 1. 在Isaac Gym生成轨迹

```bash
/home/hxy/anaconda3/envs/hexapod_rl_env/bin/python \
  src/grasp_hexapod_control/scripts/run_sim.py \
  --record-servo-trace logs/servo_walk_trace_physics240_control60.csv \
  --physics-rate 240 \
  --control-rate 60 \
  --headless
```

录制模式使用固定指令，不需要手柄。

生成可直接对比的30 Hz版本：

```bash
/home/hxy/anaconda3/envs/hexapod_rl_env/bin/python \
  src/grasp_hexapod_control/scripts/run_sim.py \
  --record-servo-trace logs/servo_walk_trace_physics240_control30.csv \
  --physics-rate 240 \
  --control-rate 30 \
  --headless
```

两组实验的PhysX配置完全相同。60 Hz控制器每4个物理帧读取一次反馈并更新
关节目标，30 Hz控制器每8个物理帧更新一次；中间物理帧保持上一次目标。
CSV只记录控制器真正更新的时刻，不记录保持目标的物理子帧。

30 Hz版本使用`GraspController.dt=1/30 s`重新执行完整控制器，不是对60 Hz
结果抽帧。换相共同支撑的目标时间为50 ms，并量化到最接近的整数控制帧：
60 Hz为3帧/50 ms，30 Hz为2帧/66.7 ms。其他以秒表示的步态参数保持相同；
量化差异是控制频率本身不可避免的影响。

| 控制频率 | 单个完整步态周期 | 每个动作的周期数 | 每个动作时间 |
|---|---:|---:|---:|
| 60 Hz | 0.700 s | 7 | 4.900 s |
| 30 Hz | 0.733 s | 7 | 5.133 s |

两份CSV评估的是“控制器在不同频率下生成的位置目标，通过ROS发送给实机后，
舵机和通信链路的跟踪表现”。由于回放时没有把实机反馈重新送回控制器，它
不是完整实机闭环控制器的频率对比；完整闭环对比需要以后让`run_real.py`
分别以30 Hz和60 Hz实时读取舵机反馈并重新解算。

## 2. 实机测试顺序

实机必须架空或可靠支撑，并准备立即断电。首次测试先只启用已经确认方向、
零位和端口的一侧，不能直接让未验证的18个舵机带载行走。
轨迹开头的`initialize`表示保持标准站姿，不负责把任意实机姿态瞬间拉到
标准站姿；回放前必须先用正常的平滑初始化流程进入标准站姿。

```bash
catkin_make
source devel/setup.bash
roslaunch grasp_hexapod_servo servo_dual_side.launch
```

先确认所有已启用腿都有连续反馈：

```bash
rostopic hz /lf_pos
rostopic echo -n 1 /lf_pos
```

当前launch文件默认只启动左侧节点；测试18个舵机前，必须确认右侧串口后再
启用右侧节点。不要凭默认`/dev/tty0`直接连接。

当前驱动还存在两个需要协作者验证或修改的实现点：

1. `power_on`是单侧节点共用的一个布尔值，使能回调只操作当前腿的第一个ID，
   不能据此确认该侧9个舵机已经全部加载。
2. 每次收到目标都会执行`rospy.loginfo`，60 Hz六腿回放会产生大量日志，
   应删除或改成只打印第一次，否则日志开销会影响延迟测量。

此外，驱动按照腿和关节逐个串口读取、再逐个发送目标，不是18个舵机硬件同步
写入。因此不同舵机的反馈天然存在总线扫描时间差，最终结果应分别报告每个
关节的延迟，而不是只给一个全机平均值。

开始保存目标和反馈。这里使用rosbag只是记录实机测试结果，Isaac Gym导出的
测试输入仍然是CSV：

```bash
mkdir -p logs
rosbag record -O logs/servo_latency \
  /lb_des /lf_des /lm_des /rb_des /rf_des /rm_des \
  /lb_pos /lf_pos /lm_pos /rb_pos /rf_pos /rm_pos
```

另开终端回放：

```bash
source devel/setup.bash
rosrun grasp_hexapod_servo servo_trace_replay.py \
  _trace:="/你复制给他的路径/servo_walk_trace.csv" \
  _start_delay:=2.0
```

回放结束后先停止回放，再停止rosbag，最后按实机安全流程卸载舵机。

## 3. 延迟含义

比较每个`target_*`与相应`/<leg>_pos`反馈曲线，通过时间平移后误差最小或
互相关峰值求延迟。该结果是控制目标发布、ROS回调、串口轮询、LX-15D内部
执行和位置反馈共同形成的端到端跟踪延迟，不等同于单个串口数据包传输时间。

当前目标和反馈消息没有`Header`，rosbag时间是消息到达记录器的时间。若后续
需要拆分ROS、串口和机械响应延迟，应给驱动增加发送/反馈时间戳和帧序号。
