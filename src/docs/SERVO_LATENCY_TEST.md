# 舵机端到端延迟测试

该测试用于比较控制目标和LX-15D位置反馈，不替代正常的B→站立→A流程。

## 生成确定性轨迹

```bash
/home/artrc/miniconda3/envs/grasp_hexapod/bin/python \
  src/grasp_hexapod_control/scripts/run_sim.py \
  --record-servo-trace logs/servo_walk_trace.csv \
  --physics-rate 240 \
  --control-rate 30 \
  --headless
```

CSV关节顺序固定为：

```text
lb, lf, lm, rb, rf, rm
```

每条腿内部为`thigh, knee, ankle`，全部角度单位为rad。回放脚本按列名读取，
不把CSV列号解释为物理舵机ID。

## 实机映射

| 腿 | 目标话题 | ID |
|---|---|---|
| lf | `/lf_des` | 1,2,3 |
| lm | `/lm_des` | 4,5,6 |
| lb | `/lb_des` | 7,8,9 |
| rf | `/rf_des` | 10,11,12 |
| rm | `/rm_des` | 13,14,15 |
| rb | `/rb_des` | 16,17,18 |

换算为：

```text
raw = 500 + direction * degrees(q) * 1000 / 240
q = direction * radians((raw - 500) / (1000 / 240))
```

默认方向在`servo.py::SIDE_CONFIG`中。不能再额外乘运动学
`JOINT_AXIS_SIGNS`。

## 测试步骤

1. 单独启动三块Servo节点：

```bash
roslaunch grasp_hexapod_servo servo_three_boards.launch
```

2. 在另一终端启动公共控制链，架空机器人并按B进入站姿：

```bash
roslaunch grasp_hexapod_control control_stack.launch
```

3. 站姿稳定后只停止第二个终端的公共控制链，保持第一个终端的三块
   Servo节点运行并维持最后站姿。
4. 记录目标和反馈：

```bash
rosbag record -O logs/servo_latency \
  /lb_des /lf_des /lm_des /rb_des /rf_des /rm_des \
  /lb_pos /lf_pos /lm_pos /rb_pos /rf_pos /rm_pos
```

5. 回放轨迹：

```bash
rosrun grasp_hexapod_servo servo_trace_replay.py \
  _trace:=logs/servo_walk_trace.csv \
  _start_delay:=2.0
```

回放脚本会直接发布`power=1`，因此只能在ID、方向、零位和站姿均已验证后使用。
延迟结果包含ROS、串口轮询、舵机内部执行和机械响应，应按每个关节分别计算。
