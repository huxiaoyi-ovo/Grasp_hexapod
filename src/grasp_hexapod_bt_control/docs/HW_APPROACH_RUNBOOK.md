# 实机操作手册：approach（接近攀爬点）测试

目标：机器人从小蓝前 1–2 m 出发，经 YOLO 粗引导、AprilTag 精对准，停到攀爬起点 P0。
标定细节见 [HW_CALIBRATION.md](HW_CALIBRATION.md)。标定程序启动时也会打印完整步骤。

手柄键位：**B** 停止，并回到站立（同时取消正在执行的序列）；**LB** 测 approach；**A** 行走；
**按住 START 2 秒** 跑全流程（含攀爬）；**X** 攀爬；**Y** 对接。

---

## 0. 准备（只做一次）

| # | 操作 | 说明 |
|---|---|---|
| 0.1 | 场地 | 平地，小蓝周围留出 2.5 m 空间；小蓝**被攀爬那一面**贴好 **255 号 tag（tag36h11，黑框外边长 50 mm）** |
| 0.2 | 机器人 | 电量充足，舵机上电，手柄已连接 |
| 0.3 | 更新代码 | `cd <仓库根目录> && git pull origin test && catkin_make && source devel/setup.bash` |

> [!WARNING]
> 机器人上的仓库有本地改动（例如上次的标定值）时，pull 会报错。先 `git stash`，pull 之后再 `git stash pop`。

## 1. 启动（4 个终端，每个先 `source devel/setup.bash`）

| 终端 | 命令 | 作用 |
|---|---|---|
| 1 | `roscore` | ROS master 单独起，重启 launch 时它不受影响 |
| 2 | `roslaunch grasp_hexapod_bt_control full_mission.launch show_windows:=false start_dock_perception:=false` | 相机、YOLO、tag 检测、控制、舵机 |
| 3 | `rosrun grasp_hexapod_bt bt_dashboard.py` | 看板：浏览器打开 `http://<机器人IP>:8080` |
| 4 | 第 3 节用 | 标定程序 |

终端 2 启动后等约 10 秒，再**按手柄 B**（或在看板点 home），机器人会站成行走站姿。

## 2. 起步前自检（1 分钟）

| # | 看什么 | 正常表现 |
|---|---|---|
| 2.1 | 终端 2 有没有红字报错 | 没有持续刷屏的 `ERROR` |
| 2.2 | 机器人站立 | 六条腿都着地，机身离地约 9.6 cm |
| 2.3 | 行走手感（可选） | 按 **A**，推摇杆慢慢走几步：动作平顺，没有卡顿或抽动；走完按 **B** |

## 3. 标定（终端 4，约 5 分钟）

```bash
rosrun grasp_hexapod_bt_control hw_calibrate.py
```

每个提示下：**回车**继续，输入 **s** 跳过，输入 **q** 退出。

| 步骤 | 你做什么 | 应该看到 |
|---|---|---|
| 1 tag 配置 | 两次直接**回车**（默认 255 / 0.050） | "配置一致，无需改动" |
| 2 话题检查 | 不用动 | 前 3 项是 `[OK ]`；IMU 显示"无"也没关系 |
| 3 YOLO 检查 | 把机器人搬到小蓝前约 **1 m**，相机对着小蓝，按回车；再搬到约 **2 m**，按回车 | `✓ 1.0 m … 深度 z≈1.0`。**出现 ✗ 先停下，把这一行发给开发** |
| 4 IMU 方向 | 没有 IMU 会自动跳过 | — |
| **5 P0 示教（必须做）** | 先按 **B** 站好，再把机器人摆到攀爬起点：<br>• 正对贴 tag 的那一面，机身与立面垂直<br>• 左右居中<br>• **两只前脚脚尖离立面约 2 cm**<br>摆好后按回车；问是否写入时答 `y` | "站姿正确"<br>深度 z 约 0.15–0.20 m<br>抖动：x、z ≤3 mm，偏航 ≤0.5°<br>"已写入" |
| 6 偏航检查 | 输入 **s** 跳过（可选） | — |

标定值**已经自动写入**配置文件，结尾打印的数值只是留个记录，不用手动复制。

## 4. 让标定生效

1. 终端 2 按 **Ctrl-C**，等它退出后，再执行一次同样的 `roslaunch …` 命令。控制节点只在启动时读参数。
2. 等约 10 秒，按手柄 **B**，让机器人站好。

终端 1（roscore）和终端 3（看板）不用重启。

## 5. approach 测试（由近到远，一轮成功再做下一轮）

| 轮次 | 起始位置 | 操作 |
|---|---|---|
| 5.1 | 正对小蓝，约 **0.6 m** | 按一下 **LB** |
| 5.2 | 约 **1 m**，左右偏 10 cm 左右 | 按一下 **LB** |
| 5.3 | 约 **1.5–2 m**，朝向偏 20°–60° | 按一下 **LB**。看不到小蓝时，机器人会先原地转身去找 |

每轮的正常过程：

1. **原地转向**，对准小蓝（自转搜索）；
2. **慢慢前进**：YOLO 粗引导，远处约 8 cm/s，1 m 以内 3 cm/s；
3. 走到约 **0.5 m** 时，终端 2 打印 `AprilTag handoff (id 255)`，切换到 tag 引导；
4. 先调整左右位置和角度，再以 2 cm/s 推进；
5. 停下，打印 `tag alignment complete: ready for climb`。

每轮结束后**量一下**：前脚脚尖离立面的距离，机身是否居中、是否正对。和示教时摆的位置比，差在 1–2 cm 以内就算合格。

> [!CAUTION]
> - **随时停止：按 B**。
> - 只测 approach 时，**不要按 X**，也**不要按住 START**，这两个都会开始实机攀爬。

## 6. 出问题时

| 终端 2 / 看板显示 | 含义 | 怎么办 |
|---|---|---|
| `no usable YOLO or tag for 5.0 s (last YOLO z=…, last tag z=…)` | 停下等了 5 秒，YOLO 和 tag 都用不了 | 看最后那两个距离：tag 距离停在某个值，说明在那里看丢了 tag（检查视野、光线、遮挡）；YOLO 显示 0.20，说明粗引导已经走到头，只差 tag |
| `approach: tag lost …, back to YOLO / wait` | tag 闪断，已退回粗引导 | 正常，会自动恢复 |
| `approach timeout (120 s)` | 总时间超过 120 秒 | 起点太远，或一直走走停停 |
| 一直原地转，找不到小蓝 | YOLO 没检出 | 单独检查 YOLO：`hw_calibrate.py --step yolo` |
| 停的位置偏差大 | 示教位置不准 | 重新摆正，只重做示教：`hw_calibrate.py --step teach`，然后做第 4 节 |

反馈时请把**失败信息**，以及终端 2 里含 `approach:` 和 `tag mode=` 的几行发给开发。

## 7. 可选：测降负载效果（approach 跑通后再做）

1. 默认配置下，开一个新终端运行：
   `python3 src/xiaolan_yolo_detection/tests/resource_profile.py --label base --out /tmp/base.json`
2. 终端 2 加 `sparse_depth_align:=true` 重启 launch，按 **B** 站好，再按 **LB** 跑一次 approach，确认还能成功。
3. 再量一次：`… --label sparse --out /tmp/sparse.json`。
4. 把两次输出的 **TOTAL** 行和 **camera** 行发给开发。

`sparse_depth_align`：相机驱动不再做整幅深度对齐（这一项约占一个 CPU 核的 87%），改成只换算 YOLO 框附近的深度点。实机对比确认之前，默认关闭。

## 8. 以后：全流程（含攀爬和对接）

approach 稳定之后再做。攀爬配置 v16.7 **还没上过实机**，第一次先悬挂空跑：

- 手柄**按住 START 2 秒**，依次执行 home → 搜索 → 接近 → 攀爬 → 对接；
- 全程有人看着，随时准备按 **B**。
