# 实机 approach 标定（`hw_calibrate.py`）

目标：让机器人从 1–2 m 外，经 YOLO 粗接近、AprilTag 对准，停到攀爬起点 P0。
标定向导一边提示一边读传感器，自动把结果写进配置。它**只读话题、不发运动指令**，
每次改文件前都会自动备份为 `<文件名>.bak_<时间戳>`。

## 0. 准备（一次）

```bash
cd <仓库根目录>
git pull origin test
catkin_make            # 或你平时的编译命令
source devel/setup.bash
```

## 1. 启动

终端 A：

```bash
roslaunch grasp_hexapod_bt_control full_mission.launch show_windows:=false
```

终端 B（运行向导）：

```bash
rosrun grasp_hexapod_bt_control hw_calibrate.py
```

## 2. 向导的 6 步

每个提示下：**回车**表示继续，输入 **s** 跳过这一步，输入 **q** 退出。

| 步骤 | 你要做什么 | 向导做什么 |
|---|---|---|
| 1 tag 配置 | 输入小蓝正面那张 tag 的编号和黑框外边长 | 写入 `phase1_visual.yaml` 的 `tag_guidance_id`，以及 `full_mission.launch` 的 `tag_id`/`tag_size_m`。**有改动时会提示你重启终端 A 的 launch** |
| 2 话题检查 | 不用动 | 检查 YOLO、tag 检测、IMU、舵机反馈有没有数据 |
| 3 YOLO 检查 | 按提示把机器人放在小蓝前 1 m、2 m | 报告能否检出小蓝、深度是否合理。检不出就不能从这个距离起步 |
| 4 IMU 方向 | 机器人静止 → 从上往下看，用手**逆时针**转约 30° | 自动设置 `tag_imu_yaw_sign`。没有 IMU 时自动跳过（tag 一路可见时用不到 IMU） |
| 5 **P0 示教**（必须做） | 把机器人摆到攀爬起点：**行走站姿**（home 后的默认站姿）、正对小蓝、前足离小蓝立面约 3 cm、横向对准攀爬位置 | 采集 30 帧 tag，检查抖动和深度（P0 处深度约 0.15–0.20 m），确认后写入 `tag_ref_x_m`、`tag_p0_z_m`、`tag_ref_yaw_rad` |
| 6 偏航-距离检查（可选） | 从 P0 沿直线后退到约 0.30 m、0.45 m | 检查 tag 偏航随距离的变化 ≤1°。超过说明 tag 打印变形、边长不对或相机标定有问题，先报告，不要上机 |

只做某一步：`rosrun grasp_hexapod_bt_control hw_calibrate.py --step teach`
（可选的步骤名：`tag`、`topics`、`yolo`、`imu`、`teach`、`yawcheck`）。

## 3. 标定完成后

1. 在终端 A 按 **Ctrl-C** 停掉，再重新启动 `full_mission.launch`。控制节点只在启动时读这些参数，不重启就不会生效。
2. 把机器人放到小蓝前 1–2 m（朝向可以偏，看不到小蓝时会原地转身搜索）。
3. 在看板上依次点 **home → spin_search → approach**。
4. 成功时 approach 会返回 `tag alignment complete: ready for climb`，机器人停在 P0。
   **只验证 approach 时，不要点 climb。**

## 常见问题

| 现象 | 处理 |
|---|---|
| 第 5 步提示"画面里看到的是其他编号 [N]" | tag 编号填错了：回到第 1 步（`--step tag`）改成 N，并重启 launch |
| 第 5 步收不到 tag | tag 不在画面里，或检测器没起来。先跑 `--step topics` |
| approach 结束时显示 `coarse stop reached without tag handoff` | YOLO 已经走近，但 0.5 m 以内没有连续 5 帧 tag 检测。检查 tag 是否在视野内、光线是否稳定 |
| approach 结束时显示 `imu unavailable for blind finish` | tag 在途中丢失，同时又没有 IMU。机器人会安全停下。保证 tag 一路可见，或者接上 IMU |
| 想恢复原配置 | 把 `config/phase1_visual.yaml.bak_<时间戳>`（或 launch 的 `.bak`）复制回原文件名 |

> 换了行走站姿（相机高度变了）或重新贴了 tag，都要重做第 5 步。
