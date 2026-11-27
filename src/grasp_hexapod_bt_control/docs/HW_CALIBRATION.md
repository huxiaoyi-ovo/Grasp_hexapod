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

> 启动 launch 时**不要**在命令行传 `tag_id:=` / `tag_size_m:=`：向导把这两个值写进 launch 的默认值，命令行参数会把它们覆盖掉。

## 2. 向导的 6 步

每个提示下：**回车**表示继续，输入 **s** 跳过这一步，输入 **q** 退出。

| 步骤 | 你要做什么 | 向导做什么 |
|---|---|---|
| 1 tag 配置 | 输入小蓝被攀爬那一面上 tag 的编号和黑框外边长（默认 255 / 0.050，已在实机确认） | 写入 `phase1_visual.yaml` 的 `tag_guidance_id`，以及 `full_mission.launch` 的 `tag_id`/`tag_size_m`。**有改动时向导会退出**：在终端 A 重启 launch，再运行 `hw_calibrate.py --from topics` 接着做（roslaunch 自带的 ROS master 会随 launch 一起重启，向导不能跨重启继续） |
| 2 话题检查 | 不用动 | 检查 YOLO、tag 检测、IMU、舵机反馈有没有数据 |
| 3 YOLO 检查 | 按提示把机器人放在小蓝前 1 m、2 m | 报告能否检出小蓝、有没有有效深度。检不出或没有深度，就不能从这个距离起步 |
| 4 IMU 方向 | 机器人静止 → 从上往下看，用手**逆时针**转约 30°，再确认方向 | 自动设置 `tag_imu_yaw_sign`（转反会写反，所以会再问一次方向）。没有 IMU 时自动跳过（tag 一路可见时用不到 IMU） |
| 5 **P0 示教**（必须做） | 先在看板点 **home**（行走站姿），再把机器人摆到攀爬起点 P0：机器人正面朝向小蓝被攀爬的那一面（贴 tag 的那一面）、机身与立面垂直；机身中心线对准这一面的中心（攀爬配置里偏右约 5 mm，可视为居中）；**两只前脚脚尖离立面约 2 cm**（机身中心离立面约 19.5 cm）；静止、六腿着地 | 检查是否在行走站姿，采集 30 帧 tag，检查坐标系、抖动和深度（P0 处深度约 0.15–0.20 m），确认后写入 `tag_ref_x_m`、`tag_p0_z_m`、`tag_ref_yaw_rad` |
| 6 偏航-距离检查（可选） | 从 P0 沿直线后退到约 0.30 m、0.45 m | 检查 tag 偏航随距离的变化 ≤1°。超过说明 tag 打印变形、边长不对或相机标定有问题，先报告，不要上机 |

只做某一步：`rosrun grasp_hexapod_bt_control hw_calibrate.py --step teach`；
从某一步往后做：`--from topics`。
（可选的步骤名：`tag`、`topics`、`yolo`、`imu`、`teach`、`yawcheck`）。
数字输错时会重新询问；每一步都可以输入 `s` 跳过。

## 3. 标定完成后

1. 在终端 A 按 **Ctrl-C** 停掉，再重新启动 `full_mission.launch`。控制节点只在启动时读这些参数，不重启就不会生效。
2. 把机器人放到小蓝前 1–2 m（朝向可以偏，看不到小蓝时会原地转身搜索）。
3. 在看板上依次点 **home → spin_search → approach**。
4. 成功时 approach 会返回 `tag alignment complete: ready for climb`，机器人停在 P0。
   **只验证 approach 时，不要点 climb。**

## 标定值与 git

向导改的是仓库里受 git 管理的 `config/phase1_visual.yaml` 和 `launch/full_mission.launch`。结束时它会打印全部标定值。
- 以后在机器人上更新代码：先 `git stash`，再 `git pull origin test`，最后 `git stash pop`（标定值会保留）。
- 最好把打印出的标定值发给开发同学入库，之后就不用每次 stash 了。

## 常见问题

| 现象 | 处理 |
|---|---|
| 第 5 步提示"画面里看到的是其他编号 [N]" | tag 编号填错了：回到第 1 步（`--step tag`）改成 N，并重启 launch |
| 第 5 步收不到 tag | tag 不在画面里，或检测器没起来。先跑 `--step topics` |
| 第 5 步提示"tag 坐标系是 [...]" | 检测器订阅的图像坐标系不是 `camera_color_optical_frame`，控制器会拒收所有 tag。检查 `full_mission.launch` 里 apriltag 节点的图像话题 |
| approach 结束时显示 `no usable YOLO or tag for 5.0 s (last YOLO z=…, last tag z=…)` | 接近逻辑是"tag 优先、YOLO 兜底、都没有就停下等"：0.5 m 内连续 3 帧 tag 就用 tag；tag 丢失 0.3 s 就退回 YOLO（YOLO 只开到 0.20 m）；两者都用不了就原地停下，等满 5 s 才失败。看报出的两个距离：tag 距离停在某个值，说明 tag 在那里看丢了（视野、光线、遮挡）；YOLO 距离是 0.20 m，说明粗引导已经到底、只差 tag |
| 想恢复原配置 | 把 `config/phase1_visual.yaml.bak_<时间戳>`（或 launch 的 `.bak`）复制回原文件名 |

> 换了行走站姿（相机高度变了）或重新贴了 tag，都要重做第 5 步。
