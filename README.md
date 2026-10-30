# Grasp Hexapod

这是一个基于 ROS Noetic 的六足机器人工作空间。项目使用手写控制器，支持 Isaac Gym 仿真和两块 LX-15D 舵机板实机运行。

## 目录

- `src/grasp_hexapod_control/`：步态、运动学、安全控制和仿真/实机入口。
- `src/grasp_hexapod_description/`：URDF、网格、RViz 和 Gazebo 配置。
- `src/grasp_hexapod_servo/`：LX-15D 两板驱动和串口协议（Python `servo.py` 与 C++ `servo_node` 两个后端，接口一致；`/grasp_hexapod/gripper_act` 仅由 C++ 后端提供）。
- `src/grasp_hexapod_msgs/`：自建消息和服务定义（msg/srv），供各包统一引用。
- `src/reference/`：参考代码，不参与运行。

仿真和实机共用 `GraspController`：前者在 Isaac 控制帧内运行，后者通过 ROS 读取反馈并发布关节目标。

控制包按职责命名：`run_*.py` 是执行入口，`control.py`、`*_mode.py` 和
`kinematics.py` 是运行时核心，`scripts/utils/` 只放共享库；
`scripts/tools/` 保留 `analyze_workspace.py` 和离线候选重建工具；旧规划、trace 和长 validator 在
`src/reference/climb_history/`，不进入实时控制链也不再维护。

## 开发协作

- `test` 是集成分支；从 `test` 创建功能分支，合并目标也是 `test`，不要直接在 `main` 开发。
- 在仓库根目录构建和运行检查。
- URDF 和 CAD 导出文件只做必要的小修改；如果以后重新导出，需同步保留这些手工修改。

### 协作者提交准则

- 最小改动：只实现当前需求，不顺带重构、格式化或修改无关代码。
- 控制文件：优先复用和合并现有文件，避免拆出零散模块；默认修改不超过 3 个文件，确需新增时说明理由。
- 边界清晰：保持现有接口、调用链和配置兼容；若必须改变公共接口或扩大范围，先沟通确认。
- 风格一致：沿用附近代码的命名、结构、注释和错误处理方式，避免过度抽象和非必要依赖。
- 可直接接入：提交前完成相关语法及针对性测试，并说明修改文件、接口影响、验证结果和未覆盖风险；不要提交日志、临时文件或无关生成物。

## 构建

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

Isaac Gym 需要安装在它支持的独立 Python 环境中。日常仿真可直接使用该环境运行入口脚本。

## 普通仿真

```bash
/home/artrc/miniconda3/envs/grasp_hexapod/bin/python \
  src/grasp_hexapod_control/scripts/run_sim.py
```

使用 ROS 手柄或导航输入时：

```bash
roslaunch grasp_hexapod_control run_sim_ros.launch
```

## 实机部署

### 阶段一视觉粗接近（诊断入口）

`phase1_visual.launch`把顶部前向相机的`/xiaolan/target`接入独立的`spin_search`
和`approach`模式；默认不启动相机、检测或动作序列。显式启动序列时只执行
`home -> spin_search -> approach`，完成后保持在 HOLD，绝不会启动 climb 或 dock：

```bash
roslaunch grasp_hexapod_bt_control phase1_visual.launch start_sequence:=true
```

相机和 YOLO 均默认关闭；需要时显式传入 `start_camera:=true`
和 `start_perception:=true`。`start_sequence` 默认也是 `false`，也可在完成
`home` 后手动运行 `rosrun grasp_hexapod_bt_control phase1_visual_sequence`。
配置默认最大前进速度为 `.03 m/s`，相机光学深度停止值为 `.60 m`。

目标的 `x/z` 是相机光学坐标（右/前），`0.60 m` 停止距离只是未标定的相机深度
诊断起点，不是机体、足端或小蓝净空。该流程不替代外参、碰撞、接触、载荷或实机安全验证；
`control_only.launch` 默认仍关闭视觉引导，原有假模式和 RTK 路径不变。

APPROACH、CLIMB与DOCK可分别调试：普通Isaac/ROS仿真使用`run_sim.py`或
`run_sim_ros.launch`，联合底部相机对接仿真使用`run_sim_dock.py`，实机统一从
`run_real.launch`进入。DOCK只使用底部USB相机；顶部Orbbec和小蓝标签链不参与
该流程。

底部相机链依赖安装包：

```bash
sudo apt install ros-noetic-usb-cam ros-noetic-image-proc ros-noetic-apriltag-ros \
  ros-noetic-nodelet ros-noetic-camera-calibration python3-opencv python3-yaml
```

首次只在机器人架空、舵机安全条件已确认时启动。必须在最终分辨率/像素格式下标定，棋盘格
参数按实际工装填写（`8x6`、`0.025 m`仅示例）：

```bash
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 \
  image:=/dock_camera/image_raw camera:=/dock_camera
```

保存内参后用`dock_camera_info_url:=file:///absolute/path/dock_camera.yaml`启动。实测相机
光学帧到卡紧机构外参，并将部署版`dock_system.yaml`以绝对路径通过
`dock_system_config:=/absolute/path/dock_system.yaml`传入；它同时供 detector 和控制器使用。
Tag size 是黑白边界有效边长。未标定的`real_calibrated: false`会阻止Y进入DockMode。
先执行`roslaunch --nodes src/grasp_hexapod_control/launch/dock_tag_system.launch`，再检查
`rostopic hz /dock_camera/image_rect_color`、`/dock_camera/camera_info`和
`/dock/tag_detections`。AprilTag检测、视觉对准或锁紧话题均不证明物理锁紧、接触、承载或
整机安全。

两块驱动板的默认映射如下。`left`、`right` 是启动参数中的板标识。

| 板 | 腿 | 舵机 ID | 默认串口 |
|---|---|---|---|
| `left` | `lf`、`lm`、`lb` | 1～9 | `/dev/ttyTHS0` |
| `right` | `rf`、`rm`、`rb` | 10～18 | `/dev/ttyACM0` |

串口名称变化时可在启动时覆盖：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  left_port:=/dev/ttyTHS0 \
  right_port:=/dev/ttyACM0
```

底部 USB 相机（Realtek `0bda:3035`）通过 udev 规则固定为 `/dev/cam_dock`，
只绑定采集节点（`index=0`，metadata 节点不参与），插拔端口或枚举序号变化都
不影响设备名。规则由 `串口配置/serial_rename.sh` 生成并安装到
`/etc/udev/rules.d/99-camera-rename.rules`：

```bash
sudo bash 串口配置/serial_rename.sh /dev/video8 cam_dock   # 按当前实际 video 序号填写
```

更换相机后重跑一次即可；dock 相关 launch 的默认设备已是 `/dev/cam_dock`，
无需再传 `dock_video_device`。

使用 C++ 版舵机驱动（`grasp_hexapod_servo_cpp`，接口一致、资源占用更低）时，
入口换成 `run_real_cpp.launch`，参数相同；也可给 `run_real.launch` 加
`servo_backend:=cpp`。

首次架空测试可显式降低速度：

```bash
roslaunch grasp_hexapod_control run_real.launch \
  max_linear_speed:=0.01 \
  max_vertical_speed:=0.003
```

启动前检查设备和 ROS 节点：

```bash
ls -l /dev/input/js*
ls -l /dev/serial/by-id/
roslaunch --nodes grasp_hexapod_control run_real.launch
```

启动后检查手柄和一条腿的反馈、目标：

```bash
rostopic echo /joy
rostopic echo /lf_pos
rostopic echo /lf_des
```

## 实机全流程（正面攀爬）调试

入口是 `full_mission.launch`，流程为：

```text
home → spin_search → approach（YOLO 粗接近 → AprilTag 精对准） → climb（正面） → dock（圆环对准 + 夹爪锁紧）
```

任何一步失败，机器人停在 HOLD 等待人工接管，不会自动重试，也不会继续下一步。正面攀爬和对接**未经实机认证**：每次都要有吊架或保护绳，急停随手可及。

### launch 默认行为

| 参数 | 默认 | 作用 |
|---|---|---|
| `start_camera` | `true` | 启动顶部 Orbbec 相机 |
| `start_perception` | `true` | 启动 YOLO 检测与深度定位 |
| `start_tag_detector` | `true` | 启动 AprilTag 检测，输出 `/xiaolan/tag_detections` |
| `start_dock_perception` | `true` | 启动底部对接相机 |
| `show_windows` | `true` | 在上位机桌面弹出 3 个画面窗口 |
| `start_sequence` | `false` | 是否自动执行整条流程；默认只启动、不动作 |
| `stop_after` | 空 | 执行到某个模式后停止，用于分段调试 |
| `home_before_dock` | `false` | 攀爬后、对接前是否先回站姿 |
| `tag_id` / `tag_size_m` | `0` / `0.060` | 检测器的 tag；id 必须与 `phase1_visual.yaml` 的 `tag_guidance_id` 一致 |
| `climb_foot_gate_m` | `0.04` | 攀爬阶段推进的足端误差门限 |

弹出的窗口：

| 窗口 | 内容 |
|---|---|
| Xiaolan YOLO RGB | 前置相机画面、YOLO 框、小蓝中心、`X_target`（横向偏移）/`Z_target`（深度），单位 m |
| AprilTag (approach alignment) | 矫正后的前置画面、tag 轮廓与 id |
| Dock camera (bottom) | 底部相机原图（圆环识别暂无标注） |

终端输出：串联节点打印每个模式的开始和结果；粗接近阶段每 2 s 打印状态、小蓝 x/z、方位角、前进与转向指令、帧率；精对准阶段每 2 s 打印 `tag_x/tag_z/tag_yaw_deg`、误差 `ex/ez/eyaw_deg` 和右移/前进/转向指令。

窗口需要在上位机自己的桌面终端里启动。经 ssh 远程启动时加 `show_windows:=false`，在有屏幕的电脑上用 `rqt_image_view` 查看 `/xiaolan_visualizer/image`、`/xiaolan/tag_detections_image`、`/dock_camera/image_raw`。

### 调试步骤

**第 0 步：编译与依赖（每次更新代码后）**

```bash
sudo apt install ros-noetic-apriltag-ros ros-noetic-image-proc ros-noetic-image-view ros-noetic-usb-cam
```

```bash
cd ~/Grasp_hexapod && catkin_make && source devel/setup.bash
```

```bash
catkin_make run_tests_grasp_hexapod_bt_control run_tests_grasp_hexapod_control
```

**第 1 步：只启动、不动作，检查感知和反馈**

机器人架空。启动后确认 3 个窗口出现、画面正常：

```bash
roslaunch grasp_hexapod_bt_control full_mission.launch
```

另开终端检查数据频率和模式状态：

```bash
rostopic hz /xiaolan/target /xiaolan/tag_detections /dock_camera/image_raw /lb_pos
```

```bash
rostopic echo /grasp_hexapod/mode_status
```

**第 2 步：贴 tag 并示教（只需做一次，tag 或相机挪动后重做）**

1. 用 tag36h11、id 0、边长 60 mm 打印 tag，贴在小蓝正对机器人一侧的竖直面上，横向居中于机器人接近中线；不要贴在 0.14–0.17 m 的前足落脚斜面上。
2. 把机器人人工摆到正面攀爬起点：机器人中心在小蓝坐标系 (−0.55, −0.038) m，正对小蓝 −x 侧面。
3. 保持第 1 步的 launch 运行，另开终端记录参考位姿（取 30 帧平均）：

```bash
rosrun grasp_hexapod_bt_control record_tag_reference.py _tag_id:=0 _samples:=30
```

4. 把输出的 `tag_ref_x_m`、`tag_ref_z_m`、`tag_ref_yaw_rad` 三行填进 `src/grasp_hexapod_bt_control/config/phase1_visual.yaml`，重启 launch 生效。
5. 示教时顺带确认：tag 窗口里能稳定框出 tag；把机器人往后移约 1 m，tag 仍能被识别（这样才能在 0.20 m 停止距离之前完成接管）。

**第 3 步：单模式手动调试（逐个验证）**

保持 launch 运行（`start_sequence` 为默认 false），另开终端逐个调用。服务是阻塞的，模式结束才返回 `success` 和 `message`。第一个必须是 `home`，否则其他模式会被拒绝。

```bash
rosservice call /grasp_hexapod/switch_mode "target_mode: 'home'"
```

```bash
rosservice call /grasp_hexapod/switch_mode "target_mode: 'spin_search'"
```

```bash
rosservice call /grasp_hexapod/switch_mode "target_mode: 'approach'"
```

```bash
rosservice call /grasp_hexapod/switch_mode "target_mode: 'climb'"
```

```bash
rosservice call /grasp_hexapod/switch_mode "target_mode: 'dock'"
```

中途要停下：另开终端调用 `home`，会抢占当前模式并平滑回站姿；手柄 B 键同样调用 `home`。紧急情况直接断电或拍急停。

**第 4 步：用串联节点分段跑**

每段稳定后再加下一段：

```bash
roslaunch grasp_hexapod_bt_control full_mission.launch start_sequence:=true stop_after:=spin_search
```

```bash
roslaunch grasp_hexapod_bt_control full_mission.launch start_sequence:=true stop_after:=approach
```

```bash
roslaunch grasp_hexapod_bt_control full_mission.launch start_sequence:=true stop_after:=climb
```

```bash
roslaunch grasp_hexapod_bt_control full_mission.launch start_sequence:=true
```

串联节点的退出码：0 完成；1 某个模式失败（日志写明哪个模式和原因）；2 找不到 `switch_mode` 服务；3 控制节点参数不满足（日志写明缺哪个）；4 `stop_after` 写错。

### 常见失败信息

| 终端信息 | 含义与处理 |
|---|---|
| `call home first` | 上电后还没执行 `home`，先调用 `home` |
| `visual search timeout` | 自转 60 s 没找到小蓝：检查 YOLO 窗口是否有框、相机朝向 |
| `perception startup unavailable` / `perception stale/unusable` | 相机或 YOLO 没起来或掉帧：看 `rostopic hz /xiaolan/target` |
| `visual target lost or approach timeout` | 粗接近途中丢目标或超时 |
| `coarse stop reached without tag handoff` | 走到 0.20 m 仍没稳定识别 tag：检查 tag 窗口、tag id、光照、tag 尺寸 |
| `tag lost` | 精对准中 tag 丢失超过 1 s |
| `tag alignment timeout` | 精对准 60 s 未进容差：检查示教值、相机是否朝机身正前方 |
| `climb rejected: ...` | 攀爬入口被拒：确认已 HOLD、`enable_real_climb`、`allow_front_hardware` 为 true |
| `dock rejected: ...` | 对接入口被拒：确认已 HOLD、`dock_system.yaml` 已标定 |

### 已知限制

- 正面攀爬在仿真里按 6 rad/s 排时，实机限 4 rad/s，推进会变慢，尚未在 Isaac 按实机速度复验。
- 正面攀爬结束时机身离对接点约 13 cm，超出对接原地搜索范围，暂未处理。
- C++ 行走步态与 Python 在金标第 75 帧差 0.085 rad（既有问题，未查）。
- 以上代码只在无 ROS 环境做过单元测试和金标对比，`catkin_make` 和实机首次运行需要现场确认。

## 安全操作

机器人必须架起后，先确认串口、舵机 ID、安装方向和机械零位，再进行行走测试。

```text
B -> 等待回到标准站姿 -> A -> 接受运动指令
```

- B：取消当前行为并平滑回到标准站姿。
- A：站姿初始化完成后启用或暂停运动。
- 第一次按 B 前，高层控制器不发送目标，舵机保持卸力。

## 攀爬预览

Isaac Gym 仍是攀爬动作和物理行为的预览权威。`run_real.launch`也提供默认启用的
C1--C27 实机诊断回放入口：完成 B 回站后按 X 启动，A 暂停或恢复，B 可随时中止。
它只按 18 个关节位置反馈的实际关节跟踪与 FK 足端目标误差连续稳定门限推进；这些
关节反馈门限不证明接触、承载或稳定性，也不构成实机攀爬安全授权。

```bash
conda run --no-capture-output -n grasp_hexapod python3 \
  src/grasp_hexapod_control/scripts/run_sim.py \
  --climb-start --climb-speed 1 --climb-joint-speed 1.2
```

可只回放一个闭区间：`C1` 至 `C27` 是 compact 数组顺序的固定用户别名，也可
使用运行时阶段名。中途入口会把 Isaac root、18 个关节和足端锚点同步到该阶段
的规划起点；它仍只是仿真预览。

```bash
conda run --no-capture-output -n grasp_hexapod python3 \
  src/grasp_hexapod_control/scripts/run_sim.py \
  --climb-start --climb-from C13 --climb-to C15 \
  --climb-speed 1 --climb-joint-speed 1.2 \
  --climb-metrics logs/c13_c15_metrics.json
```

指标 JSON 是 simulation-only 诊断，记录关节跟踪、运动学足端目标误差和关节
限位余量；不证明接触、载荷或稳定性。`--climb-from`、`--climb-to` 和
`--climb-metrics` 只能与 Isaac 的 `--climb-start` 或 `--climb-scene` 一起使用，
不进入 ROS 或实机路径。

离线候选先由 `rebuild_climb_preview.py snapshot` 生成模型/profile，再由 `build`
仅重定向显式编辑的阶段终点；它不会自动选落脚点、改变阶段顺序、限位或时序。

## 包内说明

- [启动命令与参数](src/grasp_hexapod_control/launch/README.md)
- [自建消息与服务定义](src/grasp_hexapod_msgs/README.md)
- [舵机协议、ID 和方向](src/grasp_hexapod_servo/README.md)
- [C++ 版舵机驱动](src/grasp_hexapod_servo/README_servo_cpp.md)
- [参考：编码器帧解析与角度发布](src/reference/encoder_driver/README.md)

## Repository Activity

![Repobeats analytics image](https://repobeats.axiom.co/api/embed/859fd5df85ba8f10eab3b9abe78ae79040f68f89.svg)
