# grasp_hexapod_bt_control — 行为树服务化控制栈（模式框架版）

面向行为树（`grasp_hexapod_bt`，现有 Python/py_trees 版，**零改动**）的全链路
C++ 控制栈：控制节点完全不依赖手柄话题，由服务切换状态机；手柄功能整体移到
独立的遥控器节点。与 `src/docs/README.md` 描述的模式契约一致。

`bt_control_node` 内部已重构为**统一模式框架**：每个模式（home/walk/approach/
climb/dock/release/spin_search）是 `src/modes/` 下一对独立的 `mode_<name>.h/.cpp`
（不同作者可分别维护），通过文件末尾 `REGISTER_MODE` 宏自注册接入调度器；
所有话题名统一在 `config/mode_topics.yaml` 配置（yaml-cpp 直读）。新增模式的
三步接入清单与接口契约见 **[docs/MODE_DEV_GUIDE.md](docs/MODE_DEV_GUIDE.md)**。
对外接口（`switch_mode` 服务 + `hold_motion` 租约）与响应报文保持不变。

> **与旧链路互斥**：`bt_control_node` 与 `grasp_hexapod_control` 的
> `run_real.py`/`run_real_cpp` 提供同一个 `/grasp_hexapod/switch_mode` 服务，
> 同一时刻只能运行一个。旧链路（含 /joy 手动模式）原样保留，按需二选一。

## 架构

```
手柄 → joy_node(/joy) → remote_control_node ─┬─ /cmd_vel（walk 速度，平面三自由度）
                                             ├─ 服务 /grasp_hexapod/switch_mode（抢占式）
                                             └─ 服务 /grasp_hexapod/gripper_act
现有 Python 行为树 run_real_bt.py ── /grasp_hexapod/switch_mode、/hold_motion ──┐
                                                                              ▼
bt_control_node（模式调度器：ModeContext 注入各模式实例）
  ├─ modes/mode_home|walk|approach|climb|dock|release（每模式一对文件，自注册）
  ├─ 共享输入：六腿反馈/IMU/导航（安全环路常驻，模式只读快照访问）
  └─ 话题名统一来自 config/mode_topics.yaml（~topics_config 指定路径）
                              ▼ /{leg}_des
        servo_two_boards_cpp（左板含夹爪 ID 99）→ /{leg}_pos → bt_control_node
```

## 节点

### bt_control_node（`node/bt_control_main.cpp`）

模式调度器 + 共享安全环路，数值核心复用 `grasp_hexapod_control_core`
纯计算库。各模式逻辑在 `src/modes/`（见模式开发指南）：

- **不订阅** `/joy`、`/grasp_hexapod/remote_cmd`；手柄映射参数全部删除。
- **`/grasp_hexapod/switch_mode` 带抢占语义**：进行中的不同模式请求被立即终结
  （`success=false, "preempted by X"`），新模式先平滑回正到 HOLD 再进入；
  同模式重复调用按等待者合并。单调用方（行为树）感知不到抢占。
- **`/cmd_vel` 驱动 walk**（订阅由 WalkMode 自持）：平面三自由度
  `linear.x/linear.y/angular.z`，**无 z 轴线速度**；按 `max_linear_speed`
  向量模限幅、`max_yaw_rate` 航向限幅。`~max_cmd_vel_age`（默认 0.2s）内
  无新速度指令 → 零命令停步回 HOLD，walk 以 `"cmd_vel lost"` 结束。
- **WAIT_B 安全门**：上电初始态不发布目标（舵机卸力）；只接受 `home`/`release`
  （release 免门控，仅松夹爪），其余模式拒绝 `"call home first"`。
- **home 顺序**：先松夹爪再平滑回正；夹爪失败仍完成回正（安全优先）但响应
  报失败（夹爪错误消息透传）。
- **approach/spin_search 假实现**（2026-09-18）：导航/感知栈未就绪时用于
  整链联调——保持当前姿态定时返回成功。approach 真实导航在
  `~approach_fake:=false` 时启用；时长 `~approach_fake_duration_s` /
  `~spin_search_fake_duration_s`（默认 5.0s）。
- **dock 末端夹持必然执行**：dock 只能经服务进入（原手柄 Y 直连路径已删），
  `DockMode` 到 `success` + HOLD 后调用 `gripper_act clamp`，夹爪结果即
  BT 服务的最终结果；失败 `ROS_ERROR` 并透传。`release` → open、
  `home` 完成 → open 同链路。
- **`/grasp_hexapod/mode_status` 模式状态反馈**（`ModeStatus.msg`）：当前模式/
  状态（idle|running|paused|success|failed|preempted）/调用代次/原因，
  变化即发 + 空闲 1Hz 心跳，供仪表盘与监控消费。
- **`/grasp_hexapod/hold_motion` 租约 → pause/resume**：心跳期间活动模式原地
  冻结（目标保持上一帧、持续发布保持上力，不取消模式）；心跳过期后从冻结点
  恢复（对应行为树 PauseGate）。
- 状态机仍为 `WAIT_B/RESETTING/HOLD/RUNNING`；七个模式均可经 switch_mode
  调用（approach/spin_search 当前为假实现，见上）。`approach` 为接近导航到攀爬起点（RTK 粗导航 + 视觉 tag
  伺服，原 `tag_nav` 已并入本模式，不再是独立模式）。
- 保留：六腿反馈双板门控、反馈丢失 → HOLD、攀爬相对运动监控与速度诊断
  （已随 climb 模式迁入 `modes/mode_climb.cpp`，逻辑不变）。

### remote_control_node（`node/remote_control_main.cpp`）

手柄 → 服务的映射层（参数沿用原命名，`~button_*`/`~axis_*`/`~axis_*_scale`/
`~max_linear_speed`/`~max_yaw_rate`/`~rate_hz`）：

| 输入 | 动作 |
|---|---|
| 左摇杆（axis_right/axis_forward） | `/cmd_vel` 平面速度（30Hz；手柄失效发零速） |
| 右摇杆横向（axis_yaw） | `/cmd_vel` 角速度 |
| A(0) / B(1) / X(2) / Y(3) 按下沿 | 后台线程调 `switch_mode(walk/home/climb/dock)` |
| 方向键轴 6：左(-1) / 右(+1) | `gripper_act(clamp/open)`，回中后可重复触发 |

## 启动

```bash
# 全链路（含现有 Python 行为树；remote_test 为空则跑主链，默认 walk 测试链）
roslaunch grasp_hexapod_bt_control full_chain.launch remote_test:=walk

# 仅控制栈 + 遥控器（无行为树，联调/排查用）
roslaunch grasp_hexapod_bt_control control_only.launch
```

### Phase-1 visual coarse approach

The diagnostic path is `/xiaolan/target` → C++ `spin_search` / coarse
`approach` → existing `/<leg>_des` joint-target path. Bring it up with:

```bash
roslaunch grasp_hexapod_bt_control phase1_visual.launch
```

This production entry does not start a sequence automatically, but includes
`control_only.launch` and therefore servo/joy nodes; do not use it for PC
offline checks. Camera and YOLO are optional and off by default; request them
explicitly with `start_camera:=true` and `start_perception:=true`. Override
`yolo_python:=/absolute/path/to/python3` when the TensorRT runtime lives
outside the default package-relative launcher. After the controller reports
feedback-ready, start only the bounded sequence manually:

```bash
rosrun grasp_hexapod_bt_control phase1_visual_sequence
```

It executes `home -> spin_search -> approach -> HOLD`; it never requests climb
or dock. Its deployment-start limits are `.03 m/s` maximum forward motion and
`.60 m` camera-optical stopping depth. Existing perception accepts depth only
in `(0.2, 1.0) m`. This batch adds no camera-to-body extrinsic transform, hot
extrinsic update, or AprilTag alignment. `visual_guidance_enabled` stays false
in the legacy launch, so its default fake/RTK behavior remains unchanged.

Validation is limited to the current code path: 9 visual-guidance gtests, 13
mode-framework gtests, 7 routing gtests, and isolated fake-feedback ROS E2E.
Final synthetic receipts are `/tmp/grasp_phase1_startup_e2e_20261030T010110`,
`/tmp/grasp_phase1_visual_e2e_20261030T010126`, and
`/tmp/grasp_phase1_perception_e2e_20261030T010148`; they are not physical
camera, servo, collision, contact, load, or hardware validation.

### Full mission (front climb)

`launch/full_mission.launch` starts the phase-1 stack with climb and dock
enabled, plus the standalone AprilTag detector (`tag_id`, default 0; output
`/xiaolan/tag_detections`). It is **not certified for hardware**: front
climbing and docking are not certified. Run only with harness/support and the
e-stop within reach. `climb_front_direct` timing was previewed at 6 rad/s in
simulation, while hardware is capped at 4 rad/s.

```bash
roslaunch grasp_hexapod_bt_control full_mission.launch
# Sequence only when requested (default start_sequence:=false):
roslaunch grasp_hexapod_bt_control full_mission.launch start_sequence:=true
```

Staged usage with `stop_after` (empty by default; stops successfully after the
named mode completes):

```bash
roslaunch grasp_hexapod_bt_control full_mission.launch start_sequence:=true stop_after:=approach
roslaunch grasp_hexapod_bt_control full_mission.launch start_sequence:=true stop_after:=climb
```

The sequence is `home -> spin_search -> approach -> climb -> dock`, with an
extra `home` before `dock` when `home_before_dock:=true`. Preflight requires
`visual_guidance_enabled` and `tag_guidance_enabled`, plus `enable_real_climb`
when climb is in the planned sequence and `enable_real_dock` when dock is in
it. A failed mode stops the sequence with the robot in HOLD for operator
takeover; nothing is retried.

Desktop view (`show_windows:=true`, default; needs the robot computer's own display): the YOLO window shows the box, Xiaolan centre and `X_target`/`Z_target`; the AprilTag window shows the detected tag outline and id; the dock window shows the raw bottom camera (no ring overlay yet). The terminal prints the mode progress, the visual approach state with target x/z, bearing and commands, and during alignment the tag x/z/yaw, errors and commands every 2 s. Over ssh without a display, pass `show_windows:=false` and use `rqt_image_view` on `/xiaolan_visualizer/image`, `/xiaolan/tag_detections_image` or `/dock_camera/image_raw`.

Teach-in reminder: `tag_ref_*` in `config/phase1_visual.yaml` are placeholders.
Teach them with `scripts/record_tag_reference.py` before `approach` can
succeed.

The launch sets `climb_config_file:=climb_front_direct.json`,
`allow_front_hardware:=true` (explicit hardware opt-in for the front route; the
shared 4 rad/s joint step cap applies) and `climb_foot_gate_m:=0.04`. The tag
id is `tag_guidance_id` in `config/phase1_visual.yaml`; the detector `tag_id`
arg must match it. Camera and YOLO stay off unless `start_camera:=true
start_perception:=true` are passed, as in `phase1_visual.launch`.

### AprilTag handoff inside visual `approach`

With `tag_guidance_enabled: true` (set in `config/phase1_visual.yaml`; code
default false, and false leaves the visual branch unchanged) the YOLO approach
now stops at the coarse `visual_stop_distance_m: 0.20` and hands off to the tag:

- Input: `apriltag_ros/AprilTagDetectionArray` on `approach.tag_detections` of
  `config/mode_topics.yaml` (default `/xiaolan/tag_detections`). After
  `tag_handoff_confirm_frames` (5) consecutive NEW detections of
  `tag_guidance_id` (0) with a finite pose in front of the camera, age
  `<= tag_max_age_s` (0.25), frame `visual_expected_target_frame`, the run
  switches permanently to TagGuidance; YOLO is then ignored for control.
- Servo (camera optical frame, camera axis assumed along body +y, pitch
  ignored): `right = k_lat*ex`, `forward = k_fwd*ez`, `yaw = +k_yaw*eyaw`,
  with `ex = x - tag_ref_x_m`, `ez = z - tag_ref_z_m`,
  `eyaw = wrap(yaw - tag_ref_yaw_rad)`, `yaw = atan2(R(2,0), R(0,0))`
  (R = tag frame in camera frame; tag x axis, independent of the tag z
  convention). A CCW body yaw
  lowers the observed tag yaw, so positive `eyaw` commands positive (CCW) yaw;
  the closed-loop sign is checked in `test/test_tag_guidance.cpp`. A pose with
  non-finite values or `|R(1,0)| > 0.5` (bogus / heavily rolled) is rejected.
- Params (defaults): `tag_lateral_gain/forward_gain/yaw_gain` 0.5/0.5/0.8,
  `tag_max_lateral_speed_m_s` 0.02, `tag_max_forward_speed_m_s` 0.02,
  `tag_max_yaw_rate` 0.10, `tag_position_tolerance_m` 0.01,
  `tag_yaw_tolerance_rad` 0.035, `tag_settle_s` 0.5, `tag_loss_timeout_s` 1.0,
  `tag_align_timeout_s` 60.
- Success: inside tolerance continuously for `tag_settle_s` with the gait
  stopped -> `approach_plan.ready_for_climb = true`, HOLD, outcome
  `"tag alignment complete: ready for climb"`. Failures (HOLD): `"tag lost"`,
  `"tag alignment timeout"`, and `"coarse stop reached without tag handoff"`.
  Pause/resume discards pre-pause detections and restarts the loss clock.
- `tag_ref_x_m / tag_ref_z_m / tag_ref_yaw_rad` (defaults 0.0 / 0.15 / 0.0) are
  PLACEHOLDERS and must be taught. Teach-in: put the robot at the desired final
  pose, then
  `rosrun grasp_hexapod_bt_control record_tag_reference.py _tag_id:=0`
  (optional `_samples:=30 _topic:= _frame:=`); paste the printed yaml lines.
- The 0.20 m stop and the taught reference are camera-frame diagnostics, not
  body clearance, contact, or hardware-safety evidence. Not yet verified on
  hardware or with catkin/e2e (the legacy visual e2e scripts assume the old
  0.60 m stop and Success outcome).

### Hardware-free Phase-1 bench

For offline integration, use the bench launch on an isolated ROS master.  It
starts only `bt_control_node`, a guarded fake joint-feedback/gripper fixture,
and—when requested—the existing Python `Target3DNode`; it never starts serial,
servo, joy, camera, or a synthetic `/xiaolan/target` publisher.

```bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11331
roscore -p 11331
# Terminal A runs one selected bench launch; terminal B uses the same ROS_MASTER_URI.
# Choose one bench command; do not run both simultaneously.
roslaunch grasp_hexapod_bt_control phase1_visual_bench.launch
# Or add only depth localization for a replay/external detection-depth source:
roslaunch grasp_hexapod_bt_control phase1_visual_bench.launch start_depth:=true
# Terminal B, after the controller reports feedback-ready:
rosrun grasp_hexapod_bt_control phase1_visual_sequence _controller_node:=/bt_control_node
```

`start_depth:=true` runs `xiaolan_target_3d_node.py` only (`start_yolo=false`,
`start_visualizer=false`).  By default the bench has no target publisher:
an external replay or perception process must supply `/xiaolan/detection`,
`/camera/depth/image_raw`, and matching depth/color `CameraInfo`; Target3DNode
then remains the sole `/xiaolan/target` publisher.  The fake fixture refuses a
graph that already publishes a joint-feedback topic or offers the gripper
service, and requires its explicit private `allow_fake_feedback` launch
parameter.  This is integration evidence only, not authorization for camera,
servo, motion, or hardware operation.

The CPU-only model smoke test uses the isolated runtime and actual `best.onnx`:

```bash
python3 src/grasp_hexapod_bt_control/test/phase1_yolo_smoke.py \
  --python "$PWD/.venv/xiaolan-ort/bin/python3" \
  --model "$PWD/src/xiaolan_yolo_detection/models/best.onnx"
```

It requires an isolated ROS master and tests blank synthetic RGB only. See
[`PHASE1_VISUAL_PIPELINE_AUDIT.md`](../docs/PHASE1_VISUAL_PIPELINE_AUDIT.md)
for timing, frames, limits, and non-hardware boundaries.

服务切换状态机与原按键语义对照：

| 原按键 | 服务调用 | 行为 |
|---|---|---|
| B | `switch_mode("home")` | 无条件抢占回正，完成后张开夹爪 |
| A | `switch_mode("walk")` | 步态行走，速度来自 `/cmd_vel` |
| X | `switch_mode("climb")` | 需 `~enable_real_climb`，且已在 HOLD |
| Y | `switch_mode("dock")` | 视觉对接 → 六腿同高 → **夹爪闭合** |

## 测试

```bash
# gtest（反馈门控 + cmd_vel 限幅，无 ROS master 依赖）
catkin_make run_tests_grasp_hexapod_bt_control_gtest_test_bt_control_routing

# gtest（模式框架：注册表 / YAML 加载器 / ModeLifecycle 状态机）
catkin_make run_tests_grasp_hexapod_bt_control_gtest_test_mode_framework

# bt_control_node e2e：WAIT_B 门(含 release 免门控) / home 先夹爪后回正 /
# walk+cmd_vel / 抢占 / hold 租约冻结恢复 / approach 假模式 / release
python3 src/grasp_hexapod_bt_control/test/fake_hardware_e2e.py

# 遥控器 e2e：cmd_vel 映射 / 失效归零 / 按键服务 / 方向键夹爪
python3 src/grasp_hexapod_bt_control/test/remote_control_e2e.py
```

（e2e 需要 roscore；伪舵机板 + 伪夹爪服务由脚本自带，不需真实硬件。）

## 已知边界

- `dock` 全流程依赖真实视觉（底部相机圆环识别，进程内 RingPerceptionRos，
  `dock_system.yaml` 实标定）与可选锁紧确认话题
  `/grasp_hexapod/dock/lock_confirmed`；e2e 只验证到夹爪服务
  链路（release 与 dock clamp 走同一 `actuateGripper` 实现）。
- `climb` 需 `~enable_real_climb:=true` 且 IMU/RTK 数据可用（仅影响可选监控）。
- `approach` 需 `~control_source:=navigation`。
