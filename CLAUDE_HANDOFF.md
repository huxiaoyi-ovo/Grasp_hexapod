# Claude Code 交接记录

用途：记录 Claude Code 的工作内容与对应仓库版本，供 Codex（Sol/Terra）和后续 Claude Code 会话交接。
规则：按时间追加条目，不改写旧条目；本文件不覆盖 `AGENTS.md`、安全规则或稳定事实。

条目模板：

```text
## YYYY-MM-DD 简题
- 基线：分支 / 起始提交 → 结束提交
- execution_owner：Sol / Luna，一句理由
- 目标与范围：
- 修改文件：
- 实际运行的验证：（未运行的写明未运行）
- 证据层级：CPU / CAD / GPU / 实机
- 发现与未决事项：
- 下一步：
```

---

## 2026-10-30 通读仓库、建立入口与交接文件

- 基线：`test`，起始 `f161d6e`（Track governing docs and Phase-1 evidence receipts）。
- execution_owner：Sol，仅 2 个文档文件、无运行时影响。
- 目标与范围：通读代码与治理文档；按用户指定建立 Claude Code 角色映射（主会话 = Sol，轻量子代理 = Luna）和交接文件。未改任何源码、配置、launch。
- 修改文件：`CLAUDE.md`（路由到 `AGENTS.md`）、`CLAUDE_HANDOFF.md`（本文件），均 `git add -f` 入库。
- 实际运行的验证：无代码验证（纯文档）；只做了只读核对（sha256、JSON 比对、grep）。
- 证据层级：不涉及。

### 只读核对得到的事实（供 Sol 决定是否修文档）

1. `climb_compact.json` 实际 sha256 为 `dc4eff1308e99369e815dc754037fc3fd11e0657297adb3fb6dc738ed600c711`；`AGENTS.md` 记为 `5f8edbc…`。逐键比对：与 `src/docs/evidence/climb_c2_speed_20260909/candidate.json`（`5f8edbc`）仅差顶层 `front_v1_receipt` 历史元数据键，阶段内容一致；该键运行时不消费，仅测试/验证器读取。`climb_rebuild_profile.json` 的 `template_sha256` 已是 `dc4eff…`。
2. `PROJECT_MEMORY.md` 有多段自称 current/active 的 SHA（`af3dca`、`6761`、`f8da`、`5f8edbc`）及旧的 C1–C36、2084 ticks 数值；`AGENTS.md` 有两个“（当前）”小节。实际当前为 C2 提速版本。
3. `AGENTS.md`“There is no dedicated test suite”已过时（现有 control/bt_control/servo/ring gtest 与 pytest、e2e）。
4. `HANDOFF.md` 未入库，本容器不可得。
5. 代码层（未修）：
   - `grasp_hexapod_bt_control/config/mode_topics.yaml` 与 `MODE_DEV_GUIDE.md` 注释写腿序 `lf,lm,lb,rf,rm,rb`，实际列表为控制器序 `lb,lf,lm,rb,rf,rm`（列表正确）。
   - 根 `README.md` 多处写 C1–C35，当前 compact 为 27 阶段。
   - `/grasp_hexapod/gripper_act` 仅由 C++ 舵机节点提供，`run_real.launch` 默认 `servo_backend:=python`；按代码阅读，默认后端下夹爪服务不存在（未运行验证）。舵机包只读，若修只能改 launch 默认值。
   - `launch_real_cpp.sh` 实际 `exec` 的是 `run_real.launch`（Python 控制器）。
   - `grasp_hexapod_control/scripts/dock_mode_tag.cpp` 内容为 Python（旧 AprilTag DockMode），无引用。
   - README 引用的 `scripts/configure_autostart.sh`、`src/docs/BT_INTERFACES.md`、`src/grasp_hexapod_servo_cpp/README.md` 不存在。
   - `real_control_node` / `ring_control_node` 的 switch_mode 仍为 busy 拒绝式（文档已注明）。

### 下一步

- 等待用户给出本轮目标。
- 2026-10-30 追加：用户确认 Claude Code 主会话按 Sol 职能负责统筹、规划与审查；Luna 默认最轻档位，能力不足时升到中间档位，已写入 `CLAUDE.md`。
- ~~待 Sol 决策：是否在 `AGENTS.md` 登记 Claude Code 路由~~ → 已完成，见下条。

## 2026-10-30 在 AGENTS.md 登记 Claude Code 路由

- 基线：`test`，起始 `b01853f`。
- execution_owner：Sol，用户明确同意修改治理规则；2 个文档文件。
- 修改：`AGENTS.md` Active Worker Routing —— 活跃路由改为 Codex（Sol+Terra）与 Claude Code（Sol+Luna，Luna 沿用全部 Terra 规则）；新增“同一时刻只有一条路由写仓库，切换前 pull 并读 `HANDOFF.md`/`CLAUDE_HANDOFF.md`”；`CLAUDE.md`/`CLAUDE_HANDOFF.md` 定位改为入口与交接日志，`CLAUDE_CODE_HANDOFF.md` 仍为历史文件。DeepSeek 仍非活跃路由。
- 验证：纯文档，无代码验证。
- 下一步：等待用户给出本轮目标。

## 2026-10-30 实机全流程：正面攀爬 + tag 精对准 + 串联（W1–W3）

- 基线：`test`，起始 `b075cd8` → 结束 `807e428`（本条随后单独提交）。
- 用户决定：正面攀爬（默认步态已完成）；YOLO 粗接近停止 0.20 m，稳定识别侧面 AprilTag 后自动切 tag 引导、不再用 YOLO；tag 尚未粘贴；用简单串联节点，不用完整行为树；dock 前的 ~13 cm 位置缺口用户要求暂不处理。
- 路由：Sol 冻结合同并审查；Luna 三个合同。W1、W2 用中间档位、effort high（相机/机身符号推导与闭环测试；数值移植与金标等价，判定最轻档位不足）；W3 用最轻档位、effort medium（机械性节点与 launch）。W1 与 W2 应用户要求并行，W2 在独立 worktree，文件不重叠，由 Sol 合并。
- 提交：
  - `f124673` W1：`approach` 视觉分支加 AprilTag 接管（`tag_guidance.h/.cpp` 无 ROS 策略 + `mode_approach` 胶水 + `record_tag_reference.py` 示教脚本）；yaw 用 tag x 轴，不依赖 z 轴约定；`phase1_visual.yaml` 停止 0.20 m、开启接管；`phase1_perception_e2e.py` 固定为仅粗接近配置（Sol 修改）。
  - `fc4f441` W2：C++ 支持 front compact（6 维位姿、active base knots、contact roles/segment 支撑钉、front 校验、无右镜像、非硬件 feedback-checkpoint 路径）；front+hardware 仅在 `allow_front_hardware` 时接受，硬件强制 4 rad/s；`mode_climb` 新参数 `~climb_config_file`、`~allow_front_hardware`；金标重录。
  - `807e428` W3：`full_mission_sequence` 节点（home→spin_search→approach→climb→[home]→dock，失败即停 HOLD，`stop_after` 分段）+ `full_mission.launch`（AprilTag 检测、front_direct、foot gate 0.04、dock 开启）；Sol 删去过时说明和被 YAML 覆盖的 tag_id 参数。
- 实际运行的验证（本容器无 ROS/GPU/硬件，g++ 独立编译）：
  - tag_guidance gtest 17/17，无警告（Sol 复跑）。
  - C++ 金标 12 通过；front_direct 873 帧与 Python 误差 ≤1e-6；front+hardware 开关与 4 rad/s 限速单测通过；侧向回放与旧 C++ 逐字节一致（Luna 报告，Sol 未重跑）。
  - launch xmllint 通过；sequencer 仅用桩头文件做语法检查。
- 证据层级：CPU 单元/金标等价。未运行：catkin_make、roslaunch、现有 gtest/e2e、Isaac、实机。
- 未决与风险：
  1. 既有问题未修：C++ `reset_then_walk` 与 Python 在第 75 帧差 0.085 rad（行走步态不一致，影响自转/接近/tag 伺服）；FK ~1e-17 为本机 numpy 末位差。
  2. tag 参考位姿 `tag_ref_*` 为占位值，必须在正面 P0 示教；相机光轴沿机身 +y、忽略俯仰的假设未在实机确认。
  3. 0.20 m 停止距离贴近 YOLO 感知深度门限 (0.2, 1.0) m；tag 应在更远处完成接管。
  4. front_direct 仿真按 6 rad/s / 1.8 倍速排时，实机 4 rad/s，未在 Isaac 按 1.2 倍复验。
  5. 正面攀爬终态机身中心距对接点约 13 cm（小蓝系 x≈−0.132 m），超出 dock 约 20 mm 搜索；用户要求暂不处理。
  6. 正面攀爬与 dock 实机未认证，需吊架/保护与急停。
- 下一步（用户机器）：`catkin_make` 与 `run_tests_grasp_hexapod_bt_control`、`run_tests_grasp_hexapod_control`；贴 tag 并示教；`stop_after:=approach` 分段实机测试。

## 2026-10-30 低风险清理（W4）

- 基线：`test`，起始 `59edf36`。
- execution_owner：Luna（最轻档位、effort medium；>3 文件的机械删改），Sol 审查并补两处文档。
- 删除：`scripts/dock_mode_tag.cpp`（无引用、内容为 Python）；`test/reference/dock_mode_python.py`（与 `scripts/dock_mode.py` 逐字节相同，`generate_ring_fixtures.py` 改为直接 `import dock_mode`）。
- 修改：`launch_real_cpp.sh` 改为启动 `run_real_cpp.launch`；README/文档中 C1–C35→C1–C27、删除不存在的开机自启小节、`BT_INTERFACES.md`→`BT_INTERFACE_GUIDE.md`、servo C++ 链接与目录说明；`mode_topics.yaml`、两份 `MODE_DEV_GUIDE.md`、`bt_control_node.cpp` 报错字符串的腿序改为 `lb,lf,lm,rb,rf,rm`（仅字符串）；`MODE_DEV_GUIDE` 的 spin_search 描述更新；`AGENTS.md` 测试套件描述与 compact 字节 SHA 说明。
- 验证：`py_compile`、`bash -n`；在临时副本中重生成圆环 fixtures，PNG 逐字节一致，`expected.json` 仅浮点差 ≤1.8e-11（未覆盖提交文件）；两份 MODE_DEV_GUIDE `cmp` 一致。未运行 catkin/ROS。
- 剩余（未改，非本批范围）：`grasp_hexapod_bt` 脚本与 `hexapod_mission.xml`、`BT_INTERFACE_GUIDE.md`、`run_sim_ros_climb.launch`、`climb_retime.py` 仍有 `BT_INTERFACES.md` / C1–C35 字样；`CLIMB_ENGINEERING_RULES.md` 中为历史记录，保留。

## 2026-10-30 全流程桌面可视化

- execution_owner：Sol（5 个小文件、纯显示与日志，不改控制逻辑）。
- `full_mission.launch` 新增 `show_windows`（默认 true）：YOLO 标注窗口（框、中心、X/Z）、AprilTag 标注图 `/xiaolan/tag_detections_image` + 窗口、底部 dock 相机原图窗口；`mode_approach` tag 日志增加 tag_x/tag_z/tag_yaw；`package.xml` 加 `image_view`；README 说明。
- 验证：xmllint、tag_guidance gtest 17/17。未运行 catkin/roslaunch/实机显示。
- 未做：圆环对接感知无标注图（需改 ring_perception C++）；攀爬阶段无专门画面。

## 2026-10-30 全流程默认行为与调试手册

- execution_owner：Sol（文档 + 一处 launch 默认值）。
- `full_mission.launch` 默认值（`75f53f5` 起）：`start_camera`、`start_perception`、`start_tag_detector`、`start_dock_perception`、`show_windows` 均为 true；`start_sequence` 保持 false（用户可先看画面再开跑）；`stop_after` 空；`home_before_dock` false；`tag_id` 0 / `tag_size_m` 0.060；`climb_foot_gate_m` 0.04。
- 根 `README.md` 新增“实机全流程（正面攀爬）调试”：launch 默认表、窗口与终端输出说明、步骤 0–4（依赖与编译测试 → 只启动检查感知 → 贴 tag 与示教 → `rosservice call /grasp_hexapod/switch_mode` 单模式调试 → `stop_after` 分段串联）、串联节点退出码、常见失败信息对照、已知限制；`grasp_hexapod_bt_control/README.md` 指向该节。
- 调试顺序（Codex 接手时照此执行）：
  1. `catkin_make` + `run_tests_grasp_hexapod_bt_control run_tests_grasp_hexapod_control`；
  2. `roslaunch grasp_hexapod_bt_control full_mission.launch`（架空，看 3 个窗口与 `rostopic hz`）；
  3. 正面 P0（小蓝系 (−0.55, −0.038) m）示教：`rosrun grasp_hexapod_bt_control record_tag_reference.py _tag_id:=0 _samples:=30`，结果填 `phase1_visual.yaml`；
  4. 单模式：先 `home`，再 spin_search / approach / climb / dock；中止用 `home` 抢占或手柄 B；
  5. 分段：`start_sequence:=true stop_after:=spin_search|approach|climb`，最后完整流程。
- 状态：以上全部未在实机或 ROS 环境运行过；首个现场结果应追加到本文件。

## 2026-10-30 W5：tag 对准 + 末段盲走到 P0

- 背景：正面 P0 处前置相机贴近小蓝、无法成像 tag，无法在 P0 示教或伺服。用户确认采用"tag 对中推进 + 末段盲走"，并说明越近越好、碰到小蓝可接受。
- execution_owner：Luna（中间档位、effort high：状态机、几何符号、腿里程与 IMU 耦合）；Sol 逐行审查三轮（首次实现 → 4 项修正 → 航向目标修正）。
- 行为：Aligning（先对横向/yaw，2× 容差门内才前进）→ Advancing（tag 修正下前进）→ tag 不可靠（丢失 >0.3 s 或可选深度下限）时 Handover：最后一帧需在 1× 容差内；remaining = z_last − (tag_p0_z_m − 0.01) − 最后一帧后已走腿里程；>0.25 m 失败；≤0 直接停稳 → Blind：0.015 m/s 前进，腿里程（关节反馈 FK、前后两帧均支撑的足）计距，IMU 航向锁定到"最后一帧 IMU 航向 + tag yaw 误差"，指令积分仅监控（差 >3 cm 失败）→ 停稳 → ready_for_climb。tag 可见时比目标更近也算成功。伺服死区为 0.5× 容差。
- 新/改参数：`tag_ref_z_m` → `tag_p0_z_m`；新增 overshoot、handover_loss、min_reliable_z、max_blind_distance、blind_speed、heading_gain、max_heading_error、odom_mismatch、blind_timeout、`tag_imu_yaw_sign`（±1，IMU 安装方向）。
- 示教：P0 地面标记 → 沿朝向线直线后退到 tag 清晰处，量 D → `record_tag_reference.py _offset_from_p0_m:=D`，输出 `tag_ref_x_m`、`tag_ref_yaw_rad`、`tag_p0_z_m`（= 示教深度 − D）及标准差。D 误差 1:1 传到停车位置。
- 控制包仅新增只读 `ApproachMode::stanceMask()`。
- 验证：tag gtest 50/50（Sol 复跑）；平面闭环（0.5 m 起点、±5 cm、±8°、航向漂移 ≤0.02 rad/s）最终前后误差 <1 mm、横向 ≤7.5 mm、yaw ≤1.82°；金标与基线一致。理想化模型，不含真实步态跳变、打滑、相机延迟噪声。未运行 catkin/ROS/实机。
- 实机待确认：`tag_imu_yaw_sign`（README 第 1 步手转检查）；步态在 0.015 m/s 低速与停/启时的实际响应；腿里程在真实打滑下的误差；tag 实际丢失距离（看 handover 日志的 z_last）。
