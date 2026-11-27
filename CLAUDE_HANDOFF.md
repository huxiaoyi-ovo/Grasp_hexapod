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

## 2026-11-03 正面攀爬：direct 终态后接“走到中心”尾段（原型，暂停中）

- 基线：`test` 快进 `f161d6e → 84a3bc8`（拉取 14 个远端提交，无冲突）；本条之前未改任何源码/配置。
- execution_owner：Sol（几何与支撑推理紧耦合，原型在仓库外）。
- 用户决定：(1) 不重做 platform 路线，从 `climb_front_direct.json`（SHA `4421aff…`，终态机身 x .318）向后接尾段；(2) 13 cm 缺口通过“六足上小蓝后自行前走尽量接近中心”解决；(3) Claude 自行工作（非导师模式）。
- 关键事实（模型/CAD 诊断）：
  - direct CPU 回放复现 873 tick DONE。
  - 终态 LM/RM thigh .651，限位 .698：六足固定时机身前移约 1 cm 即触限，尾段第一步必须先迈中腿。
  - 机身底面 base z −.0097、前后 ±.086 m；小蓝中心凸台顶 .248 m（世界 x .43–.47）。必须先抬身再前移，否则底面撞凸台。
  - x≈.45 横向中缝；顶面 (≈.375,.03) 有小孔（platform 的 LB 终点正落在孔上）。
- 原型位置（仓库外）：`/home/artrc/grasp_sync_backups/claude_front_tail_20261103/`（`build_tail.py` 生成器、`run.sh` 生成+CPU 回放、`plan_v*.json`、地形与可达探针、`safe.png` 安全落脚图）。
- 候选序列：BODY_RAISE z .28 → MID_PAIR_UP (.385) → BODY_ADVANCE x .42 → REAR_PAIR_UP (.35) → LF_STEP (.55) → RF_STEP (.55) → BODY_CENTRE (.45, z .30)。静态关键状态与机身路径全部通过。
- 连续 CPU 回放（复用 `docs/evidence/climb_front_direct_20260924/cpu_direct_audit.py`）：v1 中腿抬腿速度截断；v2 中腿跨越时踝触限（机身过低）；v3（先抬身）前三个尾段通过，失败在 REAR_PAIR_UP 跨越段 joint_limit。v4a（后腿净空 .022）与 v4b（前移时机身升到 .30）已发起，结果未读取。
- 证据层级：CPU/CAD 模型诊断；未运行 GPU、未写入仓库配置、无实机授权。
- 下一步：读 v4a/v4b 结果并定位后腿触限关节；全链 CPU 通过后，把生成器放入 `scripts/tools/`、配置放入 `config/`，再做一次 GPU 预览，并检查实际足端落面、承载与中缝净空。
- 追加（暂停后后台结果）：v3 的后腿触限是 LB/RB 踝关节（ankle 余量 0）。**v4b（BODY_ADVANCE 终点 z .30）全链 CPU DONE：1257 tick、28 阶段、0 截断**，终态机身 (.45,−.0677,.30)；尾段最差：FK 9.15 mm（MID_PAIR_UP，<15 mm 硬门但 >2 mm 优选）、关节余量 .047 rad（BODY_RAISE/MID）、摆腿 COM 余量 23.7 mm、活动足球隙 ≥ −.1 mm。v4a 在 LF_STEP 触限。BODY_ADVANCE 之后各尾段有控制器胶囊 proxy 警告（仅提示，非 STL 自碰撞结论），下一步须先用指定连杆 STL 查询确认。`tail_v4b.json`、`cpu_v4b.json` 已存入上述备份目录。

## 2026-11-04 正面攀爬：用户否决 v4b 尾段，改为"姿态优先"重新规划（原型，暂停中）

- 基线：`test` @ `84a3bc8`；仓库源码/配置未改。execution_owner：Sol（物理推理紧耦合，原型在仓库外）。
- 用户反馈：v4b（direct+尾段）虽上去但姿态怪异——只追落点不顾姿态与稳定，六足上台后尤甚。要求从头优化；成对迈腿；在稳定基础上越快越好。用户确认机身离支撑面 ~85–90 mm。
- 姿态量化（相对 Q_STAND）：direct 前缀前腿前伸 93–152 mm、机身相对足平面倾 17–26°；v4b 终态六足内收 65–68 mm、机身高出自然 42 mm（"高跷"）。
- 原型：`/home/artrc/grasp_sync_backups/claude_front_gait_20261104/`。主程序 `build_gait3.py`（束搜索、姿态优先、对角/对称成对），`workspace.py`（髋系舒适工作空间，支撑 0.10 rad、摆动 0.05 rad 余量），`screen.py`（运行时参考逐帧 IK、限速重定时、机身底净空），`probe_*.py` 诊断脚本。最新命令：`python3 build_gait3.py --output cfg.json --beam 6 --max-depth 70 --free-y --mid-reach 0.13`。
- 已确认的物理约束（均为模型诊断）：
  1. 舒适工作空间呈月牙形，髋旁有不可达空洞；脚靠近髋（径向 <~0.11–0.16 m、髋下 <0.07 m）时不可达。固定步幅波浪步态会让前脚在台阶边落入空洞。
  2. 中腿 thigh 限位 ±0.698 rad 是前倾攀爬的主约束；名义足迹应沿机身法向而非世界竖直投影（后腿反之），落点候选同时围绕三种投影。
  3. 双前腿同时抬起需质心在中腿线后 ≥20 mm，需机身先后移；前脚未离地前机身不可先抬头（前腿会过伸）。
  4. 上台阶只能左右对称成对；对角成对仅用于同层行走，否则出现左右错位死锁。
  5. 机身 y=-0.0677 相对小蓝偏右 3.8 cm，小蓝左缘 y≈0.14，左中腿无横向空间；机身 y 需在拟合中优化（±2 cm）。右侧台阶比左侧高约 24 mm。
  6. 束搜索评分需奖励爬升高度（5×平均足高），否则会冲到墙边陷入死角。
- 进度：最后一次搜索（b16，`--free-y`）在 depth 9 已推进到机身 x=0.18（越过此前 x≈0.07/0.14 的所有卡点），被用户暂停；beam 状态已存 `cfg_b16.json.beam.npy`。尚未生成完整配置，未跑 CPU 连续回放/GPU。
- 下一步：续跑 b16 到终点 → screen 重定时 → `cpu_direct_audit` CPU 回放 → `posture.py` 姿态对比图 → GPU 可视回放给用户看。之后才考虑入库（`scripts/tools/` 与 `config/`）。

## 2026-11-10 正面攀爬：改为"模板 + 连续优化"（原型，进行中）

- 基线：`test` @ `84a3bc8`（与 origin 一致）；仓库源码/配置未改。execution_owner：Sol（目标函数/约束与物理推理不可分，原型在仓库外）。
- 用户决定：(1) 放弃 11-04 离散束搜索，改为固定模板 + 连续参数优化，连续轨迹检查放进优化回路，复用 SIDE 机身相对摆腿结构；(2) 落足确认将来由**足端接触传感器硬件**解决（尚未安装；装上后需更新 `PROJECT_MEMORY.md`/`AGENTS.md` 硬件边界）；(3) 仿真接触检测可作为**离线设计轨迹的测量工具**（记录触地瞬间足端相对机身坐标，修正规划），不作运行时输入；(4) 要求泛化：实机起点不一定精准，按小蓝坐标系规划、PREP 与首步吸收起点误差，鲁棒优化默认 x/y ±3 cm、yaw ±3°（待用户确认范围，尚未实现）。
- 原型：`/home/artrc/grasp_sync_backups/claude_front_opt_20261110/`：`geom.py`（闭式腿 IK、精确向量化 COM/连杆点/支撑余量，均已对照仓库 kinematics 验证；地形高度图、平整安全掩膜、带符号内缩、连通落脚区域）、`plan.py`（模板、与运行时 `_stage_reference` 同式的 Hermite/五次曲线、逐采样约束）、`opt.py`（CMA-ES、逐阶段顺序优化+一步前瞻）、`refine.py`（全链并行 CMA-ES）、`itinerary.py`（冻结接触区域表）、`export.py`（导出 COMPACT_V2 + contact_plan_version 1 配置并调用 `cpu_direct_audit`）。
- 关键发现（模型诊断）：
  1. 对称成对波浪步态下，抬前对时 COM 须在中腿线后 ≥20 mm，抬后对时须在其前 ≥20 mm，所以 F 与 R 摆动之间必须有中腿摆动或六足机身平移；模板定为 F→M→(B)→R→M。
  2. 斜面 G5 有 ≤5 mm 内部凹槽；按 AGENTS.md"首个落脚斜面忽略内部小特征"填平（≤8 mm），仍保留外缘/台阶边界。
  3. 原 18 mm 内缩与 15 mm 平整圆盘重复计边距（约 33 mm），改为 5 mm 额外内缩（总计约 20 mm）。
  4. 落脚连通区域：1 地面、2 斜面（x .194–.290）、6 顶面（含孔、中缝、凸台）；左中腿名义 y .126 超出顶面左界 y≈.10，需内收或机身右移。
  5. 修复 bug：`FINAL` 曾被误判为前对摆动阶段。
  6. 导出配置通过运行时校验；CPU 回放与模型 COM 指标逐毫米一致；参考限速 3.0 rad/s 时跟踪误差达 13.4 mm（硬门 15 mm），已降到 2.5 rad/s。
- 状态：尚无全约束可行链；瓶颈在坡道段后对抬腿 COM 与中腿过渡。未运行 GPU；无实机授权。
- 2026-11-10 追加（用户叫停）：在此方法下又做了三轮。
  1. 第一轮全链 CPU 回放 DONE（`cfg_t_s0.json`，21 s）。GPU 中机身俯仰在 −15°～+33° 来回摆、腿间穿模，F11 后翻摔倒。原因：目标函数没有约束机身姿态幅度/变化，也没有腿间连杆碰撞约束；CPU 已报胶囊碰撞警告，我当成"仅提示"放过去了。
  2. 补上腿间碰撞（与控制器 `_link_collision_free` 逐样本一致）、俯仰 ±20°/每阶段 ≤8°、机身 x 单调前进三条约束后，四组模板全部不可行，最好的 `cfg_m5b.json` 在 CPU R08 停住。
  3. 用户看 GPU：前 5 个阶段都是在小蓝前方地面上原地倒腿，脚根本没有搭上小蓝。用户判断思路从一开始就错了，**已叫停**。所有优化进程已停止；仓库源码/配置未改。
- 根因复盘（Sol）：让通用优化器在抽象模板上"发明"攀爬策略，而不是先按物理推理冻结关键接触状态（准则"接触状态表先于轨迹""终端优先"）；P0 前脚离斜面仅约 13 cm，一个循环就应搭上小蓝，模板却均匀分配了 5 个循环；每次 GPU 暴露的问题都靠事后补约束追，花了数小时却没有早期可视验证。
- 原型保留在 `/home/artrc/grasp_sync_backups/claude_front_opt_20261110/`，仅作工具参考（闭式 IK、向量化 COM/胶囊碰撞、地形区域、导出、`viz.py` 姿态图）。下一步方向等用户决定。

## 2026-11-10 正面攀爬：以 v10（optimized）为基线逐段提鲁棒性（原型，进行中）

- 用户决定：不再从零规划，在 v10 `climb_front_optimized.json`（de855ddf…，GPU DONE 39.3 s）上逐段优化；目标是实机起点误差下的泛化（设计范围 前后/左右 ±2 cm、yaw ±3°）。execution_owner：Sol（物理推理紧耦合；原型在仓库外，仓库配置未改）。
- 原型目录：`/home/artrc/grasp_sync_backups/claude_front_v10_stagewise_20261110/`。工具：`landing_tolerance.py`（每次落脚到台阶/边缘距离）、`chain.py`（阶段链锚点/位姿顺延重建，未改阶段字节不变，含 pose_knots）、`edit_*.py`（各步修改）、`retime.py`（预接触点+限加速度下降、空中按参考关节速度压缩）、`cpu.sh`（cpu_direct_audit 汇总）、`gpu_check.sh`/`envelope.sh`/`yaw_sim.py`（GPU 起点偏移扫描；yaw 用运行时替换小蓝放置函数旋转三角面，不改仓库）、`slip.py`（逐阶段支撑脚打滑）。
- v10 基线扫描：仅"偏后"不鲁棒——偏后 1 cm C14 RB 上台翻倒、偏后 2 cm C2 前脚从斜面下沿滑下；偏前 ±2 cm、左右 ±1.5 cm 通过。
- 已做修改（均 CPU DONE、零截断、零胶囊警告）：
  1. C2/C3 对调（中腿先前移），前对落斜面平整区中部 LF(.242,.100)/RF(.212,−.165)：抬前对 COM 21→33 mm、关节余量 .047→.117。
  2. C4 只迈 RF 到翼面 (.30,−.165) 并带机身前移（原 LF 落点在台阶边，偏前容差 5 mm）；C5 延长到 .7 s、C8×1.5。
  3. 用户要求"最后一对腿上台前中腿前移"：几何上后腿在地面时中腿无法上顶面（髋下不足 7 cm 时需水平 ≥15 cm）；改为 RM 沿翼面前移到 .315 + 机身预移到 (.28,z .22,pitch .40)，LB 接力 .148：抬 RB COM 18.6→53.8 mm。C14/C15 高弧降 30 mm 外移 10 mm（消除机身胶囊警告）。
  4. C16 LM 终点 .425→.40（大腿贴限位、离平整区边 11 mm）：关节余量 .005→.052。
  5. 用户要求"后两腿上台不往里收、小腿垂直落面"：RB→(.212,−.225) 轴角 31°→6°，LB→(.225,.105) 31°→13°；末段机身前移拆成两段避免 RB 拉直。
  6. 用户要求"空中快、落地慢"：`retime.py`（8 mm 预接触点、下降加速度 ≤1.5 m/s²、触地 .2–.3 s；空中 2.5–3.0 rad/s 参考上限）。
- GPU 鲁棒性结果（仿真诊断）：`v11_rear2b`（步骤 1–3，未重定时）名义/−1/−2/+2 cm 全过；`v11_rt`（+C16+2.5 rad/s 重定时）名义、±2 cm、±1.5 cm、−2±1.5 cm、yaw±3° 全过；`v11_rt3/rt4`（+后腿外放+更快重定时）偏后 1–2 cm 与组合失败。
- 关键发现：名义下支撑脚累计后滑约 30 mm（v10 同），偏后起点误差叠加后超出落点余量，是剩余鲁棒性瓶颈；放慢机身加速度（≤.25 m/s²）未减少总打滑（PREP 放慢后反而多滑），更像持续受载蠕滑，根因待查（可能含仿真接触参数伪影）。提速会降低偏差容忍度。
- 下一步待用户决定：以 `v11_rt` 为鲁棒基线重新逐项加入后腿外放并单项扫描验证；单独排查打滑根因；接近阶段把 P0 目标前移约 1 cm 让误差窗口居中。均未运行实机；仓库配置未写入。

## 2026-11-12 追加：速度/流畅性优化与鲁棒性现状（原型，未入库）

- 用户认可 v11_rt4 整体动作，要求"每步再优化速度和衔接流畅性"。
- 新增工具（同原型目录）：`smooth.py`（摆腿改机身相对 Hermite，中间节点带通过速度，越台阶后节点不允许提前向下切）、`merge_body.py`（机身阶段并入相邻摆腿）、`edit_c13.py`（C13 前对中间落点避开顶面横向中缝）、`edit_rear_out.py` 可传 LB x。
- 结论：
  1. 每步结束约 0.3 s 的到位确认（settle_gate persistence .25 s）累计约 6 s，是主要"顿挫"来源；属于实机安全门，未改，待足端接触传感器后改为触地即进入。
  2. 摆腿连续化提高流畅性，但会吃掉跨台阶净空 → 只用于不跨台阶的摆腿（跨台阶的 7 步保持原分段路径）。
  3. 合并 PREP 进中腿前移会在 4 足支撑下后滑约 3 cm → 撤回；C5、C9 机身阶段因 COM 约束必须单独存在。
  4. v10 遗留：C13 前对中间落点 x=.48 离顶面横向中缝（x≈.43–.47）只有 8 mm → 改为 LF(.491,.029)/RF(.52,−.165)，C17 关节余量 .034→.112。
  5. LB 上台落点 .225→.24（.25 触发 LB–LM 胶囊警告）。
- 当前最好候选 `v11_c13_rt.json`：CPU DONE；GPU 名义 38.5 s（v10 39.3 s），11 组起点偏差扫描 7 组通过（名义、+2 cm、±1.5 cm、yaw ±3°、−2/−1.5/+3° 组合）；偏后 1/2 cm 在 C18 三脚支撑时翻倒（RB 停在右侧仅约 36 mm 宽的窄斜面上，累计后滑 27–34 mm + 起点偏后后滑出边沿），另两组组合在 C13、LB 地面接力超时。
- 单次仿真结果对小改动敏感（同一条件在相邻版本间翻转），说明多处处于临界状态；结论均为仿真诊断，无实机授权，仓库配置未写入。
- 待用户决定的下一步：(a) 查累计后滑根因（PD 刚度/摩擦/接触参数对比，区分仿真伪影）；(b) 改 RB 停靠：缩短 RB 在右窄斜面的停留（顺序调整或直接上翼面）；(c) 攀爬中途用反馈重新对齐剩余计划（需可部署的位姿估计）；(d) 接近阶段 P0 目标前移约 1 cm 使误差窗口居中。

## 2026-11-12 冻结 front v11 部署版本并推送 test（实机部署前交接）

- execution_owner：Sol 写部署配置 `config/climb_front_v11.json`、`full_mission.launch`、`PROJECT_MEMORY.md`（实机关键文件，必须 Sol 审定）；Luna（最轻档位，effort medium：机械性归档/清理/编译/测试）负责证据归档、清理未提交文件、文档同步、编译与全部测试。Sol 审查 diff 后提交并推送。
- 部署配置：`src/grasp_hexapod_control/config/climb_front_v11.json`，SHA-256 `1683c12cf8f36626d789267f6258c1cac6628940916e84ba3f3820078797c6a8`，20 阶段，计划动作 32.7 s，`simulation_joint_velocity_limit_rad_s=4.0`（与实机 4 rad/s 一致），`terminal_q_rad` 已按最终锚点重算。`full_mission.launch` 的 `climb_config_file` 已改为它（`allow_front_hardware=true`、`climb_foot_gate_m=0.04` 不变）。
- 冻结验证（均已通过）：C++ 核心库按当前源码重编译（旧库缺 10-30 的 front 移植）；C++ 与 Python 逐帧一致（1179 帧，≤5e-6 rad）；C++ 实机模式（反馈检查点、门限 0.04 m、4 rad/s）理想舵机下 DONE，39.3 s，指令峰值 3.57 rad/s；Python 4 rad/s 回放零截断；Isaac（4.8 rad/s）名义 39.3 s DONE。
- 起点误差鲁棒性（Isaac 4.8 rad/s，仿真诊断）：通过 名义、+2 cm、−1 cm、±1.5 cm 横向、−3° yaw；失败 +3° yaw（C13）、−2 cm（后段 RB 窄斜面，累计后滑约 3 cm）；三组组合结果见证据目录 `sweep_summary`。
- 清理：Codex 10-26/27 未提交的 v10 optimized 路线（配置、生成器、落脚审计工具、测试与 `climb_front_rework_20261026` 证据）作为 v11 来源入库；已否决的 platform 路线（47 MB 证据、配置、生成器、测试）移到仓库外 `/home/artrc/grasp_sync_backups/uncommitted_platform_route_20261027/`（保留原相对路径，未删除）。`src/grasp_hexapod_mujoco/` 非本次产物，未改动、未提交。
- 全链路审查结论（ROS1/C++）：编排 home→spin_search→approach→climb→[home]→dock，每步阻塞 switch_mode，失败即 HOLD 不重试；tag 示教 P0（小蓝系 −0.55,−0.038）与攀爬 P0 一致；攀爬相位按控制帧推进（反馈慢只会变慢）；段内节点为反馈检查点（0.04 m / 0.08 rad，超计划时长 5 s 失败）；舵机每帧 33 ms；对接从当前关节姿态开始、底部相机 tag 驱动、dock_system 已标定；攀爬入口要求 18 关节距 Q_STAND ≤0.08 rad，接近结束无显式回站。

### 实机上线前必须做（顺序执行，任一级不过就停）

0. **机器人端**：`git pull origin test` → `catkin_make` → `catkin_make run_tests_grasp_hexapod_control run_tests_grasp_hexapod_bt_control && catkin_test_results`；`roslaunch --nodes grasp_hexapod_bt_control full_mission.launch`；确认参数 `climb_config_file=climb_front_v11.json`、`allow_front_hardware=true`、`climb_foot_gate_m=0.04`。
1. **tag 示教（阻断项）**：`config/phase1_visual.yaml` 的 `tag_ref_x_m/tag_p0_z_m/tag_ref_yaw_rad` 仍是占位值。按根 README 步骤：P0 地面标记 → 沿朝向线后退量 D → `rosrun grasp_hexapod_bt_control record_tag_reference.py _tag_id:=0 _samples:=30 _offset_from_p0_m:=D`，写回 yaml；同时手转确认 `tag_imu_yaw_sign`。
2. **悬空空跑攀爬**：机器人吊起、脚离地，`home` 后单独 `switch_mode climb`（P0 处、Q_STAND 姿态）。看终端 climb 速度诊断与检查点停顿：记录每阶段跟踪误差，确认无超时。若带载跟踪 >0.08 rad，先放慢对应阶段再继续。
3. **只跑接近**：`full_mission.launch start_sequence:=true stop_after:=approach` 重复 5–10 次，测量停车误差（前后/左右/yaw）与进入攀爬时关节偏差（需 ≤0.08 rad）。仿真可容忍：前后 −1~+2 cm、横向 ±1.5 cm、yaw −3°~+2°；若系统性偏后，把示教 P0 前移约 1 cm；yaw >2° 收紧 `tag_yaw_tolerance_rad`。
4. **实攀**：安全绳 + 急停，`stop_after:=climb`。观察 RTK/IMU 相对运动监控（位置 5 cm / 姿态 10° / 角速度 30°/s 持续超限会 HOLD）。
5. **对接**：完整流程。
- 未关闭：累计后滑根因；RB 停靠右侧窄斜面；步间到位确认约 6 s（待足端接触传感器）；+3° yaw。均为仿真诊断，不是实机安全证明。
- 冻结批次测试结果（本机 x86_64；Orbbec SDK 只带 arm64 库，本机只能按包编译 `grasp_hexapod_control`/`grasp_hexapod_bt_control`，机器人 arm64 不受影响）：
  - gtest 250 项，6 项失败，**均为本批之前就存在**（本批未改任何 C++/测试代码）：
    1. `KinematicsGolden`：FK 与 Python 差 2.8e-17，测试要求逐位相等——浮点舍入，无害。
    2. **`GoldenTrajectory.ResetAndWalkMatchPython`：C++ 步态在回站后开始行走的第 75 帧与 Python 差 0.085 rad**。已核实 Python 与金标逐帧一致（6.7e-15），即 **C++ 接近步态与 Python 参考不一致**；10-30 之后 C++ 仅加了只读 `stanceMask()`，问题更早存在。climb 金标（ClimbLeft/ClimbFrontDirect）通过。实机接近阶段运行的是 C++ 步态 → 上线第 3 级（只跑接近）必须实测停车精度；待单独排查。
    3. **`RingNative.MiddleFirstThenFourLegBodyRaise`：对接抬身时中腿关节速度 31–45 °/s，超过测试规定 30 °/s**（对接代码最后修改 9-18）。仍远低于 4 rad/s 硬件上限，但与设计意图不符 → 上线第 5 级（对接）重点观察；待单独排查。
  - bt_control gtest 全过（含 tag_guidance 50/50）；pytest：`test_climb_front_optimized.py`、`test_climb_front_direct.py` 全过；`test_real_mode_routing.py` 8 项失败与改动前 HEAD 完全相同（侧向 run_real/Isaac 旧断言），非本批引入。
  - `roslaunch --nodes full_mission.launch` 通过；v11 配置校验通过；catkin 白名单已复位。
- 分支：本工作区 2026-11-10 02:32 被切到 `codex/mujoco-interface`（非本会话操作；未提交的 `src/grasp_hexapod_mujoco/` 属于该工作）。本批通过独立 worktree 提交到 `test`，未切换本工作区分支、未动 mujoco 目录。

## 2026-11-13 Claude Code（Sol）：攀爬默认改为正面 v11

- 用户指定：实机手动入口是现有 `grasp_hexapod_bt_control/control_only.launch`（只按手柄才动）；直接把攀爬模式默认改成最新正面攀爬。
- 修改：`src/grasp_hexapod_bt_control/src/modes/mode_climb.cpp` 默认 `~climb_config_file` `climb_compact.json`→`climb_front_v11.json`、`~allow_front_hardware` false→true、`~climb_foot_gate_m` 0.02→0.04；`control_only.launch` 的 `climb_foot_gate_m` 默认 0.02→0.04（它显式传参，会覆盖节点默认）。`full_mission.launch` 未改（仍显式设同样的值）。侧面旧配置需显式 `climb_config_file:=climb_compact.json`。bt_control README 补手动攀爬用法与安全提醒。
- 影响：所有 include `control_only.launch` 的入口（full_chain、phase1_visual、full_mission）攀爬默认均为 v11 + 0.04；phase1_visual 仍 `enable_real_climb=false`。旧 `grasp_hexapod_control/run_real*.launch`（`run_real_cpp`）不读这些参数，不能用于正面攀爬。
- 验证：worktree 内 `catkin_make --only-pkg-with-deps grasp_hexapod_bt_control`（需 `-DPYTHON_EXECUTABLE=/usr/bin/python3`，conda python 缺 empy）通过；`run_tests_grasp_hexapod_bt_control` 180 项 0 失败。假舵机 ROS e2e（节点不设任何攀爬参数）：日志 `Compact climb config=climb_front_v11.json`，`switch_mode climb` → `climb complete`，40.17 s，指令峰值 3.57 rad/s，与冻结时一致。未在实机运行。
- 仍未修的阻断项：攀爬/对接中任何新 switch_mode（含手柄 B/A/Y）触发 2 s 全腿关节空间回正，挂在小蓝上会摔落；暂停只能用 `/grasp_hexapod/hold_motion` 租约（20 Hz 持续发布）。
- 另记：接近/行走步态 C++（实机，c3f6c81：`phase_duration 0.25`、`max_linear_acceleration 0.60`、`max_foot_planar_speed 0.30`、比例 2.5）与 Python（仿真：0.70/0.40/0.12/1.5）分叉，即 `ResetAndWalkMatchPython` 失败的来源；攀爬不受影响。待用户决定是否把 Python 对齐到实机值并重生成金标。

## 2026-11-13 Claude Code（Sol）：正面攀爬开发 SOP

- 新增根目录 `CLIMB_FRONT_SOP.md`：把 v10→v11 的逐阶段开发、检查阶梯、冻结、实机上线流程整理成 SOP，含尺寸变更时需同步的几何来源清单（kinematics.py / types.h / URDF×4 / workspace_bounds.csv / 金标）、P0 重算、v11 20 阶段接触序列模板、落脚/机身/时序规则、验收标准、`cpp_replay` 编译命令、踩坑表、工具现状与未解决项。
- 路由：`CLIMB_ENGINEERING_RULES.md` 开头和 `CLAUDE.md` 第 4 条指向该 SOP。`AGENTS.md` 未改。
- 只是文档整理，无代码/配置改动；数值均取自 v11 证据目录 README、`CLIMB_ENGINEERING_RULES.md` 11-12 关闭卡和本会话验证记录。
- 建议后续：把 v11 原型工具（chain、landing_tolerance、retime、hw_retime、cpp_replay、起点误差扫描）提升为 `scripts/tools/` 维护工具，参数化路径与阶段名。

## 2026-11-24 非攀爬：BT 遗留清理 + Python 行走步态对齐实机 C++

- 基线：`origin/test` `43d6444`（主目录 detached；`test` 分支被攀爬 worktree `/home/artrc/grasp_wt_test` 占用，故以 `git push origin HEAD:test` 推送）。用户要求本对话不动攀爬；MuJoCo 放弃（未入库的 `src/grasp_hexapod_mujoco/` 保留不动）；dock 不管。
- `7519524` BT 遗留清理（execution_owner：Sol）：`hexapod_mission.xml`、`hexapod_bt.py`、`run_real_bt.py`、`sim_manual.py`、`bt_mock_world.py` 的 `BT_INTERFACES.md`→`BT_INTERFACE_GUIDE.md`；`bt_mock_world.py` 与 `BT_INTERFACE_GUIDE.md` 的 C1→C35 改为 compact 阶段序列（默认正面 v11）。验证 py_compile + xmllint。**流程偏差**：该批 6 个文件按 AGENTS.md 应交 Luna，Sol 直接改了（纯文本替换），在此记录。有意未改：`run_sim_ros_climb.launch`、`climb_retime.py`（攀爬）、`CLIMB_ENGINEERING_RULES.md` 与 `src/docs/evidence/`（历史）。
- 第 1 项诊断：C++ `reset_then_walk` 金标第 75 帧差 0.085 rad 的根因是 `c3f6c81`（2026-09-14 “修改移动速度”）只改了 C++ `ApproachMode`：`phase_duration` 0.70→0.25、`max_foot_planar_speed` 0.12→0.30、`max_linear_acceleration` 0.40→0.60、自动接近比例项 1.5→2.5（另 `run_real_ring_cpp.launch` max_linear_speed 0.30）。C++ 副本改回三参数后 6/6 金标通过，算法本身一致。
- 用户决定：实机（C++，所有实机入口 `control_stack_cpp`/`bt_control_node` 都用它）不改，实机快步态走过没问题；Python 默认跟 C++。
- `920f129`（execution_owner：Luna，最轻档位，effort medium；测试更新 + 金标重录）：Python `approach_mode.py` 四个值同步；`test_real_mode_routing.py` 三个行走测试按快步态实测重设门槛（关节余量 ≥0、足端单帧步长 ≤25/31 mm、限速截断 ≤220、锁存检查移到第一相内部）——这些是回归报警，不是安全裕度；金标只替换 `reset_then_walk`，其余键逐字节不变。
- 验证：pytest `test_real_mode_routing.py`+`test_approach_navigation.py` 83 通过 / 9 失败（与基线相同，均为本机无 isaacgym）；C++ `test_golden_trajectory` 6/6、`test_golden_config` 7/7；`test_kinematics_golden` 1 失败为既有 2.8e-17 FK 末位差（改前构建同样失败）。未运行 catkin/ROS/Isaac/实机。
- 风险记录（理想伺服 CPU 回放，非实机证据）：快步态 T=0.25 s 下 lm 踝关节起步第 2 帧压到 2.094 rad 软件限位，每帧 3–5 个关节被 4 rad/s 指令限速截断，足端跟踪误差峰值 44–67 mm（T=0.70 时 14 mm）。用户实机经验优先；若实机出现抬腿不足、落脚错乱或踝关节撞限位，从这里查。
- 待办：Isaac 仿真快步态未跑（本机无 isaacgym），需在用户机器上看一次行走；`run_real.py`（Python 实机入口）行走也随之变快。
- 补充（同日）Isaac 验证：isaacgym 在 conda 环境 `grasp_hexapod`（无 cv2）。该环境下 pytest 两文件 84 通过 / 8 失败，8 个失败在改前 `43d6444` 上完全相同（launch `enable_link_collision_check` 默认值 + compact 攀爬/硬件攀爬测试，属攀爬侧既有问题，本对话未动）。
- `run_sim.py --headless --record-servo-trace` 快（当前）/慢（`43d6444`）步态各跑一次，均正常结束。FK 足端误差峰值 前进/右移/旋转：快 33/30/17 mm，慢 27/21/18 mm；限速饱和帧比例 快 79–93%，慢 17–64%。**快步态实际抬腿只有约 22–29 mm**（关节目标本身被 4 rad/s 限速截断，指令 40 mm 达不到），慢步态约 43 mm。平地可走；越障或不平地面的抬腿余量偏小。trace 无机身位姿，未量实际行进距离。

## 2026-11-16 Claude Code（Sol）：正面攀爬 v12 候选（按实机反馈改 C8/C12/后腿上台；原型，未入库）

- 基线：`test` @ `7012f36`，在独立 worktree `/home/artrc/grasp_wt_test` 中工作；主工作区仍是 `codex/mujoco-interface`，未动。仓库配置/源码未改。execution_owner：Sol（物理推理紧耦合）。
- 用户实机反馈（v11）：(1) C12 LF 在顶面前伸太低、贴面被挡；(2) C8 LF 从斜面迈顶面抬高不够、被台阶挡；(3) 两条后腿上台前移不足、落在小蓝前踩空，RB 先踩空即失稳。用户建议"中腿先前移再上后腿"。
- 根因（模型）：跨越段规划净空仅 20 mm（`clearance.py` 实测 C8/C12 均 20.0 mm），小于实机舵机下垂/滞后且小于 0.04 m 段内门限；RB 落在右侧窄斜面 (0.212,−0.225)，离坠落边仅 42 mm，且边前移边下降。
- 原型目录 `/home/artrc/grasp_sync_backups/claude_front_v12_20261116/`：`edit_front.py`（C8 顶点 .25 平移后竖直落；C12 原地抬 20 mm→斜爬到 x+.04 顶点 .25→平移→竖直落）、`edit_rear.py`（C14 RM→翼 .36；C15 RB→右翼 (.285,−.262)；C16 LB→(.255,.10)；后腿高处平移到落点正上方再竖直落）、`edit_tripod.py`（C17 机身 +.025，C18 三足抬腿前不回退）、`clearance.py`（跨越段足球/小腿对 STL 竖直净空）、`commargin.py`（逐帧 COM 余量）、`capdiag.py`（控制器胶囊项分解）。候选 `v12_hw2.json`（hw_retime 后，计划 33.0 s）。
- 结果：C8 LF 净空 20→53.5 mm，C12 LF/RF 平移段 20→约 53 mm；CPU DONE 1183 tick、零截断、零胶囊警告；4 rad/s 零截断。Isaac 4.8 rad/s 名义 DONE 39.4 s、五门 PASS；仿真中 RB 实际落在翼面（落短 16 mm 仍在翼面，离坠落边 99 mm）。
- 起点误差扫描（11 组）：7/11，与 v11 冻结版同数但分布不同。通过：名义、−1、**−2 cm**、±1.5 cm、−2/−1.5/+3°、**−2/+1.5/−3°**（v11 后两组/偏后 2 cm 失败）；失败：+2 cm、−3°（C15 翻）、+3°、+2/+1.5/+3°（C16 翻）。
- 失败根因：C15/C16 在 5 足摆腿中同时做大机身动作（C15 前移 5 cm+侧倾 .2；C16 前移 3 cm+俯仰 .36），违反 SOP 5.4"机身大动作只在六足支撑下"。静态分析：RB 在地面 0.055 时六足机身最多到 x≈.30，RB 上翼面需 ≈.33；加 RB 地面中继(.13,−.18)后，六足机身 (.34,.23,俯仰.45) 可让 RB 静态机身上翼面，但胶囊余量仅约 2 mm。"LM 先上顶面再上 LB"静态不可行（LB 在地面时 COM 在 RB–LM 边外 10–27 mm）。
- 等用户决定：是否接受新增 RB 地面中继 + 两个六足机身阶段（改接触序列，约 +4 s），或退回 RB 斜面方案 A（前移到 .228 + 竖直落，改动小、容差提升有限）。前腿 C8/C12 修改可独立保留。
- 未运行 C++ 逐帧/gtest/ROS e2e；无实机授权；所有结论为 CPU/Isaac 仿真诊断。

### 2026-11-16 补充（Sol）：后腿落点按“不碰撞不倾倒前提下尽量往前”重新评估
- 用户反馈：实机后腿约落短 2–3 cm。用户定原则：后腿落点不按刚好设计，在不碰撞、不倾倒前提下尽量往前。
- 纠正：v12_hw2 的 RB (0.285,−0.262) 离右翼后沿台阶仅约 30 mm（inset 18 mm），此前“落短 115 mm 才掉”的说法错误；方案 2（RB 斜面 0.228）也在斜面安全区外（y−0.24 处斜面安全仅到 x≈0.22），作废。
- 静态机身（SOP 5.4）+RB 地面中继 (0.13,−0.18) 搜索（scratchpad/plan/maxfwd*.py）：不转机身时 RB 落翼面 x≥0.28 均与机身圆柱碰撞（capsule −0.4~−13 mm）；机身 yaw +10°（pose yaw −80°）后 RB (0.29,−0.30) capsule 7.9 mm / COM 89.6 mm / jm 0.075，(0.31,−0.30) capsule 5.1 mm，离翼面后沿约 40–60 mm。
- LB 左斜面是有界区（后沿 0.19 安全起，前方 0.30 起为台阶），最远可用约 0.26–0.27。
- 待用户确认接触序列变更（RB 地面中继 + 六足支撑机身前移/转 yaw + 静态摆腿）后再实现与验证。

### 2026-11-17 Sol：v12 后腿静态序列（用户 2026-11-16 确认序列；原型，未入库）
- 用户确认序列：RB 地面中继 → 六足机身前移/抬头/转 yaw → 机身静止 RB 上翼面 → （六足微调）机身静止 LB 上斜面 → 六足转回 yaw → 原三足阶段。脚本 `grasp_sync_backups/claude_front_v12_20261116/edit_seq.py`（+edit_tripod.py dx0.04 修了 pose_end/rebuild、edit_diag.py LM/RF 抬脚点前移 0.03/0.02）。
- 新增硬约束：机身底面（按六边形外接圆 r0.122 圆盘）离地 ≥15 mm（v11 实机最小 16 mm）。未加此约束时 PB 位姿机身后右离翼面 1.6 mm，已排除。
- 约束下最远落点：RB (0.29,−0.30)（翼面，后沿约 40 mm，capsule 5.5 mm），LB (0.23,0.10)（斜面，离安全后沿 40 mm；被 LB–机身碰撞+LB 地面起脚需低机身卡住）。位姿 PB=(.33,z.22,抬头.36,侧倾−.1,yaw+10°)，PC=(.34,.23,.36,0,+5°)，PD=(.365,.265,0,0,0)。
- 候选 `v12_s4hw.json`：CPU DONE 0 clip 0 capsule，min jm 0.043，机身离地全程 ≥16 mm，计划 44.8 s；Isaac 名义 DONE 46.7 s 五门 PASS。dx0.03 版本 Isaac 在 TRIPOD 翻倒（COM 41.7 mm），dx0.04 后 49.2 mm 通过。
- 发现：Isaac 中 RB/LB 实际落短约 2 cm，但支撑脚（LF/RM）同样整体落后约 2 cm，是前段累积打滑导致的整体后移，非摆腿误差；可能就是实机落短 2–3 cm 的来源。落点前移受机身相对碰撞边界限制，整体后移无法靠前移落点抵消。
- 原型 edit_reorder.py（RB_FINAL 提前）未采用：机身 0.36 时 RB 目标关节限位+capsule WARN。
- 11 组起点误差扫描进行中（gpu_v12s4_sweep/）。
- 2026-11-17 扫描结果 v12_s4hw：6/11（v11 7/11，v12_hw2 7/11）。通过：名义、−1cm、±1.5cm(y)、−2/−1.5/+3°、−2/+1.5/−3°。失败：−2cm、±3° yaw 在 TRIPOD（LM/RB 抬起=翻倒，三足 COM 余量不足）；+2cm、+2/+1.5/+3° 在 SIX_BODY_RB_WING/SIX_BODY_LB/LB_SLOPE_STATIC（地面中继脚滑移 25–35 mm；LB 地面中继 0.148 离斜边 +x 容差仅 4 mm，v11 遗留）。六足机身动作放慢（v12_s5hw）无效，+2cm 反而更早失败。
- 待用户定：三足 COM 余量需改接触顺序（RB 最后一步提前到对角阶段前，需重定 RB 目标）或其他方案。
- 2026-11-17 用户选 A（RB 提前）。静态检查否定 A：三足翻倒方向是越过 LM–RB 边，该边绕 LM 转，RB 落点前移（含 (.335,−.165)）三足 COM 只在 46–50 mm 间变化；机身 0.365 时 RB 前移 >0.30 与机身碰撞。未实现 A。
- 真实起因：三足失败工况（−2cm、±3°）均为 RF 先滑 5–10 cm；RF 对角落点 (.618,−.16) 在顶面前沿下倾斜边、inset 16 mm（v11 遗留）。改 RF 落点 x0.59（edit_rf.py，tol 四向 40 mm），CPU 余量不变 → `v12_rfhw.json`（hw_retime 零拉伸，计划 39.1 s 段时长合计，CPU DONE 0 clip，min jm 0.043）。Isaac 复测进行中（gpu_v12rf/）。
- 机身横移 −y（edit_lat.py）三足 COM 每 cm 仅 +2.4 mm 且对角 COM 下降，未采用。LB 地面中继后移（edit_lbrelay.py）≥6 mm 即 LB 起脚 IK 失败，保持 0.148（+2 cm 工况仍受限）。
- 2026-11-17 `v12_rfhw.json` 起点误差扫描 **9/11**（v11 7/11，v12_hw2 7/11）：通过 名义、−1/−2 cm、±1.5 cm(y)、±3°、−2/−1.5/+3°、−2/+1.5/−3°；失败 +2 cm（SIX_BODY_LB）、+2/+1.5/+3°（LB_SLOPE_STATIC），原因为 LB 地面中继 0.148 贴近下沿斜边（v11 遗留，序列内不可后移）。三足 RF 滑移 3–5 mm。证据 grasp_sync_backups/claude_front_v12_20261116/gpu_v12rf/。下一步：给用户看 Isaac 名义 GUI；之后 slip 检查、冻结步骤需用户批准。
- 2026-11-17 用户：打滑不管；整体方向对；按通用原则合并阶段（摆腿时机身同步动作，不死等落地）。采用规则：机身可在摆腿抬起/平移段同步动作，但须在该腿下落段开始前到位（落地段机身静止）；逐帧 COM ≥35 mm；扫描不劣于 9/11。（与 SOP 5.4 的关系已告知用户，规则文件尚未改。）
- 合并版 `edit_seq2.py` → `v12_mhw.json`：RB 地面中继与机身前移/抬头/转身合并（RB_RELAY_BODY）；LB 上斜面时机身同步升高 PC→PL=(.35,z.27,.12,0,+5°)，LB 落点 0.23→0.24，膝部不再压缩（LB_SLOPE_BODY）；去掉 SIX_BODY_RB_WING、SIX_BODY_YAW_BACK（yaw 回正并入 DIAGONAL）。保留 0.5 s SIX_BODY_LB：LB 在 PB 起脚 COM 仅 30.6 mm。CPU DONE 0 clip 0 capsule，对角 jm 0.065→0.139，机身离地 ≥16 mm，计划 44.8 s（上一版 46.7）。
- 前段 C5（只调 roll）、C9（机身前移 7.6 cm）物理上必需：并入相邻摆腿后 COM 出支撑（C5 并入 C6：7.3 mm；C9 并入 C8/C10：COM 越过 LM/RM 线），保持不动。
- Isaac 名义 + 11 组扫描进行中（gpu_v12m/）。
- 2026-11-17 合并版 v12_mhw 扫描 8/11（+3° 在 TRIPOD 翻；+2cm、偏前组合同前）。根因：LB_SLOPE_BODY 中机身升高+yaw 回正（五足）时斜面上 LM 滑 18 mm（静态版 9 mm），对角阶段机身左偏 2.5 cm。PL yaw 0（v12_m2hw）+3° 仍失败。yaw 全放进 SIX_BODY_LB 时 LB 只能静态落 0.23。
- 折中版 v12_hhw（只合并 RB 中继+机身前移；LB 静态 0.23；保留 SIX_BODY_LB、SIX_BODY_YAW_BACK）：CPU DONE 0 clip，46.2 s（v12_rfhw 46.7 s）。全扫描进行中 gpu_v12h/。
- 2026-11-17 折中版 v12_hhw 扫描 8/11（+3° 仍在 TRIPOD 翻）。结论：+3° 三足工况处于边缘，合并任一处都可能打破；按约定回到 `v12_rfhw.json`（9/11，46.7 s）作为当前候选。合并相关脚本 edit_seq2.py / 结果 gpu_v12m*、gpu_v12h 保留作记录。
- 2026-11-17 用户要求：六腿都上背后精简阶段、最终站姿居中对称舒展；宗旨“自然、快速，不为落脚点硬加阶段”。新脚本 `edit_top.py`：LB_SLOPE_STATIC 后 = SIX_BODY_YAW_BACK(→(.38,−.10,.30) 水平 yaw0) → TRIPOD_RF_LM_RB(机身静止) → TRIPOD_LB_LF_RM(机身摆腿中同步到 (.43,−.105,.30)，下落前到位) → FINAL_HOLD；删除 RB_FINAL_ADVANCE。终点足点 lb(.267,−.058) lf(.543,−.013) lm(.405,.089) rb(.287,−.202) rf(.598,−.202) rm(.43,−.294)，落点容差全部 ≥23 mm（右侧三腿即标准六边形位置；左侧受前斜面凹槽/后台阶限制）。抬脚高度按路径地形最高点 +25 mm 逐腿给。`v13_ehw.json`：CPU DONE 0 clip，三足 COM 44.8/50 mm、jm 0.105（v12_rfhw 39.6 mm/0.064），终态 jm 0.479（原 0.128），摆腿净空 ≥30 mm，机身离地 ≥19.5 mm，总 42.6 s（v12_rfhw 44.8 s）。Isaac 11 工况扫描进行中 gpu_v13e/。
- 2026-11-17 用户提出“前中腿上去后当四足走、后腿抬起不管，到中心再放下”。静态分析（模型）结论不可行：①切换瞬间（后腿着地需机身抬头 roll≥0.2）四足 COM 余量上限约 28 mm（<35）；②四足下中腿摆动时三足 COM 为 −51/−70 mm（中腿髋在机身中线，后伸受 thigh ±0.70 rad 限制，最多到机身后 7–8 cm），机身也无法再前移。脚本 scratchpad plan/quad*.py。
- 2026-11-17 `v13_ehw.json` Isaac 11 工况扫描 **9/11**（与 v12_rfhw 相同）：失败仅 +2 cm（停在 SIX_BODY_LB）、+2/+1.5/+3°（停在 LB_SLOPE_STATIC），均在新改动之前、为 LB 地面中继 0.148 遗留问题；新的末段三足在所有通过工况中均通过。名义工况末段 C21/C22 足端误差 12 mm、root 22 mm。证据 gpu_v13e/。当前候选改为 `v13_ehw.json`（42.6 s）；冻结需用户批准。
- 2026-11-17 用户确认放弃“前中腿上去后四足行走”方案（静态不可行，见上条）。继续以 `v13_ehw.json` 为候选。
- 2026-11-17 用户反馈 v13 终态左前腿太收缩（原 lf (.543,−.013)，相对机身 +0.113 前伸，髋伸展 63 mm）。改 lf → 前斜面 (.635,−.06)（h 160 mm，容差四向 ≥30 mm；相对机身 (+0.205,+0.045)，与 LB (−0.163,+0.047) 前后对称）。`v13_fhw.json`：CPU DONE 0 clip，C22 COM 51.6 mm、jm 0.292，终态各腿 jm 0.47–0.94、胶囊 17.7 mm，LF 摆腿净空 31.5 mm。GUI 名义已开；11 工况扫描进行中 gpu_v13f/。
- 2026-11-17 `v13_fhw.json`（LF 前斜面）扫描 9/11，失败同前（SIX_BODY_LB / LB_SLOPE_STATIC），证据 gpu_v13f/。
- 2026-11-17 用户：“LB 上台时机身和其他腿同步动，它腾空不支撑”。新脚本 `edit_merge.py`（输入 v13_fhw）：RB_WING_STATIC 后 = SIX_BODY_LB(PB→PC 六足，保留：CPU 审计按活动腿全段不支撑计 COM，合进摆腿阶段会报 30.6 mm) → LB_UP_BODY_LEVEL(仅 LB 活动：抬起外绕到 (.17,.13,.20)，机身在 LB 高位时 PC→P1(.37,−.11,.29) 放平 yaw0) → TRIPOD_LB_LF_RM(机身静止；LB 从空中直接落最终点 (.267,−.058)，不再落斜面中继 (.23,.10)；LF/RM 同步到终点) → TRIPOD_RF_LM_RB(机身→PF) → FINAL_HOLD。删除 LB_SLOPE_STATIC、SIX_BODY_YAW_BACK。LB 跨阶段悬空（无 contact_plan，validate 通过）。单阶段写法失败原因：活动腿在机身转动段着地会因基座插值弦误差穿地 0.7–2 mm（actual_active_foot_sphere_gap）。`v14_ehw.json`：CPU DONE 0 clip，COM 39.4/78.8/51.1 mm，jm ≥0.107，净空 ≥31 mm，落点容差 ≥23 mm，总 41.1 s（v13_fhw 43.0）。Isaac 扫描进行中 gpu_v14e/。
- 2026-11-17 `v14_ehw` 扫描 0/11：TRIPOD_RF_LM_RB（支撑 LB 后斜面终点 + LF 前斜面终点 + RM）中 LB 承载 ~57%，Isaac 中 LB 踝关节被压 0.2 rad、机身该角下沉 4.8 cm、LF 被拖滑 26 mm（LF 因 ~2 cm 系统滞后实落 0.617 贴台阶）。机身静止也失败（v14_fhw）。教训：斜面低位终点脚不宜作三足支撑；LF 前斜面终点必须最后落。
- 修正 `v14_ghw.json`：SIX_BODY_LB → LB_UP_BODY_LEVEL → LB_RM_FINAL（LB 直接落终点、RM 终点，LF 留顶面；机身 P1→P2(.38,−.10,.29)）→ TRIPOD_RF_LM_RB（机身静止，LB 负载 ~40%）→ LF_FINAL_BODY_CENTRE（LF 最后落前斜面，机身→PF）→ FINAL_HOLD。CPU DONE 0 clip，COM 39.4/84.5/69.8/87.6 mm，jm ≥0.104，净空 ≥31 mm；Isaac 名义 SUCCESS，新阶段支撑漂移 ≤3.8 mm。总 42.9 s（v13_fhw 43.0 s，几乎无时间收益）。其余 10 工况扫描进行中 gpu_v14g/。
- 2026-11-17 Sol 补记 `v14_ghw` 扫描（gpu_v14g/，读日志汇总）：已完成 10 组中 8 组 SUCCESS（名义、−1/−2 cm、±1.5 cm(y)、−3°、−2/−1.5/+3°、−2/+1.5/−3°）；失败 +3°（C20 LB_RM_FINAL 超时 7.23 s：RB 侧机身下沉，RB 足相对机身误差约 47 mm z、LB 约 36 mm z，lb_ankle 0.133 rad）、+2 cm（C18 SIX_BODY_LB，LB 足误差 21 mm 刚过 20 mm 门，为 LB 地面中继 0.148 遗留）；+2/+1.5/+3° 日志停在 C13，未跑完。
- 2026-11-17 讨论（无仓库改动）：用户计划足端压电薄膜仅做二值接触确认；对比 RL 残差与"v11/v14 + 接触状态切换规则"，Sol 建议先做规则版，RL 暂缓。按 v14_ghw 的逐阶段分析见本会话回复。
- 2026-11-17 `v14_ghw` 全扫描 8/11（+3° 失败），弃用。
- 2026-11-18 用户反馈 5 条：①仿真整体变慢（实为计划变长：v11 37.4 s → v13/v14 ~43 s，增量全在后腿段：摆腿时机身静止、机身单独成段、空中多途经点）；②后腿上台时机身保持倾斜；③支撑腿小腿杆尽量垂直接触面，只许外撇不许内撇（六足上台后尤甚）；④六足上台后：原地正常站立 → 走两步到中心，不要磨叽；⑤LB 抬起后空中不要再调整。
- 新度量 `plan/shank.py`（备份目录内 plan/，原 scratchpad 已被清空，已重建 reach/state/seq/stagejm/capwho）：支撑腿小腿相对接触面法线的径向倾角，+外撇/−内撇；标准站姿 = 0，机身每高 3 cm 内撇约 7°。v13/v14 终态机身 z 0.30 → 内撇 7–12°，中间阶段达 −22°。
- `edit_v15.py`（输入 v13_fhw）→ `v15_ihw.json`：RB_WING_BODY（RB 一次斜向外上抬到 (.155,−.31,.235)→直线到翼上方→落；横移段机身 P15→PB，最短 1.2 s：0.49 s 时 RM 滑 25 mm）→ SIX_BODY_LB → LB_SLOPE_BODY（LB 斜向外上抬到 (.15,.13,.20)→到 (.23,.125) 上方→斜向内落 (.23,.10)，机身保持倾斜 PC）→ NORMAL_STAND（六足：放平 yaw0，(.38,−.10,.265)）→ STEP_A_RF_LM_RB（机身只升到 z .28 不平移：LB 斜面支撑时平移会滑 23 mm）→ STEP_B_LB_LF_RM（机身 → (.41,−.10,.27)；走 7 cm 时 RF 被拖滑 31 mm）→ SIX_BODY_CENTRE（六足走完最后 4 cm 到 (.45,−.105,.265)）。终态足点 lb(.27,−.05) lf(.625,−.045) lm(.41,.09) rb(.30,−.24) rf(.615,−.20) rm(.45,−.30)，小腿 −1.4…+5°。过渡内撇：NORMAL_STAND rf −15 / rm −14，STEP_A 后 rm −17（需改 C12/C14 前腿落点才能根治）。CPU DONE 0 clip，COM ≥36.8 mm（C3 FRONT_TO_FACE_CENTRE 33.2 mm 为 v11 起遗留），jm ≥0.061，总 40.5 s。Isaac 名义 SUCCESS（终点 root 36 mm）。11 工况扫描进行中 gpu_v15i/。
- 2026-11-23 Sol 补记 `v15_ihw` 扫描（gpu_v15i/，读日志汇总，无新运行）：已完成 10 组中 8 组 SUCCESS（名义、−1 cm、±1.5 cm(y)、±3°、−2/−1.5/+3°、−2/+1.5/−3°）；−2 cm 攀爬 DONE 但终点 root 误差 63.5 mm 超 50 mm 门；+2 cm 在 C17 SIX_BODY_LB 失败（LB 足误差 0.41 m、root 0.36 m，即翻落；仍为 LB 地面中继 0.148 遗留）；+2/+1.5/+3° 日志停在 C13，未跑完。v15 未冻结、未入库，部署版仍为 v11。
- 2026-11-23 Sol：Isaac GUI 名义重放 `v15_ihw`（gui_v15i_1123.txt）DONE 42.3 s，各阶段跟踪 ≤0.044 rad（C16 最大）、足端 ≤13.7 mm、支撑漂移 ≤14.4 mm（C16）；窗口在门限打印前关闭（11-18 同工况 headless 五门 PASS，终点 root 36 mm）。用户看后评价“整体很流畅，比之前有很大进步”，以 v15_ihw 为基线继续优化。

## 2026-11-23 Claude Code（Sol）：正面攀爬 v16 候选（RB 直接上台 + LB 让开 LM；原型，未入库）

- 用户反馈（看 v15 GUI 后）：①RB 上台前的地面前移一步（C15 RB_GROUND_RELAY_130）没必要，直接上台；②LM 支撑位偏后，LB 上台空间小易碰 LM，建议 LM 前移并加重心转移步骤。execution_owner：Sol（物理推理紧耦合）。原型目录 `/home/artrc/grasp_sync_backups/claude_front_v16_20261123/`（`edit_v16a.py`、`edit_v16b.py`、`lblm.py`）。
- ① `v16_ahw.json`：删 C15，RB_WING_BODY 从地面 (.055,−.165) 直接抬到 (.155,−.31,.235)→翼面 (.29,−.30)，机身 P15→PB 不变。CPU DONE 0 clip 0 capsule，RB 段 COM 39.3 mm、jm .061；21 阶段 38.6 s（v15 40.5 s）。Isaac 扫描 gpu_v16a/（v12 目录下）进行中：名义、−1 cm SUCCESS，−2 cm 同 v15 仅终点 root 63 mm 超 50 mm 门。
- ② 模型核实：v15 C18 t≈1.89 s LB foot2 与 LM foot1 胶囊间隙 −0.5 mm（确实碰撞）。LM 前移不可行：LM 所在 12° 斜面带安全区前缘 x≈.29（.315 处 2 cm 台阶，外侧 y≈.17 坠落边），下一安全区为顶面 x≥.335，此时 LB 抬起 COM 最多 17 mm；斜面带内最多 +1.5 cm 且 COM 36.9 mm、间隙仍小。换序（LB 先于 RB）不可行：LB 上斜面后 RB 从地面抬起 COM −8~−11 mm。注意：landing_tolerance 在落点本身不安全时会误报 40 mm（bad[0]=0 → STEP[-1]），已在 lblm 扫描中先检查 safe。
- 采用 `v16_bhw.json`（v16a + LB_SLOPE_BODY：落点 (.23,.10)→(.22,.10)，抬起高度 .185，平移到落点外 5 mm 正上方后近竖直落）：LB 路径对 LM 间隙 −0.5→21.9 mm，对机身 9.0→12.4 mm（纯竖直落会撞机身 −6.6 mm，外侧斜落即为此）。CPU DONE 0 clip 0 capsule，LB 段 COM 39.7 mm，hw_retime 4 rad/s 零截断，21 阶段 38.3 s。GUI 名义已开（gui_v16b.txt），11 工况扫描待 v16a 扫描完成后跑。
- 遗留：BACK_TO_MINUS09 / BACK_TO_055 CPU COM 11.8/15.5 mm（v15 起相同）；LB 地面中继 0.148（+2 cm 工况）。无实机授权，结论为 CPU/Isaac 仿真诊断。
- 2026-11-23 用户同意采用 LB 中间落点 0.22（v16b），理由：0.23 的 12 mm 间隙与实机跟踪误差同量级；LB 终点 (.27,−.05) 不变；−x 容差 31 mm 仍覆盖实机落短 2–3 cm。实机测试需单独记录 LB 实际落点。
- 2026-11-23 `v16_ahw` Isaac 11 工况扫描 8/11（gpu_v16a/）：通过 名义、−1 cm、±1.5 cm(y)、±3°、−2/−1.5/+3°、−2/+1.5/−3°；失败 −2 cm（仅终点 root 63 mm 门）、+2 cm（SIX_BODY_LB 超时）、+2/+1.5/+3°（LB_SLOPE_BODY 超时），后两者同为 LB 地面中继 0.148 遗留。与 v15 已完成的 10 组结果一致，RB 直接上台未降低鲁棒性。v16b 扫描进行中 gpu_v16b/。
- 2026-11-23 `v16_bhw` Isaac 11 工况扫描 **9/11**（gpu_v16b/，v15/v16a 8/11）：新增通过 +2/+1.5/+3°（v13/v14/v16a 均在 LB 上斜面超时）；失败 −2 cm（仅终点 root 62 mm 门）、+2 cm（SIX_BODY_LB，LB 地面中继 0.148 遗留）。当前候选改为 `v16_bhw.json`（38.3 s，21 阶段）；冻结需用户批准。

### 2026-11-23 Sol：v17 统筹规划（LB 中继 + 小腿内撇；原型进行中）
- 用户要求 1（LB 地面中继 0.148 导致 +2 cm 翻落）与 2（六足上台后过渡内撇）一起处理。
- 地形核实：LB 中继前方是小蓝正面 13–14 cm 竖墙（x≈0.165），安全区止于 x 0.15；0.148 离墙约 10 mm。中继退到 0.125（+x 容差 25 mm）时，C13–C16 全部可行（COM ≥48 mm，jm ≥0.061）；LB 起脚六足姿态改为 PCN=(.345,−.078,.235,抬头.44,侧倾−.2,yaw+2.3°)：COM 40.5 mm、jm .195、胶囊 7.9 mm、机身离地 17.6 mm（`relay.py`）。
- v16b 小腿倾角（`plan/shank.py`）：C18 NORMAL_STAND rf −15.0 / rm −14.2 / lf −7.8，C19 后 rm −17.0 / lf −11.5。只调站立机身位置最多改善到 −6.7°（RM），根因是 C12 RF (.52,−.165)、C14 RM (.36,−.262) 横向比标准站姿窄 1.5–2 cm；右翼/顶面安全区宽到 y −0.32。
- 方案：v17 = v16b + LB 中继 0.125 + PCN + C12 LF/RF、C14 RM 落点外移 + 站立机身重选。联合搜索 `joint17.py`（结果 joint17_scan.txt / joint17_res.pkl）进行中。
- 2026-11-23 用户要求“用效率最高的方式”：新增 `fastgeom.py`（批量 IK/COM/胶囊/机身离地/摆腿路径间隙/小腿倾角，与逐个版本对拍差值 0，单核约 120×）+ `v17search.py`（A：LB 起脚姿态+路径，3×18750 位姿 15 s；B：LF/RF/RM × 站姿 × STEP_B 比例 α，3600 组 14 进程 5.4 min）。用户要求以后都这么提速：已写入 `CLIMB_FRONT_SOP.md` 5.6（未提交）和记忆。
- 发现：PCN 侧倾 −0.2 时 LB 在空中平移末段小腿撞机身 −8.6 mm（只查端点漏掉），改为整条路径采样。A 选中继 0.125（+x 容差 25 mm）+ PCN=(.345,−.083,.23,抬头.44,侧倾−.15,yaw+3.4°)，LB 路径外抬 (.15,.15,.185)→落点外 25 mm 上方→落：COM 40.4 mm、jm .259、路径对 LM 15.2 mm/对机身 9.7 mm（`v17_A_pick.json`）。
- B 候选：C1（α=1.0，最差小腿 −2.0°）CPU 逐帧 STEP_B COM 34.0 mm/jm .040，弃；**C2**（α=0.8，LF (.52,.06)、RF (.52,−.20)、RM (.36,−.30)、站姿 (.39,−.09,.265,yaw+5°)，静态最差小腿 −4.8°，v16b −17°）CPU DONE 0 clip 0 capsule，各阶段 COM ≥39.7 mm，37.2 s（`v17_c2.json`，参数 `v17_c2_params.json`）。
- 已用 `setsid nohup` 脱离会话启动 `pipeline_v17.sh v17_c2`（hw_retime → CPU → 小腿/净空/落点容差 → Isaac 11 工况 3 路并行），日志 `claude_front_v16_20261123/pipeline_v17_c2.log`，扫描汇总 `gpu_v17_c2_summary.txt`。下次开工先读这两个文件。
- 2026-11-23 Sol 补记 `v17_c2` 流水线结果（pipeline_v17_c2.log，读日志，无新运行）：CPU DONE，计划 37.3 s；Isaac 名义 DONE 38.9 s，终点 root 34.8 mm。11 工况扫描 **8/11**（v16b 9/11）。新增通过 −2 cm（终点 root 37 mm，v16b 62 mm 超门）、**+2 cm**（LB 中继 0.125 解决了 v11 起的遗留问题）。新增失败：+3° 与 +2/+1.5/+3° 在 C12 FRONT_DIRECT_FINAL_480 超时，LB（地面支撑脚）相对机身误差 20.4/20.1 mm，只比 20 mm 门多 0.1–0.4 mm，主要是 y 向 19 mm；v16b 同工况峰值 20.3 mm、末值 18.8 mm 刚好过，属边缘问题，C12 LF/RF 外移后略变差（LF 世界系滑移 31/43 mm；LF (.52,+.06) 落点 −y 容差仅 6 mm）。−3° 在 C16 SIX_BODY_LB 翻倒：t≈0.87 s LB 踝实际 −1.29 rad、目标 +0.15 rad（被卡住），t≈1.33 s 机身姿态误差 109°，向后翻；推测是新 PCN 起脚姿态（抬头 .44）+ −3° 偏航下 LB 小腿/足卡到中继前方竖墙或地面，**未核实**。candidate 仍为 v16_bhw（9/11），v17_c2 未达标（目标 ≥9/11）。

### 2026-11-24 Sol：v17 失败根因诊断（原型目录 `/home/artrc/grasp_sync_backups/claude_front_v18_20261124/`，未入库）
- 新工具：`trace_sim.py`（测试 harness，不改仓库：yaw_sim 旋转 + 每 4 物理帧记录足端位置/全身连杆接触力 → npz）、`trace_an.py`/`trace_ts.py`/`gaps.py`/`links.py`（分析）、`sdf.py`（地形 2 mm 符号距离场 + URDF 碰撞尺寸的腿连杆间隙）、`yawcheck.py`/`hover.py`/`wallclr.py`、`v18searchA.py`/`relayscan.py`/`pbscan.py`/`lbslope.py`（可行性扫描）、`edit_v18.py`（消融编辑）。注意 Isaac 接触力 API 的足端力基本只有法向分量。
- −3°（C16 翻倒）根因：LB `knee_link` 在 C16 机身前移抬头时压入中继前竖墙顶边（Isaac 接触 18 kN；SDF 名义 −22.8 mm）。v16b 同样接触（85 N），被仿真整机落后 ~24 mm 掩盖。静态扫描：LB 在墙前地面（中继 0.085–0.125）时，不存在“LB 抬起 COM≥35 mm 且小腿稳健间隙 ≥5 mm”的机身姿态；LB 直接上斜面低位带（x .196–.204）多数 IK 无解，外侧 y≥.09 可解但 C15 RB 抬起 COM 仅 7 mm。→ 需改接触状态，待用户决定。
- 次要机制：整机落后 ~24 mm（C3 −4、C11→C13 −10 mm）使翼面/斜面落点 RB/RM/LM 悬空 2–5 mm、承载 0；v17 RF (.52,−.20) 被压入 1.5 mm。A2（斜面落点预压 2.5 mm，用户 2026-11-24 批准为原型门限偏离）`v18_a2hw.json` CPU DONE 0 clip，未上 Isaac。
- +3°（C12 超时）机制：LF+RF 双摆且机身前移 3 cm 时，LM/RM/RB 滑 11–17 mm（名义 9–12 mm），LB 相对机身误差 20.4 mm > 20 mm 门，下落段前等待超时，LF/RF 停在顶面上方 37–43 mm。A1（C12 机身静止）不可行：机身 x .195 时 LF/RF 够不到 x .52（IK 无解，CPU 卡 RUNNING）。备选：把前移挪到 C11（支撑在平地/顶面）。
- 当前候选仍为 v16_bhw（9/11）；v17_c2 8/11。工程准则已追加未关闭记录。
- 2026-11-24 补充：全链支撑腿连杆检查（`chainclr.py`）显示部署版 v11 C15 RB_DIRECT_PLATFORM_225 中 LB 小腿对墙顶边名义 +1 mm、偏前 2 cm −10 mm（边缘，实机观察项）；v16b/v17 C16 −20/−23 mm 为 v12 起新增 SIX_BODY_LB 带入；C21 LF −14 mm 各工况相同，疑为斜面上 L 形踝部模型假象，待复核。
- 2026-11-24 联合扫描 `jointback.py`（中继 × LM x × RB x；systemd-run MemoryMax=12G，12 进程，81 s）：不改接触顺序时，最优 = 中继 0.125 不变、**LM 0.285→0.255、RB 0.29→0.28** → LB 小腿稳健间隙 +8.5 mm（COM≥35）；代价 RB 边缘余量 34→24 mm、LM 7 mm（现 5 mm）。待用户决定。
- 2026-11-24 用户：上一轮 `jointback.py` 14 进程整网格评估爆内存（~3.6–7 GB/进程）。已改 `fastgeom.py`（备份 fastgeom.py.bak_20261124）：`CHUNK=8192`、`chunked()`、`safe_procs()`、`pool_map` 默认按内存/核数限进程；SOP 5.6 第 7 条（资源预算，未提交）；记忆已更新。
- 2026-11-24 v18 落点后移方案（用户同意 LM .255 / RB .28）结果：**不可行**。`v18searchA/B/C.py`（Q=C15 终止且 LB 起脚、去掉 SIX_BODY_LB；LB 斜面中间落点 T；站姿 T0；STEP_A/B COM；LB 两段路径；全部按 SDF 稳健检查）。小腿问题可解（Q≈(.31,−.06,.215,.32,−.05,yaw+10°)，knee 稳健 +5.7–8.5 mm），但 LB 落到斜面时对 LM 最好 9.5 mm、对机身 5.9 mm（v16b 用户采纳 21.9/12.4，认为 12 mm 已过小）；LM .265/.27 + RB .275/.28、LM 内收 y .09–.10 均无解；左侧（y≥.14）全为 75–109 mm 结构。搜索器对拍：v17 落点 + 去掉小腿约束能找回 v17 类解（knee −17.6 mm），说明无解为真。
- 结论：在“LB 在墙前地面中继、RB 先上翼面、机身前移后 LB 抬起”的接触顺序下，LB 小腿碰墙顶与 LB 抬起 COM、LB 落斜面空间三者冲突。下一步需用户决定是否做后腿接触顺序级搜索（例如 LB 在机身前移前、RB 仍在地面时先上斜面；之前同类换序被 RB 抬起 COM −8~−11 mm 否决，需在新约束下重查）。
- 搜索资源：均在 systemd-run MemoryMax 10–12G、pool_map safe_procs=12 下运行，单次 20–45 s，无内存问题。

### 2026-11-24 补记（Sol）：v16 已推送为默认（`8b0374a`），推送后补验
- 用户要求立即上实机，Luna 合同中途停止，Sol 直接提交推送（Luna 已完成：同步 dbf00ab、生成 `climb_front_v16.json`（阶段与 v16_bhw 逐字相同，`terminal_q_rad` 重算——原值是 v11 残留，最大差 1.11 rad）、改 `mode_climb.cpp` / `full_mission.launch` / `control_only.launch` 默认）。
- 推送后补验：Python 校验器 OK；CPU 30 Hz 回放 DONE 1200 帧 0 截断；worktree `catkin_make --only-pkg-with-deps grasp_hexapod_bt_control`（全量编译因 OrbbecSDK 扩展文件缺失失败，与攀爬无关）；C++ `cpp_replay` 仿真/实机模式均 DONE 1200 帧，峰值 3.60 rad/s，末帧对 terminal_q 5.4e-6 rad。
- 未做：catkin 测试、假舵机 ROS e2e、Isaac 名义复跑（扫描 9/11 为 v16_bhw 原值）、证据目录（说明稿在 scratchpad `v16_candidate_README.md`）。
- 实机注意：C16 LB knee_link 压墙顶边（模型 ~20 mm）；C19 RF / C20 LF 抬脚 ~22 mm（用户决定不改）。

## 2026-11-24 Claude Code（Sol）：v16 实机反馈 → 正面攀爬默认改为 v16.1（后腿落点推深）
- 用户实机反馈（v16）：①行走步子小、频率高、机身晃；②恢复站立时腿在地上硬拧，要改为抬腿回正；③C11 右中腿上台时小腿几乎水平，可能撞右侧黑色立柱（地形中 x .200–.225、y −.355…−.330、高 224 mm）；④**最优先**：左右后腿落点尽量往小蓝里面（实机多次因后腿踩空摔倒）。
- ④ 已做：`climb_front_v16_1.json`（来源 `v161ahw.json`，原型目录 `claude_front_v18_20261124/`，`edit_v161.py`）：C14 RM→(.38,−.29)、C15 RB→(.30,−.30)（C15 终止机身 (.32,−.088,.23,.36,−.15,yaw+15°)）、C17 LB→(.225,.10)、C20 LB 终点→(.26,−.07)。防落短余量：C15 RB 34→44 mm、C17 LB 32→38、C20 LB 24→70。CPU DONE 1173 帧 0 截断；C++ 仿真/实机模式 DONE，峰值 3.60 rad/s，末帧对 terminal_q 4.3e-6 rad；Isaac 11 组 **10/11**（v16 9/11，v11 7/11；失败 +2/+1.5/+3° 于 LB_SLOPE_BODY）。默认已切到 v16.1（mode_climb.cpp / full_mission.launch / control_only.launch 注释），v16、v11 保留回退。
- ③ 尝试（未采用）：C11 RM 离地即翻成"小腿朝上"（踝关节反折，关节空间加密节点 `rmtuck.py`）。v16.1b 9/11 且 CPU 审计 C11 IK 跟踪误差 15.5 mm（翻转过直腿奇异点时 DLS 落后）；翻转放慢 ×1.6 的 v16.1c CPU 通过（13.4 mm）但 Isaac 8/11（C11 +2.35 s 后 C12/C17/C20 边缘工况失败）。下一步：不翻转、只整体抬高且小腿不朝外伸的方案。
- ① Luna `b05b7f3`：phase 0.25→0.40 s、step_height 0.04→0.06（C++/Python/金标/测试同步）。Sol 复核（`/home/artrc/grasp_sync_backups/gait_20261124/gaitsweep.py`）：限速截断旧步态本就严重（0.12 m/s 180 帧 592 次，新 627 次）；根因是站姿踝关节 1.72 rad 离 ±2.094 限位仅 0.37 rad，行走时踝关节频繁顶限位（120 帧 49 帧），落点投影截短频繁（旧 90/200、新 139/200）。根治需调站立姿态，待用户决定；`b05b7f3` 可 `git revert` 回退。
- ② 方案已定未实现：六足先只升降机身 → 三角组 A（LF/RM/LB）抬 4 cm→空中移到站位→放 → 组 B；仅六足着地时用，悬空等情况保留关节插值。
- 遗留：C15/C16 LB knee_link 对墙顶边（模型 C15 名义 −1 mm、C16 −20 mm，被落后掩盖，用户 v16 实机未报此处卡腿）；C19 RF / C20 LF 抬脚 ~22 mm。

## 2026-11-25 Claude Code（Luna，合同由 Sol 冻结）：压缩攀爬阶段新增关节空间摆动段 `joint_space`
- execution_owner: Luna (sonnet, effort high) — 理由：C++/Python 控制器耦合特性，约 10 个文件 + 测试。
- 目的：摆动腿踝关节翻到另一侧要经过伸直奇异点，位置 IK 滞后；用关节节点直接给目标。未配置 `joint_space` 的配置行为字节不变。
- Schema（阶段级可选键）：`joint_space: {legs, from_knot, to_knot, q_knots_rad[(to-from+1, len(legs), 3)], q_velocities_rad_s? }`，仅 `piecewise_base_quintic` 阶段。校验（C++ `validateClimbCompact` / Python `ClimbMode._validate_joint_space`）：腿 ⊂ active_legs 且唯一；0≤from<to≤K-1；形状；有限；关节限位；速度首末行为零；首末节点 FK（hip_to_base(forward)）与 active_base_knots_m 差 ≤1e-3 m；若有 active_base_velocities，from/to 节点这些腿速度为零。侧向右镜像遇 joint_space 直接拒绝。
- 运行时：T(from)≤phase_time≤T(to) 内，用与 active-base 槽位相同的插值（C++ `piecewiseSlots` / Python `_piecewise(elapsed=…)`）得 q_ref；该腿足端目标与世界锚点 = FK(q_ref)；控制器 `joint_override[leg]=q_ref`，`calJointPoses`/`cal_joint_poses` 对该腿跳过 DLS，限位、单帧速度限幅、碰撞守卫仍作用于所有腿。覆盖每个 update 开头清空，setMode/abortClimb（含 resetToStand）清空；HOLD/FAILED 帧重发上次覆盖。实机模式同一路径。
- 文件：include/climb_compact.h、climb_mode.h、control.h；src/climb_compact.cpp、climb_mode.cpp、control.cpp；scripts/climb_mode.py、control.py、utils/climb.py；CMakeLists.txt；test/test_climb_joint_space.cpp/.py。
- 验证：新 gtest 5/5（MID_PLATFORM_285 腿 5 knot 1→2，区间内 25 帧 max|q_des−q_ref|=6.7e-16，其余腿与去掉 joint_space 的回放差 0）；新 pytest 14/14（同样 0 / 0）；golden_trajectory 6/6、golden_config 7/7、KinematicsGolden 失败为已知项；pytest 集合 17 failed/146 passed（基线 17 failed/132 passed，失败列表相同，+14 新测试）；cpp_replay v16_1 sim/hw 与改前参考（`/home/artrc/grasp_sync_backups/joint_space_20261125/ref_{sim,hw}.csv`）逐字节相同，1173 帧，max diff 0；Python 与 C++ 对同一 joint_space 配置回放差 5.0e-6（与无 joint_space 基线的既有差异相同）。
- 未做：config 中尚无使用 joint_space 的配置；未推送，待 Sol 审查。

## 2026-11-24 非攀爬：approach / 小蓝 YOLO 审查与远距离起步

- 基线 `8b0374a`（已含攀爬侧 v16 部署，本条未碰）。
- 审查结论（CPU 单测 + 代码审读，无实机）：(1) 深度有效范围 (0.2,1.0) m 使 1 m 外看到小蓝即 TARGET_PENDING → VisualGuidance 0.5 s 失败；(2) 粗停 0.20 m 贴深度下限，tag 必须在 z≈0.22 m 前接管；(3) 模型仅有 6 张图 CPU/GPU 一致性记录，无 mAP/召回/误检证据，本机无数据集与 best.pt；(4) 深度只取 bbox 中心 9×9（斜看/镂空会量到背景）——用户决定暂不改；(5) 小问题：`depth_median` regions 残留循环、yolo 节点 max_input_age 默认 0.5 与 yaml 0.25 不一致。
- 本次修改（execution_owner：Luna，中间档位 sonnet、effort medium：6+ 文件 C++/配置/测试耦合）：`xiaolan_yolo.yaml` max_depth 2.50；`phase1_visual.yaml` 接近超时 120 s，新增 `visual_far_distance_m 1.0`、`visual_far_ramp_m 0.3`、`visual_far_max_forward_speed_m_s 0.08`；`VisualGuidance` 前进上限按 z 分段线性（默认 far=near，行为不变）；+6 gtest；两份 README 同步。用户输入：开局距小蓝 1–2 m，远处 0.08 m/s。
- 验证：visual gtest 26/26、tag 50/50（独立 g++ + /opt/ros/noetic 头文件），YOLO pytest 18 通过 / 3 GPU 跳过。未运行 catkin/roslaunch/实机。
- 待办：用户提供小蓝实拍与“有光伏板无小蓝”负样本后，做遮挡敏感图 + 颜色反事实（担心模型主要靠蓝色，光伏板误检风险）。实机需确认 1–2.5 m 处 bbox 中心深度稳定性与 0.08 m/s 下 YOLO 跟踪。

## 2026-11-25 Claude Code（Sol）：AprilTag 链路推理 + 合成图检测实测（无仓库代码改动）

- execution_owner：Sol（只读推理 + scratchpad 测试 + 本文件追加）。基线 detached `ca59b7f`；本机工作区未编译（devel/ 仅 setup 脚本），控制器侧未运行。
- `apriltag_ros` 3.2.1 / `apriltag` 3.2.0 为本机 apt 安装（2026-08-18，x86_64）；Jetson 是否安装未核实。
- 合成图实测（scratchpad `tagtest/`，独立 roscore 端口，参数同 full_mission.launch，fx 690，无畸变）：位置 x/z 亚毫米准确；官方图案（libapriltag `apriltag_to_image`）正装 yaw≈0、+10° 偏转测得 +9.7…+12.3°，控制器 yaw 符号定义与 apriltag_ros 一致；OpenCV aruco 的 APRILTAG_36h11 图 = 官方图转 180°（yaw≈±180°，示教后 wrap 可处理）；平面内转 90° 安装 |R10|=1 → 全部拒收。
- 风险（推理 + 合成证据，未实机）：(1) 近正对 yaw 病态：6 cm tag 在 0.5 m 约 0.35 px 边缘误差 ≈ 2°，合成图已出现随距离变化的 0.1–2.3° 偏差，与 tag_yaw_tolerance 0.035 rad 同量级；(2) z=0.8 m、偏 10° 时 40/40 帧 yaw 在 ±10.5° 间翻转（平面位姿二义性），而 Coarse→tag 交接无距离门；(3) 交接无距离门 + tag 前进上限 0.02 m/s + align_timeout 60 s（覆盖 Aligning+Advancing）→ 远处（约 >1.2 m）交接会超时；(4) 相机俯仰被忽略：p0_z=z_D−D 在俯仰 θ 时多走 D(1/cosθ−1)（θ20°/D0.3 m ≈19 mm），URDF 无相机外参，θ 未知；(5) odom_at_last_valid 用处理时刻而非图像时刻，约 2–3 mm 多走（次要）。
- 待用户决定：交接距离门/近距离才交接、yaw 容差或改用多 tag/更大 tag、俯仰补偿（需相机外参）。

### 2026-11-25 Sol：关节角摆腿功能 + 上台/顶面摆腿"抬高、小腿朝上"（原型，未推送）
- 用户：实机舵机受载下垂，足端蹭小蓝背面；所有上台和小蓝背面上的摆腿必须先抬到较高高度再移动，小腿方向朝上或水平，不朝下/朝前；用关节角控制；最简单做法是只转踝关节。
- Luna 本地提交 `70d631e`（未推送）：compact climb 阶段可选 `joint_space`（legs/from_knot/to_knot/q_knots_rad/q_velocities_rad_s），窗口内该腿直接用关节角插值，跳过 DLS；C++/Python 对称；无 joint_space 的配置 C++ 重放与改前逐字节相同（v16.1 1173 帧，差 0）；新 gtest 5/5、pytest 14/14，金标 6/6、7/7。
- 审查（`lift_audit_20261125/swing_audit.py`）：v16.1 全部 19 次上台/顶面摆腿小腿朝下（竖直分量均值 −0.6…−0.99）或朝前（最大 +0.98），空中离地 ~20 mm。
- 工具 `lift_audit_20261125/tuck_all.py`（翻小腿生成）、`clip_retime.py`（关节窗口感知的自动重定时）、`path_terrain.py`（控制器逐帧回放的扫掠检查）。注意：`hw_retime.py` 不认 joint_space（会拉长时长但不改关节速度→过冲），joint_space 配置用 `clip_retime.py`，流水线 `NO_RETIME=1`。
- v16.3：离地 1.5 cm 即翻，C3 翻转扫进前坡面（Isaac 11/11 在 C3 翻倒）。v16.4：加扫掠检查，C3/C11 改为抬高后再翻 → Isaac **10/11**（仅 +2/+1.5/+3° C12 失败），CPU 0 截断，C++ 仿真/实机 1422 帧一致、峰值 3.80 rad/s。
- v16.5（进行中）：按用户规则"先抬高再移动"——先只转踝关节（必要时加少量膝）把脚抬到 50 mm，再翻小腿平移，落点上方 ≥ 落点+50 mm（关节限位内）再竖直落；C3/C17/C19 沿用 v16.4。扫掠（抬起后）脚 46–56 mm（C11 28、C20 LF 24），计划 46.5 s。Isaac 11 组运行中。

## 2026-11-25 Claude Code（Sol + Luna）：tag 交接距离门 + yaw 跳变拒收（本地提交，未推送）

- 用户决定：直接在 `test` 开发、可本地提交、推送等用户指令（与攀爬一起推）。相机光轴平行 base_link +y、俯仰≈0（用户确认，已记入 PROJECT_MEMORY），俯仰补偿不做。
- Sol：cherry-pick `cb5be0d`→`81643cd`、`ca59b7f`→`02eae62`（交接文件冲突两边保留），YOLO 两处小修 `9ac4606`。
- execution_owner：Luna（sonnet，effort medium；9 文件 C++/配置/测试/文档耦合）。`TagGuidance`：`tag_handoff_max_z_m`（yaml 0.5，代码默认 0=关）——Coarse 只有 z≤门限的检测计入连续确认，门外清零；`tag_max_yaw_jump_rad`（yaml 0.10，默认 0）——与参考样本 yaw 差超限即拒收，Coarse 中参考超过 max_age_s 失效（自恢复）、清零计数，交接后参考不失效、容许量 +max_yaw_rate·dt，拒收帧零命令，持续拒收走原 "tag lost"/handover 分支。README 补参数、示教（示教深度约 0.3 m，先测 yaw–距离，散布 ≤1°）、安装（正装/倒装，禁 90°；OpenCV 36h11 图为官方转 180°）。感知节点代码默认值对齐 yaml（max_result_age .25、max_depth 2.5、深度话题 /camera/depth/*、intra_op 2），test_target_timing 显式配 0.8 模拟 NX CPU 慢节奏。
- 验证（`grasp_wt_test` build 目标 + 二进制）：tag gtest 58/58（原 50）、visual 26/26、mode_framework 13/13、routing 7/7、bt_control_node 编译通过；YOLO pytest 18 通过 / 3 GPU 跳过；yaml 可加载。未跑 roslaunch / 实机。
- 注意：本工作树同时有另一会话（攀爬，grasp-hexapod-a3）在写，其 CLAUDE_HANDOFF 条目未被本提交纳入。
- 待办：实机示教 tag、yaw–距离检查、IMU yaw 符号；大 tag / 双 tag 需硬件。

## 2026-11-25 Claude Code（Sol，c0）：平地行走站姿 + 步态分析（仅原型，无仓库代码改动）
- 用户要求：专门的行走站姿和行走步态，平稳快速、减少机身晃动、稳定跟踪给定机身速度；完成后做 approach ROS 闭环联调（A）。用户思路：踝基本不动、支撑腿小腿竖直，速度由大腿摆动和膝抬落决定。
- 原型目录 scratchpad `gait/`（会话临时，可能被清）：harness.py（Python 控制器镜像，理想伺服 30 Hz）、stance.py、jsgait*.py、compare.py、compare.png。
- 根因（模型）：现有笛卡尔摆动在 Q_STAND（ρ .109、髋下 .0886）原半径竖直抬 6 cm → 未截断需求 膝 12–18 rad/s、踝 6–9 rad/s（指令限速 4），每条腿每 180 帧膝截断约 70 次、踝顶限位；0.05 m/s 也每帧 3 次截断。摆动腿落后 → 换相时未落地即另一组抬起，推测为实机机身晃动来源。
- 新方案原型：支撑腿笛卡尔刚体平移；摆动腿关节空间五次 Hermite（端点关节速度 = α×支撑速度，α=.5）+ 只用膝抬 4 cm（sin² 包络，FK 标定）；行走站姿 ρ .125–.14、髋下 .11（机身升高约 2 cm）、β=0。T .5 s、τ .05 s。前进 0.14 m/s 时峰值 大腿 2.76/膝 2.78/踝 2.47 rad/s（≤3），横移 .12 ≤2.8，转向 .5 rad/s 踝≈0。关节余量 ≥.37 rad（Q_STAND .20）。
- 限制：3 自由度下前后腿支撑末端小腿倾角随步长增大（0.12 m/s 约 15°），仅中立位竖直；抬腿 4 cm（低于 b05b7f3 的 6 cm，5 cm 需 T .6）。
- 待用户决定后实施：站姿参数、抬腿高度、最大速度上限、Q_STAND↔行走站姿切换（三角组抬腿放置，进入/离开 approach 时）、tag 示教须在行走站姿下做。实施涉及 C++ approach_mode/control + Python 镜像 + 金标 + 测试（Luna），control.* 需先通知 a3。
- 2026-11-25 续：只转踝关节抬脚（v16.5/5b）会让脚绕踝关节画圆弧，在 4 cm 以下横扫 6–8 cm，Isaac 7/11，弃用（工具备份 `lift_audit_20261125/tuck_all_ankle_v165.py`）。**v16.6**（用户选）：先按脚位置竖直抬到关节限位允许的最高处（目标 50 mm；C12 前腿只到 23–28、C14 30、C20 LF 38），再关节角翻小腿；落点上方 ≥ 落点+50 mm（限位内）后竖直落；C3/C11/C19 沿用 v16.4。低处横移（`low_drag.py`，脚 <40 mm 时的水平移动）C4/C8/C15/C17/C20 ≤1 mm；C12 32–43、C14 20、C19 RF 112（未解决：RF 伸直只能竖直抬 13 mm，需改该步机身动作）。CPU DONE 1523 帧 0 截断，计划 48.9 s。Isaac 核心 3 组（名义、±3°）全部 SUCCESS；其余 8 组在后台跑，结果见 `claude_front_v18_20261124/full_v166.log` 与 `gpu_v166_summary.txt`。未推送；推送需带上 `70d631e`（关节角功能）并经用户同意；C++ 重放 v16.6 尚未做（v16.4 已验证 1422 帧一致）。
- 2026-11-25 v16.6 结果：Isaac 11 组 **10/11**（仅 +2/+1.5/+3° 在 C17 LB_SLOPE_BODY 失败）；C++ cpp_replay（含 70d631e）仿真/实机模式 DONE 1523 帧，与 Python CPU 一致，峰值 3.80 rad/s，末帧对 terminal_q 4.3e-6。配置 `claude_front_v18_20261124/v166hw.json`，未入库、未推送；远端仍为 v16.1（7596500）。

## 2026-11-26 攀爬会话对步态会话（Q_STAND → Q_CLIMB_PREP）的答复

（对方 socket 已失效，消息没送达，答复记在这里。）
- (a) 攀爬会话没有改对方清单里的文件，以后也不打算改；未提交的只有本文件（只追加）。对方可以把本文件连同 v16.5/v16.6 段落一起提交。攀爬会话下一步要写的文件（climb_front_v16_6.json，以及 mode_climb.cpp、full_mission.launch、control_only.launch 里的默认攀爬配置那一行）要等用户同意推送，并且等对方这一批提交完成后才动。
- (b) 依赖旧站姿的地方：control.py:263 / control.cpp:162 的 PREPARE_CLIMB 门限（拿 q_cur 和 q_init 比）；run_sim.py:1319-1347 prepare_compact_stage_entry 用 base_height_at_stand，run_sim.py:1409 从 P0 起步时用 q_init 生成机器人；run_sim_dock.py:261-270 会覆盖 q_init、foot_init_base、base_height_at_stand。只影响行走：mode_walk.cpp:17、bt_control_node.cpp:177、run_real.py:496、run_sim.py:1471 用 foot_init_base 半径推算偏航角速度上限，rho 一变，上限也会跟着变。
- 换站姿动作的物理要求：两组三角步态交替换腿时，机身漂移会占用起点误差余量（dx ±20 mm、dy ±15 mm、yaw ±3°，v16.6 最极端那组已经失败），Isaac 里要报告漂移量，建议控制在 5 mm、0.5° 以内；前腿往外摆向小蓝正面时，要检查脚和小腿离台阶立面、边沿的间隙；最后机身下降要准确落到 Q_CLIMB_PREP，settle_gate 门限不放宽。
- (c) 同意把攀爬入口门限从 q_init 改为 Q_CLIMB_PREP。条件：容差不变；C++ 和 Python 一起改；P0、compact 配置检查和攀爬金标逐字节不变；test_climb_joint_space（cpp 5/5、py 14/14）和攀爬金标测试保持通过。

### 2026-11-26 walking gait run 1 (walk session grasp-hexapod-cd)
- execution_owner: Luna (sonnet, effort high — two-language coupled controller change with golden parity). Contract: /home/artrc/grasp_sync_backups/claude_walk_gait_20261125/CONTRACT_GAIT_RUN1.md.
- Scope: ApproachMode swing becomes joint-space (quintic Hermite liftoff→touchdown, α=0.5 boundary velocity, knee-only lift 64u³(1−u)³ calibrated to 4 cm foot-tip clearance), T 0.50 s, max_foot_planar_speed 0.14. Files: approach_mode.{h,cpp,py}, golden reset_then_walk only, walking-envelope tests in test_real_mode_routing.py.
- Next (run 2, pending climb-session confirmation): Q_STAND := walking stance (ρ .125, z −.115, β −10°; body 96 mm); old values → Q_CLIMB_PREP; walk→climb-prep tripod transition (A then B lift-and-place to the climb-prep xy, then a 26 mm six-foot body lowering) before every climb entry; climb P0 / golden byte-identical.
- Run 1 accepted after 3 iterations (1 → 1b → 1c) and committed locally (no push). The final design:
  - every walking leg gets an exact joint target (stance = Newton IK of the rigid-twist foot, swing = joint Hermite + knee lift governed to ≤3 rad/s), so walking no longer goes through DLS;
  - no per-phase command latch: stance follows the slewed command every frame, and the touchdown uses a look-ahead of the slew profile;
  - accelerations 0.25 m/s² / 1.2 rad/s², T 0.5, h 0.04, budget 0.14.
  CPU metrics at the walking stance (ideal servo, 8-case matrix):
  - 0 clips, tracking 0 %, settled slip < 0.6 mm, steady lift 40 mm;
  - overall peak ≤ 2.98 rad/s except the combined start-up at 3.09;
  - lateral 0.14 trims some lifts to 37 mm (25.5 mm at start-up).
  Metrics: claude_walk_gait_20261125/run1/metrics_b.json.
- Isaac baseline of the OLD gait (HEAD before this commit, 0.12 m/s): forward reaches only 42 mm/s (35 %) with roll p2p 5.8° and height p2p 12 mm. At 0.06 m/s it is fine (95 %). Harness: claude_walk_gait_20261125/isaac/{walk_trace.py,analyze.py}.
## 2026-11-26 ZCode 主会话（Sol）：xiaolan_yolo_detection 新建 tag36h11 单 ID 检测 launch

- 基线：`test` @ `7596500`，本批未提交。
- execution_owner：Sol，新建 1 个 launch、`package.xml` 加两行依赖、追加本条，共 ≤3 个文件的琐碎配置，不涉及控制/攀爬/舵机。
- 目标与范围：用户先要求把本地 apriltag 入库 `src/reference/`，随后改为不入库，用本机 apt 的 `ros-noetic-apriltag-ros 3.2.1`，在 `xiaolan_yolo_detection` 新建 launch，从 `/camera/color/image_raw` + `/camera/color/camera_info` 检测 `tag36h11` 的单个 ID（默认 255）。本机没有 apriltag 源码，只有 `/opt/ros/noetic` 下的 apt 二进制与头文件，所以没有东西可入库。
- 修改文件：`src/xiaolan_yolo_detection/launch/xiaolan_apriltag.launch`（新建）；`src/xiaolan_yolo_detection/package.xml`（`exec_depend` 加 `apriltag_ros`、`image_view`）。
- 用法：先启动 `xiaolan_orbbec_camera.launch`，再 `roslaunch xiaolan_yolo_detection xiaolan_apriltag.launch`。参数：`image_topic`、`camera_info_topic`、`tag_id`（255）、`tag_size_m`（0.060）、`publish_tf`（true）、`show_window`（true）。输出 `/tag_detections`、`/tag_detections_image`，TF `camera_color_optical_frame -> tag_255`。节点名 `xiaolan_apriltag_detector_$(arg tag_id)`，与 `full_mission.launch` 的 `xiaolan_apriltag_detector` 不同名，不会互相踢掉。
- 实际运行的验证（本机实机 ROS 环境；相机已在运行，master 上只有 `/camera/camera` 与 `/rosout`，无控制/舵机节点）：
  - `xmllint`、`roslaunch --nodes`（`/xiaolan_apriltag_detector_255`、`/tag_255_view`）、`roslaunch --dump-params`：`standalone_tags` = `id: 255`（整数）/`size: 0.06`/`name: tag_255`。
  - 真实相机（848×480 rgb8，约 13 Hz；tag36h11 255 号贴在小蓝侧面）：节点日志 `Loaded tag config: 255, size: 0.06, frame_name: tag_255`，无告警；最终版 5 s 内 74 条 `/tag_detections`，全部 `id=[255]`；同一帧用 libapriltag 离线检测为 `id=255 hamming=0 margin=123.2`。检测进程约 113% CPU（6 核）。
  - 合成图（libapriltag `apriltag_to_image` 渲染 255 与 0，发布到私有话题）：仅 255 时 56 条全为 255；255 与 0 同图时 60 条全为 255，日志 `Requested description of standalone tag ID [0], but no description was found`。
  - 未运行：带窗口的 `image_view`（本会话无 DISPLAY）；`catkin_make`（本批未改编译内容）。
- 证据层级：实机相机 + 实机 ROS，仅感知节点，无执行器。测试脚本与合成图在仓库外 `/tmp/zc_apriltag/`（易失）。
- 发现与未决事项：
  1. **`full_mission.launch` 的 AprilTag 检测器不会发布任何检测**：其 `standalone_tags` 的 `<rosparam>` 没写 `subst_value="true"`，roslaunch 不替换 `$(arg tag_id)`/`$(arg tag_size_m)`。`roslaunch --dump-params` 得到 `id: '$(arg tag_id)'`、`size: '$(arg tag_size_m)'`；把该节点块原样复制到临时 launch 实测，启动即 `[ERROR] Error loading standalone tag descriptions: type error`，画面含 0/255 号 tag 时零发布。即经 `full_mission.launch` 启动时，approach 的 tag 引导收不到检测（approach 具体行为未实测）。**未修**（不在本轮范围，属实机任务关键文件，待用户决定）。修法是给该 `<rosparam>` 加 `subst_value="true"`（一行）。
  2. `tag_size_m` 默认 0.060 沿用仓库其他 tag；255 号 tag 实际边长未确认，只影响位姿平移，不影响 ID 识别。
  3. 订阅的是未矫正的 `image_raw`（`plumb_bob`，k1≈−0.032）：ID 识别不受影响，位姿在画面边缘有偏差；需要精确位姿时用 `image_proc` 矫正话题（`xiaolan_tag_system.launch`/`full_mission.launch` 的做法）。
  4. 该包 `README.md` 未补这个 launch 的说明（本轮未动，超出 3 文件边界）。
- 下一步：用户确认 tag 边长；决定是否修 `full_mission.launch` 的 `subst_value`。

## 2026-11-26 追加：xiaolan_apriltag.launch 无 DISPLAY 弹窗崩溃修复

- 基线：同日条目之上，本批未提交。execution_owner：Sol，单文件 launch 一处条件 + 本条。
- 现象：用户在无 DISPLAY 的终端启动，`image_view` 段错误（exit -11，`cannot open display`），检测器不受影响继续发布。
- 环境核实：机器人本机有 Xorg + gnome-shell，但桌面停在 GDM 登录界面（gnome-shell 属 gdm），`DISPLAY=:0` 未登录时不可用；用户 shell 无 DISPLAY。
- 修改：`xiaolan_apriltag.launch` 的 image_view 条件改为 `if="$(eval arg('show_window') and optenv('DISPLAY', '') != '')"`——无 DISPLAY 自动跳过弹窗；`show_window:=false` 仍可强制关。用户同批把 `tag_size_m` 默认改为 `0.050`，保留。
- 验证：`--nodes` 三种情况（无 DISPLAY / 有 DISPLAY / 有 DISPLAY + show_window:=false）节点列表均正确；无 DISPLAY 对真实相机实跑：只起检测器、无崩溃，5 s 74 条检测全为 `id=[255]`，日志 `Loaded tag config: 255, size: 0.05`。另清理了用户上次崩溃启动遗留在 master 上的 `/tag_255_view` 注册与参数。
- 下一步不变：等用户决定是否修 `full_mission.launch` 的 `subst_value`。

## 2026-11-26 追加：xiaolan_apriltag.launch 检测器占用 140% → 60%

- 基线：同日之上，未提交。execution_owner：Sol，单个 launch 改参数 + 本条。
- 用户反馈：apriltag 占用非常多；用户确认 15 Hz 够用，不降帧率。
- 事实：本机 `apriltag_ros` 只读 `tag_family / tag_threads / tag_decimate / tag_refine_edges / max_hamming_dist / remove_duplicates / publish_tf / standalone_tags / tag_bundles / tag_debug / publish_tag_detections_image / transport_hint`；`tag_blur` 不被读取（四个二进制里没有 "blur" 字符串）。
- 基准方法：真实相机录 7.7 s / 117 帧回放（含 255 号 tag，黑边框外宽约 52 px，距 0.385 m），每个配置单起节点、回放只放一遍，按 `/proc` 进程 CPU 时间算 ms/帧；同会话、同主频 1.42 GHz，基线复测 144.9/144.7/145.5% 一致。注意：**回放不能 `--loop`**，时间戳倒跳会让节点的 image/camera_info 时间戳配对卡住（日志 `Topics ... do not appear to be synchronized`）。脚本与回放在仓库外 `/tmp/zc_apriltag/`（易失）。
- 结果（4 线程）：decimate 1.0 = 145% / 96 ms，1.5 = 87% / 58 ms，2.0 = 65% / 44 ms，3.0 = 50% / 33 ms，4.0 = 45% / 30 ms；全部 83/83 帧检出，x 相同，z 0.3846–0.3847 m，yaw 标准差 0.14–0.31°。decimate 2.0 下线程 4/2/1 = 65.5/60.0/55.8%。离线：decimate 2.5 → 0/12 检出（非整数走插值把 tag 糊掉）；`tag_refine_edges=0` 丢一半检测；标注图发布开/关无差别。
- 修改：launch 新增参数 `tag_decimate`（默认 2.0）、`tag_threads`（默认 2，原 4），删除无效的 `tag_blur`。用户此前把 `tag_size_m` 改为 0.050、并把 image_view 节点整块注释掉，均保留。
- 真实相机前后对照（同文件、同会话、交替两次）：旧配置 140.3% / 139.6%，新配置 60.5% / 59.6%；5.5 s 内均 82 帧 82 检测（≈15 Hz），x −0.0369 → −0.0368 m，z 0.3848 m 不变，yaw 标准差 0.25–0.27°，节点日志无告警。
- 边界：只在当前场景（一个 tag、约 0.4 m、正对相机）实测。tag 更远或在画面里更小时，decimate 2.0 的找 tag 步骤会比 1.0 更早失败；按等价关系推算约 2 倍距离（0.8 m）内仍可检出，但未实测。需要旧行为时用 `tag_decimate:=1.0`。
- 未解决不变：`full_mission.launch` 的 `subst_value`。

## 2026-11-26 ZCode 主会话（Sol）：camera_stream 远程查看推流节点（bt_control）

- 基线：`test` @ `7596500`，本批未提交（工作区另有今日 xiaolan_yolo_detection 的未提交改动，与本批无关）。
- execution_owner：Sol，理由：≤3 个文件的琐碎工具（1 个 ~110 行 Python 节点 + 1 个 launch + CMakeLists 一行安装），不碰控制/攀爬/舵机，且需要与实机 ROS 在线联测反复迭代，拆给 Luna 往返成本更高。
- 目标与范围：用户要求把 `/camera/color/image_raw` 推流给其他电脑远程查看，指定推 compressed。新建独立节点，不进 full_mission/autostart，按需手动启动。
- 修改文件：`src/grasp_hexapod_bt_control/scripts/camera_stream.py`（新建）、`src/grasp_hexapod_bt_control/launch/camera_stream.launch`（新建）、`src/grasp_hexapod_bt_control/CMakeLists.txt`（`catkin_install_python` 加一行）。
- 设计与实测约定：
  - 订阅 `~input_topic`（默认 `/camera/color/image_raw`）→ 发布 `~output_topic`（默认 `/camera/stream/compressed`，与驱动的 lazy `.../image_raw/compressed` 不同名，避免双发布者）。参数 `~fps`=8.0、`~jpeg_quality`=70，无 cv_bridge 依赖（numpy+cv2，支持 rgb8/bgr8/mono8）。
  - 编码约定逐字节模仿本机驱动：format 串 `"<encoding>; jpeg compressed <encoding>"`，rgb8 先转 BGR 再 imencode。用 C++ 探针链接 image_transport 的 compressed 订阅插件实测：插件交付 `encoding=rgb8` 且通道均值与原图不交换 → rqt_image_view/image_view 颜色正确（这条链路的行为 2026-11-26 实测确认，不是文档推断）。
  - 双端惰性：无人订阅输出时输入订阅也注销（0.5 s 定时器管理）。rospy 收 848x480@15 Hz raw 的反序列化固定吃 ~17% 单核，不能常驻订阅。
  - `cv2.setNumThreads(1)`：opencv-python 线程池对 JPEG 编码是纯开销，本机实测默认单帧 wall 4.0 ms 烧 23.4 ms CPU（线程池自旋），单线程 wall 3.9 ms、CPU 3.8 ms。
- 实际运行的验证（本机实机相机 + 实机 ROS；Jetson 6 核 1.42 GHz 节流频率）：
  - `py_compile`、`xmllint`、`roslaunch --nodes`（`/camera_stream_node`）、`catkin_make --only-pkg-with-deps grasp_hexapod_bt_control`（白名单与工作区原配置一致，未复位）。
  - 在线联测：无人订阅 0–0.2% 核；一个订阅者 22.6% 核（收图 ~17% + 编码 ~5%）；退订 1 s 后回 0%；输出 7.4–7.5 Hz（8 上限）；带宽 323–438 KB/s（≈2.6–3.5 Mbps）；订阅后首帧 0.6 s；解码通道校验与驱动同构（swap 误差 2.95 < 同序 10.05）；C++ 插件探针消费端颜色正确；节点日志无 ERROR。
  - 优化前对照：编码态 87% 核（OpenCV 线程池 5 worker 占 43%，主线程仅 22.5%），即每帧 ~88 ms CPU 时间摊 6 线程。
  - 未运行：全量 catkin（既有 OrbbecSDK 缺扩展文件问题，与本批无关）、catkin 测试（纯新增脚本，无 C++/配置改动）。
- 发现与备注：
  1. `catkin_install_python` 的 devel 包装器是运行时 relay（exec 源文件），改脚本无需重编。
  2. 本机后台进程跨工具调用存活不稳定：测试期间 roscore/相机两次被外部清掉，验证改为单命令自包含流程；结束后 roscore+相机保持运行（与会话开始时一致）。
  3. `/tmp` 会话中途被清两次，测试脚本统一放 `~/zc_tmp/`。
  4. 远程电脑（同一 ROS master）查看：`rqt_image_view` 选 `/camera/stream/compressed`；或 `rosrun image_view image_view image:=/camera/stream _image_transport:=compressed`。
- 下一步：等用户决定是否 `git commit`；如需更低 CPU 可 `_fps:=5` 或把 jpeg_quality 降到 60。

## 2026-11-26 追加（Sol）：相机推流合并进 bt_dashboard，独立 camera_stream 撤回

- 基线同上条（`test` @ `7596500`，未提交）。execution_owner：Sol（单文件功能合并 + 撤回本会话自己的未提交文件，与实机在线联测紧耦合）。
- 用户决定：把相机远程查看合并进 Web 看板（浏览器直接看，远端电脑不再需要 ROS）。
- 修改：`src/grasp_hexapod_bt/scripts/bt_dashboard.py`
  - 新增 `CameraFeed` 类（~110 行）：raw→JPEG，惰性订阅（0 观看者退订，省 ~17% 单核 raw 反序列化常驻开销），`cv2.setNumThreads(1)`（线程池对 JPEG 纯开销，实测 23.4→3.8 ms/帧 CPU），编码约定与驱动 compressed 插件一致（rgb8→BGR）。
  - HTTP：`GET /camera.mjpg` multipart/x-mixed-replace（ThreadingHTTPServer 每观看者一线程，断开 BrokenPipe→release 退订）；无 CameraFeed 时 503。
  - UI：主控台 sideB 顶部 `camCard`（MJPEG `<img>` + 开启/关闭按钮，默认关闭）；CSS、JS 就位；`/remote` 共用同一页面。
  - run()：`~camera_topic`（默认 /camera/color/image_raw）、`~camera_fps`（8）、`~camera_quality`（70）→ `Handler.camera`；selftest 页面 marks 增 camCard/camBtn/camera.mjpg + 无 CameraFeed 时 /camera.mjpg 503 断言。
- 撤回：`grasp_hexapod_bt_control` 的 `scripts/camera_stream.py`、`launch/camera_stream.launch` 删除，CMakeLists 回到与 HEAD 逐字一致（均未提交过，git status 已确认）。ROS 话题侧观看需求仍可用驱动自带的 lazy `/camera/color/image_raw/compressed`。
- 实际运行的验证（实机相机 + 实机 ROS，_port:=8090 临时实例）：
  - 页面含相机卡片；无观看者时 bt_dashboard 不出现在 `/camera/color/image_raw` 订阅者列表；空闲 CPU 1.4%（看板自身基线）；curl 拉流 6 s 得 39 帧（≈6.5 fps）1.95 MB（≈325 KB/s），拉流期间 22.2% 单核；断开 1.5 s 后订阅注销、CPU 回落 1.5%；首帧 JPEG ffd8，解码 848×480，通道序与驱动一致（swap 更小）。
  - `py_compile` 通过；自检（--selftest）在第 7 步树模板断言失败——**HEAD 同样失败（既有问题）**：`hexapod_bt` 现树与看板内置 `FALLBACK_TEMPLATES_JSON` 兜底快照节点数不一致；此前"HEAD 通过"是从非 scripts 目录运行导致 sys.path 没 import 到真实树的假象。本批新增的第 9 步 marks/503 检查因断在更早步骤未跑到，改由上面的实机联测覆盖。
  - 未运行：浏览器端人工确认（本机无桌面会话），建议用户打开看板点一次"开启画面"。
- 遗留/发现：selftest 树模板快照脱节为既有问题，待用户决定是否重生成 `FALLBACK_TEMPLATES_JSON`；camera feed 的 rospy.loginfo 在 nohup stdout 未见（日志走向），功能以 rostopic info 订阅者增减与帧流为证。

## 2026-11-27 追加（Sol）：修复相机"关了再开没画面"死锁 + 画面放大功能

- 基线同前（未提交）。execution_owner：Sol（单文件 bug 修复 + 小 UI 功能，需复现用户精确操作路径联测）。
- **根因（用户报障：开→关→再开无画面）**：`CameraFeed.release()` 持有 `threading.Lock` 期间调用 `_unsubscribe()`，后者再次 `with self._lock`——`Lock` 不可重入，第一次关闭的 HTTP 线程在 `unregister()` 之后、清 `_jpeg/_gen` 之前死锁并**永久持锁**；第二次打开的 `acquire()` 等锁无响应 → 请求挂起无画面。此前 curl 联测只测了一次打开+断开（死锁发生在断开之后，订阅者消失/CPU 回落都"正常"），没测第二次打开，盲区。修复：退订折叠进 `release()` 的同一 `with` 块（并留注释禁止再拆再入锁子函数）；`_unsubscribe()` 删除。另加固：`_camera_mjpeg` 设 `connection.settimeout(10)`，半关闭僵死客户端最迟 10 s 报错释放名额。
- 客户端配套：每次开启用 `camera.mjpg?t=<Date.now()>` 换 URL（浏览器对中断过的同 URL multipart 流不会重新拉）。
- **放大功能**：画面右上角半透明 ⤢ 按钮 → 全屏遮罩 `#camOverlay` 放大；放大用 DOM 移动 `<img>`（不重载、不开第二条流、订阅数不变）；✕ 按钮/Esc/点背景收起，收起后 `<img>` 移回卡片。未开启画面时点 ⤢ 提示"先开启画面再放大"。
- 实际运行的验证（实机相机 + 实机 ROS，_port:=8090）：
  - py_compile 通过；页面含 camZoom/camOverlay/camShrink/camWrap（20 处匹配）。
  - **开→关→开→关→开 三轮回归**：三次拉流分别 14/18/19 帧（669/873/922 KB），每次关闭后订阅者消失，看板 `/state.json` 始终 200（锁健康）；拉流期间 CPU 8.2%（采样窗口与前次 22.2% 不同，量级一致：空闲基线 + 编码，均远低于修复前 87%）。
  - 未运行：浏览器人工点按（本机无桌面）；放大交互（DOM 移动不断流为标准行为，建议用户实点一次 ⤢ / Esc）。
- 备注：`camera feed: 订阅/退订` 的 rospy.loginfo 在 nohup stdout 仍未见（日志走向未深究，功能以 rostopic info 与帧流为证）。

## 2026-11-27 Luna 合同：xiaolan_yolo_detection 感知链路降资源（P1/P2/P3）

- 基线：分支 `perf/xiaolan-yolo-resource`，基线 `test` @ `7596500`；本批未提交。
- execution_owner：Luna，冻结合同机械执行；P1/P2/P3 具体改法均由 Sol 合同逐条给定，不做设计决策。
- 目标与范围：
  - P1 可视化节点默认关闭+按需订阅：`xiaolan_yolo.launch` 的 `start_visualizer` 默认 `true→false`；`xiaolan_visualizer.py` 的 `__init__` 不再建相机 RGB 订阅（话题名存 `self.rgb_topic`），`run()` 每拍按需管理——窗口开着或 `~image` 有订阅者才订阅，不需要时 unregister 并清空 `latest_rgb`/`colors`，不需要时不渲染不发布不收图；`rgb_cb` 缓存上限 30→6。其余四个订阅、`render()`、DISPLAY 守卫与窗口退出逻辑语义不变。
  - P2：`config/xiaolan_yolo.yaml` `depth_cache_size: 45→6`。
  - P3 RGB 直通：`yolo_core.py` `Letterbox.prepare` 改收 RGB（去掉通道反转，docstring 注明输入须为 RGB，mapping 不变）；`xiaolan_yolo_node.py` rgb8 走 `passthrough` 零拷贝、其他编码仍转 rgb8；`test_core.py` 新增 `test_prepare_rgb_channel_passthrough`（通道均值直通 + 补边 114 + 通道反转后张量不同）。
- 修改文件：`src/xiaolan_yolo_detection/` 下 `launch/xiaolan_yolo.launch`、`scripts/xiaolan_visualizer.py`、`scripts/xiaolan_yolo_node.py`、`src/xiaolan_yolo_detection/yolo_core.py`、`config/xiaolan_yolo.yaml`、`tests/test_core.py`、`README.md`（start_visualizer 默认值、可视化话题按需订阅说明、`depth_cache_size` 配置行、`max_result_age` 笔误 0.8→0.25）+ 本文件追加本节。均未 git add。
- 实际运行的验证（隔离 master 全程用 `roscore -p 11312` + `ROS_MASTER_URI=http://localhost:11312`；本机 ros_comm 1.17.4 的 `roscore` 会忽略 `ROS_MASTER_URI` 强回 11311，必须用 `-p`）：
  - `python3 -m py_compile`（4 个 py）与 `xmllint --noout` launch：通过。
  - `roslaunch --nodes`：默认仅 `/xiaolan_yolo_node` `/xiaolan_target_3d_node`；`start_visualizer:=true` 多出 `/xiaolan_visualizer`；均退出码 0。
  - unittest：`Ran 22 tests, OK (skipped=3)`。合同预估基线 19+3，实测基线 21+3（18 通过+3 GPU 跳过），加新测试后 22，3 项 GPU 照旧跳过。
  - smoke（`phase1_yolo_smoke.py`，ONNX CPU）退出码 0：`fresh=15 rate_hz=6.350 inference_ms_min=148.592 inference_ms_max=161.491 provider=CPUExecutionProvider`，stale 路径 `STALE_OR_INVALID_RGB_TIMESTAMP` 正常。
  - e2e（`phase1_perception_e2e.py`）退出码 0：steady 期 detection/depth/target 均 15 Hz、state 2；故障恢复 `VALID_MEASURED`；`healthy_valid_count=53`。
  - 两次隔离 master 用后均清理；11311 生产 master 与其上的相机 roslaunch 未触碰。
- 证据层级：CPU（ONNX CPU 推理 + 隔离 ROS master 功能链路）。未运行 GPU/TensorRT 推理、未做实机资源测量、未运行 `resource_profile.py`。
- 发现与未决事项：
  1. 合同的 unittest 基线预估（Ran 19）与实测（Ran 21）不符，按合同"以实际为准"；无本批外失败。
  2. 合同给出的活动区域参考值（nh=235/top=90）与 480x640→416 letterbox 实际几何（ratio .65，nh=312/top=52）不符；新测试改为从 `prepare` 返回的 mapping 推导活动区域，通道均值与补边 114 断言仍按合同。
  3. 新测试初版两处断言自身写错（border 均值分母、可复用张量两次调用返回同一缓冲需先 copy 快照），只改测试，实现未改。
  4. `CLAUDE_HANDOFF.md` 当前实际被 git 跟踪且 `git check-ignore` 未命中（合同写"已被 .gitignore 忽略"与现状不符）；按合同只追加、未 git add。
- 未做的事：未 git add/commit/push；未动批准清单外任何文件（工作区中 `bt_dashboard.py`、`package.xml`、`xiaolan_apriltag.launch`、`tests/resource_profile.py`、`src/docs/evidence/xiaolan_yolo_resource_20261127/` 为本批之前已有改动/文件，未触碰）；未运行 catkin_make（无 C++/消息/编译输入改动）。

## 2026-11-27 ZCode 主会话（Sol）：xiaolan_yolo 链路降资源（基线→P1/P2/P3→复测→探针）

- 分支 `perf/xiaolan-yolo-resource`（自 `test` @ `7596500`），已提交见 git log；`CLAUDE_HANDOFF.md` 本条与上方 Luna 条目因文件含他人未提交改动而**未 git add**。
- execution_owner：第 0 步与第 2 步 Sol（测量工具 + 实机联测）；第 1 步 Luna（合同：P1 可视化默认关+按需订阅、P2 深度缓存 45→6、P3 RGB 直通、测试与 README，8 文件）。合同外修正一次：`window_open` 死锁（GTK 首次 imshow 前 `getWindowProperty`=0 → 永不渲染），改为 `needed = self.show_window or pub.get_num_connections()>0`。
- 新增 `src/xiaolan_yolo_detection/tests/resource_profile.py`：按节点 XMLRPC 取 PID，/proc 差分测 CPU%/RSS/线程 + maps 库来源 + 小话题监视 + tegrastats 侧录；不订阅相机原始流。双跑作噪声门槛。
- **基线（SW 相机，3 节点+可视化默认开）**：CPU%/核 camera 89.1、yolo 42.2、td3 22.3、visualizer 31.5（链路 185.1）；RSS 共 ~2448 MB（yolo ~2010 = TRT/cuDNN 映射）；detection/target 14.95 Hz，时延 p50 140 ms p99 168 ms；双跑噪声门槛通过。**场景无小蓝目标**（0/903 检出，无故障）。
- **复测（P1+P2+P3 后）**：默认无可视化节点；camera 87.0、yolo 19.7、td3 20.3（三进程合计 127.0，同口径基线 153.6，-26.6 点；可视化进程整 省 31.5 点）；RSS 共 2235.7 MB（-212 MB：可视化进程 -172、td3 缓存 -33）；detection/target 仍 14.95 Hz，时延 p50 131 ms p99 166 ms，无新故障。yolo 降 22.5 点的来源：rgb8 passthrough 免掉整幅 cvtColor2 + letterbox 乘法从负 stride 视图变连续视图。
- **P1 专项**：`start_visualizer:=true` 无观看者、无 DISPLAY 时 20 s CPU 2.7%（≤3 门）、不出现在 RGB 订阅者里；接入查看器后 2 s 内出现在订阅者列表，输出 4.5 Hz（45/10 s）、45/45 帧时间戳在 `/xiaolan/detection` 命中；断开后下一检查点已退订。
- **P5 关闭（不触发）**：maps 显示每进程只有一套 `libopencv_core.so.4.2` + 一个 cv2 绑定（`~/.local` 的 `cv2.abi3.so`），无双 OpenCV；交接里早先"双 OpenCV"推断不成立。
- **P6 关闭（探针失败）**：相机 89% ≥ 30% 触发 HW D2C 探针；驱动反复报 `Selected profiles do not support HW D2C. color=848x480@15, depth=848x480@15`（`p6_hw_camera.launch.log:294-308`），管道 0 帧。该分辨率组合硬对齐不支持，SW 对齐成本在该 profile 集合下属固有；相机保持 `align_mode:=SW`。
- 验证：unittest Ran 22 OK (skipped=3)；`roslaunch --nodes` 默认 2 节点/显式 3 节点；隔离 master（`roscore -p 11312`，ros_comm 1.17.4 忽略 ROS_MASTER_URI 必须用 `-p`）跑 smoke（onnx）与 perception e2e 均退出码 0；`tests/test_visualizer.py` 未改一字通过。Sol 复核最终态 unittest 通过。
- 证据：`src/docs/evidence/xiaolan_yolo_resource_20261127/`（6 个 JSON + tegrastats 侧录 + launch 日志 + validation.txt + manifest.sha256，校验通过）。
- **遗留（P3 检出等价性）**：实机场景无目标，"同场景检出率相同、框中心 ≤1 px、置信度 ≤0.01"无法测；目标放回视野后按 validation.txt 的 baseline-style 流程补测。相机进程仍在跑（SW），供补测直接用。
- 台架诊断，不构成硬件运动授权；`full_mission.launch` 的感知邻接项（image_proc 矫正链、AprilTag 无 decimate/threads 参数、show_windows 默认 true、standalone_tags 缺 subst_value）未动，仍待用户决定。

## 2026-11-27 Claude Code (Sol): approach deployment prep finished on local test (not pushed)
- Local commits on top of origin/test (537d929 merged in f0ab125):
  - c46e48e: walking gait.
  - d98c163: Q_STAND = walking stance, Q_CLIMB_PREP = old stance, controller climb-prep transition and mission path.
  - a366723: every climb entry runs the transition (X / BT climb / BT mode_climb / run_real / run_sim `--climb-from-walk-stance`). Sol finished the Luna run 2b, which stopped on a rate limit. C++ gtests 5/9/7/13 pass.
  - c31743f: tag chain.
    - full_mission.launch standalone_tags gains subst_value. Before this the detector published nothing.
    - New tag_decimate 2 and tag_threads 2.
    - image_view starts only when DISPLAY is set.
    - Tag teaching happens at P0 (offset 0 by default; hardware: tag visible to ~0.15 m).
    - PROJECT_MEMORY: the camera is pitched slightly down.
    - New `src/grasp_hexapod_bt_control/test/approach_closed_loop_e2e.py`.
- Closed loop (isolated master, compiled bt_control_node, fake servos, kinematic world with synthetic YOLO/tag/IMU, tag taught at P0): 4/4 PASS against P0 with a budget of right 15 mm, forward 20 mm, yaw 3°.
  - straight: −1.6 / 5.4 mm / 0.25°.
  - far offset (1.0 m, 12 cm, 20°): −3.3 / 5.5 mm / −0.30°.
  - search (target outside the FOV): 3.7 / 6.7 mm / 0.26°.
  - tag hidden below 0.24 m, blind finish: −5.6 / 10.1 mm / 0.52°.
  - The tag handoff happens at 0.49 m (the 0.5 m gate).
- Timing: the C++ gait alone costs mean 0.023 ms and max 0.36 ms per frame on the desktop (600 frames walking). The 30–100 ms control-loop spikes seen in the closed-loop run come from host load (Python world + ROS), not from the gait. Isaac `--climb-start --climb-from-walk-stance`: the prep takes 78 frames, then C1..C27 complete; PREVIEW VERDICT SUCCESS.
- Hardware TODO after the push: teach the tag at P0; confirm tag ID (0 vs 255) and size (0.05 vs 0.06), IMU yaw sign and camera pitch; launch with show_windows:=false over SSH.
- Climb session closed. Its v16.6 is still a prototype (`claude_front_v18_20261124/v166hw.json`, 10/11 Isaac, not in the repo). Control/node/BT/launch files are free again for climb work.

## 2026-11-27 攀爬会话（Sol）：v16.6 入库并设为默认（本地提交，未推送）
- 新文件 `src/grasp_hexapod_control/config/climb_front_v16_6.json`，来源：原型 `claude_front_v18_20261124/v166hw.json`（sha256 c9a1b482…），内含 `front_v16_6_provenance`。P0、xiaolan_translation、settle_gate 与 v16.1 相同。
- 摆腿改动（C4/C8/C12/C14/C15/C17/C20）：先按脚位置竖直抬到关节限位允许的最高处（目标 50 mm），再用 joint_space 把小腿翻成朝上，平移过去，在落点上方 ≥50 mm 处展开，最后竖直落下。C3/C11/C19 沿用 v16.4。计划时长 48.9 s。
- 默认配置切换只改了这几处：`mode_climb.cpp` 的 climb_config_file 默认值和注释；`full_mission.launch` 的参数和注释；`control_only.launch` 的注释。v16.1/v16/v11 保留，可回退。
- 在 eccc25d 上重新验证：CPU 30 Hz DONE，1523 帧，0 次截断；用当前源码重编的 C++ cpp_replay 在仿真/实机模式下都是 DONE，与 70d631e 时的结果逐帧一致，峰值 3.80 rad/s。Isaac 11 组 10/11 是之前测的；那之后攀爬代码只做了 Q_CLIMB_PREP 改名。
- 注意：仓库里的证据脚本 `src/docs/evidence/climb_front_direct_20260924/cpu_direct_audit.py` 仍用 Q_STAND 做攀爬起点，在新站姿下会报 "compact entry joint error exceeds simulation gate"。复跑时换成 Q_CLIMB_PREP（这次用的是 scratchpad 里改过的副本，证据目录本身没动）。
- 遗留问题：C19 RF 只能先竖直抬 13 mm，随后在低处横移 112 mm；C12 前腿只能抬 23–28 mm；C3/C11 低处横移 10–30 mm。
