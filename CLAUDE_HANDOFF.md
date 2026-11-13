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
