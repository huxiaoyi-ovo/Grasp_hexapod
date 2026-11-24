# 正面攀爬开发 SOP

本文整理 v11（2026-11-12 冻结、2026-11-13 设为默认）从"已跑通的 v10"逐步改到可上实机的开发思路和流程。用途：机器人尺寸、关节、质量或小蓝几何变了以后，照这份思路和流程把攀爬重新做到可部署。第 1 节是思路和原则，第 2 节以后是具体步骤、工具和 v11 的参考数值。

- 权威顺序不变：`AGENTS.md` > `AGENT_HARNESS.md` > `PROJECT_MEMORY.md` > `CLIMB_ENGINEERING_RULES.md`。本文只描述流程，规则冲突以前者为准。
- 每修一个问题，按 `CLIMB_ENGINEERING_RULES.md` 的八项模板追加卡片，本文不重复。
- 当前基线：`src/grasp_hexapod_control/config/climb_front_v11.json`（SHA-256 `1683c12c…c6a8`），证据目录 `src/docs/evidence/climb_front_v11_20261112/`。
- 所有仿真/回放结果都只是诊断，不证明接触、载荷、摩擦或实机安全。

---

## 1. 整体开发思路与原则

这一节是本文的核心。第 2 节以后的步骤、工具和数值都会随版本变化，这里的思路不会。尺寸变了、小蓝换了、甚至换一种攀爬方向，都先按这一节想清楚，再去动配置。

### 1.1 怎么看待攀爬这个问题

- **攀爬是一串准静态支撑状态的切换。** 每个阶段只做一类事：要么六足都踩着、只动机身；要么一部分腿摆动、其余腿当支撑。每一帧机器人都必须靠当前支撑腿站稳，不能指望动态平衡。
- **实机是开环位置控制。** 舵机只有关节位置反馈，没有力和接触传感（足端接触传感器还没装）。控制器不知道脚有没有真的踩实、有没有打滑，段内检查点只能确认"关节到了"，不能确认"踩稳了"。
- 所以**鲁棒性只能来自几何余量，不能指望反馈纠错**：落点离边缘够远、重心离支撑多边形边界够远、关节离限位够远、腿和腿之间够远。起点误差、打滑、舵机跟踪误差都会把这些余量吃掉一部分，余量不够的地方在实机上就会出事。
- 推论：**名义轨迹能跑通不算完成。** 要在设计起点误差（前后/左右 ±2 cm、yaw ±3°）下仍然跑通，才算一个可部署的方案。

### 1.2 分四层决策，从上往下定，从下往上改

| 层 | 决定什么 | 谁来定 | 变动代价 |
|---|---|---|---|
| 策略层 | 接触序列：哪条腿、什么顺序、落到小蓝哪个面，机身在哪些阶段动 | **和用户一起，靠物理推理定** | 最大，改了等于重做 |
| 几何层 | 每个落点的具体坐标、机身位姿、抬腿高度和越障路径 | 按第 1.3 节的余量原则选 | 中，改一个落点要重跑后续阶段 |
| 时序层 | 每段时长、速度曲线、哪里快哪里慢 | 在几何定好之后调 | 小，但会影响鲁棒性 |
| 执行层 | C++ 实机怎么解释配置：检查点、门限、4 rad/s 上限、超时 | 代码固定，规划必须服从 | 规划时当约束，不当变量 |

- 上层没定稳之前不要去优化下层。时长调得再好，落点贴着台阶边也没用。
- 出问题时从下往上排查：先看是不是时序（太快、截断），再看几何（落点、余量），最后才动策略。能在下层解决就不动上层。
- **策略层不交给优化器。** 2026-11-10 试过"固定模板 + CMA-ES 连续优化"，让优化器从代价函数里自己找策略，结果机身晃、腿互穿、脚在地上来回挪始终没搭上小蓝，被用户叫停。优化器可以在几何层、时序层做局部微调，但哪条腿去哪里必须是人想清楚、说得出物理理由的。

### 1.3 原则

1. **从已经跑通的版本出发，逐阶段局部修改。**
   不从空白开始。v11 = v10 + 9 项局部修改，每项只动出问题的那几个阶段，其余阶段保持字节级不变。这样每次修改的影响范围清楚，出了问题也知道是哪一项引起的。

2. **一次只改一件事，改完跑完整回归。**
   攀爬各阶段是串联的：前一阶段的终点就是后一阶段的起点。改一个落点会改变后面所有阶段的初始状态；给摆腿加平滑会吃掉跨台阶的净空；提速会降低起点误差下的成功率。所以每次改动后都要从头回放，并重跑起点误差扫描，不能只看改动的那一段。

3. **落点选在平整面的中心，用"到边缘的距离"衡量。**
   起点误差会让所有落点整体偏移。落点必须在每个方向上都留有余量（目标 ≥ 20 mm），离台阶、边缘、缝、凹槽都要够远。不要用 bbox 或"名义上在面内"来判断。几何上实在给不了这么多余量的地方（比如右侧约 36 mm 宽的窄斜面），如实标注，不要硬塞。

4. **重心余量优先于速度。**
   抬腿阶段，重心到支撑多边形边界的距离是防翻倒的唯一保障。宁可多加一步把支撑腿挪到位，也不要在余量很小的状态下抬腿。v11 的几处关键改动都是为了这个：先把中腿前移再抬前腿、最后一对腿上台前先把中腿再往前送、机身预偏移。

5. **机身大动作只在六足支撑下做。**
   机身大幅抬升或前移时，如果同时有腿在空中，剩下的支撑腿受力不均，会打滑（Isaac 中约 3 cm 后滑）。把机身阶段和摆腿阶段分开，看起来多停一次，实际更稳。

6. **落地姿态要有利于站稳。**
   上台的腿让小腿尽量垂直于落脚面，不要向内收：这样接触更稳、不容易滑，也避免关节被拉到全伸。

7. **空中快，落地慢。**
   摆腿在空中可以快，接近落点时减速、低冲击落地。跨台阶的摆腿保持已验证净空的分段路径，平地摆腿才做连续化。

8. **按实机的能力设计，而不是按仿真的能力设计。**
   实机是 30 Hz、全关节共享 4 rad/s 上限、段内反馈检查点、舵机有跟踪延迟。规划从一开始就按这些约束来，不要先在仿真里用 6 rad/s 调好，再指望实机跟得上。

9. **实机跑的是 C++，仿真和 Python 只是设计工具。**
   部署前必须证明 C++ 和 Python 逐帧一致，并用当前源码重新编译。所有派生字段（如 `terminal_q_rad`）在改动后都要重算，不能留旧值。

10. **证据分级，先便宜后昂贵，每级只信它能证明的东西。**
    配置校验 → CPU 回放 → 落点容差 → Isaac 可视化 → 起点误差扫描 → C++ 一致性 → ROS 端到端 → 实机悬空 → 实机实爬。便宜的检查先挡掉明显错误；仿真通过只说明运动学和仿真物理上可行，不证明真实接触、载荷、摩擦和安全。CPU 回放的碰撞警告是硬失败，不要"先看看 GPU 再说"。

11. **尽早给用户看，用户的观察是最重要的输入。**
    改完前几个阶段就开 Isaac 可视化给用户看，不要先自动跑几个小时。v11 的大部分改动来自用户看仿真后的判断，比如"最后两条腿落点太极限"、"中腿落在竖坡附近容易踩空"、"后腿不用往里收"。

12. **知道什么时候该退回。**
    一项改动如果用鲁棒性换速度，就退回；如果几何上限达不到余量要求，就标注而不是硬改。每个问题修完后都写进 `CLIMB_ENGINEERING_RULES.md`，能自动检查的做成工具或断言。

13. **不用舵机误差或 IMU 推断着地。**
    着地确认将来由足端接触传感器负责。现在规划时只需要给"下探到接触"留出余量，不要发明传感器的频率、延迟或噪声。

### 1.4 一次修改的完整循环

```text
观察（仿真画面 / 用户反馈 / 扫描失败）
  → 定位：哪个阶段、哪条腿、哪一层（时序 / 几何 / 策略）
  → 找根因：坐标、几何余量、接触、时序还是控制语义
  → 最小局部修改（只动这几个阶段，后续阶段自动重新衔接）
  → 检查阶梯（第 5.5 节）：便宜的先跑，失败就停在那一级
  → 给用户看
  → 记录：改动写进 provenance，问题写成关闭卡
```

### 1.5 尺寸变化时，哪些能沿用，哪些要重做

| 能直接沿用 | 需要重新验证或重做 |
|---|---|
| 本节的思路和原则 | 每个落点的具体坐标（可达范围变了） |
| 接触序列（先试着沿用 v11 的 20 阶段） | 机身位姿：高度、俯仰、前移量 |
| 落点、机身、时序规则 | 抬腿高度和越障路径（腿长变了净空就变） |
| 检查阶梯和验收流程 | 时长（关节速度需求随连杆长度变） |
| 执行层语义（检查点、4 rad/s、超时） | 重心余量（质量分布变了） |
| | P0 起点和接近阶段的 tag 示教 |

顺序：先同步机器人模型（第 3 节）→ 检查 v11 的每个落点在新尺寸下是否可达、关节余量是否够 → 沿用接触序列，从第一个失败阶段开始逐个修几何 → 几何都通过后再调时序 → 最后做鲁棒性和冻结。只有当某个阶段在几何上无论如何都做不到时，才回到策略层，和用户一起重新定接触序列。

## 2. 运行时语义（配置在实机上是怎么被执行的）

改配置前要先知道 C++ 实机链路怎么解释它（`grasp_hexapod_control/src/climb_*.cpp`，Python 镜像 `scripts/climb_mode.py`）：

| 项 | 实机行为 |
|---|---|
| 控制频率 | 30 Hz；每帧阶段相位前进一个 dt |
| 段内节点 | **反馈检查点**：所有活动足 FK 误差 ≤ `climb_foot_gate_m`（实机 0.04 m）且关节误差 ≤ 0.08 rad 才越过；舵机跟得慢只会变慢，不会跳步 |
| 阶段结束 | 达标后持续 0.25 s 才进下一阶段 |
| 超时 | 阶段计划时长 + 5 s 仍未完成 → 失败，进 HOLD，不重试 |
| 速度上限 | 全关节共享 4 rad/s 步进上限（实机忽略配置里的 `simulation_joint_velocity_limit_rad_s`） |
| 进入条件 | 已 HOLD；18 关节距 Q_STAND ≤ 0.08 rad；front 配置需 `allow_front_hardware=true` |
| 舵机指令 | 每帧 `command_duration_ms=33` |
| IMU/RTK 监控 | 可选；缺失时只打警告并关闭（5 cm / 10° / 30°/s 持续超限进 HOLD） |
| 对接 | 从攀爬结束时的当前关节姿态开始，不读 `terminal_q_rad` |

配置结构（`SIMULATION_ONLY_CLIMB_COMPACT_V2`）：

- 顶层：`p0`（`q_rad` 起始关节、`base` 起始机身位姿 `[x,y,z,roll,pitch,yaw]`、`anchors_world_m` 起始足端世界坐标）、`xiaolan_translation`（小蓝相对世界的平移，决定起点距离）、`stages`、`settle_gate`、`terminal_q_rad`、`simulation_joint_velocity_limit_rad_s`、`front_vN_provenance`。
- 每阶段：`pose_start/pose_end`（可选 `pose_curve=piecewise_quintic` + `pose_knots`）、`active_legs`（腿序 `lb,lf,lm,rb,rf,rm` = 0..5）、`anchor_knots`（世界坐标节点，非活动腿保持上一阶段终点）、`segment_durations_s`、`anchor_curve`（`piecewise_base_quintic` 时活动腿按 `active_base_knots_m` + `active_base_velocities_m_s` 机身相对 Hermite 曲线运动）、`settle_s`。
- 校验器：Python `ClimbMode(None)._validate_config(cfg)`；C++ `loadClimbCompact` + `enterClimb`。两者都必须通过。

---

## 3. 尺寸变更：先同步机器人模型（任何攀爬改动之前）

机器人几何在多处重复定义，**必须一起改**，否则仿真、Python、C++、实机各算各的。

| 内容 | 位置 |
|---|---|
| 髋位置/朝向、关节轴符号、连杆长度、足端偏移、足球半径、站立角、关节限位、速度限 | `src/grasp_hexapod_control/scripts/kinematics.py`（`HIP_XYZ`、`HIP_YAW`、`JOINT_AXIS_SIGNS`、`THIGH_TO_KNEE_LENGTH`、`KNEE_TO_ANKLE_LENGTH`、`FOOT_OFFSET_ANKLE`、`FOOT_RADIUS`、`STAND_*_ANGLE`、`JOINT_LOWER/UPPER`、`JOINT_VELOCITY_LIMIT`） |
| 同上（C++） | `src/grasp_hexapod_control/include/grasp_hexapod_control_cpp/types.h`（`kHipXyz`、`kHipYaw`、`kJointAxisSigns`、`kThighToKneeLength`、`kKneeToAnkleLength`、`kFootOffsetAnkle`、`kFootRadius`、`kStand*AngleDeg`、限位） |
| 质量/质心估计（COM 裕度用） | `kinematics.py` 的 `ESTIMATED_*` |
| URDF（Isaac 用 `hexapod_isaacgym_view.urdf`） | `src/grasp_hexapod_description/urdf/`：`grasp_hexapod.urdf`、`hexapod_collision.urdf`、`hexapod_isaacgym_view.urdf`、`hexapodurdf.urdf.xacro`、`grasp_hexapod_export.csv` |
| 工作空间边界 | `src/grasp_hexapod_control/config/workspace_bounds.csv`，用 `scripts/tools/analyze_workspace.py` 重新生成 |
| C++/Python 一致性金标 | `src/grasp_hexapod_control/test/golden_control_trajectories.json`，用 `scripts/tools/export_golden_control.py` 重新导出 |
| 舵机零位/方向映射 | `grasp_hexapod_servo`（只读包，改动需与负责人协调） |

同步后的检查：

1. `catkin_make` 后跑 `test_kinematics_golden`、`test_golden_config`、`test_golden_trajectory`：C++ 与 Python 的 FK/控制轨迹一致。
2. Isaac 里站立（Q_STAND）不穿地、不打滑，机身高度与 `kinematics` 预测一致。
3. 小蓝几何变了：替换 `grasp_hexapod_description/meshes/xiaolan/base_link_xiaolan.STL`，并更新 `PROJECT_MEMORY.md` 的 Xiaolan Geometry 段（落脚面、台阶高度、平整区）。

---

## 4. 重新确定起点 P0

P0 同时被攀爬和接近（tag 示教）使用，二者必须是同一个物理位置。

1. 重算 `p0.q_rad = Q_STAND`、`p0.base`（站立机身高度 z 由新 FK 得到）、`p0.anchors_world_m`（Q_STAND 在 `p0.base` 下的 FK）。
2. 检查 `xiaolan_translation`（v11 为 `[0.45, -0.03, 0]`）：腿变长/变短后，前腿第一次落到 12° 斜面的可达性会变，必要时调整起点距离，并同步修改接近阶段的 P0。
3. 实机：按根 `README.md` 的 tag 示教步骤重录 `phase1_visual.yaml` 的 `tag_ref_x_m / tag_p0_z_m / tag_ref_yaw_rad`。

---

## 5. 开发流程（逐阶段）

### 5.0 环境

- 只在 `test` 分支开发；工作区不是 test 时用独立 `git worktree`。
- Python 用系统 `/usr/bin/python3`（conda 环境缺 `rospkg`/`empy`），`catkin_make` 加 `-DPYTHON_EXECUTABLE=/usr/bin/python3`。
- 本机 x86_64 只能按包编译：`catkin_make --only-pkg-with-deps grasp_hexapod_bt_control`（Orbbec SDK 只有 arm64 库）。编完用 `cmake -DCATKIN_WHITELIST_PACKAGES=""` 复位白名单。
- **验证前必须用当前源码重编 C++**（曾用过期库验证，见第 8 节）。

### 5.1 接触序列模板（v11，20 阶段，计划 32.7 s，实机约 39–40 s）

尺寸变了先沿用这个序列，只改坐标和时序；要改序列先和用户确认。

| # | 阶段 | 活动腿 | 作用 |
|---|---|---|---|
| 1 | PREP | — | 六足支撑下抬身、调俯仰 |
| 2 | MID_GROUND_FIRST | lm rm | 中腿先在地面前移，给前腿让空间 |
| 3 | FRONT_TO_FACE_CENTRE | lf rf | 前腿落到 12° 斜面平整区中心 |
| 4 | RF_TO_WING | rf | RF 单独移到右翼，同时机身前移 |
| 5 | SIX_SUPPORT_BODY_ADVANCE | — | 六足支撑机身前移（不可并入相邻抬腿阶段） |
| 6 | BACK_TO_MINUS09 | lb rb | 后腿地面跟进 |
| 7 | MID_GROUND_TO_080 | lm rm | 中腿地面前移 |
| 8 | FRONT_TO_034 | lf rf | 前腿上移 |
| 9 | BODY_TO_105 | — | 六足支撑机身前移/抬升 |
| 10 | BACK_TO_055 | lb rb | 后腿跟进 |
| 11 | MID_PLATFORM_285 | lm rm | 中腿上平台 |
| 12 | FRONT_DIRECT_FINAL_480 | lf rf | 前腿到顶面（避开横向中缝） |
| 13 | LB_GROUND_RELAY | lb | LB 地面中继 |
| 14 | RM_WING_FORWARD_BODY_PRESHIFT | rm | 最后一对上台前，中腿再前移 + 机身预偏移 |
| 15 | RB_DIRECT_PLATFORM_225 | rb | RB 上台，小腿尽量垂直落地面 |
| 16 | LB_PLATFORM_225 | lb | LB 上台，同上 |
| 17 | DIAGONAL_RF_LM_ADVANCE | rf lm | 对角前移 |
| 18 | TRIPOD_LB_LF_RM_ADVANCE | lb lf rm | 三足前移 |
| 19 | RB_FINAL_ADVANCE | rb | RB 到终点（机身前移分两段，避免把 RB 拉到全伸） |
| 20 | FINAL_HOLD | — | 保持 |

### 5.2 编辑方法

- 每项修改写成一个独立脚本（`edit_xxx.py in.json out.json [参数]`），只改目标阶段，然后调用 `chain.rebuild(stages, i)` 从改动处向后传播：非活动腿继承上一阶段终点，活动腿首节点继承上一阶段终点，内部/终点世界节点保留，机身相对节点按各节点时刻的机身位姿重算；输入未变的阶段字节级不变。
- 新增摆腿用 `chain.swing_stage(...)`：抬起 → 越过 → 落下，落点高度从 STL 精确包络取。
- 每个脚本把改动写进 `front_vN_provenance.edits`（或临时 `vN_edits`），最后 `chain.validate(cfg)`。

### 5.3 落脚点规则

- 落点取**平整区中心**，在设计起点误差（前后/左右 ±20 mm、yaw ±3°）内必须仍在同一平整面上。用 `landing_tolerance.py <cfg>` 输出每次落脚四个方向的容差，<20 mm 的必须改或如实标注（右侧窄斜面约 36 mm 宽，几何上限不足，已标注）。
- 离台阶、边缘、横向缝、凹槽的距离是硬指标，不要用 bbox 或名义 patch 判断。
- 最后两条上台腿（RB、LB）不要向内收：小腿尽量垂直落地面（v11 RB 小腿-法向夹角 31°→6°，LB 30°→13°）。
- 关节裕度 ≥ 0.05 rad（v11 C16 LM 曾只有 0.005 rad）。
- 抬腿阶段的 COM 支撑裕度要留够：v11 前腿抬起时 21→33 mm、RB 抬起时 18.6→53.8 mm，都是因为原值偏险才改的。

### 5.4 机身与时序规则

- 机身大幅抬升/前移只在六足支撑下做；把机身动作并入四足摆腿会导致约 3 cm 后滑。
- 最后一对腿上台之前，先把中腿再往前转移一段（v11 RM 沿翼前移到 .315，机身预偏移到 x .28 / z .22 / pitch .40）。
- 空中快、落地慢（`retime.py`）：落地前 8 mm 处插入预接触节点，下降加速度 ≤ 1.5 m/s²，预接触速度 0.02 m/s，最后 8 mm 用 0.25 s；其余段按参考关节速度 2.75–3.3 rad/s 缩放。
- 平地摆腿可以连续化（`smooth.py`，中间节点给通过速度，只在离地和落地停）；**跨台阶的摆腿保持分段零速路径**，且过了抬腿节点后不允许向下的通过速度。
- 最后做硬件定时（`hw_retime.py`）：按 4 rad/s 上限回放全链，只拉长原始峰值超过上限 90% 的阶段，直到零截断。

### 5.5 每次修改后的检查阶梯（任何一级失败就停在这一级修）

| 级 | 检查 | 工具 | 通过标准 |
|---|---|---|---|
| 1 | 配置校验 | `chain.validate` | 无异常 |
| 2 | CPU 30 Hz 回放 | `cpu.sh cfg out.json` | DONE；无 capsule 警告；逐阶段看 COM 裕度、关节裕度、FK 误差、腿间距、截断 |
| 3 | 落脚容差 | `landing_tolerance.py cfg` | 每个落点各方向 ≥ 20 mm（或已标注的例外） |
| 4 | Isaac 名义可视化 | `run_sim.py --climb-start --climb-config cfg --climb-speed 1 --climb-joint-speed 1.2 --climb-metrics m.json` | DONE，五项 preview gate PASS；**给用户看** |
| 5 | 起点误差扫描 | 平移：改 `xiaolan_translation`（机器人误差 e → `xiaolan_translation -= e`）；yaw：`YAW_DEG=±3 python3 yaw_sim.py --headless …`；脚本 `envelope_hw.sh cfg tag "dx dy yaw;…"` | 设计范围 前后/左右 ±20 mm、yaw ±3°；记录通过/失败及失败阶段，不能比上一版差 |
| 6 | 滑移诊断 | `slip.py metrics cfg` | 记录支撑足累计滑移（v11 名义约 27–34 mm） |

Isaac 时用 `--climb-joint-speed 1.2`（4 rad/s × 1.2 = 4.8 rad/s）检查实机定时，不要用 1.8（6 rad/s）的预览结果判断实机。

### 5.6 计算提速（默认做法，2026-11-23 用户要求以后都这么做）

静态搜索（落点、机身位姿、摆腿路径、站姿）一律**向量化 + 多进程**，不要写逐个调用 `evaluate()` 的 Python 循环。

1. **向量化**：`geom` 的 `world_from_base / inv_rigid / apply / base_to_hip / leg_ik / com_base_exact / capsule_clearance_fast / seg_dist` 都支持前置批量维度。把成千上万个候选位姿堆成 `(B,6)`，足端堆成 `(B,6,3)`（或广播 `(6,3)`），一次调用算完。现成封装在 `fastgeom.py`：
   - `evaluate_batch(poses, feet, swing)`：IK 可达、关节余量、COM 余量、胶囊余量、机身离地，与 `plan/state.py evaluate` 结果逐位一致；
   - `swing_path_clear(pose, feet, leg, knots, other)`：一条摆腿路径上对另一条腿和对机身的最小间隙、路径关节余量；
   - `shank_batch(poses, feet)`：小腿相对接触面法线的倾角，与 `plan/shank.py` 一致；
   - `body_clear(poses)`：机身底面圆盘离地。
2. **多进程**：外层候选（落点组合、中继位置等）用 `fastgeom.pool_map`（`multiprocessing` fork 池；进程数受第 7 条资源预算约束，16 核 31 GB 机器上通常 ≤ 12）。每个进程设 `OMP_NUM_THREADS=1`，避免 numpy 线程与进程抢核。
3. **先粗筛后细查**：先对整张网格做端点检查（起脚/落地两个时刻，向量化），只对幸存者做路径和后续阶段检查。
4. **新写批量函数先对拍**：和逐个版本在几十个样本上比较，差值必须为 0 才能用。
5. 多个候选的 CPU 审计（`cpu.sh`）互相独立，可以同时启动多个。
6. 长任务要在关掉 Claude Code 后继续跑时，用 `setsid nohup bash pipeline.sh > log 2>&1 &`，不要挂在会话的后台任务里（会话退出时会被一起结束）。
7. **资源预算：不要把机器用到极限**（2026-11-24 用户要求；当天 `jointback.py` 14 进程 × 每进程一次评估 86400 个位姿，约 3.6–7 GB/进程，把 31 GB 内存撑爆）。
   - **分块**：任何批量调用每次 ≤ `fastgeom.CHUNK` = 8192 个位姿（实测约 41 kB/位姿，峰值约 0.35 GB），用 `fastgeom.chunked(fn, poses, ...)`。禁止在 worker 里把整张大网格一次送进 `evaluate_batch` / `leg_clear`。
   - **按内存定进程数**：`pool_map` 默认用 `safe_procs(per_proc_gb)`，取“核数 − 4”和“可用内存 × 50% ÷ 单进程峰值”中的较小者。新脚本先在一个 worker 上用 `resource.getrusage(...).ru_maxrss` 实测单进程峰值，再填 `per_proc_gb`。
   - **硬上限**：长任务或新脚本用 `systemd-run --user --scope -p MemoryMax=12G -p MemorySwapMax=0 python3 ...` 启动（脱离会话时放在 `setsid nohup` 里面）。超限时只杀这一个任务，不会拖垮桌面和 Isaac。
   - **与 Isaac 并存**：Isaac 3 路并行在跑时，CPU 搜索 ≤ 6 进程、MemoryMax ≤ 8G；GPU 仿真保持 3 路（GPU 利用率已约 91%）。
   - 启动前用 `free -g` 看一眼可用内存；预计峰值超过可用内存的一半时，先缩小网格或改成分段搜索。

实测（v17，16 核）：批量评估单核快约 120 倍；LB 起脚姿态 + 路径搜索（3×18750 个位姿）15 s；落点 × 站姿 × 两步联合搜索（3600 组）5.4 min。原来逐个循环同规模要数小时。工具在 `/home/artrc/grasp_sync_backups/claude_front_v16_20261123/`（`fastgeom.py`、`v17search.py`），仍是原型路径。

---

## 6. 冻结（准备上实机）

全部满足才能冻结：

1. 重算 `terminal_q_rad`：由最终机身位姿 + 最终锚点 IK 得到，与回放末帧差 ≤ 1e-5 rad。改过终点的任何编辑都要重算所有派生字段（`terminal_q_rad`、`stage_count`、来源 SHA）。
2. `simulation_joint_velocity_limit_rad_s = 4.0`；Python 4 rad/s 回放零截断。
3. 当前源码重编 C++，用 `cpp_replay.cpp` 逐帧对比：
   ```bash
   R=/home/artrc/Grasp_hexapod
   g++ -O2 -std=c++17 cpp_replay.cpp -I$R/src/grasp_hexapod_control/include \
     -I$R/src/grasp_hexapod_control/vendor -I/usr/include/eigen3 -L$R/devel/lib \
     -lgrasp_hexapod_control_core -lyaml-cpp -Wl,-rpath,$R/devel/lib -o cpp_replay
   ./cpp_replay cfg.json $R/src/grasp_hexapod_control/config 0 cpp_sim.csv   # 与 Python 逐帧比，≤1e-5 rad、帧数相同
   ./cpp_replay cfg.json $R/src/grasp_hexapod_control/config 1 cpp_hw.csv    # 实机模式：检查点 + 0.04 m 门限，DONE、峰值 < 4 rad/s
   ```
4. Isaac 名义（1.2）DONE + 五项 gate PASS；起点误差扫描结果写入 `sweep_summary.json`。
5. `catkin_make run_tests_grasp_hexapod_control run_tests_grasp_hexapod_bt_control`：不引入新失败（已知旧失败见 `CLAUDE_HANDOFF.md`）。
6. ROS 端到端（假舵机）：`bt_control_node` + 回显 `/<leg>_des → /<leg>_pos` 的假舵机，`switch_mode home` → `switch_mode climb` 返回 `climb complete`，时长与 C++ 回放一致。
7. 部署：
   - 配置存为 `src/grasp_hexapod_control/config/climb_front_vN.json`，写 `front_vN_provenance`（来源配置与 SHA、证据目录、编辑列表、硬件定时、证据边界）。
   - 改 `grasp_hexapod_bt_control/src/modes/mode_climb.cpp` 的 `~climb_config_file` 默认值，以及 `full_mission.launch` 的 `climb_config_file`。
   - 证据目录 `src/docs/evidence/climb_front_vN_<日期>/`：README（编辑列表 + 验证表）、`tools/`、`configs/`、`sweep_summary.json`、`MANIFEST.sha256`。
   - 更新 `PROJECT_MEMORY.md` 快照、`CLIMB_ENGINEERING_RULES.md` 关闭卡、`CLAUDE_HANDOFF.md`（`*.md` 被 gitignore，需 `git add -f`）。

---

## 7. 实机上线阶梯（每级不过就停）

1. 机器人端 `git pull origin test` → `catkin_make` → 跑测试；确认参数 `climb_config_file`、`allow_front_hardware=true`、`climb_foot_gate_m=0.04`。
2. **悬空空跑**：`roslaunch grasp_hexapod_bt_control control_only.launch`，脚离地按 B 回正，按 X 攀爬；看每阶段跟踪误差和检查点停顿，带载跟踪 > 0.08 rad 的阶段先放慢。
3. **手动 P0 实爬**：人工摆到 P0（误差控制在前后 −1~+2 cm、左右 ±1.5 cm、yaw ±2°），挂保护绳，按 X。
4. **只跑接近**：`full_mission.launch start_sequence:=true stop_after:=approach` 重复 5–10 次，测停车误差。
5. **全流程**。

安全：攀爬/对接中**不要按任何手柄键、不要调用其他模式**，任何新的 `switch_mode` 都会触发 2 s 全腿关节空间回正，会从小蓝上摔落（未修复）。暂停用 `rostopic pub -r 20 /grasp_hexapod/hold_motion std_msgs/String "data: 'hold'"`；紧急只能断舵机电源。

---

## 8. 踩过的坑（都已写入 `CLIMB_ENGINEERING_RULES.md`）

| 坑 | 后果 | 对策 |
|---|---|---|
| 优化器自己发明策略 | 机身晃、腿互穿、够不到小蓝 | 人定接触序列，逐阶段改 |
| 落点贴横向缝（8 mm） | yaw +3° 时超时 | 落点取平整区中心，`landing_tolerance.py` |
| 改终点没重算 `terminal_q_rad` | LM 差 0.26 rad，仿真测不出 | 冻结检查 ≤1e-5 rad |
| 按 6 rad/s 预览 | 实机 4 rad/s 第一阶段就截断 | `hw_retime.py`，Isaac 用 1.2 |
| 跨台阶摆腿连续化 | 净空变小，起点偏差下擦边失败 | 只连续化平地摆腿 |
| 机身动作并入四足摆腿 | 后滑约 3 cm | 机身大动作只在六足支撑下做 |
| 用过期 C++ 库验证 | 结论无效 | 先重编，再 C++/Python 逐帧比 |
| 提速过头 | 起点误差鲁棒性下降 | 速度改动后必须重跑第 5 级扫描 |
| `rospack` 缓存/主工作区路径 | worktree 校验时 `$(find)` 指向旧文件 | 干净环境：只 source `/opt/ros/noetic` 并把 `ROS_PACKAGE_PATH` 指向 worktree |
| `pkill -f` 匹配到自身 shell | 命令被杀（exit 144） | 用 `[e]2e` 这种方括号模式 |
| `landing_tolerance.py` 落点本身不安全时 | 误报四向 40 mm（`bad[0]=0` 取到 `STEP[-1]`） | 先查落点本身在 `safe` 掩码内 |
| 只查起脚/落地两个时刻 | 中间路径小腿撞机身（v17 C17 −8.6 mm） | 摆腿路径整条采样查间隙（`swing_path_clear`） |
| 逐个 Python 循环做静态搜索 | 一次搜索几十分钟到数小时，超时 | 按 5.6 向量化 + 多进程 |

---

## 9. 工具现状

v11 的工具归档在 `src/docs/evidence/climb_front_v11_20261112/tools/`：`chain.py`、`edit_*.py`、`landing_tolerance.py`、`retime.py`、`smooth.py`、`hw_retime.py`、`cpu.sh`、`gpu_check.sh`、`envelope*.sh`、`yaw_sim.py`、`slip.py`、`refspeed.py`、`body_accel.py`、`merge_body.py`、`cpp_replay.cpp`。

- 它们是**原型**，写死了原型目录路径（`/home/artrc/grasp_sync_backups/...`）和 v11 的阶段名/坐标。复用前先复制到工作目录，修 `sys.path` 和输出路径。
- `edit_*.py` 里的坐标是针对当前尺寸和小蓝几何的数值，尺寸变化后要按第 5.3 节规则重新选点，不能直接套用。
- 建议（未做）：把 `chain.py`、`landing_tolerance.py`、`retime.py`、`hw_retime.py`、`cpp_replay.cpp` 和起点误差扫描提升为 `scripts/tools/` 下的维护工具，参数化路径和阶段名。

---

## 10. 尚未解决（下一版的起点）

- 支撑足累计后滑约 3 cm，限制"起点偏后"鲁棒性（−2 cm 失败）。需区分仿真接触参数伪影和真实物理。
- RB 停靠在右侧约 36 mm 宽的窄斜面。
- yaw +3° 在 C13 失败（v11 扫描 11 组通过 7 组）。
- 每阶段结束约 0.25 s 持续 + 检查点停顿，总计约 6 s，等足端接触传感器后可缩短。
- 攀爬/对接中任何新模式请求触发全腿回正（安全阻断项）。
- 接近/行走步态 C++（实机）与 Python（仿真）参数分叉，`ResetAndWalkMatchPython` 失败；攀爬不受影响。

---

## 尺寸变更速查清单

```text
[ ] 3   同步 kinematics.py / types.h / URDF×4 / workspace_bounds.csv / 金标，跑 golden gtest
[ ] 3   Isaac 站立检查
[ ] 4   重算 p0（q_rad / base / anchors），检查 xiaolan_translation，同步接近 P0
[ ] 5.2 以 v11 为源，chain.rebuild 从阶段 1 传播新 p0
[ ] 5.5 CPU 回放，找第一个失败阶段 → 按 5.3/5.4 规则只改该阶段 → 重复，直到 DONE
[ ] 5.5 landing_tolerance、Isaac 名义（给用户看）、起点误差扫描
[ ] 5.4 retime / smooth（仅平地）/ hw_retime 零截断
[ ] 6   terminal_q 重算、C++ 重编 + 逐帧对比 + 实机模式、gtest、ROS e2e、证据目录、改默认配置
[ ] 7   悬空 → 手动 P0 → 接近 → 全流程
```
