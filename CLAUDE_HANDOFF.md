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
