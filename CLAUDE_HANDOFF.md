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
- 待 Sol 决策：是否在 `AGENTS.md` 的 Active Worker Routing 中登记 Claude Code（Sol）/ Luna 路由并指向本文件（当前 `AGENTS.md` 仍写 Claude Code 非活跃路由）。
