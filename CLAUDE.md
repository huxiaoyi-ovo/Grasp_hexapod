# Claude Code 入口

本文件只做路由，不定义项目规则。权威顺序：

1. `AGENTS.md`：工作流、阶段表、硬件与证据边界（最高权威，本文件不覆盖它）。
2. `AGENT_HARNESS.md`：决策优先级、实现与验证原则、输出结构。
3. `PROJECT_MEMORY.md`：稳定物理事实与当前仿真快照。
4. `CLIMB_ENGINEERING_RULES.md`：任何攀爬阶段规划或修改前必须先读并执行。
5. `HANDOFF.md`（若存在，未入库）→ `CLAUDE_HANDOFF.md`：跨会话状态，不得覆盖上述规则。

## Claude Code 的角色映射（用户 2026-10-30 指定）

| AGENTS.md 角色 | Claude Code 对应 |
|---|---|
| Sol | 主会话：需求、架构、物理/安全推理、冻结合同、最终验收 |
| Luna（按 Terra 合同规则执行） | 主会话新建的子代理（不用 fork），单线程、单写者 |

- Sol 主要负责整体统筹规划、冻结合同和审查；可分离的执行工作交给 Luna。
- Luna 模型：默认 Agent 工具最轻档位；Sol 判断该档能力不足（多文件耦合、难诊断、需较强推理的实现）时升到中间档位，并在合同和 `CLAUDE_HANDOFF.md` 中注明档位与理由。
- Luna effort：每次显式设置，按 `AGENTS.md` 的 Terra reasoning 规则（low/medium/high/xhigh）。
- Sol/Luna 分工、`execution_owner` 声明、三文件边界、合同字段（目标、批准文件、不变量、禁止项、验证层级、证据、停止条件）完全沿用 `AGENTS.md` 的 Sol/Terra 规则，只把 Terra 换成 Luna。
- 子代理无上下文启动：合同必须自包含，写清仓库路径、文件和验证命令；Luna 不做设计决策，越界即停。
- `grasp_hexapod_servo` 只读；只在 `test` 分支开发；默认导师模式（见 `AGENTS.md`）。

## 交接

- 每轮有仓库写入、关键发现或未完成事项时，结束前更新 `CLAUDE_HANDOFF.md`（追加，不覆盖历史）。
- 与 Codex 交接：Codex 侧读取 `CLAUDE_HANDOFF.md` 获知 Claude Code 的工作与版本；Claude Code 开工前先 `git pull origin test` 并读 `HANDOFF.md`（若有）和 `CLAUDE_HANDOFF.md`。
- 本文件与 `CLAUDE_HANDOFF.md` 被 `.gitignore` 的 `*.md` 规则忽略，需 `git add -f` 入库。
