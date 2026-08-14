# 攀爬配置分层

- `climb_compact.json` 是 Isaac Gym 预览唯一的当前运行计划。
- `climb_rebuild_profile.json` 是当前 compact 的可编辑模型/profile 快照；它只用于离线候选重建。
- snapshot 自动从 compact 的 `xiaolan_translation` 写入模型 `world_from_model` 平移；换模型时编辑其表面、endpoint 与 `surface_id`，再重建候选。`surface_id: unassigned` 时 build 只警告；必须人工赋面并另做 CAD 检查，profile 不会自动选点。
- 旧规划、trace 和长 validator 已归档到 `src/reference/climb_history/`，不是运行时、不会维护，仅是 simulation-history evidence。
- 当前 compact schema 已移除 `expected_q_end`：运行时不消费它，历史终点快照也不再匹配手调预览阶段。

日常预览检查：

```bash
python3 src/grasp_hexapod_control/scripts/validate_climb_preview.py
```

离线快照/候选工作流（不自动选落脚点、阶段顺序或时序）：

```bash
python3 src/grasp_hexapod_control/scripts/tools/rebuild_climb_preview.py snapshot \
  --config src/grasp_hexapod_control/config/climb_compact.json \
  --model src/grasp_hexapod_description/meshes/xiaolan/base_link_xiaolan.STL --output /tmp/profile.json
python3 src/grasp_hexapod_control/scripts/tools/rebuild_climb_preview.py build \
  --template src/grasp_hexapod_control/config/climb_compact.json --profile /tmp/profile.json \
  --output /tmp/candidate.json --scope-report /tmp/candidate_scope.json
python3 src/grasp_hexapod_control/scripts/validate_climb_preview.py \
  --generic --config /tmp/candidate.json --report /tmp/candidate_report.json
```
