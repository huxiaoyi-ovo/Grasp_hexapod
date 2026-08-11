# 攀爬配置分层

- `climb_compact.json` 是 Isaac Gym 预览唯一的当前运行计划。
- `climb.json` 保存未解决的实机与离线分析输入，不能授权运行时攀爬。
- `climb_simple.json` 与三份 `*_trace.json` 是历史/生成证据，不是运行时依赖，也不会安装进控制包。
- 当前 compact schema 已移除 `expected_q_end`：运行时不消费它，历史终点快照也不再匹配手调预览阶段。

日常预览检查：

```bash
python3 src/grasp_hexapod_control/scripts/validate_climb_preview.py
```

历史认证检查，仅在明确需要时运行。它仍检查旧认证字段，不能认证当前 compact 预览：

```bash
python3 src/grasp_hexapod_control/scripts/tools/validate_climb.py
```
