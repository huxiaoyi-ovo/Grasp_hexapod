# 27 阶段并发紧凑攀爬

当前 SHA 为 `67616b7056329c0f65723962befa5166c4d162c45ca0b85fc79aad516aed9edf`。它从 68f 基线合并五个有界组合：`RB_BODY_ADVANCE`（RB/身体）、`LM_RM_PRE_ADVANCE`（LM/RM 预推进）、`LM_BODY_TRANSFER`（LM/身体；RM 在此阶段保持固定支撑）、`RM_BODY_REPOSITION`（RM/身体重定位）及 `LB_LF_DOCK_TRANSFER`（LB/LF/最终身体对接）。八个关键左转移阶段以及最终身体和六足状态仍与 68f 字节一致。

名义 GPU 五门通过：运动时间由 `64.233333 s` 降至 `57.033333 s`，节省 `7.2 s`（`11.2%`）；完整时间由 `68.533333 s` 降至 `61.333333 s`。`.75` 摩擦、`.10 s` RM 延迟的 stress GPU 五门通过，完整时间为 `57.066667 s`；RM 延迟仅在 `LM_LEFT_FINAL_LAND` 阶段才激活。该结果仍是仿真诊断，不是硬件接触、载荷、摩擦或稳定性认证。

`test_climb_concurrency.py` 与曲线连续性测试共 16 项通过；27 阶段 strict、final-transfer 入口及默认 retime audit 也通过。被拒绝的 26 阶段 ABC 候选虽然名义通过，但在低摩擦 stress 的 `LF_LOW_STEP`、RM 延迟激活前失稳；移除 B 后 AC 通过。这只说明组合 B 与结果相关，不构成已测得接触力机制或单一根因。

局部 CAD 接受路径共采样 11 个节点与 15 个段中点：A/C 复用与被拒绝 ABC 几何相同的 9 节点和 12 中点，B1 自身贡献 2 节点和 3 中点；被拒绝 ABC 的 B 样本不作为当前 B1 证据。这不是连续全机器人 CAD 证明。曲线速度只是参考轨迹，不是带载舵机实测速率。

左转移密集离线门仍未通过且未放宽：冻结 68f 与当前 6761 在相同 `LB_LOW_STEP` `t=1.700 s` 都得到 leg 0 / joint 2 裕度 `0.07606457 rad < 0.08 rad`。左转移阶段字典未变，因此这不是本轮并发新增回归；两版均不记录为 left-gate PASS。
