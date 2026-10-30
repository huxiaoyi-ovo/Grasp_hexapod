# LM 调平同步抬脚（已推广）

SHA `f8da102282bffef261bfa0a2fe94b07558121f6c4bb38bc97c1c8c1c3061440f` 已推广为默认配置，并从冻结 6761 生成。C20 将固定足短 x 预载缩为 `.5 s`，保持 `z=.226 m`、roll `.16 rad`，不下降或调平；C21 在 LM 抬起的首 `1.2 s` 同步调平到 roll `0`，高度仍为 `.226 m`，随后保留原 `.45 s`；C22 从该状态接回原终点。C23 之后及终态完全冻结。该调整使路径更平顺，计划时长只少 `.15 s`，不作为显著提速结论。

30 Hz 连续 seed DLS 的局部结果：LM 离地后五支撑最小 `.03079897 m`，关节最小裕度 `.13666168 rad`，最大 IK 残差 `4.78e-08 m`，峰值 secant `2.87290096 rad/s`；generic replay DONE 1743 ticks、最小关节裕度 `.03117174 rad`、capsule hold 为零。名义全链可视 GPU 五门通过：C1--C26 为 `57.033333 -> 56.866667 s`，全流程含终态保持为 `61.333333 -> 61.166667 s`，只省 `.166667 s`；其内部 `motion_progress` 分类总量为 `56.100000 s`。终末 foot/root/global-root 为 `.00352759/.00788508/.00847891 m`，最小关节裕度 `.0304283 rad`。`.75` 摩擦、`.10 s` RM-delay stress 也通过五门，`motion_progress/full` 为 `56.100000/61.200000 s`，终末 foot/root/global-root 为 `.00168006/.00654630/.01207420 m`，最小关节裕度 `.0304276 rad`；RM-delay 仅在 `LM_LEFT_FINAL_LAND` 激活一次。所有结果仍是仿真诊断。

DOCK 只审计正常完成攀爬的 terminal 入口：40 mm BODY_RAISE 加默认 20 mm 搜索圆的 243 个 30 Hz 样本中，六足 true-COM 支撑最小 `.13520475 m`、关节最小 `.38631758 rad`、IK 最大 `5.95e-09 m`。base-z 轴倾角不是接触角；TF 驱动的预对准和下降未纳入。任意 HOLD 的 Y 入口仍可回缓存 terminal，自动 mission CLIMB→DOCK 仍未调用 `DockMode.enter()`，均是未解决路由风险。

既有左转移离线门仍不通过：6761 和 68f 均在 `LB_LOW_STEP t=1.7 s` 得到 `.07606457 rad < .08 rad`，没有放宽，也不归因于本候选。
