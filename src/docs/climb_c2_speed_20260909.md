# C2 RM 降低 apex 与提速（已推广）

SHA `5f8edbc7ba46e5776e800cdfb7b786234ea50e844cf617aaf20e6bbd5489fcce` 已推广为默认配置，从冻结 f8da 生成，只改 C2 `RM`：第三个 active-base knot 降 `20 mm`，第二段由 `2.10 s` 缩为 `1.70 s`，再按实际机身位姿回写 world knots 和 `.4` 内部 Hermite tangent。其他阶段、终态、物理频率、30 Hz 控制和 PD 均冻结。C2 仿真时间（非屏幕墙钟播放时间）`3.766667 -> 3.366667 s`，全流程同口径 `61.166667 -> 60.766667 s`，属于小幅提速。

连续 seed DLS：120 Hz 最小关节裕度 `.1405513 rad`、IK 最大 `4.88e-08 m`、峰速 `3.25347 rad/s`；30 Hz 分别为 `.1408393 rad`、`4.35e-08 m`、`3.24511 rad/s`，原速度门均通过。C2 9 个节点/分段中点的局部 visual-CAD 查询没有 RM 非足垫命中；可投影 crossing 点的最小 footpad gap 为 `.0875013 m`。一个 crossing 中点未落在小蓝 STL 的 XY 投影域，记录为 `null`，不算作 15 mm 净空通过，也不解释为接触。

名义全链 GPU 通过五门：C2 peak foot error `.007005 m`（f8da `.009544 m`），global root `.00847525 m`、end root `.00785956 m`、end foot `.00352879 m`、最小关节裕度 `.0304276 rad`。`.75` 摩擦、`.10 s` RM-delay stress 也通过五门：全流程 `60.800000 s`，end foot/root/global-root `.00162178/.00651317/.0119757 m`，最小关节裕度 `.0304278 rad`。这些仍只是仿真预览诊断。

headless 原生计时仅用于播放性能：C1--C4 的 2928 个物理帧中，`simulate` 为 `14.249851 s`、`fetch_results(True)` 为 `.003903 s`、30 Hz controller 块为 `.435807 s`，C1 到 C4 每物理步 `simulate+fetch` 从 `4.066 ms` 增至 `5.625 ms`；无 viewer 时 render/sync 均为零。这表明本样本的掉速在 PhysX `simulate`，不是历史 metrics、fetch、渲染或 DLS；它不代表真实执行速度。

既有 `LB_LOW_STEP t=1.7 s` 的 `.07606457 rad < .08 rad` 离线左转移门仍未通过，未放宽，也不归因于本 C2 候选。所有证据都只是模型/仿真诊断，不构成接触、载荷、摩擦、稳定性或硬件授权。
