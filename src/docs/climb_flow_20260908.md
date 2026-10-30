# 攀爬尾段与停顿优化

候选从 33 阶段变为 32 阶段。C2 的第二段时长为 2.10 s；C2 第二 air knot、C8 两个 air knot，以及 RM/LB-LF 尾段的两个 air knot 各下调 5 mm。C25 将 LM 在空中的末点接近保留的首个中间支撑，C26 只作 0.8 s 竖直下降。旧 `LM_DIRECT_FINAL` 删除，之后 LM 固定在该中间支撑；C27 右侧双腿与机身同步、C28 RM、C29 机身重位、C30 左侧双腿、C31 dock、C32 hold。最终机身和其余五足终点不变。

此前 LM 45 度问题来自强制使用旧最终 LM 落点；固定首个中间支撑后，LM 终态轴角为 25.215 度，边缘余量 19.021 mm，低侧 gap 0.995 mm。C25 的 sampled CAD foot-bottom 最小 gap 为 17.054 mm；它是局部采样，不代表全程连杆净空。落脚阶段 persistence、checkpoint 和 collision gate 未改，也没有新增 IMU 到位门。C24--C25 共享非零的 world-frame 参考速度，runtime 在既有 settled 且无 collision 后同 tick 下发下一阶段首个点，避免重复终点；C26 是普通落脚而没有 marker。

每个可选 `piecewise_base_quintic` 摆动保存内部 Hermite 速度：原曲线内部点采用相邻 unit-chord bisector 的 `.4 × min` 速度；尾段 RM/LB-LF 的修正曲线保留已验证的首点 incoming、末点 outgoing 定向。C2/C8 在原速度求出后降低 knot；尾段 lowering 后仅重算尾段速度。这是工作空间取舍，参考轨迹的非零速度不等于带载舵机实测速度。

候选 SHA `68f60b97b167ca644f0cebe5fd4de1016b9705764a0376b0431459b0e818346c`。名义及 `.75` friction/`.10 s` RM-delay GPU 均以五个 preview gates 到达 DONE；名义 motion/full 为 64.2333/68.5333 s。stress 的 C28 RM 峰值 foot-target error 为 52.43 mm，因此该通过结果不认证全程跟踪鲁棒性。小蓝背面沿其 y 轴的锁紧对接突起已包含在实际 STL；没有另建几何或传感器模型。所有结果仅为仿真诊断，不构成真实接触、载荷、稳定或硬件授权。
