# bt_control_node 模式状态反馈（调度器发布，dashboard/监控消费）。
# 话题名见 grasp_hexapod_bt_control/config/mode_topics.yaml 的 bt.mode_status。
# 发布时机：状态变化立即发；空闲期 1Hz 心跳。
Header header
string mode      # 当前/最近一次模式名（无任何调用时为空）
string status    # idle / running / paused / success / failed / preempted
uint64 run       # 调用代次（每次 switch_mode 注册/抢占时 +1）
string message   # 终态原因或暂停原因；running 时为空
