# 行为树实时状态快照（运行器每 tick 发布一次，≤ ~5Hz）
# 唯一数据源：/grasp_hexapod/bt_state —— bt_monitor.py（终端）与
# bt_dashboard.py（Web）均订阅本话题渲染。
#
# 注意：root_status 是 py_trees 树根状态；主链根为失败回退 Selector，
# 任务失败回退后根状态仍为 SUCCESS，故另用 mission_status 区分任务结果
# （取自运行器 ctx.status_log 最后上报值）。

std_msgs/Header header
string tree_name        # "主链" / "遥控测试链"
string root_status      # 树根状态 INVALID | RUNNING | SUCCESS | FAILURE
string mission_status   # 任务结果：status_log 最后上报 DONE/CLAMPED/RELEASED/FAILED（空=尚未上报）
string active_phase     # 当前阶段 = 最深 RUNNING 节点名（无则空）
string active_feedback  # active_phase 节点的 feedback_message
BtNodeState[] nodes     # 前序遍历（含根，depth=0）
