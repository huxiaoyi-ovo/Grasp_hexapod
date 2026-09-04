# 行为树单节点实时状态（/grasp_hexapod/bt_state 数组元素）
# 由行为树运行器每 tick 快照发布，供终端监视 / Web 看板消费。

string name          # 节点名（如 "执行 攀爬到小蓝上 ㉘"）
string status        # INVALID | RUNNING | SUCCESS | FAILURE（py_trees Status 名）
string feedback      # feedback_message 文本（叶子节点多为中文状态说明）
uint32 depth         # 树深度，根=0（前序遍历重建缩进/父子关系）
bool   is_leaf       # true=无子节点（叶子）
