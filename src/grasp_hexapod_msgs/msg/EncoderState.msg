# 编码器当前状态（encoder_status_node 持续发布，行为树/其他节点订阅）。
# 两态：最近有效角度 ∈ [90,180] deg 视为落地；否则（含串口无数据）未落地。
Header header
bool landed       # 已落地：最近有效角度在落地角度范围内
float64 angle     # 最近有效角度（deg）
string reason     # 人类可读说明（角度范围判定 / 串口无数据）
