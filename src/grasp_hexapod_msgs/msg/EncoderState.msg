# 编码器当前状态（encoder_status_node 持续发布，行为树/其他节点订阅）。
# 三态互斥语义同原 EncoderStatus 服务；normal=false 时 landed/not_landed 无意义。
Header header
bool normal       # 串口读取正常且角度值在合理范围
bool landed       # 已落地：出现超过阈值的突变且随后稳定（锁定）
bool not_landed   # 未落地：窗口内角度固定（max-min < 阈值）
float64 angle     # 最近有效角度（deg）
string reason     # 人类可读说明（如 "未检测到突变"、"串口无数据"）
