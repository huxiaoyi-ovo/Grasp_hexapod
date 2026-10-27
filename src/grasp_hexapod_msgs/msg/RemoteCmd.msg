# 遥控器语义命令（remote_control 节点把 /joy 转换后发布于 /grasp_hexapod/remote_cmd）。
string mode        # idle / walk / climb / dock（当前激活的手动模式）
float32 vx         # 前后速度（walk 模式，m/s）
float32 vy         # 左右速度（walk 模式，m/s）
float32 vyaw       # 偏航角速度（walk 模式，rad/s）
bool reset_edge    # B 键边沿：复位请求（本条消息内为单次事件）
bool enable_edge   # A 键边沿：使能请求
bool climb_edge    # X 键边沿：攀爬触发
bool dock_edge     # Y 键边沿：对接触发
