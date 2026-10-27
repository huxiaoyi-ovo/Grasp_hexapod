# 单个传感器的健康状态（由 sensor_health_monitor 统一判定后随 SensorHealthArray 发布）。
string name      # 传感器名：imu / gps / rtk / servo / stereo / mono
bool online      # 是否收到过消息（通信在线）
bool fresh       # 新鲜度是否达标（age_s <= max_age）
float32 freq_hz  # 滑窗实测频率（Hz）
float32 age_s    # 距最近一条消息的时长（秒）
string reason    # 人类可读异常原因（如 "超时 0.8s"、"频率 2Hz < 5Hz"）；正常为空
