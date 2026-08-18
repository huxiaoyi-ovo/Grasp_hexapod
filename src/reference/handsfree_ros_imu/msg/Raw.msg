# CombinedData.msg

std_msgs/Header header

# 原始时间戳（us）
uint32 timeStamp_us
# 原始系统状态字（低 3 位为 QoS）
uint16 raw_sys_state
uint8  qos

# 姿态角
float32 roll_deg
float32 pitch_deg
float32 yaw_deg
# 四元数（xyz, w）
geometry_msgs/Quaternion orientation

# 角速度（rad/s）
geometry_msgs/Vector3 angular_velocity
# 线加速度（m/s^2）
geometry_msgs/Vector3 linear_acceleration
# 磁场（T）
geometry_msgs/Vector3 magnetic_field
# 重力（m/s^2）
geometry_msgs/Vector3 gravity

# 温度（°C）
float32 temperature_C
# 更新频率（Hz）
float32 update_rate_hz

# 备用和校验
uint16 reserved1
uint16 simple_checksum
