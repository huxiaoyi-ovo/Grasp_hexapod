std_msgs/Header header

uint8 WAITING=0
uint8 NO_TARGET=1
uint8 TARGET_VALID=2
uint8 TARGET_PENDING=3
uint8 FAULT=4

uint8 state
string reason
time rgb_stamp
time depth_stamp
float32 detection_hz
float32 depth_hz
float32 target_hz
float32 inference_ms
