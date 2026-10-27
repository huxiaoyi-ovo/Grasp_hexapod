# Coordinates and header belong to the original RGB image, NOT letterbox/depth.
std_msgs/Header header
bool detected
float32 confidence
int32 bbox_x
int32 bbox_y
int32 bbox_width
int32 bbox_height
int32 center_u
int32 center_v
uint32 image_width
uint32 image_height
float32 inference_ms
string reason
