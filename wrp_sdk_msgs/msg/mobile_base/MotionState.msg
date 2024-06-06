uint32 CONTROL_SOURCE_RC = 0
uint32 CONTROL_SOURCE_CAN = 1
uint32 CONTROL_SOURCE_ETHERNET = 2

std_msgs/Header header
geometry_msgs/Vector3 desired_linear
geometry_msgs/Vector3 desired_angular
uint32 source
bool collision_detected
bool assisted_mode_enabled