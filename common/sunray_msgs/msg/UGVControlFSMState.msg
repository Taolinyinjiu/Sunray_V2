std_msgs/Header header

uint8 FSM_INIT=0
uint8 FSM_HOLD=1
uint8 FSM_RETURN=2
uint8 FSM_MOVE=3

float64 geo_fence_x_min
float64 geo_fence_x_max
float64 geo_fence_y_min
float64 geo_fence_y_max
float64 geo_fence_z_min
float64 geo_fence_z_max

geometry_msgs/Point home_point
UGVControlCMD active_control_cmd
uint8 fsm_state

geometry_msgs/Point desired_pos
geometry_msgs/Vector3 desired_vel
float64 desired_yaw

geometry_msgs/Point current_pos
geometry_msgs/Vector3 current_vel
float64 current_yaw

geometry_msgs/Twist controller_cmd_vel

bool odom_valid
bool control_cmd_valid
bool inside_geo_fence
