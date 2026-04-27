std_msgs/Header header

uint8 FRAME_LOCAL=0
uint8 FRAME_BODY=1

uint8 MECANUM=0
uint8 DIFFERENTIAL=1

uint8 CONTROL_HOLD=0
uint8 CONTROL_POINT=1
uint8 CONTROL_VELOCITY=2
uint8 CONTROL_VELOCITY_BODY=3

uint8 reference_frame
uint8 ugv_type
uint8 control_mode

geometry_msgs/Point desired_pos
geometry_msgs/Vector3 desired_vel
geometry_msgs/Vector3 desired_linear
geometry_msgs/Vector3 desired_angular
float64 desired_yaw

geometry_msgs/Point current_pos
geometry_msgs/Vector3 current_vel
float64 current_yaw

geometry_msgs/Vector3 pos_error
geometry_msgs/Vector3 vel_error
float64 yaw_error

geometry_msgs/Twist cmd_vel
