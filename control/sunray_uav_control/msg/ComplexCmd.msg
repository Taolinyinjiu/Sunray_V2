# Direct control passthrough to MAVROS setpoint topics.
# mode: 0=POSITION_TARGET (setpoint_raw/local)
#       1=ATTITUDE_TARGET (setpoint_raw/attitude)
#       2=BOTH
std_msgs/Header header
uint8 mode

# ---- setpoint_raw/local (mavros_msgs/PositionTarget) ----
uint8 coordinate_frame
uint16 position_type_mask
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
float32 yaw
float32 yaw_rate

# ---- setpoint_raw/attitude (mavros_msgs/AttitudeTarget) ----
uint8 attitude_type_mask
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 body_rate
float32 thrust
