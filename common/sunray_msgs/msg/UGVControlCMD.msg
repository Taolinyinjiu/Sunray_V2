std_msgs/Header header

uint8 UNDEFINE=0

uint8 CMD_SOURCE_RC=1
uint8 CMD_SOURCE_DEMO=2
uint8 CMD_SOURCE_GROUND_STATION=3
uint8 CMD_SOURCE_PLANNER=4
uint8 CMD_SOURCE_MODULE=5
uint8 CMD_SOURCE_SWARM_CONTROL=6
uint8 cmd_source

uint8 HOLD=1
uint8 RETURN=2
uint8 MOVE_POINT=3
uint8 MOVE_VELOCITY=4
uint8 MOVE_VELOCITY_BODY=5
uint8 MOVE_WGS84=6
uint8 control_cmd

geometry_msgs/Point desired_pos
geometry_msgs/Vector3 desired_vel
geometry_msgs/Vector3 desired_linear
geometry_msgs/Vector3 desired_angular
float64 desired_yaw
geographic_msgs/GeoPoint desired_wgs84_pos
