# -------------Px4StateCore:-------------
# 1 "Core" 代表控制闭环强依赖的数据，不等价于"全部高频"
# 2 数据可能来自不同频率的话题，订阅端应结合 *_stamp 做判断
std_msgs/Header header

# -------------local_position/odom-------------
geometry_msgs/Vector3 position	# position: [x, y, z]，单位 m，局部 ENU

geometry_msgs/Quaternion attitude_q	# 姿态四元数，与 attitude 对应

geometry_msgs/Vector3 velocity	# velocity: [vx, vy, vz]，单位 m/s，局部 ENU

geometry_msgs/Vector3 attitude_rate	# attitude_rate: [p, q, r]，单位 rad/s，机体系角速度

# -------------setpoint/local-------------
uint8 setpoint_coordinate_frame	# 设定值坐标系（PositionTarget.coordinate_frame）

uint16 setpoint_type_mask	# 设定值掩码（PositionTarget.type_mask）

geometry_msgs/Vector3 pos_setpoint	# pos_setpoint: [x, y, z]，单位 m，局部 ENU

geometry_msgs/Vector3 vel_setpoint	# vel_setpoint: [vx, vy, vz]，单位 m/s，局部 ENU

geometry_msgs/Vector3 acc_setpoint	# acc_setpoint: [ax, ay, az]，单位 m/s^2 ，局部 ENU

float32 yaw_setpoint	# 偏航角设定，单位 rad

float32 yaw_rate_setpoint	# 偏航角速度设定，单位 rad/s

# -------------setpoint/attitude-------------
geometry_msgs/Quaternion q_setpoint	# 姿态设定值四元数

geometry_msgs/Vector3 body_rate_setpoint	# 机体系角速度设定 [p, q, r]，单位 rad/s

float32 thrust_setpoint	# 归一化推力设定值，典型范围 [0, 1]

# -------------timestamp-------------
time odom_stamp	# odom_stamp: local_position 对应时间

time setpoint_stamp	# setpoint_stamp: setpoint feedback 对应时间
