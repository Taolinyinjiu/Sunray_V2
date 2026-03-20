# -------------Px4State-------------
# 统一状态消息：合并原 Px4StateCore 与 Px4StateHealth，便于单话题订阅与记录
std_msgs/Header header

# -------------/mavros/state-------------
bool connected	# mavros 是否与 px4 建立链路连接

bool armed	# 飞控解锁状态

string mode	# PX4 当前飞行模式（如 OFFBOARD / POSCTL / AUTO.LAND）

uint8 system_status	# 飞控系统状态（mavros_msgs/State.system_status）

# -------------/mavros/extended_state-------------
uint8 landed_state	# 起降状态（mavros_msgs/ExtendedState.landed_state）

# -------------/mavros/sys_status-------------
bool rc_available	# RC遥控器链路是否连接

float32 battery_voltage_v	# 当前电压

float32 battery_current_a	# 当前电流

float32 battery_percentage	# 电压百分比

float32 fcu_load	# 飞控cpu负载百分比

# -------------/mavros/rc/in-------------
uint16[] rc_channels	# 遥控通道原始值（长度与机型配置相关）

# -------------/mavros/global_position-------------
uint8 satellites	# 可见卫星数量（来自 /mavros/global_position/raw/satellites）

int8 gps_status	# GPS 状态（sensor_msgs/NavSatStatus.status，常见值 -1/0/1/2）

uint8 gps_service	# GPS 服务位掩码（sensor_msgs/NavSatStatus.service）

float64 latitude	# 融合/输出纬度，单位 deg

float64 longitude	# 融合/输出经度，单位 deg

float64 altitude	# 融合/输出高度，单位 m

float64 latitude_raw	# 原始 GNSS 纬度，单位 deg

float64 longitude_raw	# 原始 GNSS 经度，单位 deg

float64 altitude_amsl	# 海拔高度（平均海平面 AMSL），单位 m

uint8 gps_fix_type	# GPS 定位类型（建议映射 PX4 GPS fix type）

float32 gps_eph	# 水平定位精度估计，单位 m（常用含义）

float32 gps_epv	# 垂直定位精度估计，单位 m（常用含义）

# -------------/mavros/estimator_status-------------
bool estimator_healthy	# 估计器健康状态简化布尔量

uint64 estimator_status_flags	# 估计器状态位图（mavros_msgs/EstimatorStatus.flags）

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

geometry_msgs/Vector3 acc_setpoint	# acc_setpoint: [roll, pitch, yaw]，单位 m/s^2 ，局部 ENU

float32 yaw_setpoint	# 偏航角设定，单位 rad

float32 yaw_rate_setpoint	# 偏航角速度设定，单位 rad/s

# -------------setpoint/attitude-------------
geometry_msgs/Quaternion q_setpoint	# 姿态设定值四元数

geometry_msgs/Vector3 body_rate_setpoint	# 机体系角速度设定 [p, q, r]，单位 rad/s

float32 thrust_setpoint	# 归一化推力设定值，典型范围 [0, 1]

# -------------timestamp-------------
time state_stamp	# state_stamp: /mavros/state 对应时间

time landstate_stamp	# landstate_stamp : /mavros/extended_state

time sys_stamp	# sys_stamp: /mavros/sys_status

time rc_stamp	# rc_stamp: /mavros/rc/in

time gps_stamp	# GPS 相关数据源时间戳

time estimator_stamp	# estimator 相关数据源时间戳

time odom_stamp	# odom_stamp: local_position 对应时间

time setpoint_stamp	# setpoint_stamp: setpoint feedback 对应时间
