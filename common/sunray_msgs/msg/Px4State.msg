# -------------Px4State-------------
# 使用一个大而全的ros消息，统一表示PX4飞控的所有状态信息

std_msgs/Header header

# ---------- px4类型枚举 -------------

# FlightMode
uint8 FLIGHT_MODE_UNDEFINED=0
uint8 FLIGHT_MODE_MANUAL=1
uint8 FLIGHT_MODE_ACRO=2
uint8 FLIGHT_MODE_ALTCTL=3
uint8 FLIGHT_MODE_POSCTL=4
uint8 FLIGHT_MODE_OFFBOARD=5
uint8 FLIGHT_MODE_STABILIZED=6
uint8 FLIGHT_MODE_RATTITUDE=7
uint8 FLIGHT_MODE_AUTO_MISSION=8
uint8 FLIGHT_MODE_AUTO_LOITER=9
uint8 FLIGHT_MODE_AUTO_RTL=10
uint8 FLIGHT_MODE_AUTO_LAND=11
uint8 FLIGHT_MODE_AUTO_RTGS=12
uint8 FLIGHT_MODE_AUTO_READY=13
uint8 FLIGHT_MODE_AUTO_TAKEOFF=14

# LandedState
uint8 LANDED_STATE_UNDEFINED=0
uint8 LANDED_STATE_ON_GROUND=1
uint8 LANDED_STATE_IN_AIR=2
uint8 LANDED_STATE_TAKEOFF=3
uint8 LANDED_STATE_LANDING=4

# -----------setpoint 坐标系与掩码枚举 -------------

uint8 FRAME_LOCAL_NED=1
uint8 FRAME_LOCAL_OFFSET_NED=7
uint8 FRAME_BODY_NED=8
uint8 FRAME_BODY_OFFSET_NED=9

uint16 IGNORE_PX=1
uint16 IGNORE_PY=2
uint16 IGNORE_PZ=4
uint16 IGNORE_VX=8
uint16 IGNORE_VY=16
uint16 IGNORE_VZ=32
uint16 IGNORE_AFX=64
uint16 IGNORE_AFY=128
uint16 IGNORE_AFZ=256
uint16 FORCE=512
uint16 IGNORE_YAW=1024
uint16 IGNORE_YAW_RATE=2048

# -------------/mavros/state-------------
bool connected	# mavros 是否与 px4 建立链路连接
bool rc_available	# RC遥控器链路是否连接
bool armed	# 飞控解锁状态
uint8 flight_mode # PX4 当前飞行模式枚举
string flight_mode_name	# PX4 当前飞行模式（如 OFFBOARD / POSCTL / AUTO.LAND）
uint8 system_status	# 飞控系统状态（mavros_msgs/State.system_status）

# -------------/mavros/extended_state-------------
uint8 landed_state	# 起降状态枚举（mavros_msgs/ExtendedState.landed_state）
string landed_state_name # 起降状态以字符串的方式显示

# -------------/mavros/sys_status-------------
float32 battery_voltage_v	# 当前电压
float32 battery_current_a	# 当前电流
float32 battery_percentage	# 电压百分比
uint16 fcu_load	# 飞控cpu负载百分比,这里用uint16是和mavros对齐

# ------------/vision_pose 原始值--------------
bool enable_vision_fuse	# 是否启用视觉融合
geometry_msgs/PoseStamped vision_pose

# -----------/local_position/pose-------------
geometry_msgs/PoseStamped local_pose

# -----------/local_position/velocity------------
geometry_msgs/TwistStamped local_velocity # 惯性系
geometry_msgs/TwistStamped body_velocity # 机体系

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



# -------------/mavros/global_position-------------
uint8 satellites	# 可见卫星数量（来自 /mavros/global_position/raw/satellites）
int8 gps_status	    # GPS状态: -1：无法定位 0：普通定位 1：SBAS增强（WAAS, EGNOS） 2：GBAS增强
uint8 gps_service	## GPS服务: 1：美国GPS  2：俄罗斯GLONASS  4：中国北斗  8：欧盟Galileo
# ------- 经纬高 --------
float64 latitude	# 融合/输出纬度，单位 度 (deg)
float64 longitude	# 融合/输出经度，单位 度 (deg)
float64 altitude	# 融合/输出高度，单位 米 (m)
# ------- 经纬高原始值 --------
float64 latitude_raw	# 原始 GNSS 纬度，单位 度 (deg)
float64 longitude_raw	# 原始 GNSS 经度，单位 (deg)
float64 altitude_amsl	# 海拔高度（平均海平面 AMSL），单位 米 (m)

# -------------timestamp-------------
time state_stamp	# state_stamp: /mavros/state 对应时间

time landstate_stamp	# landstate_stamp : /mavros/extended_state

time sys_stamp	# sys_stamp: /mavros/sys_status

time rc_stamp	# rc_stamp: /mavros/rc/in

time gps_stamp	# GPS 相关数据源时间戳

time estimator_stamp	# estimator 相关数据源时间戳

time odom_stamp	# odom_stamp: local_position 对应时间

time setpoint_stamp	# setpoint_stamp: setpoint feedback 对应时间
