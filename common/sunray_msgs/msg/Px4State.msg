# -------------Px4State-------------
# 使用一个大而全的ros消息，统一表示PX4飞控的所有状态信息

std_msgs/Header header

# 请注意，PX4State.msg中的模式，坐标系，掩码等，与mavros中一一对应
# 在这里没有使用掩码的原因，是掩码只是方便代码写入，但对理解没有太多的用途，我们尝试在common提供更好的理解msg的工具

# -------------/mavros/state-------------
bool connected	            # mavros 是否与 px4 建立链路连接
bool rc_available	        # RC遥控器链路是否连接
bool armed	                # 飞控解锁状态
uint8 flight_mode           # PX4 当前飞行模式枚举
string flight_mode_name	    # PX4 当前飞行模式（如 OFFBOARD / POSCTL / AUTO.LAND）
uint8 system_status	        # 飞控系统状态（mavros_msgs/State.system_status）

# -------------/mavros/extended_state-------------
uint8 landed_state	        # 起降状态枚举（mavros_msgs/ExtendedState.landed_state）
string landed_state_name    # 起降状态以字符串的方式显示

# -------------/mavros/sys_status-------------
float32 battery_voltage_v	# 当前电压
float32 battery_current_a	# 当前电流
float32 battery_percentage	# 电压百分比
uint16 fcu_load	            # 飞控cpu负载百分比,这里用uint16是和mavros对齐

# ------------外部里程计--------------
geometry_msgs/Pose external_pose        # 外部里程计位姿
geometry_msgs/Twist external_velocity   # 外部里程计速度

# -----------/local_position/pose-------------
geometry_msgs/Pose local_pose           # 惯性系位姿

# -----------/local_position/velocity------------
geometry_msgs/Twist local_velocity      # 惯性系速度

# -------------setpoint/local-------------
uint8 setpoint_coordinate_frame	        # 坐标系（PositionTarget.coordinate_frame）
uint16 setpoint_local_type_mask	        # 掩码（PositionTarget.type_mask）
geometry_msgs/Vector3 pos_setpoint	    # pos_setpoint: [x, y, z]，单位 m，局部 ENU
geometry_msgs/Vector3 vel_setpoint	    # vel_setpoint: [vx, vy, vz]，单位 m/s，局部 ENU
geometry_msgs/Vector3 acc_setpoint	    # acc_setpoint: [roll, pitch, yaw]，单位 m/s^2 ，局部 ENU

float32 yaw_setpoint	                # 偏航角，单位 rad
float32 yaw_rate_setpoint	            # 偏航角速度，单位 rad/s

# -------------setpoint/attitude-------------
uint16 setpoint_att_type_mask	                # 掩码（AttitudeTarget.type_mask）
geometry_msgs/Quaternion orientation_setpoint	# 姿态 四元数
geometry_msgs/Vector3 body_rate_setpoint	    # 机体系角速度设定值（绕机体 x/y/z 轴的旋转速度），
                                                # x=p=roll_rate, y=q=pitch_rate, z=r=yaw_rate，单位 rad/s
float32 thrust_setpoint	                        # 归一化推力设定值，典型范围 [0, 1]

# -------------/mavros/global_position-------------
uint8 satellites	# 可见卫星数量（来自 /mavros/global_position/raw/satellites）
int8 gps_status	    # GPS状态: -1：无法定位  0：普通定位 1：SBAS增强（WAAS, EGNOS） 2：GBAS增强
uint8 gps_service	# GPS服务:  1：美国GPS  2：俄罗斯GLONASS  4：中国北斗  8：欧盟Galileo
# ------- 经纬高 --------
float64 latitude	# 融合/输出纬度，单位 度 (deg)
float64 longitude	# 融合/输出经度，单位 度 (deg)
float64 altitude	# 融合/输出高度，单位 米 (m)
# ------- 经纬高原始值 --------
float64 latitude_raw	# 原始 GNSS 纬度，单位 度 (deg)
float64 longitude_raw	# 原始 GNSS 经度，单位 (deg)
float64 altitude_amsl	# 海拔高度（平均海平面 AMSL），单位 米 (m)
