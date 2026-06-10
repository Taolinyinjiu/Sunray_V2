# -------------Px4State-------------
# 使用一个大而全的 ROS 消息，统一表示 PX4/MAVROS 相关状态信息。
# 本消息由 sunray_uav_control::MavrosHelper 周期发布到：
#   /<agent>/sunray/px4_state
#
# 设计目的：
#   1. 把常用 MAVROS 状态、定位、RC、GPS、setpoint 回显整合到一个话题。
#   2. 方便地面站、监控工具、调试脚本和二次开发模块直接订阅。
#   3. 避免上层模块同时订阅大量 /mavros/* 话题。

std_msgs/Header header

# 注意：
#   1. flight_mode / landed_state / coordinate_frame / type_mask 等枚举值保持与 MAVROS/MAVLink 语义一致。
#   2. ROS/MAVROS 侧常用 ENU 或 ROS frame；PX4/MAVLink 内部常用 NED。
#      MAVROS 会做部分坐标转换，使用时仍应结合具体插件确认 frame 语义。
#   3. RC 通道是原始通道数组，不在本消息中解释为 roll/pitch/throttle/yaw。
#      通道含义由遥控器、接收机、PX4 RC_MAP_* 参数和 MAVROS 配置共同决定。

# ------------- /mavros/state : mavros_msgs/State -------------
bool connected	            # MAVROS 是否与 PX4/FCU 建立 MAVLink 连接，对应 State.connected
bool rc_available	        # PX4/MAVROS 是否认为存在手动输入，对应 State.manual_input；不等价于 rc_channels 一定有效
bool armed	                # 飞控解锁状态，对应 State.armed
uint8 flight_mode           # PX4 当前飞行模式，MavrosHelper 将 State.mode 字符串转换为内部 FlightMode 枚举值
uint8 system_status	        # 飞控系统状态，对应 State.system_status，语义为 MAVLink MAV_STATE

# ------------- /mavros/rc/in : mavros_msgs/RCIn -------------
uint16[] rc_channels        # RC 原始通道值数组，直接来自 RCIn.channels；通常为 PWM 或 MAVLink RC_CHANNELS 原始值
uint8 rc_rssi               # RC 信号强度，直接来自 RCIn.rssi；取值含义依 MAVLink/MAVROS 实现

# ------------- /mavros/extended_state : mavros_msgs/ExtendedState -------------
uint8 landed_state	        # 起降状态枚举，对应 ExtendedState.landed_state，如 ON_GROUND / IN_AIR / TAKEOFF / LANDING

# ------------- /mavros/sys_status : mavros_msgs/SysStatus -------------
float32 battery_voltage_v	# 电池电压，单位 V；MavrosHelper 由 SysStatus.voltage_battery 转换，当前按 mV / 1000 处理
float32 battery_current_a	# 电池电流，单位 A；MavrosHelper 由 SysStatus.current_battery 转换，当前按 mA / 1000 处理
float32 battery_percentage	# 电池剩余比例，范围通常为 [0, 1]；MavrosHelper 由 SysStatus.battery_remaining(%) / 100 转换
uint16 fcu_load	            # 飞控负载；MavrosHelper 由 SysStatus.load / 1000 得到 system_load 后写入，字段类型为历史兼容保留

# ------------- Sunray -> MAVROS 外部定位输入快照 -------------
# 来源不是 PX4 回传，而是 Sunray 控制器传给 MavrosHelper::pub_vision_pose() 的外部定位数据。
# 根据 basic_param.fuse_odom_type，该数据会发布到：
#   1: /mavros/vision_pose/pose
#   2: /mavros/odometry/in
geometry_msgs/Pose external_pose        # 外部定位/里程计位姿，位置单位 m，姿态为四元数
geometry_msgs/Twist external_velocity   # 外部定位/里程计速度；linear 单位 m/s，angular 单位 rad/s

# ------------- /mavros/local_position/odom : nav_msgs/Odometry -------------
geometry_msgs/Pose local_pose           # PX4/MAVROS 局部位姿快照，来自 Odometry.pose.pose
geometry_msgs/Twist local_velocity      # PX4/MAVROS 局部速度快照，来自 Odometry.twist.twist；linear 单位 m/s，angular 单位 rad/s

# ------------- /mavros/setpoint_raw/target_local : mavros_msgs/PositionTarget -------------
# 该组字段是 PX4/MAVROS 最近一次位置类 setpoint 回显/缓存，不是上层 UAVControlCMD 原始命令。
# 对应 Sunray 发布到 /mavros/setpoint_raw/local 的结果或 PX4 target_local 回显。
uint8 setpoint_coordinate_frame	        # 坐标系，对应 PositionTarget.coordinate_frame，如 LOCAL_NED / BODY_NED 等 MAVLink frame 枚举
uint16 setpoint_local_type_mask	        # PositionTarget.type_mask，表示哪些 position/velocity/accel/yaw/yaw_rate 字段被忽略
geometry_msgs/Vector3 pos_setpoint	    # 位置 setpoint: [x, y, z]，单位 m；frame 语义由 coordinate_frame 决定
geometry_msgs/Vector3 vel_setpoint	    # 速度 setpoint: [vx, vy, vz]，单位 m/s；frame 语义由 coordinate_frame 决定
geometry_msgs/Vector3 acc_setpoint	    # 加速度或力 setpoint，单位通常为 m/s^2；是否表示 force 取决于 type_mask
float32 yaw_setpoint	                # 偏航角 setpoint，单位 rad；是否有效取决于 type_mask
float32 yaw_rate_setpoint	            # 偏航角速度 setpoint，单位 rad/s；是否有效取决于 type_mask

# ------------- /mavros/setpoint_raw/target_attitude : mavros_msgs/AttitudeTarget -------------
# 该组字段是 PX4/MAVROS 最近一次姿态类 setpoint 回显/缓存。
# 对应 Sunray 发布到 /mavros/setpoint_raw/attitude 的结果或 PX4 target_attitude 回显。
uint16 setpoint_att_type_mask	                # AttitudeTarget.type_mask，表示忽略姿态、body rate 或 thrust 等字段
geometry_msgs/Quaternion orientation_setpoint	# 姿态 setpoint 四元数；是否有效取决于 type_mask
geometry_msgs/Vector3 body_rate_setpoint	    # 机体系角速度设定值（绕机体 x/y/z 轴的旋转速度），
                                                # x=p=roll_rate, y=q=pitch_rate, z=r=yaw_rate，单位 rad/s
float32 thrust_setpoint	                        # 归一化推力设定值，典型范围 [0, 1]；PX4 是否接受反推取决于参数/配置

# ------------- /mavros/gpsstatus/gps1/raw : mavros_msgs/GPSRAW -------------
uint8 satellites	    # 可见卫星数量，来自 GPSRAW.satellites_visible
int8 gps_status	        # GPS fix_type，来自 GPSRAW.fix_type；常见值: 0 no GPS, 1 no fix, 2 2D fix, 3 3D fix 等
uint8 gps_service	    # 历史兼容字段；当前 MavrosHelper 直接填入 GPSRAW.fix_type 的无符号副本，不是 GNSS 星座 service bitmap

# ------- 经纬高 --------
float64 latitude	    # GNSS 纬度，单位 deg；MavrosHelper 由 GPSRAW.lat * 1e-7 转换
float64 longitude	    # GNSS 经度，单位 deg；MavrosHelper 由 GPSRAW.lon * 1e-7 转换
float64 altitude	    # GNSS 高度，单位 m；MavrosHelper 由 GPSRAW.alt * 1e-3 转换

# ------- 经纬高原始值 / AMSL --------
float64 latitude_raw	# 原始 GNSS 纬度整数值，直接由 GPSRAW.lat 转 double，未乘 1e-7
float64 longitude_raw	# 原始 GNSS 经度整数值，直接由 GPSRAW.lon 转 double，未乘 1e-7
float64 altitude_amsl	# 海拔高度 AMSL，单位 m；MavrosHelper 当前同样由 GPSRAW.alt * 1e-3 填充
