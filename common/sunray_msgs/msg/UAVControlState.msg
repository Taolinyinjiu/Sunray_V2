# -------------UAVControlState-------------
std_msgs/Header header  # 话题头

# UAVControlState 是 sunray_uav_control 对外发布的完整运行状态。
# 典型话题：
#   /uav1/sunray/uav_control/control_state

#【基本参数】
string agent_name                 # 机器人名称前缀，例如 uav
uint8 agent_id                    # 机器人编号，默认从 1 开始
uint8 controller_types            # 控制器类型
float64 takeoff_relative_height     # 当前起飞相对高度
float64 takeoff_max_velocity        # 当前起飞最大速度
uint8 land_type                 # 当前降落类型
float64 land_max_velocity       # 当前降落最大速度
geometry_msgs/Vector3 home_point    # 当前返航目标点

# Sunray控制模块状态枚举
uint8 OFF=0     # OFF状态表示当前不满足正常工作的条件,这里也许需要引入一个错误码，来表示那些条件没有被满足?
uint8 INIT=1    # INIF状态表示当前初始化完成，PX4接受到外部定位信息，切换为Position模式,可以通过遥控器进行控制
uint8 TAKEOFF=2 
uint8 HOVER=3
uint8 RETURN=4
uint8 LAND=5
uint8 MOVE=6
uint8 EMERGENCY_KILL=7

# 当前的sunray控制模块状态
uint8 control_state

# 当前执行的控制指令
UAVControlCMD last_cmd

# ---------- current odom ----------
# self_odom 为原始本机里程计，包含位置、姿态和速度。
nav_msgs/Odometry self_odom

# ---------- controller output ----------
# 实际发布到底层 MAVROS setpoint 话题的控制量
# PX4_OriginController 通常发布 mavros_msgs/PositionTarget 到 /mavros/setpoint_raw/local
# GeometricController / sunray_attitude_controller 通常发布 mavros_msgs/AttitudeTarget 到 /mavros/setpoint_raw/attitude
uint8 OUTPUT_NONE=0
uint8 OUTPUT_POSITION_TARGET=1
uint8 OUTPUT_ATTITUDE_TARGET=2
uint8 controller_output_type

# 最近一次发布的 PositionTarget
mavros_msgs/PositionTarget position_target

# 最近一次发布的 AttitudeTarget
mavros_msgs/AttitudeTarget attitude_target

# 以下为报错信息
bool odometry_lost  # 里程计丢失
bool odometry_valid # 里程计无效，通常为里程计xyz为NAN或者姿态非单位四元数
# 电子围栏/地理围栏跃过？
