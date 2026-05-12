# 话题头
std_msgs/Header header

# 当前起飞相对高度
float64 takeoff_relative_height
# 当前起飞最大速度
float64 takeoff_max_velocity
# 当前降落类型
uint8 land_type
# 当前降落最大速度
float64 land_max_velocity

# 当前返航目标点
geometry_msgs/Vector3 home_point

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

# 以下为报错信息
bool odometry_lost  # 里程计丢失
bool odometry_valid # 里程计无效，通常为里程计xyz为NAN或者姿态非单位四元数
# 电子围栏/地理围栏跃过？
