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
geometry_msgs/PoseStamped home_point

# 当前无人机的状态PX4State
sunray_msgs/Px4State px4_state

# ------------- 控制命令枚举 ------------
uint8 UNDEFINE=0    # 未知指令
uint8 TAKEOFF=1     # 起飞
uint8 LAND=2        # 降落
uint8 RETURN=3      # 返航
uint8 KILL=4        # 紧急锁桨
uint8 HOVER=5       # 当前位置悬停
uint8 MOVE_POINT=6  # 移动到某一点
uint8 MOVE_VELOCITY=7   # 使用速度模式控制
uint8 MOVE_TRAJECTORY=8 # 追踪轨迹
uint8 MOVE_POINT_BODY=9 # 机体系下移动到某一点
uint8 MOVE_VELOCITY_BODY=10  # 机体系下速度模式控制
uint8 MOVE_POINT_WGS84=11   # WGS84 经纬高定位

# 当前的控制指令
uint8 control_cmd

# SunrayFSM状态枚举
uint8 FSM_OFF=0
uint8 FSM_INIT=1
uint8 FSM_TAKEOFF=2
uint8 FSM_HOVER=3
uint8 FSM_RETURN=4
uint8 FSM_LAND=5
uint8 FSM_MOVE=6
uint8 EMERGENCY_KILL=7

# 当前的sunray状态机状态
uint8 sunray_fsm_state

# 当前误差
geometry_msgs/Vector3 pos_err

# 当前是否进入运动稳态
bool move_stable
