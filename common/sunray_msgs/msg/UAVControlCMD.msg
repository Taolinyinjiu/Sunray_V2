# -------------UAVControlCMD-------------
std_msgs/Header header	# 消息头（时间戳与坐标系名）

# UAVControlCMD 是 sunray_uav_control 接收的外部控制指令（控制接口）
# 典型话题：
#   /uav1/sunray/uav_control/control_cmd

# 枚举中，使用UNDEFINE = 0标识未定义
uint8 UNDEFINE=0 			# 未定义的来源/命令

# ---------- command source ----------
# ----------- 命令来源枚举 -------------
uint8 SUNRAY_STATION=1  	# SUNRAY地面站
uint8 RC_CONTROLLER=2    	# RC遥控器控制
uint8 TERMINAL=3  		    # 终端TUI/命令行控制
uint8 SWARM_CONTROL=4       # 集群控制模块
uint8 PLANNING=5            # 规划模块
uint8 EXAMPLE_DEMO=6        # 控制案例模块
# 命令来源
uint8 cmd_source

# ---------- control command ----------
# ------------- 控制命令枚举 ------------
# 发布频率规则：
# - TAKEOFF / LAND / RETURN / KILL / HOVER / MOVE_POINT / MOVE_POINT_BODY / MOVE_POINT_WGS84：
#   事件型命令，发送一次即可。
# - MOVE_VELOCITY / MOVE_VELOCITY_BODY：
#   连续型速度命令，需要持续发布，建议 >=10Hz；停止发布后控制节点会按超时逻辑切回悬停。
# - MOVE_TRAJECTORY：
#   连续型轨迹命令，需要持续发布，建议 >=100Hz 或与规划器输出频率一致。
uint8 TAKEOFF=1                     # 起飞：发送一次即可，无人机原地起飞至起飞高度
uint8 LAND=2                        # 降落：发送一次即可，无人机原地降落
uint8 RETURN=3                      # 返航：发送一次即可，无人机返回起始点并降落
uint8 KILL=4                        # 紧急锁桨：危险操作！发送一次即可，无人机立刻停止转动，自由落体掉落
uint8 HOVER=5                       # 当前位置悬停：发送一次即可，无人机会读取当前位置并在当前位置悬停
uint8 MOVE_POINT=6                  # 惯性系位置控制：发送一次即可，配合desired_pos、desired_yaw使用
uint8 MOVE_VELOCITY=7               # 惯性系速度控制：需要持续发布，建议>=10Hz；配合desired_vel、fixed_height、yaw_mode使用
uint8 MOVE_TRAJECTORY=8             # 惯性系轨迹控制：需要持续发布，建议>=100Hz或与规划器频率一致；配合desired_pos、desired_vel、desired_acc、yaw_mode使用
uint8 MOVE_POINT_BODY=9             # 机体系位置控制：发送一次即可，配合desired_body_xy_pos、fixed_height、desired_yaw使用（偏航角为惯性系设定值）
uint8 MOVE_VELOCITY_BODY=10         # 机体系速度控制：需要持续发布，建议>=10Hz；配合desired_body_xy_vel、fixed_height、yaw_mode使用
uint8 MOVE_POINT_WGS84=11           # WGS84系经纬高控制：发送一次即可，暂未支持。
# 控制命令
uint8 control_cmd

# ---------- control data  ----------
# ----------- 控制数据 ----------------
# 惯性系
geometry_msgs/Vector3 desired_pos
geometry_msgs/Vector3 desired_vel
geometry_msgs/Vector3 desired_acc
geometry_msgs/Vector3 desired_jerk

# 机体系
Vector2 desired_body_xy_pos
Vector2 desired_body_xy_vel

# MOVE_POINT_BODY和MOVE_VELOCITY_BODY模式下必须要填写fixed_height
# MOVE_VELOCITY模式下，当fixed_height不为零和负值时，使用fixed_height锁定z轴高度，不再使用desired_vel字段中的z轴数据
float32 fixed_height

# TAKEOFF模式下可选填写。大于0时覆盖控制节点yaml中的默认起飞参数；小于等于0时使用yaml默认值。
float32 takeoff_relative_height
float32 takeoff_max_velocity
# LAND模式下可选填写。大于0时覆盖控制节点yaml中的默认降落速度；小于等于0时使用yaml默认值。
float32 land_max_velocity

geographic_msgs/GeoPoint desired_wgs84_pos # 经纬高

# ---------- yaw mode ----------
# ------------偏航角控制模式---------
uint8 KEEP_YAW=0 # 保持当前的yaw角不变
uint8 SET_YAW=1     # 控制yaw角
uint8 SET_YAWRATE=2 # 控制yaw_rate
uint8 yaw_mode

# yaw角相关控制指令
float32 desired_yaw
float32 desired_yaw_rate

# ---------- YunLink metadata ----------
# 协议无关的本地命令跟踪标识；由 bridge 或上游本地发布者生成并透传回本地状态链路
uint64 tracking_token
