std_msgs/Header header

# 枚举中，使用UNDEFINE = 0标识未定义
uint8 UNDEFINE=0 					# 未定义的来源

# ---------- command source ----------
# ----------- 命令来源枚举 -------------
uint8 SUNRAY_STATION=1  	# SUNRAY地面站
uint8 RC_CONTROLLER=2    	# RC遥控器控制
uint8 TERMINAL=3  				# 终端TUI/命令行控制
uint8 CONTROL_CMD=4    		# Sunray系列代码控制
# 命令来源
uint8 cmd_source

# ---------- control command ----------
# ------------- 控制命令枚举 ------------
uint8 TAKEOFF=1 # 起飞
uint8 LAND=2    # 降落
uint8 RETURN=3  # 返航
uint8 KILL=4    # 紧急锁桨
uint8 HOVER=5   # 当前位置悬停
uint8 MOVE_POINT=6  # 移动到某一点
uint8 MOVE_VELOCITY=7   # 使用速度模式控制
uint8 MOVE_TRAJECTORY=8 # 追踪轨迹
uint8 MOVE_POINT_BODY=9 # 机体系下移动到某一点
uint8 MOVE_VELOCITY_BODY=10  # 机体系下速度模式控制
uint8 MOVE_POINT_WGS84=11   # WGS84 经纬高定位
# 控制命令
uint8 control_cmd

# ---------- control data  ----------
# ----------- 控制数据 ----------------
geometry_msgs/Vector3 desired_pos
geometry_msgs/Vector3 desired_vel
geometry_msgs/Vector3 desired_acc
geometry_msgs/Vector3 desired_jerk

float32 desired_yaw
float32 desired_yaw_rate

geographic_msgs/GeoPoint wgs84_pos # 经纬高

# ---------- yaw mode ----------
# ------------偏航角控制模式---------
uint8 KEEP_YAW=0 # 保持当前的yaw角不变
uint8 SET_YAW=1     # 控制yaw角
uint8 SET_YAWRATE=2 # 控制yaw_rate
uint8 yaw_mode

# ---------- mode flag ----------
bool fixed_height
