std_msgs/Header header

# ---------- command source ----------
# ----------- 命令来源枚举 -------------
uint8 SUNRAY_STATION=0  # SUNRAY地面站
uint8 PLANNING=1    # 规划模块
uint8 RC=2  # 遥控器
uint8 DEMO=3    # 示例demo
# 命令来源
uint8 cmd_source

# ---------- control command ----------
# ------------- 控制命令枚举 ------------
uint8 TAKEOFF=0 # 起飞
uint8 LAND=1    # 降落
uint8 RETURN=2  # 返航
uint8 KILL=3    # 紧急锁桨
uint8 HOVER=4   # 当前位置悬停
uint8 MOVE_POINT=5  # 移动到某一点
uint8 MOVE_VELOCITY=6   # 使用速度模式控制
uint8 MOVE_TRAJECTORY=7 # 追踪轨迹
uint8 MOVE_POINT_BODY=8 # 机体系下移动到某一点
uint8 MOVE_VELOCITY_BODY=9  # 机体系下速度模式控制
uint8 MOVE_POINT_WGS84=10   # WGS84 经纬高定位
# 控制命令
uint8 control_cmd

# ---------- control data  ----------
# ----------- 控制数据 ----------------
geometry_msgs/Point desired_pos
geometry_msgs/Vector3 desired_vel
geometry_msgs/Vector3 desired_acc
geometry_msgs/Vector3 desired_jerk
geometry_msgs/Vector3 desired_att   # 如果这里其实是 rpy
geometry_msgs/Quaternion desired_att_quat # 如果这里是四元数

float32 desired_yaw
float32 desired_yaw_rate

geographic_msgs/GeoPoint wgs84_pos # 经纬高

# ---------- yaw mode ----------
# ------------偏航角控制模式---------
uint8 SET_YAW=0     #控制yaw角
uint8 KEEP_YAW = 1  # 保持当前的yaw角不变
uint8 SET_YAWRATE=2 #控制yaw_rate
uint8 yaw_mode

# ---------- mode flag ----------
bool fixed_height
