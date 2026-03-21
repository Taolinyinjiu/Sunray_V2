# -------------OdomStatus-------------
# 外部定位源状态消息：用于描述来源类型、能力、发布模式与当前状态量
std_msgs/Header header	# 消息头（时间戳与坐标系名）

# -------------source-------------
uint8 external_source	# 外部定位数据来源类型

uint8 VIOBOT = 0	# VIOBOT 系列定位源（含 VIOBOT2、Baton mini）
uint8 MOCAP = 1	# 动作捕捉定位源
uint8 VINS = 2
	
# -------------capability-------------
bool provides_local	# 是否提供局部定位能力

bool provides_global	# 是否提供全局定位能力

bool provides_loopcheck	# 是否支持回环检测

# -------------status-------------
bool odom_valid	# 外部定位数据是否有效（通常基于数据间隔超时判定）

bool loopcheck_valid	# 回环检测是否有效

# -------------publish_mode-------------
uint8 publish_mode	# 当前发布模式

uint8 SINGLE_COPY = 0	# 单路复制输出
uint8 DUAL_DIRECT = 1	# 双路直出
uint8 INIT_TRANSFORM = 2	# 初始化变换后输出

# -------------topic-------------
string external_local_topic	# 外部局部定位话题名

string external_global_topic	# 外部全局定位话题名

string external_loopcheck_topic	# 外部回环检测话题名

# -------------data-------------
geometry_msgs/Vector3 local_position	# 位置 [x, y, z]，单位 m
geometry_msgs/Vector3 local_velocity	# 速度 [vx, vy, vz]，单位 m/s
geometry_msgs/Vector3 local_attitude_rpy	# 姿态角[roll pitch yaw] 单位deg
geometry_msgs/Quaternion local_attitude_quat	# 姿态角 四元数

geometry_msgs/Vector3 global_position	# 位置 [x, y, z]，单位 m
geometry_msgs/Vector3 global_velocity	# 速度 [vx, vy, vz]，单位 m/s
geometry_msgs/Vector3 global_attitude_rpy	# 姿态角[roll pitch yaw] 单位deg
geometry_msgs/Quaternion global_attitude_quat	# 姿态角 四元数

string loopcheck_tf_parent_frame
string loopcheck_tf_child_frame

geometry_msgs/Transform loopcheck_tf