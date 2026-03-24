# -------------OdomStatus-------------
# 外部定位源状态消息：用于描述来源类型、能力、发布模式与当前状态量
std_msgs/Header header	# 消息头（时间戳与坐标系名）

# -------------source-------------
string external_source_name 
uint8 external_source_id	# 外部定位数据来源

string localization_mode_name 
uint8 localization_mode # 重定位模式

# external_source_id 重定位数据来源枚举
uint8 VIOBOT = 0	# VIOBOT 系列定位源（含 VIOBOT2、Baton mini）
uint8 MOCAP = 1	# 动作捕捉定位源
uint8 VINS = 2
uint8 GAZEBO = 3
uint8 GAZEBO_ARUCO = 4

# localization_mode 重定位模式枚举
uint8 LOCAL = 0
uint8 GLOBAL = 1
uint8 LOCAL_AND_GLOBAL = 2
uint8 LOCAL_WITH_ARUCO = 3

# -------------status-------------
bool has_odometry	# 是否收到里程计数据
bool has_relocalization	# 是否收到重定位数据

time last_odometry_time	# 最新的里程计数据发送时间

# 由于localization模块只负责聚合，转发，不对里程计数据进行分析，也不进行有效性判断

bool odom_timeout	# 外部定位数据是否有效（通常基于数据间隔超时判定）

# 由于重定位涉及到了tf转换，当重定位数据错误而继续时，会导致程序发生异常，因此这里对重定位数据进行一些校验
bool relocalization_data_valid

# -------------- tf --------------
string global_frame_id
string local_frame_id
string base_frame_id
