# -------------OdomState-------------
# 外部定位源状态消息：用于描述来源类型、能力、发布模式与当前状态量
std_msgs/Header header	# 消息头（时间戳与坐标系名）

# -------------source-------------
# external_source 重定位数据来源枚举
uint8 VIOBOT = 0	# VIOBOT 系列定位源（含 VIOBOT2、Baton mini）
uint8 MOCAP = 1	# 动作捕捉定位源
uint8 VINS = 2
uint8 GAZEBO = 3
uint8 GAZEBO_ARUCO = 4
uint8 PENGYU_SIM = 5     ## 借助pengyu_sim仿真包进行仿真
uint8 FASTLIO_EFK = 6
# 当前使用的外部定位源
uint8 external_source	

# localization_mode 定位模式枚举
uint8 LOCAL = 0
uint8 GLOBAL = 1
uint8 LOCAL_AND_GLOBAL = 2
uint8 LOCAL_WITH_ARUCO = 3
# 当前使用的定位模式
uint8 localization_mode 

# -------------state-------------
# 当前定位模块状态
bool odometry_valid	                # 里程计数据有效性,仅根据时间戳判断
bool relocalization_valid           # 重定位数据有效性,比如姿态是否为单位四元数,位置是否是有限值
time odometry_received_stamp        # 最新的里程计数据接受到的时间
time relocalization_received_stamp  # 最新的重定位数据接受到的时间
float32 odometry_update_hz           # 里程计更新频率

# -------------output odom-------------
# localization_fusion 当前发布的局部/全局里程计快照
# local_odom.header.stamp 或 global_odom.header.stamp 为 0 时，表示对应 odom 尚未输出有效数据
nav_msgs/Odometry local_odom         # 当前发布到 sunray/localization/local_odom 的局部里程计
nav_msgs/Odometry global_odom        # 当前发布到 sunray/localization/global_odom 的全局里程计

# 由于localization模块只负责聚合，转发，不对里程计数据进行分析，因此这里的有效性判断为对超时的判断，
# 里程计超过0.2秒没有收到新数据，会判定为无效
# 由于重定位自身的特点,重定位数据没有更新频率
