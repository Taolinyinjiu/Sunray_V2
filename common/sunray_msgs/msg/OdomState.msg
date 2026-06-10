# -------------OdomState-------------
std_msgs/Header header	# 消息头（时间戳与坐标系名）

# OdomState 是 localization_fusion 对外发布的完整运行状态。
# 典型话题：
#   /uav1/sunray/localization_fusion/odom_state

#【基本参数：外部定位来源】
# external_source 重定位数据来源枚举
uint8 VIOBOT = 0	    # VIOBOT 系列定位源（含 VIOBOT2、Baton mini）
uint8 MOCAP = 1	        # 动作捕捉定位源
uint8 VINS = 2          # VINS算法
uint8 GAZEBO = 3        # Gazebo仿真
uint8 GAZEBO_ARUCO = 4  # Gazebo中的Aruco定位
uint8 PENGYU_SIM = 5    # 借助pengyu_sim仿真包进行仿真
uint8 FASTLIO_EKF = 6   # Fastlio算法+EKF高频
# 当前使用的外部定位源
uint8 external_source	

#【输入信息：话题名&话题状态】
string subtopic_name_external_odom
bool odometry_valid	                # 外部定位里程计数据的有效性,仅根据时间戳判断
float32 odometry_update_hz          # 外部定位里程计的更新频率

string subtopic_name_external_relocalization # 如果为空，则代表没有配置重定位话题

#【输出信息：话题名&信息】
string pubtopic_name_local_odom
string pubtopic_name_global_odom

#【输出信息：里程计信息】
# localization_fusion 当前发布的局部/全局里程计快照
# local_odom.header.stamp 或 global_odom.header.stamp 为 0 时，表示对应 odom 尚未输出有效数据
nav_msgs/Odometry local_odom         # 当前发布到 sunray/localization/local_odom 的局部里程计
nav_msgs/Odometry global_odom        # 当前发布到 sunray/localization/global_odom 的全局里程计

#【输出信息：TF坐标名字及变化关系】
# localization_fusion 当前维护的 TF 快照，仅透传当前内部缓存，不额外改写时间戳
string world_frame_name             # 当前world坐标系名称
string global_frame_name            # 当前全局坐标系名称
string local_frame_name             # 当前局部坐标系名称
string base_frame_name              # 当前机体系名称
geometry_msgs/TransformStamped world_to_global_tf   # world -> {agent}/sunray_global
geometry_msgs/TransformStamped global_to_local_tf   # {agent}/sunray_global -> {agent}/sunray_local
geometry_msgs/TransformStamped local_to_base_tf     # {agent}/sunray_local -> {agent}/base_link
