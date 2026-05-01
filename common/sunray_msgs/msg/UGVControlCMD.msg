std_msgs/Header header

# UGVControlCMD 是发给 sunray_ugv_control 的底层无人车控制指令。
# 典型发布话题：
#   /ugv1/sunray/ugv_control/control_cmd
#
# 使用规则：
# 1. header.stamp 建议填写当前 ROS 时间；速度类指令会用 stamp 做超时保护。
# 2. cmd_source 表示指令来源，便于调试、仲裁和日志显示。
# 3. control_cmd 表示控制模式，不同模式只读取对应字段，未使用字段保持默认值即可。
# 4. 坐标系约定：
#    - desired_pos / desired_vel 使用本地世界坐标系，单位 m / m/s。
#    - cmd_vel 使用车体坐标系，单位 m/s / rad/s，与底盘 /cmd_vel 语义一致。
#    - desired_yaw 使用本地世界坐标系 yaw，单位 rad。

# 枚举中使用 UNDEFINE=0 标识未定义/无效。
uint8 UNDEFINE=0

# ---------- command source：命令来源枚举 ----------
uint8 SUNRAY_STATION=1     # Sunray 地面站/Qt 面板等上位机控制
uint8 RC_CONTROLLER=2      # RC 遥控器控制
uint8 TERMINAL=3           # 终端/TUI/命令行测试工具
uint8 CONTROL_CMD=4        # Sunray 内部控制模块直接下发
uint8 SWARM_CONTROL=5      # 集群控制模块
uint8 PLANNING=6           # 规划模块
uint8 EXAMPLE_DEMO=7       # 示例 Demo / 教程测试程序
uint8 cmd_source

# ---------- control command：控制命令枚举 ----------
uint8 HOLD=1                # 保持/停车：控制器持续发布零速度；不读取其他期望字段
uint8 RETURN=2              # 返航：移动到 sunray_ugv_control 内部记录的 return point
uint8 MOVE_POINT=3          # 点位控制：读取 desired_pos.x/y 和 desired_yaw；desired_pos.z 通常填 0
uint8 MOVE_VELOCITY=4       # 世界系速度控制：读取 desired_vel.x/y 和 desired_yaw；适合全向底盘，差速底盘默认不支持
uint8 MOVE_VELOCITY_BODY=5  # 车体系速度控制：读取 cmd_vel；差速底盘只使用 cmd_vel.linear.x + cmd_vel.angular.z
uint8 MOVE_WGS84=6          # 经纬高控制预留：读取 desired_wgs84_pos；当前 UGV 控制器仅保留接口，暂不执行
uint8 control_cmd

# MOVE_POINT 使用：
# - desired_pos.x/y：目标点在本地世界坐标系下的位置，单位 m
# - desired_pos.z：UGV 通常填 0，当前到点判定只使用 x/y
# - desired_yaw：目标偏航角，单位 rad
geometry_msgs/Point desired_pos

# MOVE_VELOCITY 使用：
# - desired_vel.x/y：本地世界坐标系速度，单位 m/s
# - desired_vel.z：UGV 不使用，建议填 0
# - desired_yaw：期望偏航角，单位 rad
geometry_msgs/Vector3 desired_vel

# MOVE_VELOCITY_BODY 使用：
# - cmd_vel.linear.x：车体前向速度，单位 m/s
# - cmd_vel.linear.y：车体侧向速度，单位 m/s；差速底盘会强制忽略
# - cmd_vel.linear.z：UGV 不使用，建议填 0
# - cmd_vel.angular.z：偏航角速度，单位 rad/s
# - cmd_vel.angular.x/y：UGV 不使用，建议填 0
geometry_msgs/Twist cmd_vel

# MOVE_POINT / MOVE_VELOCITY 使用：
# - 世界系目标 yaw，单位 rad
float64 desired_yaw

# MOVE_WGS84 使用：
# - latitude / longitude / altitude 分别为纬度、经度、高度
# - 当前 UGV 控制器仅保留接口，暂不执行
geographic_msgs/GeoPoint desired_wgs84_pos
