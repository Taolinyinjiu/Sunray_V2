# 话题头
std_msgs/Header header

# UGVControlState 是 sunray_ugv_control 对外发布的完整运行状态。
# 推荐外部 UI / monitor / RViz 只订阅本消息，不直接侵入 ugv_control_fsm 内部逻辑。
# 典型话题：
#   /ugv1/sunray/ugv_control/control_state

# ---------- robot identity ----------
string agent_name                 # 机器人名称前缀，例如 ugv
uint8 agent_id                    # 机器人编号，从 1 开始

# ---------- drive base ----------
uint8 DRIVE_UNKNOWN=0
uint8 DRIVE_MECANUM=1
uint8 DRIVE_DIFFERENTIAL=2
uint8 drive_type                  # 底盘类型枚举

# ---------- input health ----------
bool odom_valid                   # 是否已收到本机定位/里程计
bool control_cmd_valid            # 是否已收到有效 UGVControlCMD
bool inside_geo_fence             # true 表示当前在地理围栏内；odom_valid=false 时默认 true

# ---------- fsm state ----------
uint8 FSM_INIT=0                  # 未收到有效输入或尚未进入控制
uint8 FSM_HOLD=1                  # 保持/停车
uint8 FSM_RETURN=2                # 返航
uint8 FSM_MOVE=3                  # 执行 MOVE_POINT / MOVE_VELOCITY / MOVE_VELOCITY_BODY / MOVE_WGS84
uint8 fsm_state

# 最近一次收到的控制指令
sunray_msgs/UGVControlCMD active_ugv_control_cmd

# ---------- current odom ----------
# self_odom 为原始本机里程计，包含位置、姿态和速度。
nav_msgs/Odometry self_odom

# ---------- target ----------
# target_valid=false 表示当前 FSM 没有明确目标点，例如 INIT/HOLD 或速度控制。
bool target_valid
geometry_msgs/Point target_pos    # 单位 m
float64 target_yaw                # 单位 rad

# ---------- controller output ----------
# 实际发布到底盘速度话题的控制量，与 /<agent>/sunray/ugv_control/cmd_vel 一致。
geometry_msgs/Twist controller_cmd_vel

# ---------- geo fence ----------
geometry_msgs/Point geo_fence_min
geometry_msgs/Point geo_fence_max
