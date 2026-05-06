std_msgs/Header header

# swarm_control_ugv的相关参数
# 本机ID
uint8 agent_id
# 集群数量
uint32 swarm_num

# swarm_control_ugv的相关状态
# 输入准备状态
bool self_odom_ready
bool peers_odom_ready
uint32 ready_peer_num

# 本机局部里程计
# 1. self_odom_ready=false 时，本字段为默认空消息
# 2. self_odom 来自本机 /sunray/localization/local_odom，包含位置、姿态、速度等信息
nav_msgs/Odometry self_odom

# 当前选择执行的集群控制指令UGVSwarmCMD（来自外部）
sunray_msgs/UGVSwarmCMD swarm_cmd

# 当前集群状态
uint8 INIT=0
uint8 RETURN_HOME=1
uint8 ARRIVED=2
uint8 SWARM_STATIC_FORMATION=3    # 正在执行静态编队控制，目标点固定或最终会收敛到固定点
uint8 SWARM_DYNAMIC_FORMATION=4   # 正在执行动态编队控制，目标点会随时间持续变化
uint8 SWARM_DYNAMIC_FORMATION_PREPARE=5 # 动态编队就位阶段：先抵达动态阵型 t=0 初始目标点，全体就位并等待后再开始动态追踪
uint8 fsm_state

# 当前集群控制目标点（根据formation计算得到）
# 1. target_valid=false 表示当前没有明确目标点，例如 INIT
# 2. target_pos 为世界坐标系下的目标位置，单位：米
# 3. target_yaw 为世界坐标系下的目标偏航角，单位：rad
bool target_valid
geometry_msgs/Point target_pos
float32 target_yaw

# 当前发布的UGVControlCMD（发布到无人车控制器）
sunray_msgs/UGVControlCMD ugv_cmd
