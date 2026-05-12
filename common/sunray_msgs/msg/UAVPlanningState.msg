std_msgs/Header header

# 这里描述当前sunray_planning在于哪个规划器进行交互
string planner_type_string

# 这里描述规划模块状态机当前的状态
uint8 OFF=0 # OFF阶段表示当前不满足Planning基本运行条件
uint8 INIT=1 # INIT阶段表示Planning当前正常启动,并可接收UAVPlanningCMD指令
uint8 TAKEOFF=2 # TAKEOFF状态同步UAVControl部分,用于表示当前无人机处于起飞阶段
uint8 LAND=3 # LAND状态同步UAVControl部分,用于表示当前无人机处于降落阶段
uint8 PLANNING=4 # PLANNING状态表示持续的移动
uint8 ARRIVED=5 # ARRIVED表示到达了目标点
uint8 PLAN_FAILED=6 # PLAN_FAILED表示目标点无法到达

# PLAN_FAILED 需要搭配一个错误码,表示无法到达的原因,这里简单地描述可能的情况
# 1. 目标点在规划可行性空间中无法达到,比如无人机与目标点之间存在无法绕开的障碍物
# 2. 规划过程被外部指令打断

uint8 sunray_planning_state

# PLANNING模块的home点
geometry_msgs/Vector3 home_point

UAVPlanningCMD planning_cmd