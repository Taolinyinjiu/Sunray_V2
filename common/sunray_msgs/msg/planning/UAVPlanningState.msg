std_msgs/Header header
uint32 task_id

uint8 UNDEFINE=0
# 这里描述具体的planner类型
uint8 PLANNER_EGO=1
uint8 PLANNER_DIFF=2
uint8 PLANNER_FUEL=3
uint8 PLANNER_SUPER=4
uint8 planner_type
string planner_type_string

# 这里描述planner当前的状态
uint8 PLANNER_STATE_UNDEFINE=0
uint8 PLANNER_STATE_INIT=1
uint8 PLANNER_STATE_WAIT_TARGET=2
uint8 PLANNER_STATE_GENERATE=3
uint8 PLANNER_STATE_REPLAN=4
uint8 PLANNER_STATE_EXEC=5
uint8 PLANNER_STATE_PAUSE=6
uint8 PLANNER_STATE_SUCCESS=7
uint8 PLANNER_STATE_FAIL=8
uint8 PLANNER_STATE_EMERGENCY_STOP=9
uint8 planner_state
string planner_state_string

# 这里描述规划模块状态机当前的状态
uint8 INIT=0
uint8 READY=1
uint8 PLANNING=2
uint8 ARRIVED=3
# ARRIVED与HOVER在正确执行时等价，在打断时，比如轨迹执行的中段发送悬停命令，进入HOVER而非ARRIVED
uint8 HOVER=4
uint8 LAND=5
uint8 planning_fsm_state
string planning_fsm_state_string

# 这里表示当前目标点的坐标系
uint8 SUNRAY_LOCAL=1
uint8 SUNRAY_GLOBAL=2
uint8 planning_frame
string planning_frame_string

# 这里表示目标点类型的方式
uint8 GOAL_SINGLE=1
uint8 GOAL_MULTI=2
uint8 goal_type
string goal_type_string

# ----------- 命令来源枚举 -------------
uint8 SUNRAY_STATION=1  	# SUNRAY地面站
uint8 RC_CONTROLLER=2    	# RC遥控器控制
uint8 TERMINAL=3  		    # 终端TUI/命令行控制
uint8 CONTROL_CMD=4    		# Sunray系列代码控制
uint8 cmd_source
string cmd_source_string

# 总的航点以及当前执行的航点
uint32 waypoint_count
uint32 current_waypoint_index

# 当前执行的目标点
PlanningWaypoint current_target
# 任务队列中最后的目标点
PlanningWaypoint final_target
