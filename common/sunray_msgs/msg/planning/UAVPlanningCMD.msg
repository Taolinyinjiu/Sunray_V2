# 记录命令发送时间
std_msgs/Header header

# ----------- 命令来源枚举 -------------
uint8 UNDEFINE=0
uint8 SUNRAY_STATION=1  	# SUNRAY地面站
uint8 RC_CONTROLLER=2    	# RC遥控器控制
uint8 TERMINAL=3  		    # 终端TUI/命令行控制
uint8 CONTROL_CMD=4    		# Sunray系列代码控制
# 控制命令来源
uint8 cmd_source

# ------------- 控制命令枚举 ------------
uint8 TAKEOFF=1 # 起飞
uint8 LAND=2    # 降落
uint8 RETURN=3  # 特殊返航命令
uint8 KILL=4    # 紧急锁桨
uint8 HOVER=5   # 当前位置悬停
uint8 PLANNING_LOCAL=6 # 局部规划到某一点
uint8 PLANNING_GLOBAL=7 # 全局规划到某一点

# 控制命令
uint8 control_cmd

# 单航点时 size=1，多航点时 size>1
PlanningWaypoint[] waypoints
