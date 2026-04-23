# 记录命令发送时间
std_msgs/Header header

# 选择航点坐标系
# 请注意，一次命令中，不支持设置两个航点坐标系不一致
uint8 SUNRAY_LOCAL=1
uint8 SUNRAY_GLOBAL=2
uint8 planning_frame

# ----------- 命令来源枚举 -------------
uint8 UNDEFINE=0
uint8 SUNRAY_STATION=1  	# SUNRAY地面站
uint8 RC_CONTROLLER=2    	# RC遥控器控制
uint8 TERMINAL=3  		    # 终端TUI/命令行控制
uint8 CONTROL_CMD=4    		# Sunray系列代码控制
uint8 cmd_source

# 单航点时 size=1，多航点时 size>1
PlanningWaypoint[] waypoints
