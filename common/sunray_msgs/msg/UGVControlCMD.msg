std_msgs/Header header

# 枚举中，使用UNDEFINE = 0标识未定义
uint8 UNDEFINE=0 					# 未定义的来源

# ---------- command source ----------
# ----------- 命令来源枚举 -------------
uint8 SUNRAY_STATION=1  	# SUNRAY地面站
uint8 RC_CONTROLLER=2    	# RC遥控器控制
uint8 TERMINAL=3  		    # 终端TUI/命令行控制
uint8 CONTROL_CMD=4    		# Sunray系列代码控制
# 命令来源
uint8 cmd_source

# ---------- control command ----------
# ------------- 控制命令枚举 ------------
uint8 HOLD=1            # 停止（单次触发）
uint8 RETURN=2          # 返航（单次触发）
uint8 MOVE_POINT=3           # 移动到某一点（单次触发）
uint8 MOVE_VELOCITY=4        # 惯性系速度控制（频率至少为10hz）
uint8 MOVE_VELOCITY_BODY=5   # 机体系速度控制（频率至少为10hz）
uint8 MOVE_WGS84=6           # WGS84 经纬高定位（单次触发）- 暂未启用，此处保留接口
# 控制命令
uint8 control_cmd

# ---------- control data  ----------
# ----------- 控制数据 ----------------
# 惯性系
geometry_msgs/Vector3 desired_pos
geometry_msgs/Vector3 desired_vel
float32 desired_yaw

# 机体系
geometry_msgs/Vector3 desired_linear  # 机体系线速度
geometry_msgs/Vector3 desired_angular  # 机体系角速度

# WGS84 经纬高
float64 lat
float64 lon
float64 alt