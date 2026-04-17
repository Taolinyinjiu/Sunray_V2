std_msgs/Header header

# 集群指令来源
uint8 UNDEFINE=0
uint8 RC=1
uint8 DEMO=2
uint8 GROUND_STATION=3
uint8 PLANNER=4
uint8 EXTERNAL_MODULE=5
uint8 cmd_source

# 集群控制命令
uint8 HOLD=1
uint8 RETURN=2
uint8 MOVE=3
uint8 SWARM_FORMATION=4
uint8 swarm_cmd

# MOVE 指令参数
geometry_msgs/Point desired_pos
float32 desired_yaw

# 阵型枚举
uint8 FORMATION_NONE=0
uint8 RING=1
uint8 LINE=2
uint8 COLUMN=3
uint8 V_SHAPE=4
uint8 WEDGE=5
uint8 CUSTOM=6
uint8 EXPAND=7
uint8 CONTRACT=8
uint8 RESERVED_9=9
uint8 RESERVED_10=10
uint8 formation

# 阵型参数，通常表示队形尺寸或间距
float32 formation_param

# leader 目标位姿
geometry_msgs/Point leader_pos
float32 leader_yaw
