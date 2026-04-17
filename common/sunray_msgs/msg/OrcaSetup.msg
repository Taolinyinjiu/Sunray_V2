std_msgs/Header header

# ORCA 设置命令
uint8 GOAL=1
uint8 GOAL_RUN=2
uint8 RUN=3
uint8 STOP=4

# 设置命令
uint8 cmd

# 目标点（ENU）
float32[3] desired_pos
float32 desired_yaw
