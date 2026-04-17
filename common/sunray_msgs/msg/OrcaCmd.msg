std_msgs/Header header

# ORCA 输出状态
uint8 INIT=0
uint8 RUN=1
uint8 ARRIVED=2
uint8 STOP=3

# 当前 ORCA 状态
uint8 state

# 当前绑定目标点
float32[3] goal_pos
float32 goal_yaw

# 输出控制量
float32[3] linear
float32[3] angular
