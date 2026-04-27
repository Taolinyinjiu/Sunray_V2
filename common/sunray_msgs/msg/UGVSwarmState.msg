std_msgs/Header header

# 当前选择执行的集群控制指令
sunray_msgs/UGVSwarmCMD current_cmd

# 当前集群状态
uint8 INIT=0
uint8 TAKEOFF=1
uint8 LAND=2
uint8 HOVER=3
uint8 FORMATION=4
uint8 ORCA_RETURN_HOME=5
uint8 state

# 集群数量
uint32 swarm_num
