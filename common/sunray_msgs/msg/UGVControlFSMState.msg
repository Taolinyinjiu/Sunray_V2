std_msgs/Header header

# UGV控制状态机状态枚举
uint8 INIT=0    # 初始化状态
uint8 HOLD=1     # 停止状态
uint8 RETURN=2   # 返航状态
uint8 MOVE=3     # 移动状态

# 当前的状态机状态
uint8 fsm_state

## 上一条收到的指令
sunray_msgs/UGVControlCMD ugv_control_cmd

##返航点


## 下发到底层的控制指令
geometry_msgs/Twist cmd_vel

