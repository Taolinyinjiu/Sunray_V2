# 单点数据类型  
geometry_msgs/Point position
float32 yaw
# hold_time 保持时间: 作用为多航点任务时,到达该点后,在该点保持悬停多久后,再移动到下一个目标点
float32 hold_time   

# 备注：fsm下发指令到planner复用这个消息，但是忽略hold_time