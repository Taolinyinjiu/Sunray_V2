# 记录命令发送时间
std_msgs/Header header

# plan_cmd_source在sunray_planning模块中不参与到任何逻辑判断中,planning模块也没有优先级的概念
# 如果此时在执行多航点轨迹飞行任务,接受到了一个新的UAVPlanningCMD指令,则会抛弃掉原有的任务转而执行新的任务
# plan_cmd_source 仅作为展示用,对于terminal,surnay_station,uav_planning_example 他们都会显式的填写这个字段,从而在日志中或地面站显示时得到明确的反馈
string plan_cmd_source

# ------------- 控制命令枚举 ------------
uint8 TAKEOFF=1 # 起飞
uint8 LAND=2    # 降落
uint8 HOVER=3    # 如果当前在运动，则打断，切换为悬停状态
uint8 PLAN_RETURN=4 # 通过规划的方式，回到HOME点
uint8 PLAN_LOCAL_GOAL=5 # 局部规划到某一点
uint8 PLAN_GLOBAL_GOAL=6 # 全局规划到某一点

# 控制命令
uint8 plan_cmd

# 单航点时 size=1，多航点时 size>1
PlanningWaypoint[] waypoints
