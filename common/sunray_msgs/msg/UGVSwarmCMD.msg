std_msgs/Header header

# ---------- command source ----------
# ----------- 命令来源枚举 -------------
uint8 UNDEFINE=0
uint8 GROUND_STATION=1
uint8 TERMINAL=2          # 终端TUI/命令行控制
# 命令来源
uint8 cmd_source

# 智能体ID：
# 1. 只有指定ID的智能体才会响应该命令
# 2. agent_id=99 表示所有智能体均响应
uint8 agent_id

# ---------- swarm control command ----------
# ------------- 集群控制命令枚举 ------------
# SWARM_HOLD/SWARM_RETURN 为单次触发指令
# SWARM_FORMATION 需要配合 formation_cmd 使用
uint8 SWARM_HOLD=1
uint8 SWARM_RETURN=2
uint8 SWARM_FORMATION=3
# 集群控制命令
uint8 swarm_cmd

# 阵型控制参数
sunray_msgs/Formation formation_cmd
