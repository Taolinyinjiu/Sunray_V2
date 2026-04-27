std_msgs/Header header

# 集群指令来源；用于标记该指令来自遥控器、demo、地面站、规划模块或外部模块
uint8 UNDEFINE=0
uint8 RC=1
uint8 DEMO=2
uint8 GROUND_STATION=3
uint8 PLANNER=4
uint8 EXTERNAL_MODULE=5
uint8 cmd_source

# 集群控制命令；决定本条指令的主动作
# SWARM_TAKEOFF/SWARM_LAND/SWARM_HOVER/SWARM_RETURN 为单次触发指令，只使用 cmd_source/swarm_cmd
# SWARM_FORMATION 需要配合 formation 使用，可同时配合 formation_param、leader_pos、leader_yaw、custom_offsets
uint8 SWARM_TAKEOFF=1
uint8 SWARM_LAND=2
uint8 SWARM_HOVER=3
uint8 SWARM_RETURN=4
uint8 SWARM_FORMATION=5
uint8 swarm_cmd

# 阵型枚举；仅在 swarm_cmd=SWARM_FORMATION 时使用
# KEEP_FORMATION：保持当前物理阵型，可配合 leader_pos/leader_yaw 实现保持阵型移动
# RING/LINE/COLUMN/V_SHAPE/WEDGE：切换到内置阵型，可配合 formation_param 设置阵型间距
# CUSTOM：切换到自定义阵型，需要配合 custom_offsets 使用，也可配合 leader_pos/leader_yaw 指定虚拟 leader
# EXPAND/CONTRACT：在当前阵型基础上按比例扩大/缩小间距，不需要 custom_offsets
uint8 KEEP_FORMATION=0
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

# 阵型参数；仅在 swarm_cmd=SWARM_FORMATION 时使用
# 当 formation_param > 0 时，作为阵型间距/尺寸参数覆盖当前 spacing
# 对 RING/LINE/COLUMN/V_SHAPE/WEDGE/CUSTOM 生效；对 KEEP_FORMATION 可用于只调整当前阵型间距
float32 formation_param

# 虚拟 leader 目标位置；仅在 swarm_cmd=SWARM_FORMATION 时使用
# 配合 leader_yaw 指定阵型参考中心；可与 KEEP_FORMATION 一起实现保持当前阵型移动
# 当前代码中 leader_pos/leader_yaw 全为 0 时会被视为未显式指定 leader 目标
geometry_msgs/Point leader_pos

# 虚拟 leader 目标航向；仅在 swarm_cmd=SWARM_FORMATION 时配合 leader_pos 使用
float32 leader_yaw

# 自定义阵型偏移量；仅在 swarm_cmd=SWARM_FORMATION 且 formation=CUSTOM 时使用
# 坐标定义在虚拟 leader 体坐标系下：x 前方为正，y 左方为正，z 保留
# custom_offsets[i] 对应 agent_id=i+1 的目标偏移，并会继续乘以 formation_param/当前 spacing
geometry_msgs/Point[] custom_offsets
