std_msgs/Header header

# ----------- 阵型枚举 -------------
uint8 STATIC_KEEP_FORMATION=0           ## 静态阵型：抓拍当前全集群的相对位置关系，并将该相对关系整体平移到虚拟 leader 附近
uint8 STATIC_FORMATION_LINE=1           ## 静态阵型：所有智能体在惯性系下排成一条直线，直线方向由 static_line_angle 指定
uint8 STATIC_FORMATION_POLYGON=2        ## 静态阵型：所有智能体位于正 N 边形顶点，N=当前集群数量，虚拟 leader 位于多边形几何中心
uint8 STATIC_FORMATION_RANDOM=3         ## 静态阵型：在场地安全范围内随机生成目标点，并保证任意两个目标点间距在 [3*orca_radius,10*orca_radius]
uint8 STATIC_FORMATION_CUSTOM=9         ## 静态阵型：完全由外部自定义每个智能体相对虚拟 leader 的位置/yaw 偏移量，点间距需在 [3*orca_radius,10*orca_radius]

uint8 DYNAMIC_FORMATION_RING=11         ## 动态阵型：所有智能体均匀分布在圆周上，并绕圆心连续运动；虚拟 leader 位于圆心
uint8 DYNAMIC_FORMATION_POLYGON=12      ## 动态阵型：所有智能体沿正 N 边形边线运动，N=当前集群数量；虚拟 leader 位于多边形几何中心
uint8 DYNAMIC_FORMATION_LEMNISCATE=13   ## 动态阵型：所有智能体沿 8 字形 Lemniscate 轨迹运动；初始相位位于同一侧半瓣，虚拟 leader 位于轨迹中心

# 阵型类型：
# 1. 仅在 swarm_cmd=SWARM_FORMATION 时使用
# 2. static 类阵型默认最终会收敛到某个固定目标点
# 3. dynamic 类阵型会随时间持续更新目标点，直到达到 dynamic_time 后结束
uint8 formation_type

# 虚拟 leader 目标位置与航向：
# 1. leader_pos 始终表示阵型参考点在世界坐标系下的位置
# 2. 对于大多数静态阵型，leader_yaw 主要作为最终目标偏航角参考
# 3. 对于动态圆环阵型，leader_yaw 还作为圆周初始相位偏置使用
geometry_msgs/Point leader_pos
float32 leader_yaw

# 动态阵型持续时间，单位：秒
# 1. 仅在 dynamic 类阵型中使用
# 2. swarm_control 会在 [0, dynamic_time] 时间段内持续刷新动态目标点
# 3. 超过该时间后，状态机切换回 ARRIVED
float32 dynamic_time

# ----------- STATIC_FORMATION_LINE 参数-------------
float32 static_line_spacing               ## 直线阵型中相邻两个智能体之间的间隔距离，单位：米，限制范围 [3*orca_radius,10*orca_radius]
float32 static_line_angle                 ## 直线阵型与世界坐标系 X 轴的夹角，单位：deg；0 表示沿 X 轴，90 表示沿 Y 轴
# ----------- STATIC_FORMATION_POLYGON 参数-------------
float32 static_polygon_spacing            ## 正 N 边形相邻两个智能体之间的边长，单位：米，限制范围 [3*orca_radius,10*orca_radius]；N=当前集群数量
# ----------- STATIC_FORMATION_CUSTOM 参数-------------
# 自定义阵型偏移量：
# 1. 仅在 STATIC_FORMATION_CUSTOM 时使用
# 2. custom_offsets_pos[i] / custom_offsets_yaw[i] 对应 agent_id=i+1
# 3. 偏移量直接定义在世界坐标系下，不再额外乘以任何 spacing，也不会按 leader_yaw 旋转
# 4. 任意两个自定义目标点的三维距离必须位于 [3*orca_radius,10*orca_radius]
geometry_msgs/Point[] custom_offsets_pos
float32[] custom_offsets_yaw

# ----------- DYNAMIC_FORMATION_RING 参数-------------
float32 dynamic_ring_radius                   ## 圆半径，单位：米；相邻弦长需位于 [3*orca_radius,10*orca_radius]
float32 dynamic_ring_move_speed               ## 沿圆周的切向速度，单位：米/秒；大于 0 表示逆时针，小于 0 表示顺时针
# ----------- DYNAMIC_FORMATION_POLYGON 参数-------------
float32 dynamic_polygon_spacing               ## 动态正 N 边形的边长，单位：米，限制范围 [3*orca_radius,10*orca_radius]；N=当前集群数量
float32 dynamic_polygon_move_speed            ## 智能体沿正 N 边形周长循环运动时的线速度，单位：米/秒；大于 0 表示按顶点序号递增方向前进，小于 0 表示反方向
# ----------- DYNAMIC_FORMATION_LEMNISCATE 参数-------------
float32 dynamic_lemniscate_x_radius           ## 8 字轨迹 X 方向尺度，单位：米，限制范围 [3*orca_radius,10*orca_radius]；轨迹大致覆盖 [-x_radius, +x_radius]
float32 dynamic_lemniscate_y_radius           ## 8 字轨迹 Y 方向尺度，单位：米，限制范围 [3*orca_radius,10*orca_radius]；轨迹大致覆盖 [-0.5*y_radius, +0.5*y_radius]
float32 dynamic_lemniscate_move_speed         ## 8 字轨迹相位推进的等效线速度，单位：米/秒；大于 0 表示正向，小于 0 表示反向；初始相位由 formation 内部按单侧半瓣队列分配
