<!-- title: Formation/ORCA 与扩展 -->

<section id="swarm-formation-orca">

## Formation/ORCA 与扩展

集群模块的核心算法分成两层：

- `formation`：根据 `sunray_msgs/Formation` 生成每个 agent 的目标点。
- `ORCA`：根据当前位置、速度、目标点和障碍物，生成避碰后的 XY 速度。

这两个库都不直接发布或订阅 ROS 话题，ROS 状态机节点负责把消息和算法连接起来。

### Formation 消息

`sunray_msgs/Formation` 是阵型描述接口，由 `UAVSwarmCMD` 或 `UGVSwarmCMD` 携带。

关键字段：

| 字段 | 说明 |
| --- | --- |
| `formation_type` | 阵型类型。 |
| `leader_pos` | 虚拟 leader 世界系位置，也是阵型参考点。 |
| `leader_yaw` | 虚拟 leader yaw，部分动态阵型也作为初始相位。 |
| `dynamic_time` | 动态阵型持续时间。 |
| `static_line_spacing` | 静态直线阵型间距。 |
| `static_line_angle` | 静态直线阵型角度，单位 deg。 |
| `static_polygon_spacing` | 静态多边形边长。 |
| `custom_offsets_pos/yaw` | 自定义阵型中每个 agent 相对 leader 的偏移。 |
| `dynamic_ring_radius` | 动态圆环半径。 |
| `dynamic_ring_move_speed` | 动态圆环切向速度。 |
| `dynamic_polygon_spacing` | 动态多边形边长。 |
| `dynamic_polygon_move_speed` | 动态多边形沿边运动速度。 |
| `dynamic_lemniscate_x/y_radius` | 动态 8 字轨迹尺度。 |
| `dynamic_lemniscate_move_speed` | 动态 8 字轨迹运动速度。 |

### 自定义消息

集群模块依赖 `common/sunray_msgs` 中的自定义消息完成跨包通信。二次开发时应优先改消息定义，再同步控制节点、工具节点和可视化节点。

| 消息 | 作用 | 主要使用者 |
| --- | --- | --- |
| `sunray_msgs/Formation` | 描述静态/动态阵型参数，是 formation 算法的核心输入。 | `UAVSwarmCMD`、`UGVSwarmCMD`、`formation_tui`、Qt 面板。 |
| `sunray_msgs/UAVSwarmCMD` | UAV 集群命令，包含起飞、降落、悬停、返航、编队等命令。 | 终端工具、Qt 面板、上层任务、`swarm_control_uav_node`。 |
| `sunray_msgs/UGVSwarmCMD` | UGV 集群命令，包含保持、返航、编队等命令。 | 终端工具、Qt 面板、上层任务、`swarm_control_ugv_node`。 |
| `sunray_msgs/UAVSwarmState` | UAV 集群状态快照，包含 odom 就绪、目标点、FSM、下发给单机控制器的命令。 | monitor、RViz、Qt 面板、YunLink 桥接。 |
| `sunray_msgs/UGVSwarmState` | UGV 集群状态快照，包含 odom 就绪、目标点、FSM、下发给单机控制器的命令。 | monitor、RViz、Qt 面板、YunLink 桥接。 |

消息修改原则：

- 新增阵型参数时，优先加到 `Formation.msg`，不要只在某个工具节点里加私有参数。
- 新增需要上报给地面站或可视化的运行状态时，优先加到 `UAVSwarmState.msg` 或 `UGVSwarmState.msg`。
- 修改消息后必须重新编译 `common/sunray_msgs`，再编译 `swarm` 和依赖它的工具。

统一入口：

```cpp
bool GetFormationGoal(const sunray_msgs::Formation &formation_cmd,
                      int agent_id,
                      double formation_time,
                      double &target_x,
                      double &target_y,
                      double &target_z,
                      double &target_yaw);
```

约定：

- `agent_id` 从 `1` 开始，内部转换成 `agent_index=agent_id-1`。
- `leader_pos` 是世界系参考点。
- 当前 offset 直接定义在世界系下，不再按 `leader_yaw` 旋转。
- 静态阵型通常传入 `formation_time=0`。
- 动态阵型使用 `formation_time` 计算当前相位。
- 目标点必须满足场地边界和静态障碍物约束。

### 安全约束

Formation 初始化时会读取：

| 参数 | 作用 |
| --- | --- |
| `orca/radius` | 作为智能体等效避碰半径。 |
| `orca/max_speed` | 限制动态阵型速度上限。 |
| `field/x_min~x_max` | 目标点 X 边界。 |
| `field/y_min~y_max` | 目标点 Y 边界。 |
| `field/z_min~z_max` | 目标点 Z 边界。 |
| `static_obstacles/*` | 拒绝落入障碍物安全区的目标点。 |

间距约束：

```text
minSafeDistance = 3 * orca_radius
maxSafeDistance = 10 * orca_radius
```

阵型间距、边长、部分动态轨迹尺度需要落在这个安全范围内。目标点如果进入 `obstacle_radius + orca_radius` 范围，`GetFormationGoal()` 会返回 false。

### Formation 设计

`formation` 库只负责从“阵型描述 + agent_id + 时间”计算目标点，不负责避障速度、不发布 ROS 话题、不直接控制无人机或无人车。

核心设计可以理解为：

```text
sunray_msgs/Formation
  + agent_id
  + formation_time
  -> target_x / target_y / target_z / target_yaw
```

这样状态机可以在不同阶段复用同一个函数：

| 场景 | `formation_time` 用法 |
| --- | --- |
| 静态阵型 | 固定传入 `0`，目标点不随时间变化。 |
| 动态阵型 PREPARE | 传入 `0`，先让所有 agent 到动态轨迹起点。 |
| 动态阵型运行 | 传入动态阵型开始后的 `elapsed_time`，持续刷新目标点。 |

Formation 的设计重点是“目标点合法性”：

- 目标点必须在 `field/*` 边界内。
- 目标点不能落入静态障碍物安全范围。
- 多个 agent 的目标点间距要满足安全范围。
- 动态轨迹的完整运动范围要能放进场地。
- 速度类参数不能超过 ORCA 的最大速度约束。

如果 `GetFormationGoal()` 返回 `false`，状态机不会盲目发布目标点。此时应优先检查阵型参数、场地边界、静态障碍物和 `orca/radius`。

### 静态阵型

#### STATIC_KEEP_FORMATION

抓拍当前全集群相对位置关系，然后把这组 offset 整体平移到新的虚拟 leader 附近。

| 项 | 说明 |
| --- | --- |
| 枚举 | `STATIC_KEEP_FORMATION=0` |
| 参数 | `leader_pos`、`leader_yaw` |
| 特点 | 保持当前形状，不重新规划几何队形。 |
| 限制 | 只检查最终目标点是否在场地内、是否避开障碍物。 |

#### STATIC_FORMATION_LINE

所有智能体沿一条直线等间距分布，虚拟 leader 位于队列几何中心。

| 项 | 说明 |
| --- | --- |
| 枚举 | `STATIC_FORMATION_LINE=1` |
| 参数 | `static_line_spacing`、`static_line_angle`、`leader_pos`、`leader_yaw` |
| 角度 | `static_line_angle` 为直线方向与世界 X 轴夹角，单位 deg。 |
| 限制 | `1 <= swarm_num <= 10`，间距必须在安全范围内。 |

#### STATIC_FORMATION_POLYGON

`swarm_num=N` 时，所有智能体位于正 N 边形顶点，虚拟 leader 位于几何中心。

| 项 | 说明 |
| --- | --- |
| 枚举 | `STATIC_FORMATION_POLYGON=2` |
| 参数 | `static_polygon_spacing`、`leader_pos`、`leader_yaw` |
| 限制 | `3 <= swarm_num <= 10`，边长必须在安全范围内。 |

#### STATIC_FORMATION_RANDOM

在场地安全范围内随机生成每个 agent 的最终目标点，再转换成相对虚拟 leader 的 offset。

| 项 | 说明 |
| --- | --- |
| 枚举 | `STATIC_FORMATION_RANDOM=3` |
| 参数 | `leader_pos`、`leader_yaw`、消息时间戳 |
| 限制 | `1 <= swarm_num <= 10`，任意两目标点距离必须在安全范围内。 |
| 场地 | 采样范围会向内收缩 `2 * orca_radius`，避免贴边。 |

#### STATIC_FORMATION_CUSTOM

外部直接指定每个智能体相对虚拟 leader 的位置和 yaw 偏移。

| 项 | 说明 |
| --- | --- |
| 枚举 | `STATIC_FORMATION_CUSTOM=9` |
| 参数 | `custom_offsets_pos`、`custom_offsets_yaw`、`leader_pos`、`leader_yaw` |
| 数组规则 | `custom_offsets_pos[i]` 对应 `agent_id=i+1`。 |
| 限制 | 两个数组长度必须等于 `swarm_num`，目标点距离和场地/障碍物检查都必须通过。 |

### 动态阵型

#### DYNAMIC_FORMATION_RING

虚拟 leader 为圆心，智能体均匀分布在圆周上，并按切向速度绕圆运动。

| 项 | 说明 |
| --- | --- |
| 枚举 | `DYNAMIC_FORMATION_RING=11` |
| 参数 | `dynamic_ring_radius`、`dynamic_ring_move_speed`、`dynamic_time` |
| 限制 | `swarm_num >= 2`，相邻弦长必须在安全范围内。 |
| 速度 | `0.05 <= abs(dynamic_ring_move_speed) <= orca_max_speed`。 |

#### DYNAMIC_FORMATION_POLYGON

所有智能体沿正 N 边形周长运动，不同智能体在周长参数上错开一个边长。

| 项 | 说明 |
| --- | --- |
| 枚举 | `DYNAMIC_FORMATION_POLYGON=12` |
| 参数 | `dynamic_polygon_spacing`、`dynamic_polygon_move_speed`、`dynamic_time` |
| 限制 | `3 <= swarm_num <= 10`，边长必须在安全范围内。 |
| 场地 | 正 N 边形外接圆必须能放进场地安全范围。 |

#### DYNAMIC_FORMATION_LEMNISCATE

所有智能体沿同一条 8 字轨迹运动。初始相位分布在同一侧半瓣内部，避免偶数机在中心自交点重叠。

| 项 | 说明 |
| --- | --- |
| 枚举 | `DYNAMIC_FORMATION_LEMNISCATE=13` |
| 参数 | `dynamic_lemniscate_x_radius`、`dynamic_lemniscate_y_radius`、`dynamic_lemniscate_move_speed`、`dynamic_time` |
| 限制 | `1 <= swarm_num <= 10`，X/Y 尺度必须在安全范围内。 |
| 场地 | 8 字轨迹必须能放进场地 XY 安全范围。 |

### ORCA 设计

ORCA 用于在“想去目标点”和“避免与邻居/障碍物碰撞”之间折中，输出当前控制周期的平面速度。它不改变 formation 给出的最终目标点，只改变每个周期的走法。

当前 Sunray 集群中的 ORCA 设计边界：

| 项 | 说明 |
| --- | --- |
| 输入 | 每个 agent 的 XY 位置、XY 速度、目标点、静态障碍物。 |
| 输出 | 当前 agent 的 XY 速度命令。 |
| 不处理 | UAV 高度控制、yaw 控制、PX4 模式、底盘运动学约束。 |
| 调用者 | `swarm_control_uav_node` 和 `swarm_control_ugv_node`。 |

UAV 的高度和 yaw 仍由集群状态机结合 formation 目标填入 `UAVControlCMD`。UGV 的底盘能力差异由 `sunray_ugv_control` 处理，ORCA 本身只给出世界系平面速度建议。

### ORCA 封装

`swarm/ORCA/orca_lib` 是原生 RVO2/ORCA 源码，业务开发时不建议直接修改。

`swarm/ORCA/ORCA.h` 和 `swarm/ORCA/ORCA.cpp` 是 Sunray 上层封装入口，不依赖 ROS，公开接口只使用基础数值类型。

典型调用：

```cpp
orca_.init(agent_num,
           neighbor_dist,
           time_horizon,
           radius,
           max_speed,
           time_step,
           max_neighbors);

for (int i = 0; i < agent_num; ++i) {
  orca_.setAgentState(i, x, y, vx, vy);
}

orca_.setAgentGoal(self_idx, goal_x, goal_y);
orca_.GetOrcaVelCmd(self_idx, cmd_vx, cmd_vy);
```

当前 ORCA 只输出 XY 平面速度：

- UAV 的 Z 方向由 `swarm_control_uav_node` 根据高度误差生成，并通过 `fixed_height` 或控制字段交给 UAV 控制器。
- UAV/UGV 的 yaw 指令来自 formation 生成的 `target_yaw`。
- 圆形静态障碍物通过 `addCircleObstacle()` 加入，内部近似为多边形障碍物。

### SwarmCMD 和 SwarmState

`UAVSwarmCMD` 常用命令：

| 枚举 | 含义 |
| --- | --- |
| `SWARM_TAKEOFF=1` | UAV 集群起飞。 |
| `SWARM_LAND=2` | UAV 集群降落。 |
| `SWARM_HOVER=3` | UAV 集群悬停。 |
| `SWARM_RETURN=4` | UAV 集群返航。 |
| `SWARM_FORMATION=5` | 执行 `formation_cmd` 指定阵型。 |

`UGVSwarmCMD` 常用命令：

| 枚举 | 含义 |
| --- | --- |
| `SWARM_HOLD=1` | UGV 集群保持/停车。 |
| `SWARM_RETURN=2` | UGV 集群返航。 |
| `SWARM_FORMATION=3` | 执行 `formation_cmd` 指定阵型。 |

`UAVSwarmState` 和 `UGVSwarmState` 关键字段：

| 字段 | 说明 |
| --- | --- |
| `agent_id` / `swarm_num` | 本机 ID 和集群数量。 |
| `self_odom_ready` | 本机 odom 是否可用。 |
| `peers_odom_ready` | 邻居 odom 是否全部可用且未超时。 |
| `ready_peer_num` | 当前有效邻居数量。 |
| `self_odom` | 本机 odom，RViz mesh、速度、轨迹来自这里。 |
| `swarm_cmd` | 当前执行的集群命令。 |
| `fsm_state` | 当前集群状态机状态。 |
| `target_valid` | 当前是否有有效目标点。 |
| `target_pos` / `target_yaw` | 当前追踪目标点和 yaw。 |
| `uav_cmd` / `ugv_cmd` | 当前发布给单机控制器的命令。 |

### UAVSwarmCMD 详细说明

`UAVSwarmCMD` 通常由 terminal、TUI、Qt 面板、YunLink 桥或其他上层任务模块发布。

字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 消息时间戳。 |
| `cmd_source` | `uint8` | 命令来源，常见为 `GROUND_STATION=1`、`TERMINAL=2`。 |
| `agent_id` | `uint8` | 目标智能体 ID，`99` 表示广播。 |
| `swarm_cmd` | `uint8` | 集群命令。 |
| `formation_cmd` | `sunray_msgs/Formation` | 阵型参数，仅 `SWARM_FORMATION` 时使用。 |

UAV 命令到单机命令的大致映射：

| 集群命令 | 单机输出 |
| --- | --- |
| `SWARM_TAKEOFF` | `UAVControlCMD::TAKEOFF` |
| `SWARM_LAND` | `UAVControlCMD::LAND` |
| `SWARM_HOVER` | `UAVControlCMD::MOVE_POINT` 到当前悬停点 |
| `SWARM_RETURN` | ORCA 速度控制返回记录点 |
| `SWARM_FORMATION` | formation 目标点 + ORCA 避碰速度 |

### UGVSwarmCMD 详细说明

`UGVSwarmCMD` 面向无人车集群。

字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 消息时间戳。 |
| `cmd_source` | `uint8` | 命令来源，常见为 `GROUND_STATION=1`、`TERMINAL=2`。 |
| `agent_id` | `uint8` | 目标车辆 ID，`99` 表示广播。 |
| `swarm_cmd` | `uint8` | UGV 集群命令。 |
| `formation_cmd` | `sunray_msgs/Formation` | 阵型参数，仅 `SWARM_FORMATION` 时使用。 |

UGV 命令到单机命令的大致映射：

| 集群命令 | 单机输出 |
| --- | --- |
| `SWARM_HOLD` | `UGVControlCMD::HOLD` |
| `SWARM_RETURN` | ORCA 速度控制返回记录点 |
| `SWARM_FORMATION` | formation 目标点 + ORCA 避碰速度 |

### UAVSwarmState 状态说明

`UAVSwarmState` 是 monitor、RViz 和 Qt 面板的主要数据源，不建议显示工具直接读取 swarm control 内部变量。

状态枚举：

| 枚举 | 含义 |
| --- | --- |
| `INIT=0` | UAV 集群控制初始化状态。 |
| `TAKEOFF=1` | 已发送起飞指令，等待底层进入 hover。 |
| `LAND=2` | 已发送降落指令，等待底层回到 init。 |
| `RETURN_HOME=3` | 正在返航。 |
| `ARRIVED=4` | 已到达目标点，持续发布悬停点。 |
| `SWARM_STATIC_FORMATION=5` | 正在执行静态阵型。 |
| `SWARM_DYNAMIC_FORMATION=6` | 正在追踪动态阵型。 |
| `SWARM_DYNAMIC_FORMATION_PREPARE=7` | 动态阵型准备阶段。 |

字段使用建议：

| 字段 | 使用建议 |
| --- | --- |
| `self_odom_ready` | 判断本机定位是否就绪。 |
| `peers_odom_ready` | 判断集群邻居输入是否完整。 |
| `ready_peer_num` | 排查少了哪几台时配合 monitor 使用。 |
| `target_valid` | RViz/面板是否显示目标点。 |
| `uav_cmd` | 调试当前实际下发给 UAV 控制器的命令。 |

### UGVSwarmState 状态说明

UGV 状态枚举：

| 枚举 | 含义 |
| --- | --- |
| `INIT=0` | UGV 集群控制初始化状态。 |
| `RETURN_HOME=1` | 正在返航。 |
| `ARRIVED=2` | 已到达目标点，持续保持。 |
| `SWARM_STATIC_FORMATION=3` | 正在执行静态阵型。 |
| `SWARM_DYNAMIC_FORMATION=4` | 正在追踪动态阵型。 |
| `SWARM_DYNAMIC_FORMATION_PREPARE=5` | 动态阵型准备阶段。 |

UGV 的 `ugv_cmd` 字段记录当前下发给 `sunray_ugv_control` 的单机命令。排查“集群状态正常但车不动”时，应同时看 `UGVSwarmState.ugv_cmd` 和 `UGVControlState.controller_cmd_vel`。

### 新增阵型

推荐修改范围：

| 文件 | 修改内容 |
| --- | --- |
| `common/sunray_msgs/msg/Formation.msg` | 增加阵型枚举和必要参数。 |
| `swarm/formation/formation.h` | 声明新的 offset 计算函数。 |
| `swarm/formation/formation.cpp` | 实现目标点计算，并接入 `GetFormationGoal()`。 |
| `swarm/utils/uav_swarm_cmd_pub_terminal.cpp` | 增加 UAV 终端参数输入和显示。 |
| `swarm/utils/ugv_swarm_cmd_pub_terminal.cpp` | 增加 UGV 终端参数输入和显示。 |
| `swarm/utils/swarm_control_panel_node.cpp` | 增加 Qt 面板参数编辑。 |
| `swarm/utils/*monitor*.cpp` | 增加阵型名称显示。 |
| `swarm/utils/rviz_visualization_*_node.cpp` | 增加阵型名称显示。 |

新增阵型时要同步考虑这些问题：

- 新阵型是否是静态还是动态。
- 是否需要新参数字段，还是复用已有 `leader_pos`、spacing、radius、move_speed。
- `swarm_num` 的合法范围。
- 目标点是否会越界。
- 目标点之间的距离是否满足 `[3 * orca_radius, 10 * orca_radius]`。
- 动态轨迹是否能完整放进场地边界。
- 终端工具和 Qt 面板是否能编辑所有必要参数。
- RViz 和 monitor 是否能显示阵型名称，避免调试时只看到数字枚举。

### 修改 ORCA 策略

推荐修改：

| 文件 | 修改内容 |
| --- | --- |
| `swarm/ORCA/ORCA.h` | 调整或新增上层 API。 |
| `swarm/ORCA/ORCA.cpp` | 实现 preferred velocity、死锁处理、障碍物处理策略。 |
| `swarm/ORCA/test/orca_test.cpp` | 增加压力测试用例。 |

不要优先修改 `swarm/ORCA/orca_lib`。该目录是原生 RVO2/ORCA 内核，除非明确要改底层算法，否则应保持稳定。

ORCA 参数调试建议：

| 现象 | 优先检查 |
| --- | --- |
| 多机靠得太近 | 增大 `orca/radius` 或阵型 spacing。 |
| 避障动作太早、路径绕得远 | 减小 `orca/time_horizon` 或 `orca/neighbor_dist`。 |
| 避障太晚 | 增大 `orca/time_horizon`。 |
| 队形移动太慢 | 增大 `orca/max_speed`，同时确认单机控制器速度限制。 |
| 密集队形卡住 | 检查 spacing 是否低于安全距离，或增加 ORCA 死锁处理策略。 |
| 目标点在障碍物内 | 检查 `static_obstacles` 和 formation 目标点合法性。 |

静态障碍物同时影响两层：

```text
formation:
  目标点合法性检查，避免生成障碍物内部目标

ORCA:
  速度层避障，把圆形障碍物近似成多边形障碍物
```

因此如果你修改障碍物参数，应同时观察目标点是否有效、ORCA 输出是否正常、RViz 是否显示一致。

### 新增平台

如果后续增加新平台，例如船、机械臂底盘或其他地面机器人，可以参考当前 UAV/UGV 分层方式：

| 层级 | 建议 |
| --- | --- |
| 纯算法 | 继续复用 `formation` 和 `ORCA`。 |
| 消息 | 新增平台自己的 `SwarmCMD` 和 `SwarmState`，或抽象公共消息。 |
| 状态机 | 参考 `swarm_control_uav` 或 `swarm_control_ugv` 新建目录。 |
| 控制输出 | 转换成该平台已有的单机控制接口。 |
| 可视化 | 新增 `rviz_visualization_xxx_node`。 |
| 控制面板 | 在 Qt panel 中增加平台 tab 或平台选择项。 |

新增平台时不要让 swarm 直接控制硬件。仍然应该保持：

```text
swarm 控制
  -> 平台单机控制接口
  -> 平台驱动
```

### 新增状态量

如果某个过程量需要被 monitor、RViz 或 Qt 面板显示，应优先加入 `UAVSwarmState.msg` 或 `UGVSwarmState.msg`。

推荐流程：

```text
修改 SwarmState.msg
  -> 重新编译 common/sunray_msgs
  -> 在 swarm_control_uav/ugv 中填充字段
  -> 在 monitor/RViz/panel 中读取显示
```

不要让显示节点直接读取控制节点内部变量，否则后续远程监控、YunLink 桥接和日志记录都无法复用这些状态。

### 调试建议

指令没有响应：

```text
检查 swarm monitor
确认 self_odom_ready 是否正常
确认 peers_odom_ready 是否正常
确认 agent_id 是否匹配，或是否使用 99 广播
确认当前 FSM 是否允许接收该指令
```

阵型目标点无效：

```text
检查 Formation 参数是否合理
检查 swarm_num 是否与实际启动数量一致
检查 field 边界是否过小
检查静态障碍物是否挡住目标点
检查 spacing、radius、move_speed 是否超出 formation 内部约束
```

动态阵型一直不开始：

```text
观察 FSM 是否停在 SWARM_DYNAMIC_FORMATION_PREPARE
检查所有 agent 是否抵达各自 t=0 初始点
检查 goal_xy_tolerance / goal_z_tolerance / goal_yaw_tolerance 是否过严
检查某个邻居 odom 是否超时
检查 dynamic_prepare_wait_time 是否配置过大
```

RViz 看不到模型：

```text
确认 rviz_visualization 节点已启动
确认 MarkerArray 话题已加入 RViz
确认 Fixed Frame 为 world
确认 self_odom_ready=true
确认 mesh_resource 路径有效
```

ORCA 输出不明显或车辆不动：

```text
确认 goal_point 是否有效
确认 odom_caches_ 中所有 agent 的 odom 都已收到
确认 orca/max_speed 是否过小
确认 orca/radius 是否过大导致可行空间太小
确认静态障碍物是否把目标点或路径完全堵住
确认单机控制器是否接受当前类型的速度命令
```

集群状态正常但单机不执行：

```text
检查 swarm state 中的 uav_cmd / ugv_cmd
检查单机 control_state 是否正常
检查单机控制器是否已经起飞或处于可执行状态
检查 UAV 是否处于 HOVER/MOVE，而不是 INIT/LAND
检查 UGV 是否 odom_valid=true
检查控制命令话题名是否和单机控制器订阅话题一致
```

### 修改范围速查

| 想做的事 | 改动范围 |
| --- | --- |
| 改默认队形参数 | `launch/*cmd_pub_terminal.launch` 或 Qt 面板默认值。 |
| 改仿真 ORCA 参数 | `swarm_control_uav_sim.yaml` / `swarm_control_ugv_sim.yaml`。 |
| 改 RViz 显示效果 | `rviz_visualization_uav_node.cpp` / `rviz_visualization_ugv_node.cpp`。 |
| 改 monitor 文本 | `swarm_control_uav_monitor_node.cpp` / `swarm_control_ugv_monitor_node.cpp`。 |
| 改集群状态机 | `swarm_control_uav.cpp` / `swarm_control_ugv.cpp`。 |
| 改阵型算法 | `formation/formation.cpp`。 |
| 改避碰封装 | `ORCA/ORCA.cpp`。 |
| 改底层 RVO2 | `ORCA/orca_lib/*`，通常不建议。 |

### 典型二次开发任务

#### 任务 1：新增一个 V 字形静态阵型

建议步骤：

1. 在 `Formation.msg` 中新增枚举，例如 `STATIC_FORMATION_V=4`。
2. 如果 V 字形只需要 spacing 和 leader，可以复用 `static_line_spacing`；如果需要夹角，新增 `static_v_angle` 更清晰。
3. 在 `formation.h` 声明 `computeStaticVOffset()`。
4. 在 `formation.cpp` 实现每个 `agent_index` 的 offset。
5. 在 `GetFormationGoal()` 的 switch 中接入新枚举。
6. 添加间距检查，确保左右两臂相邻 agent 距离满足安全范围。
7. 检查所有目标点是否在 field 内、是否避开障碍物。
8. 在终端工具和 Qt 面板中增加 V 字形选项。
9. 在 monitor 和 RViz 中增加阵型名称显示。
10. 用 `orca_test` 增加 V 字形切换测试。

V 字形计算时要注意 `agent_id` 从 1 开始。不要让 `agent_id=1`、`agent_id=2` 在 leader 附近重叠，也不要让偶数/奇数数量时中心 agent 的分配产生跳变。

#### 任务 2：新增一个沿直线往返的动态阵型

建议步骤：

1. 在 `Formation.msg` 中新增动态阵型枚举和必要参数，例如长度、速度。
2. 在 `formation` 中实现 `formation_time -> offset` 的函数。
3. 限制 `move_speed` 不超过 `orca_max_speed`。
4. 检查整条往返路径能否放进 field。
5. PREPARE 阶段仍然使用 `formation_time=0` 的初始点。
6. 正式运行阶段根据 `elapsed_time` 周期性更新目标点。
7. 在 `dynamic_time` 到达后让状态机回到 `ARRIVED`。

动态阵型要重点处理“轨迹终点”和“周期边界”的连续性。如果目标点突然跳变，ORCA 会计算很大的期望速度，然后再被限速，表现为队形抖动或追踪滞后。

#### 任务 3：把集群状态上报到外部地面站

推荐做法：

```text
UAVSwarmState / UGVSwarmState
  -> yunlink_ros_bridge
  -> 外部协议
  -> 地面站
```

不要让地面站直接订阅每个 swarm 节点内部变量。需要新增字段时，应先加到 `SwarmState.msg`，再由桥接模块读取。

需要同步的地方：

| 文件/模块 | 改动 |
| --- | --- |
| `common/sunray_msgs/msg/*SwarmState.msg` | 新增状态字段。 |
| `swarm_control_uav/ugv` | 填充字段。 |
| `utils/*monitor*` | 终端显示。 |
| `utils/rviz_visualization_*` | 可视化显示。 |
| `utils/swarm_control_panel_node.cpp` | Qt 面板显示。 |
| `communication/yunlink_ros_bridge` | 外部协议映射。 |

#### 任务 4：调整避障更保守

优先改 YAML，不要先改 C++：

```yaml
orca:
  neighbor_dist: 2.0
  time_horizon: 3.0
  radius: 0.4
  max_speed: 0.4
```

含义：

- `neighbor_dist` 增大：更远的邻居会参与避障。
- `time_horizon` 增大：更早规避潜在碰撞。
- `radius` 增大：等效安全距离变大。
- `max_speed` 减小：动作更慢，但可能更稳。

副作用：

- 队形更容易绕远。
- 密集阵型可能更难到达。
- `minSafeDistance=3*radius` 变大，原本合法的 spacing 可能变成非法。
- 动态阵型速度上限变低，轨迹追踪可能滞后。

#### 任务 5：接入自定义上层任务

如果你有自己的任务调度节点，不需要直接操作每个 agent 的单机控制命令。推荐发布：

```text
/sunray/swarm/uav_swarm_cmd
/sunray/swarm/ugv_swarm_cmd
```

最小逻辑：

```cpp
sunray_msgs::UAVSwarmCMD cmd;
cmd.header.stamp = ros::Time::now();
cmd.cmd_source = sunray_msgs::UAVSwarmCMD::TERMINAL;
cmd.agent_id = 99;
cmd.swarm_cmd = sunray_msgs::UAVSwarmCMD::SWARM_FORMATION;
cmd.formation_cmd.formation_type = sunray_msgs::Formation::STATIC_FORMATION_LINE;
cmd.formation_cmd.leader_pos.x = 0.0;
cmd.formation_cmd.leader_pos.y = 0.0;
cmd.formation_cmd.leader_pos.z = 1.5;
cmd.formation_cmd.static_line_spacing = 1.5;
cmd.formation_cmd.static_line_angle = 0.0;
pub.publish(cmd);
```

注意事项：

- `agent_id=99` 是广播。
- 如果只想控制某一台，填具体 ID。
- `SWARM_FORMATION` 必须填写 `formation_cmd`。
- 动态阵型必须填写 `dynamic_time` 和对应速度/半径参数。
- 发布命令后应订阅 `SwarmState` 判断是否进入目标状态，而不是假设命令一定成功。

### 代码阅读顺序

如果要深入改集群模块，建议按这个顺序读：

1. `common/sunray_msgs/msg/Formation.msg`
2. `common/sunray_msgs/msg/UAVSwarmCMD.msg`
3. `common/sunray_msgs/msg/UAVSwarmState.msg`
4. `swarm/formation/formation.h`
5. `swarm/formation/formation.cpp`
6. `swarm/ORCA/ORCA.h`
7. `swarm/ORCA/ORCA.cpp`
8. `swarm/swarm_control_uav/swarm_control_uav.h`
9. `swarm/swarm_control_uav/swarm_control_uav.cpp`
10. `swarm/swarm_control_ugv/swarm_control_ugv.cpp`
11. `swarm/utils/*cmd_pub_terminal.cpp`
12. `swarm/utils/swarm_control_panel_node.cpp`
13. `swarm/utils/rviz_visualization_*_node.cpp`

阅读时要把“目标点生成”和“避碰速度生成”分开看。formation 只回答“每个 agent 应该去哪里”，ORCA 回答“考虑避碰后当前周期应该怎么走”。

### 设计取舍

当前设计选择了分布式 swarm control：

```text
每个 agent 一个 swarm_control_xxx_node
每个节点都订阅全体 odom
每个节点只发布自己的单机 control_cmd
```

优点：

- 真机部署时每台机器可以独立运行自己的集群节点。
- 某个节点异常时，不一定影响其他节点继续发布状态。
- UAV/UGV 可以复用相同 formation 和 ORCA 算法。

代价：

- 每个节点都需要看到全体 odom。
- `swarm_num`、`agent_id` 必须配置一致。
- 动态阵型 PREPARE 依赖所有节点对“全体到达”的判断一致。
- 所有机器的定位坐标系必须语义一致。

如果未来改成中心化控制，可以减少 odom 分发复杂度，但中心节点会成为单点，真机网络中断时风险更高。当前结构更适合多机实验和逐台部署。

</section>
