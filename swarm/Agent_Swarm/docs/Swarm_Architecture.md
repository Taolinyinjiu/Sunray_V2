Swarm 架构（基于 Leader 的编队策略系统）
====================================

范围
----
本文描述基于"Leader 引导 + 编队策略（ring/line/column/v_shape/v/wedge/custom）"的 Swarm 架构。
目标是：仅给 leader 下达动作/目标，其它无人机自动保持编队移动，ORCA 负责避障。

核心原则
--------
- 所有 ID 从 **1** 开始（1-based），Leader 由 `leader_id` 参数指定（`agent_swarm.launch` 默认 `leader_id=1`，`swarm_sim.launch` 默认 `leader_id=1`）。
- 编队通过"二维偏移量表"定义，`OffsetBasedPolicy` 将偏移转换为目标点。
- ORCA 只负责避障与速度输出，编队逻辑与避障解耦。
- 阵型变换/移动到位后自动进入 HOVER。
- Leader 本身也参与 ORCA 避障计算，其目标点来自 `/sunray/leader_goal` 或 leader 当前位姿（通过 `leader_publish_goal` 参数控制）。
- 支持 UAV（无人机）和 UGV（无人车）两种 agent 类型，通过 `agent_type` 参数区分（`0`=UAV，`1`=UGV）。
- `leader_id > 100` 表示 Leader 是 UGV，实际 Leader ID 取 `leader_id - 100`。此时 LeaderTracker 订阅 UGV 状态话题。

模块协作总览（图）
------------------

```
┌─────────────────────────────────────────────────────────────────────┐
│                          外部输入（操控）                            │
│  formation_switch/tui -> /sunray/formation_cmd  (ring/line/column/v/...) │
│                     /sunray/leader_goal     (leader 目标点)         │
└─────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌────────────────────────────────────────────────────────────────────────────┐
│                        Agent_Swarm 控制层                                   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │              AgentSwarmNode (节点入口与调度)                           │   │
│  │  - 三个独立定时器:                                                     │   │
│  │      goalTimerCb       @ goal_rate Hz (编队目标计算)                   │   │
│  │      controlTimerCb    @ control_rate Hz (控制指令输出)                │   │
│  │      statePublishTimerCb @ state_pub_rate Hz (状态发布)                │   │
│  │  - 订阅: formation_cmd(/ground), formation_offsets,                    │   │
│  │         leader_goal, uav_state（orca_cmd 由 OrcaClient 订阅）           │   │
│  │  - 发布: orca/agent_state, orca/setup, uav_control_cmd，ugv_control_cmd │   │
│  └──────────────────────────────────────────────────────────────┘   │
│        │            │             │             │                    │
│        ▼            ▼             ▼             ▼                    │
│ ┌────────────┐ ┌──────────┐ ┌──────────┐ ┌──────────────────────┐    │
│ │ Leader     │ │Formation │ │Formation │ │ AgentStateCache      │    │
│ │ Tracker    │ │StateMach.│ │ Context  │ │ (全机状态缓存+发布)   │     │
│ └────────────┘ └──────────┘ │(数据结构)│ └──────────────────────┘     │
│        │            │       └──────────┘          │                  │
│        │            │            │                 ▼                 │
│        │            │            ▼        /uavX/orca/agent_state     │
│        │            │  ┌──────────────┐                              │
│        │            │  │FormationPoli-│                              │
│        │            │  │cyFactory     │                              │
│        │            │  └──────┬───────┘                              │
│        │            │         ▼                                      │
│        │            │  ┌──────────────┐                              │
│        │            │  │FormationPolicy│                             │
│        │            │  │(Ring/Line/   │                              │
│        │            │  │ Column/V/W) │                               │
│        │            │  └──────┬───────┘                              │
│        │            │         ▼                                      │
│        │            │  ┌──────────────┐   ┌──────────────────────┐   │
│        │            │  │GoalDispatcher│──▶│ /uavX/orca/setup     │  │
│        │            │  └──────────────┘   └──────────────────────┘   │
│        │            │                              │                 │
│        │            │                              ▼                 │
│        │            │                     ┌──────────────────────┐   │
│        │            │                     │ OrcaClient           │   │
│        │            │                     │ (订阅 /uavX/orca_cmd)│   │
│        │            │                     └──────────┬───────────┘   │
│        │            │                                │               │
│        │            │                                ▼               │
│        │            │                     ┌──────────────────────┐   │
│        │            │                     │ ControlCommandMapper │   │
│        │            │                     │ /uavX/uav_control_cmd│   │
│        │            │                     └──────────────────────┘   │
│        │            │                                                │
└────────┼────────────┼────────────────────────────────────────────────┘
         │            │
         ▼            ▼
┌─────────────────────────────────────────────────────────────────────┐
│                           ORCA 避障层                               │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                OrcaNode (节点入口)                             │   │
│  │  - ros::Rate(20.0) 循环驱动                                   │   │
│  └──────────────────────────────────────────────────────────────┘   │
│        │                                                            │
│        ▼                                                            │
│  ┌──────────┐    ┌──────────────┐    ┌───────────────────────────┐  │
│  │ OrcaIO   │───▶│ OrcaEngine   │───▶│ RVOSimulator             │  │
│  │(ROS适配) │    │(避障计算核心) │    │ (Agent/KdTree/Obstacle)  │  │
│  └──────────┘    └──────────────┘    └───────────────────────────┘  │
│        │                                        ▲                   │
│        │              ┌─────────────────────────┘                   │
│        │              │                                             │
│        │       ┌──────────────┐                                     │
│        │       │ObstacleBuilder│                                    │
│        │       │(围栏/障碍物)  │                                    │
│        │       └──────────────┘                                     │
│        │                                                            │
│        ▼                                                            │
│  /uavX/orca_cmd (OrcaCmd: 速度+状态)                               │
│  /uavX/orca/goal (目标点可视化 Marker)                              │
│  /uavX/orca/geo_fence (围栏可视化 MarkerArray)                      │
└─────────────────────────────────────────────────────────────────────┘
```

参数配置
--------

### Agent_Swarm 参数 (通过 launch arg 传入，agent_swarm.yaml 为注释占位)

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `agent_id` | int | 1 | 本机 ID（1-based），是否 Leader 由 `leader_id` 决定 |
| `agent_num` | int | 1 | 集群总数 |
| `agent_type` | int | 0 | 类型: 0=UAV, 1=UGV |
| `leader_id` | int | 1 | Leader 的 ID（在 `agent_swarm.launch` 默认 1；`swarm_sim.launch` 默认 1）；>100 表示 UGV Leader，实际 ID = leader_id-100 |
| `agent_name` | string | "uav" | 话题前缀名（拼接为 `/{agent_name}{id}/...`） |
| `formation_policy` | string | "ring" | 初始编队策略名 |
| `spacing` | double | 1.0 | 编队相邻间距(m) |
| `spacing_scale_up` | double | 1.2 | expand 乘法因子 |
| `spacing_scale_down` | double | 0.8 | contract 乘法因子 |
| `spacing_min` | double | 0.3 | 最小间距(m) |
| `spacing_max` | double | 5.0 | 最大间距(m) |
| `use_fixed_altitude` | bool | true | 是否强制固定高度（仅 UAV 生效） |
| `fixed_altitude` | double | 1.0 | 固定高度(m)，编队目标 Z 值 |
| `goal_rate` | double | 20.0 | 编队目标计算频率(Hz) |
| `control_rate` | double | 20.0 | 控制指令输出频率(Hz) |
| `state_pub_rate` | double | 20.0 | 状态发布到 ORCA 的频率(Hz) |
| `leader_timeout` | double | 1.0 | Leader 状态超时判定(s) |
| `orca_timeout` | double | 1.0 | ORCA 输出超时判定(s) |
| `goal_z_tolerance` | double | 0.2 | Z 到位容差(m)，仅 UAV 生效，用于 ORCA ARRIVED 切回 HOVER |
| `leader_publish_goal` | bool | true | Leader 是否直通目标点到 ORCA |

### ORCA 参数 (通过 launch arg 传入，orca.yaml 为注释占位)

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `agent_id` | int | 1 | 本机 Agent ID（1-based） |
| `agent_num` | int | 1 | 集群总数 |
| `agent_name` | string | "uav" | 话题前缀名 |
| `orca_params/neighborDist` | float | 1.5 | 邻居搜索距离 |
| `orca_params/timeHorizon` | float | 2.0 | ORCA 时间窗 |
| `orca_params/timeHorizonObst` | float | 2.0 | 障碍物时间窗 |
| `orca_params/radius` | float | 0.3 | Agent 碰撞半径 |
| `orca_params/maxSpeed` | float | 0.5 | ORCA 最大速度(m/s) |
| `orca_params/time_step` | float | 0.1 | RVO 仿真步长(s) |
| `geo_fence/min_x` | float | -5.0 | 围栏 X 最小值 |
| `geo_fence/max_x` | float | 5.0 | 围栏 X 最大值 |
| `geo_fence/min_y` | float | -5.0 | 围栏 Y 最小值 |
| `geo_fence/max_y` | float | 5.0 | 围栏 Y 最大值 |

注意：
- 主循环频率硬编码为 `ros::Rate rate(20.0)`，不可配置。
- `max_neighbors` 不是可配参数，传入 `agent_num` 作为值。
- 目标到达容差硬编码为 0.15m（`0.15f * 0.15f`）。
- 围栏始终构建，无 `fence_enabled` 开关，围栏外边距硬编码为 0.2m。

数据流（逐步详解）
-----------------

### 1 动作/目标输入

外部通过 `formation_switch` 工具或其他节点发送：
- `/sunray/formation_cmd` (`sunray_msgs::Formation`)：
  - `cmd` 字段: `TAKEOFF`(=1), `LAND`(=2), `HOVER`(=3), `FORMATION`(=4), `SET_HOME`(=5), `RETURN_HOME`(=6)
  - `name` 字段: `"ring"`, `"line"`, `"column"`, `"v_shape"`, `"wedge"`, `"expand"`, `"contract"`, `"custom"`
- `/sunray/formation_offsets` (`sunray_msgs::FormationOffsets`)：
  - `offsets` 数组：自定义阵型偏移量（leader 体坐标系，spacing=1 时的归一化偏移）
- `/sunray/leader_goal` (`geometry_msgs::PoseStamped`)：Leader 目标点

`formation_switch`（可执行文件名 `uav_command_pub`，节点名 `formation_switch_node`）是一个交互式命令行工具，从标准输入读取**整数指令**（需回车确认，输入非整数时自动恢复并提示重新输入）：
- `1`=ring, `4`=line, `5`=column, `6`=v_shape, `7`=wedge
- `2`=expand, `3`=contract
- `101`=takeoff, `102`=land, `103`=hover, `104`=set_home, `105`=return
- `201`=输入 leader 目标点 (x y z yaw)

`formation_tui` 是基于 ncurses 的 TUI 小工具，提供“影院选座”式自定义阵型编辑：
- 20x20 网格，中心为 Leader（不可选），其余格子代表 follower 偏移
- 方向定义：上方为 +x（前方），左侧为 +y（左方），与编队偏移坐标一致
- `Arrows/WASD` 移动光标
- `Space` 选中/取消座位（选中数量不能超过 follower 数量），`c` 发布自定义阵型（`name="custom"` + `/sunray/formation_offsets`）
- `x` 清除全部选区，`q` 退出
- `S` 保存、`L`/`l` 加载自定义阵型（文本文件）
- `m` 输入 leader 目标点 (x y z yaw)
- `1/4/5/6/7` 切阵型，`2/3` 扩散/聚拢，`t/g/h/o/b` 起飞/降落/悬停/设定home/返航

补充说明（formation_tui 细节）：
- `agent_num` 读取顺序：私有参数 `~agent_num` → 全局参数 `agent_num` → `/agent_num`，默认 1
- follower 数量 = `agent_num - 1`，自定义阵型座位数必须与 follower 数量一致，否则不允许发送
- 保存/加载文件路径：
  - 按 `S`/`L` 后输入的路径会原样使用
  - 直接回车时默认文件名为 `custom_formation.txt`
  - 默认文件保存到 **formation_tui 进程的当前工作目录**（具体路径取决于启动环境）
  - 需要固定位置时，请在提示中输入**绝对路径**（例如 `/home/xx/formation/custom_formation.txt`）

### 2 AgentSwarmNode 主循环

[`AgentSwarmNode`](Agent_Swarm/include/agent_swarm_node.h) 是核心调度节点，使用**三个独立定时器**驱动：

```
goalTimerCb() @ goal_rate Hz:
  1. 获取有效状态 effectiveState(leader_ok, orca_ok)（不打印状态变化日志）
  2. 若状态为 FORMATION:
     - 从 LeaderTracker 获取 leader 位姿
     - 若有 leader_goal 则以其为参考，否则以 leader 当前位姿为参考
     - FormationPolicy.computeTarget() 计算目标点
     - GoalDispatcher 下发目标
  3. 若状态为 ORCA_RETURN_HOME:
     - 以 home 位置为目标 → GoalDispatcher 下发

controlTimerCb() @ control_rate Hz:
  1. 获取有效状态（状态变化日志仅在此处打印，避免与 goalTimerCb 重复）
  2. INIT → 不发送任何控制指令，等待 TAKEOFF 指令
  3. HOVER → publishHover()
  4. TAKEOFF → handleTakeoff() (时序流程)
  5. LAND → publishLand()
  6. FORMATION/ORCA_RETURN_HOME → 从 OrcaClient 获取 orca_cmd → ControlMapper 映射
     - 到位切 HOVER 条件（全部满足时触发）：
       a. ORCA 报告 ARRIVED
       b. 当前处于活跃动作阶段（formation_change_active_ 或 leader_goal_active_ 或 return_home_active_）
       c. last_goal_valid_ 为真
       d. ORCA 目标点与 AgentSwarm 记录的 last_goal_ 一致（dist2 < 0.04，即目标一致性检查）
       e. UAV 时额外检查 Z 误差 <= goal_z_tolerance
       满足后重置各 active 标志并切状态机到 HOVER

statePublishTimerCb() @ state_pub_rate Hz:
  1. state_cache_.publishOrcaStates()  // 发布全部 agent Odom 给 ORCA
```

### 3 Leader 位姿获取

[`LeaderTracker`](Agent_Swarm/include/leader_tracker.h) 订阅 Leader 状态话题：
- 当 `leader_id <= 100` 时，订阅 `/uav{leader_id}/sunray/uav_state`（硬编码前缀，不使用 `agent_name` 参数）
- 当 `leader_id > 100` 时，订阅 `/ugv{leader_id-100}/sunray_ugv/ugv_state`
- 缓存 Leader 的当前位姿 (`leader_pose_`)
- 提供 `getLeaderPose()` 和 `isFresh(timeout_sec)` 接口
- Leader 目标点 (`leader_goal`) 的缓存和管理在 `AgentSwarmNode` 中，不在 LeaderTracker 中
- 收到 `/sunray/leader_goal` 时，`AgentSwarmNode::leaderGoalCb` 除了缓存目标外还会：
  - 设置 `leader_goal_active_ = true`，失效旧目标（`last_goal_valid_ = false`）
  - 若 `use_fixed_altitude_ && agent_type_==0`，则用 `leader_goal.z` 更新 `fixed_altitude_`
  - 强制状态机切到 `FORMATION`（确保收到目标后立即开始编队移动）

### 4 编队状态机

[`FormationStateMachine`](Agent_Swarm/include/formation_state_machine.h) 管理编队状态转换：

```
状态定义:
  INIT             = 0   // 初始化
  TAKEOFF          = 1   // 起飞中
  LAND             = 2   // 降落
  HOVER            = 3   // 悬停
  FORMATION        = 4   // 编队飞行
  ORCA_RETURN_HOME = 5   // 返航(走 ORCA 目标点)

初始状态:
  requested_state_ = INIT（节点启动后在地面待命，不发送任何控制指令）

状态转换触发:
  - onFormationCmd(cmd):
      任意 + TAKEOFF cmd → TAKEOFF
      任意 + FORMATION cmd → FORMATION
      任意 + HOVER cmd → HOVER
      任意 + LAND cmd → LAND
      任意 + RETURN_HOME cmd → ORCA_RETURN_HOME
      SET_HOME → 不改变状态（仅在 AgentSwarmNode 中记录 home 位置）
      未知指令 → 保持当前状态不变
      (EXPAND/CONTRACT 的 cmd 仍是 FORMATION，spacing 调整在 AgentSwarmNode 中处理)

  - effectiveState() 安全兜底（无 update() 方法）:
      INIT → 直接返回 INIT，等待外部 TAKEOFF 指令
      TAKEOFF/LAND/HOVER → 直接返回，不受 leader/orca 状态影响
      ORCA_RETURN_HOME + orca 超时 → HOVER
      FORMATION + (leader 超时 或 orca 超时) → HOVER

  - controlTimerCb 中的自动转换:
      FORMATION/ORCA_RETURN_HOME → ORCA ARRIVED + active 标志 + last_goal_valid_ + 目标一致性(dist2<0.04) + Z 到位 → HOVER
```

收到 `name="expand"` / `"contract"` 时，`AgentSwarmNode::onFormationCmd` 使用**乘法因子**调整 spacing：
- expand: `spacing_ = min(spacing_ * spacing_scale_up_, spacing_max_)`（默认 ×1.2）
- contract: `spacing_ = max(spacing_ * spacing_scale_down_, spacing_min_)`（默认 ×0.8）
- 限幅范围 `[spacing_min, spacing_max]`（默认 `[0.3, 5.0]`）
- expand/contract **仅调整 spacing**，不重建策略对象，保持当前阵型不变

**起飞流程**（`handleTakeoff`，仅 UAV 有效）：

基于时间的四阶段流程，不检测高度到位：

```
0s ~ 5s:   phase 1  切换飞控到 CMD_CONTROL 模式（发布 UAVSetup，已是 CMD_CONTROL 时跳过）
5s ~ 10s:  phase 2  解锁（发布 UAVSetup ARM，需 CMD_CONTROL 且未解锁时才发送）
10s ~ 15s: phase 3  发送起飞指令 publishTakeoff(fixed_altitude_)，携带目标高度（需已解锁）
15s+:      phase 4  起飞流程结束，切回 HOVER 状态（需 uav_state_ready_ 且 armed），等待外部发送 FORMATION 指令进入编队
```

注意：`publishTakeoff(altitude)` 在 `UAVControlCMD::Takeoff` 指令中设置 `desired_pos[2] = altitude`，
确保所有 UAV（包括 Leader）在起飞阶段就飞到 `fixed_altitude_` 高度，避免起飞后 Leader 与 Follower 高度不一致。

### 5 编队策略计算（OffsetBasedPolicy）

[`OffsetBasedPolicy`](Agent_Swarm/include/formation_policy.h) 提供通用 `computeTarget()`：
- 将 `generateOffsets()` 生成的二维偏移表按 `spacing` 缩放
- 根据 Leader 航向旋转偏移
- 叠加 Leader 位置得到目标点
  
`Offset2D` 是偏移向量结构（单位化或按队形尺度定义），由各阵型生成。

每种编队仅需实现 `generateOffsets(int follower_count)`：
- Ring/Line/Column/V/V-Shape/Wedge 均在 [`formation_policies.h/cpp`](Agent_Swarm/src/formation_policies.cpp) 中实现

**通用计算流程**：

```cpp
computeTarget(leader_pose, ctx, target_pose) → bool

算法（摘要）:
  leader_index = leaderIndexFromId(ctx.leader_id)   // leader_id>100 时取低位
  若 ctx.agent_id == leader_index: return false      // leader 自身不产生目标

  否则:
    // follower_index（0-based）:
    //   agent_id < leader_index → agent_id - 1
    //   agent_id > leader_index → agent_id - 2
    follower_count = ctx.agent_num - 1
    offsets = generateOffsets(follower_count)
    offset = offsets[follower_index] * ctx.spacing
    offset = rotate(offset, leader_yaw)
    target = leader_pose + offset
    // Z 轴由外层 AgentSwarmNode 根据 use_fixed_altitude/fixed_altitude 覆写
```

[`FormationContext`](Agent_Swarm/include/formation_policy.h) 是一个简单数据结构体，包含 `agent_id`/`agent_num`/`leader_id`/`spacing`，不持有策略指针。[`FormationPolicyFactory`](Agent_Swarm/include/formation_policy_factory.h) 根据 `name` 创建策略实例（支持 `ring/line/column/v_shape/v/wedge/custom`，未知名称默认返回 `ring`）。

### 6 目标点下发

[`GoalDispatcher`](Agent_Swarm/include/goal_dispatcher.h) 负责将编队计算出的目标点发送给 ORCA 层：

- 发布话题: `/{agent_name}{id}/orca/setup` (`sunray_msgs::OrcaSetup`)
- 消息字段:
  - `desired_pos[3]`: 目标位置 (x, y, z)
  - `desired_yaw`: 目标航向
  - `cmd`: 运行模式，`GOAL_RUN` 表示向目标移动, `GOAL` 表示设置目标不启动
- GoalDispatcher 是简单的 publish 封装，每次调用直接发布，**无防抖机制**
- **到位检测**在 `AgentSwarmNode::controlTimerCb` 中完成（dist2 < 0.04 即 0.2m），不在 GoalDispatcher 中

### 7 ORCA 避障

ORCA 层作为独立节点运行，每个 agent 一个实例。`orca_node.cpp` 的 main 函数中直接初始化 OrcaEngine 和 OrcaIO，然后在 `ros::Rate(20.0)` 循环中调用 `engine.step()`、`io.publishCmd()` 与 `io.publishFenceMarkers()`（无独立 OrcaNode 类，`orca_node.h` 为占位头文件）：

[`OrcaIO`](ORCA/include/orca_io.h) → [`OrcaEngine`](ORCA/include/orca_engine.h) → [`RVOSimulator`](ORCA/include/RVOSimulator.h)

**OrcaIO** 负责 ROS 通信适配：
- 订阅**所有** agent 的 `/{agent_name}{i}/orca/agent_state` (nav_msgs::Odometry)
- 订阅**所有** agent 的 `/{agent_name}{i}/orca/setup` (sunray_msgs::OrcaSetup)，用于为所有 agent 设置 RVO goal
- 发布 `/{agent_name}{id}/orca_cmd` (sunray_msgs::OrcaCmd)
- 发布可视化: `/{agent_name}{id}/orca/goal` (目标点 Marker)、`/{agent_name}{id}/orca/geo_fence` (围栏 MarkerArray)

**OrcaEngine** 封装 RVO 计算：
- `init()`: 创建 RVOSimulator，设置参数 (neighborDist, agent_num, timeHorizon, timeHorizonObst, radius, maxSpeed, timeStep)
- `setupAgents()`: 为每个 agent 创建 RVO Agent，`maxNeighbors` 传入 `agent_num`
- `updateAgentState(idx, odom)`: 更新 agent 当前 Odometry
- `handleSetup(idx, setup)`: 处理 OrcaSetup 消息，设置 RVO goal、运行状态；含**防抖机制**——若已到位且新目标仍在到达半径内，保持 ARRIVED 而不重置为 RUN
- `step(out)`: 执行一步 ORCA 计算
  1. 检查各 agent odom 是否超时（>1s 视为过期）及全局就绪状态
  2. 检查本机是否到达目标（硬编码 0.15m）
  3. 更新所有 agent 的位置和速度到 RVOSimulator（**过期 agent 的位置移至 (1e4,1e4)、速度置零**，避免幽灵位置干扰避障决策）
  4. `simulator->computeVel()` 始终执行 ORCA 避障（包括 ARRIVED/STOP 状态，确保围栏排斥生效）
  5. 按状态输出：STOP→零速度；ARRIVED→仍输出 ORCA 避障速度（正常到位时接近零，靠近围栏时产生排斥）；RUN→正常 `getAgentVelCMD(idx)` 速度

**OrcaCmd 消息** 包含：
- `linear[3]`: 避障后的速度向量 (x, y, z)
- `angular[3]`: 角速度 (x, y, z)
- `goal_pos[3]`: 目标位置
- `goal_yaw`: 目标航向
- `state`: 状态码 —— `INIT`(=0) 初始化, `RUN`(=1) 正在移动, `STOP`(=2) 停止, `ARRIVED`(=3) 已到达目标

**ObstacleBuilder** 构建虚拟围栏：
- 围栏始终构建（无 `fence_enabled` 开关）
- 在 `[min_x, max_x] × [min_y, max_y]` 范围构建四条矩形障碍物边界，外边距硬编码 0.2m
- 障碍物以顶点序列添加到 RVOSimulator

### 8 OrcaClient 接收避障输出

[`OrcaClient`](Agent_Swarm/include/orca_client.h) 订阅 `/{agent_name}{id}/orca_cmd`：
- 缓存最新的 OrcaCmd
- 提供 `getLastCmd(out)` / `isFresh(timeout_sec)` 接口供 AgentSwarmNode 主循环查询
- 仅做缓存，无回调机制。到位检测在 `AgentSwarmNode::controlTimerCb` 中完成

### 9 控制指令映射

[`ControlCommandMapper`](Agent_Swarm/include/control_command_mapper.h) 将 ORCA 输出转为最终控制指令：
- UAV 发布话题: `/{agent_name}{id}/sunray/uav_control_cmd` (`sunray_msgs::UAVControlCMD`)
- UGV 发布话题: `/{agent_name}{id}/sunray_ugv/ugv_control_cmd` (`sunray_msgs::UGVControlCMD`)
- **三种映射模式**（根据 OrcaCmd.state）：
  - `RUN` → UAV:`XyVelZPosYaw`；UGV:`VEL_CONTROL_ENU`
  - `ARRIVED` → UAV:`XyzPosYaw`；UGV:`POS_CONTROL_ENU`
  - `STOP`/其它 → UAV:`Hover`；UGV:`HOLD`
- **特殊指令**：
  - `publishTakeoff(altitude)`: 起飞指令（仅 UAV），`altitude` 参数设置 `desired_pos[2]` 确保起飞到指定高度
  - `publishLand()`: 降落指令（仅 UAV）
  - `publishHover()`: 悬停/停止
- 各方法直接调用发布，无 `publishIfNeeded()` 接口

### 10 AgentStateCache

[`AgentStateCache`](Agent_Swarm/include/agent_state_cache.h) 维护**所有 agent** 的状态并向 ORCA 报告：
- 订阅所有 agent 的状态话题：
  - UAV: `/{agent_name}{id}/sunray/uav_state`
  - UGV: `/{agent_name}{id}/sunray_ugv/ugv_state`
- 将 UAVState/UGVState 转换为 nav_msgs::Odometry 缓存
- `publishOrcaStates()`: 定时发布所有已缓存 agent 的 Odometry 到 `/{agent_name}{id}/orca/agent_state`
- 提供 `states()` 接口返回完整的 `map<int, Odometry>` 缓存

关键话题与作用（表）
-------------------

| 话题 | 方向 | 消息类型 | 发布者 | 订阅者 | 用途 |
|---|---|---|---|---|---|
| `/sunray/formation_cmd` | ext→Agent | `sunray_msgs/Formation` | formation_switch / formation_tui | AgentSwarmNode | 编队指令 |
| `/sunray/formation_cmd/ground` | ext→Agent | `sunray_msgs/Formation` | 地面站 | AgentSwarmNode | 编队指令备用通道 |
| `/sunray/formation_offsets` | ext→Agent | `sunray_msgs/FormationOffsets` | formation_tui | AgentSwarmNode | 自定义阵型偏移量 |
| `/sunray/leader_goal` | ext→Agent | `geometry_msgs/PoseStamped` | formation_switch / formation_tui | AgentSwarmNode | Leader 目标点 |
| `/{name}{id}/sunray/uav_state` | ext→Agent | `sunray_msgs/UAVState` | 飞控/仿真 | AgentStateCache, LeaderTracker, AgentSwarmNode | 无人机状态 |
| `/{name}{id}/sunray_ugv/ugv_state` | ext→Agent | `sunray_msgs/UGVState` | 车控/仿真 | AgentStateCache, LeaderTracker（仅常规前缀场景） | 无人车状态 |
| `/{name}{id}/sunray/setup` | Agent→飞控 | `sunray_msgs/UAVSetup` | AgentSwarmNode | 飞控/仿真 | UAV 模式切换（起飞流程使用） |
| `/{name}{id}/orca/agent_state` | Agent→ORCA | `nav_msgs/Odometry` | AgentStateCache | OrcaIO | 全部 agent 状态→ORCA |
| `/{name}{id}/orca/setup` | Agent→ORCA | `sunray_msgs/OrcaSetup` | GoalDispatcher | OrcaIO | 目标点与运行模式 |
| `/{name}{id}/orca_cmd` | ORCA→Agent | `sunray_msgs/OrcaCmd` | OrcaIO | OrcaClient | 避障后速度与状态 |
| `/{name}{id}/sunray/uav_control_cmd` | Agent→飞控 | `sunray_msgs/UAVControlCMD` | ControlCommandMapper | 飞控/仿真 | UAV 最终控制指令 |
| `/{name}{id}/sunray_ugv/ugv_control_cmd` | Agent→车控 | `sunray_msgs/UGVControlCMD` | ControlCommandMapper | 车控/仿真 | UGV 最终控制指令 |
| `/{name}{id}/orca/goal` | ORCA→RViz | `visualization_msgs/Marker` | OrcaIO | RViz | 目标点可视化 |
| `/{name}{id}/orca/geo_fence` | ORCA→RViz | `visualization_msgs/MarkerArray` | OrcaIO | RViz | 围栏可视化 |

注：`{name}` = `agent_name` 参数值（默认 `uav`），`{id}` = `agent_id`（1-based）。例如 `/uav1/orca_cmd`。
例外：LeaderTracker 对 Leader 状态订阅始终使用硬编码前缀：`leader_id<=100` 时为 `/uav{leader_id}/sunray/uav_state`，`leader_id>100` 时为 `/ugv{leader_id-100}/sunray_ugv/ugv_state`；不跟随 `agent_name`。

动作指令（形成编队的方式）
------------------------
- `cmd=FORMATION`：进入编队模式，`name` 指定编队或动作
  - `name=ring/line/column/v_shape/wedge`：切换阵型
  - `name=expand/contract`：调整 `spacing`（乘法因子 `spacing_scale_up`/`spacing_scale_down`，限幅 `[spacing_min, spacing_max]`）
- `cmd=HOVER`：悬停
- `cmd=TAKEOFF`：全体起飞
- `cmd=LAND`：全体降落
- `cmd=RETURN_HOME`：全体返航（走 ORCA）

状态机图示
------------------

```
                    ┌──────────┐
                    │   INIT   │ ◄─── 初始状态（地面待命，不发送控制指令）
                    └────┬─────┘
                         │ TAKEOFF cmd
                         ▼
                    ┌──────────┐
                    │ TAKEOFF  │
                    └────┬─────┘
                         │ 15s后
                         ▼
                    ┌──────────┐
                    │  HOVER   │ ◄─── 起飞完成后在此等待
                    └────┬─────┘
                         │ FORMATION cmd
                         ▼
    ┌──────────┐ ◄──── FORMATION cmd (从任意状态)
 ┌──│FORMATION │──┐
 │  └────┬─────┘  │
 │       │        │ EXPAND/CONTRACT
 │       │        │ (乘法调整 spacing, 保持 FORMATION)
 │       │ ARRIVED│
 │       ▼        │
 │  ┌──────────┐  │
 └──│  HOVER   │──┘
    └────┬─────┘
         │
         ▼          ▼
    ┌──────────┐  ┌──────────────┐
    │   LAND   │  │ORCA_RETURN_H │
    └──────────┘  └──────────────┘

  正常流程: INIT → TAKEOFF → HOVER → (FORMATION cmd) → FORMATION → HOVER (到位) → ...
  注: LAND 和 RETURN_HOME 可从任意状态进入（RETURN_HOME 走 ORCA）
  注: effectiveState() 安全兜底——INIT 直接返回不做任何操作，
      leader/orca 超时时 FORMATION→HOVER，orca 超时时 ORCA_RETURN_HOME→HOVER
```

RVO/ORCA 内核说明
-----------------

ORCA 层使用经典的 RVO2 (Reciprocal Velocity Obstacles) 算法实现多 agent 避障：

### 核心类

- [`RVOSimulator`](ORCA/include/RVOSimulator.h)：仿真器主类，管理所有 Agent、Obstacle 和 KdTree
  - `addAgent(pos, ...)`: 添加 agent 及其参数
  - `addObstacle(vertices)`: 添加多边形障碍物
  - `processObstacles()`: 构建障碍物 KdTree
  - `setAgentPrefVelocity(id, vel)`: 设置期望速度
  - `setAgentGoal(id, goal)`: 设置 agent 目标点
  - `computeVel()`: 执行一步避障计算
  - `getAgentVelCMD(id)`: 获取避障后的指令速度
  - `getAgentVelocity(id)`: 获取当前速度

- [`Agent`](ORCA/include/Agent.h)：单个 agent 的 ORCA 计算
  - `computeNeighbors()`: 通过 KdTree 搜索邻居
  - `computeNewVelocity()`: 计算 ORCA 约束平面，求解线性规划得到最优速度

- [`KdTree`](ORCA/include/KdTree.h)：空间索引，加速邻居搜索
  - `buildAgentTree()`: 构建 agent KdTree
  - `buildObstacleTree()`: 构建障碍物 KdTree
  - `computeAgentNeighbors()`: 查询 agent 邻居
  - `computeObstacleNeighbors()`: 查询障碍物邻居

- [`Obstacle`](ORCA/include/Obstacle.h)：障碍物线段
  - 以链表形式存储多边形边

- [`Vector2`](ORCA/include/Vector2.h)：2D 向量运算工具类

### ORCA 算法流程

```
每个时间步 (OrcaEngine::step):
  1. 检查各 agent odom 是否过期（>1s）、全局就绪判定
     - 未就绪 / 未启动 → 输出 INIT 零速度，提前返回
  2. 检查本机是否到达目标 (0.15m 硬编码，使用 agent_state_ 非 sim 内部位置)
  3. 更新所有 agent 位置和速度 → RVOSimulator
     - 过期 agent（odom 超时 >1s）位置移至 (1e4,1e4)、速度置零，避免幽灵干扰
  4. simulator->computeVel():  // 始终执行，包括 ARRIVED/STOP
     a. 每个 Agent::computeNeighbors()  // KdTree 搜索
     b. 每个 Agent::computeNewVelocity() // ORCA 线性规划
  5. 按状态输出:
     - STOP → 零速度
     - ARRIVED → 仍输出 ORCA 避障速度（正常到位时接近零，靠近围栏时产生排斥）
     - RUN → getAgentVelCMD(idx) 正常速度
```

Launch 配置说明
--------------

### swarm_sim.launch

集群仿真总 launch，为每个 agent 启动 agent_swarm 和 orca 节点（最多 6 台，超过需手动扩展）：

```xml
<launch>
  <arg name="agent_num" default="1"/>
  <arg name="agent_type" default="0"/>   <!-- 0=UAV, 1=UGV -->
  <arg name="leader_id" default="1"/>
  <arg name="agent_name" default="uav"/>
  <arg name="formation_policy" default="ring"/>
  <arg name="spacing" default="2.5"/>
  <arg name="spacing_min" default="0.3"/>
  <arg name="spacing_max" default="5.0"/>
  <arg name="spacing_scale_up" default="1.2"/>
  <arg name="spacing_scale_down" default="0.8"/>
  <arg name="use_fixed_altitude" default="true"/>
  <arg name="fixed_altitude" default="1.0"/>
  <arg name="goal_z_tolerance" default="0.2"/>
  <arg name="leader_publish_goal" default="true"/>
  <!-- ORCA 参数 -->
  <arg name="neighborDist" default="3.0"/>
  <arg name="maxSpeed" default="2.0"/>
  ...

  <!-- agent 1 (Leader) -->
  <group if="$(eval arg('agent_num') >= 1)">
    <include file="agent_swarm.launch">
      <arg name="agent_id" value="1"/>
      ...
    </include>
    <include file="orca.launch">
      <arg name="agent_id" value="1"/>
      ...
    </include>
  </group>

  <!-- agent 2, 3, ... 6 -->
  ...
</launch>
```

### agent_swarm.launch

单个 agent 的控制节点：
- 所有参数通过 launch arg 传入（`agent_swarm.yaml` 为注释占位）
- 启动 `agent_swarm_node`

### orca.launch

单个 agent 的 ORCA 节点：
- 所有参数通过 launch arg 传入（`orca.yaml` 为注释占位）
- 启动 `orca_node`

### formation_switch.launch

启动 `uav_command_pub` 交互控制工具（节点名 `formation_switch_node`）。

### formation_tui.launch

启动 `formation_tui` TUI 编队编辑工具。

编译说明
--------

基于 catkin/CMake 构建（ROS 1）：

```bash
cd ~/catkin_ws
catkin_make --pkg sunray_swarm
# 或
catkin build sunray_swarm
```

依赖：
- `roscpp`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `visualization_msgs`, `tf`, `tf2`, `tf2_geometry_msgs`
- `sunray_msgs`（自定义消息包：Formation, FormationOffsets, OrcaSetup, OrcaCmd, UAVState, UGVState, UAVControlCMD, UGVControlCMD, UAVSetup）
- `ncurses`（formation_tui 依赖）

代码文件职责（与文件头中文说明同步）
-------------------------------

### Agent_Swarm 层

| 文件 | 职责 |
|---|---|
| [`agent_swarm_node.h/cpp`](Agent_Swarm/src/agent_swarm_node.cpp) | 节点入口与调度，订阅/发布、定时器与状态机驱动 |
| [`formation_state_machine.h/cpp`](Agent_Swarm/src/formation_state_machine.cpp) | 编队状态机（状态转换、指令分发、安全兜底） |
| [`leader_tracker.h/cpp`](Agent_Swarm/src/leader_tracker.cpp) | Leader 位姿获取与缓存（硬编码 /uavN 或 /ugvN 前缀） |
| [`formation_policy.h`](Agent_Swarm/include/formation_policy.h) | 编队策略接口与通用偏移策略基类（`Offset2D`, `OffsetBasedPolicy`, `FormationContext`） |
| [`formation_policies.h/cpp`](Agent_Swarm/src/formation_policies.cpp) | Ring/Line/Column/V-Shape/Wedge/Custom 的偏移表生成与通用目标计算 |
| [`formation_policy_factory.h/cpp`](Agent_Swarm/src/formation_policy_factory.cpp) | 策略工厂，注册 `ring/line/column/v_shape/v/wedge/custom` |
| [`agent_state_cache.h/cpp`](Agent_Swarm/src/agent_state_cache.cpp) | 全部 agent 状态缓存与 `/orca/agent_state` Odometry 发布 |
| [`goal_dispatcher.h/cpp`](Agent_Swarm/src/goal_dispatcher.cpp) | 编队目标 → OrcaSetup 封装与发布 |
| [`orca_client.h/cpp`](Agent_Swarm/src/orca_client.cpp) | 订阅 `/orca_cmd`，缓存 ORCA 避障输出 |
| [`control_command_mapper.h/cpp`](Agent_Swarm/src/control_command_mapper.cpp) | ORCA 速度输出 → UAVControlCMD/UGVControlCMD 映射（含三种模式：RUN/ARRIVED/STOP） |
| [`formation_switch.cpp`](Agent_Swarm/utils/formation_switch.cpp) | 命令行交互控制工具（可执行文件名 `uav_command_pub`，读取整数指令） |
| [`formation_tui.cpp`](Agent_Swarm/utils/formation_tui.cpp) | ncurses TUI 编队编辑工具（自定义阵型/切阵型/起降/目标点） |

### ORCA 层

| 文件 | 职责 |
|---|---|
| [`orca_node.h/cpp`](ORCA/src/orca_node.cpp) | ORCA 节点入口（orca_node.h 为占位头文件，逻辑在 cpp 的 main 中） |
| [`orca_engine.h/cpp`](ORCA/src/orca_engine.cpp) | ORCA/RVO 避障计算核心封装 |
| [`orca_io.h/cpp`](ORCA/src/orca_io.cpp) | ROS 话题适配（订阅所有 agent 状态/目标，发布速度/可视化） |
| [`obstacle_builder.h/cpp`](ORCA/src/obstacle_builder.cpp) | 围栏/矩形障碍物构建 |
| [`RVOSimulator.h/cpp`](ORCA/src/RVOSimulator.cpp) | RVO2 仿真器主类 |
| [`Agent.h/cpp`](ORCA/src/Agent.cpp) | RVO2 单 Agent ORCA 计算（邻居搜索+线性规划） |
| [`KdTree.h/cpp`](ORCA/src/KdTree.cpp) | KdTree 空间索引 |
| [`Obstacle.h/cpp`](ORCA/src/Obstacle.cpp) | 障碍物线段数据结构 |
| [`Vector2.h`](ORCA/include/Vector2.h) | 2D 向量工具 |
| [`Definitions.h`](ORCA/include/Definitions.h) | 公共类型定义与常量 |
| [`RVO.h`](ORCA/include/RVO.h) | RVO 库统一头文件 |

扩展指南
--------

### 添加新编队阵型

1. 在 [`formation_policies.h/cpp`](Agent_Swarm/src/formation_policies.cpp) 中新增策略类，继承 `OffsetBasedPolicy`
2. 实现 `generateOffsets(follower_count)`，返回二维偏移表
3. 在 [`FormationPolicyFactory`](Agent_Swarm/src/formation_policy_factory.cpp) 中注册新类型
4. 在 `sunray_msgs/Formation` 消息中新增 `name` 约定（必要时扩展常量）
5. 在 `formation_switch` 和/或 `formation_tui` 中添加对应的指令绑定

### 添加新的外部控制指令

1. 在 [`FormationStateMachine`](Agent_Swarm/src/formation_state_machine.cpp) 中添加新状态/转换
2. 在 [`AgentSwarmNode::controlTimerCb`](Agent_Swarm/src/agent_swarm_node.cpp) 中处理新状态
3. 在 [`ControlCommandMapper`](Agent_Swarm/src/control_command_mapper.cpp) 中添加新的指令映射
