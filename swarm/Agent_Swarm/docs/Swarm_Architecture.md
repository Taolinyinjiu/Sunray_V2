# sunray_swarm 技术与架构文档

> 面向多机（UAV / UGV）编队控制与 ORCA 避障的 ROS 1 软件包。本文基于 `swarm/` 源码，覆盖模块划分、数据流、状态机、ORCA 内核、ROS 接口与参数。

---

## 1. 总体概览

`sunray_swarm` 将**编队规划**与**避障执行**合并到单个 per‑agent 节点内运行：每个 agent 启动一份 `uav_swarm_node`（或 `ugv_swarm_node`），节点内内嵌 ORCA 引擎。agent 之间通过 ROS 话题交换各自的目标点（`OrcaSetup`）与里程计（`local_odom`），构成分布式的集中式避障。

### 1.1 目录结构

```
swarm/
├── Agent_Swarm/     # 编队控制主体（Leader/Follower、FSM、策略、控制映射）
│   ├── include/     # 公共接口
│   ├── src/         # 实现 + 节点入口 (uav/ugv swarm_node)
│   ├── utils/       # 外设工具：键盘指令、Ncurses TUI
│   └── launch/      # 单 agent 启动片段
├── ORCA/            # ORCA/RVO2 纯 C++ 内核 + 封装层
│   ├── include/     # RVO2 (Agent/Obstacle/KdTree/RVOSimulator) + orca_engine 封装
│   └── src/
├── launch/          # 多机仿真一键启动 (swarm_sim.launch 等)
└── scripts/         # 辅助脚本
```

### 1.2 顶层组件图

```
┌─────────────────────── 用户层 ───────────────────────┐
│  formation_tui (ncurses)   formation_switch (stdin) │
│         │                          │                │
│         ▼                          ▼                │
│  /sunray/formation_offsets   /sunray/swarm/uav_swarm_cmd
│  /sunray/leader_goal                                │
└──────────────────────┬──────────────────────────────┘
                       │
        ┌──────────────┴──────────────┐
        ▼                             ▼
┌──────────────────────┐    ┌──────────────────────┐
│  uav_swarm_node  #i  │◀──▶│  uav_swarm_node  #j  │   (ROS 话题)
│ ┌──────────────────┐ │    │        ...           │
│ │ LeaderTracker    │ │    └──────────────────────┘
│ │ AgentStateCache  │ │
│ │ FormationPolicy  │ │  生成目标 → OrcaSetup
│ │ FormationFSM     │ │
│ │ GoalDispatcher   │─┼────▶ /{name}{id}/orca/setup
│ │ OrcaEngine       │◀┼──── (订阅其他 agent 的 setup)
│ │ ControlMapper    │─┼────▶ /{name}{id}/sunray/uav_control_cmd
│ └──────────────────┘ │
└──────────────────────┘
        ▲
        │ odom
   /{name}{id}/sunray/localization/local_odom
```

---

## 2. Agent_Swarm 内部架构

### 2.1 类职责表

| 类 | 文件 | 职责 |
|---|---|---|
| `UavSwarmNode` / `UgvSwarmNode` | `swarm_uav_control_fsm.cpp` / `swarm_ugv_control_fsm.cpp` | 节点入口；整合组件与三个定时器 |
| `FormationStateMachine` | `formation_state_machine.*` | 管理请求状态与安全兜底（leader/orca 超时 → HOVER） |
| `LeaderTracker` | `leader_tracker.*` | 订阅 leader odom，输出 leader 位姿与新鲜度 |
| `AgentStateCache` | `agent_state_cache.*` | 订阅所有 agent 的 odom，供本地 ORCA 镜像仿真 |
| `FormationPolicy` (基类 `OffsetBasedPolicy`) | `formation_policy.h`, `formation_policies.*` | 按阵型生成 follower 目标点：查表→缩放→旋转→平移 |
| `FormationPolicyFactory` | `formation_policy_factory.*` | 按字符串创建策略（ring/line/column/v_shape/wedge/custom） |
| `GoalDispatcher` | `goal_dispatcher.*` | 打包并发布 `OrcaSetup`（同时喂给本地 ORCA 与其他 agent） |
| `OrcaEngine` | `ORCA/src/orca_engine.cpp` | 本地 RVO 仿真，输出速度指令 |
| `ControlCommandMapper` | `control_command_mapper.*` | 把 ORCA 输出翻译成 `UAVControlCMD` |

### 2.2 分层依赖

```
           ┌────────────────────────────┐
           │  UavSwarmNode / UgvSwarmNode│   (编排层)
           └──────────┬─────────────────┘
                      │
   ┌──────────┬───────┴───────┬──────────────┬────────────┐
   ▼          ▼               ▼              ▼            ▼
FormationFSM LeaderTracker  AgentStateCache GoalDispatcher ControlMapper
   │          │               │              │
   └────┬─────┘               │              │
        ▼                     ▼              ▼
  FormationPolicy        OrcaEngine    (sunray_msgs::UAVControlCMD)
  (Ring/Line/Column/V/Wedge/Custom)
        │                     │
        └────── Offset2D ──────┘
                              │
                              ▼
                       RVO::RVOSimulator (ORCA 内核)
```

---

## 3. 数据流

### 3.1 每周期数据流（Follower 视角）

```
                           ┌────────────────────────────────┐
 其他 agent 的 odom ──────▶│  AgentStateCache               │
                           └──────────────┬─────────────────┘
                                          │ states()
 Leader odom ──▶ LeaderTracker ─────┐     │
                                    ▼     ▼
 /sunray/leader_goal ───▶ (覆盖)  FormationPolicy.computeTarget()
                                    │
                                    ▼ target Pose
                   ┌──── GoalDispatcher.publishGoal ───▶ /{name}{id}/orca/setup
                   │                                      (广播给所有 agent)
                   ▼
 其他 agent 的 /orca/setup ──▶ OrcaEngine.handleSetup(idx)
                                    │
                                    ▼ step()
                              OrcaOutput {linear[2], state}
                                    │
                                    ▼
                          ControlCommandMapper.publishFromOrca
                                    │
                                    ▼
                       /{name}{id}/sunray/uav_control_cmd
```

### 3.2 三个定时器（默认均 20 Hz）

| 定时器 | 回调 | 主要工作 |
|---|---|---|
| `goal_timer_` | `goalTimerCb` | 计算自身目标点并 `dispatchOrcaGoal` |
| `state_pub_timer_` | `statePublishTimerCb` → `updateLocalOrca` | 刷新本地 ORCA 仿真：喂 odom → `sim_->computeVel()` → 缓存 `OrcaCmd` |
| `control_timer_` | `controlTimerCb` | 读状态与最新 `OrcaCmd`，下发控制 |

---

## 4. 编队状态机

### 4.1 状态枚举

`SwarmState = { INIT, TAKEOFF, LAND, HOVER, FORMATION, ORCA_RETURN_HOME }`

### 4.2 状态迁移图

```
                   UAVSwarmCMD::TAKEOFF
        INIT  ───────────────────────────▶  TAKEOFF
          ▲                                   │  (底层 FSM_HOVER 到达)
          │ FSM_INIT                          ▼
         LAND ◀──── LAND cmd ────────────── HOVER ◀──────────┐
          ▲                                   │              │
          │                                   │ SWARM_FORMATION
          │                                   ▼              │
          │                               FORMATION          │
          │                                   │              │
          │     leader/orca 超时兜底          │              │
          │  ◀──────────────────────────────┼──────────────┘
          │                                   │
          │          SWARM_RETURN             │
          └─────────── ORCA_RETURN_HOME ◀────┘
                  (orca 超时则降级 HOVER)
```

### 4.3 有效状态裁决（`FormationStateMachine::effectiveState`）

```
requested == INIT/TAKEOFF/LAND/HOVER          → requested
requested == ORCA_RETURN_HOME                 → orca_ok ? ORCA_RETURN_HOME : HOVER
otherwise (FORMATION)                         → (leader_ok && orca_ok) ? FORMATION : HOVER
```

- `leader_ok = leader_goal_active || LeaderTracker.isFresh(leader_timeout)`
- `orca_ok   = has_orca_cmd && (now - stamp) <= orca_timeout`

---

## 5. 编队策略

### 5.1 通用流程 (`OffsetBasedPolicy::computeTarget`)

```
followerIndex(agent_id, leader_id)
        │
        ▼
generateOffsets(follower_count)  ──▶  Offset2D{ox, oy}  (归一化, spacing=1)
        │
        ▼ × spacing
(ox', oy')
        │
        ▼ 绕 leader yaw 旋转
(rx, ry)
        │
        ▼ + leader.position
 target_pose (z 继承 leader；必要时会被 fixed_altitude 覆盖)
```

- `leader_id > 100` 表示 Leader 是 UGV，索引取 `leader_id - 100`，支持异构混合编队。
- Leader 本身 `computeTarget` 返回 false；节点按 `leader_publish_goal` 决定是否透传自身位姿或 `/sunray/leader_goal`。

### 5.2 五种内置阵型偏移量（归一化）

| 策略 | 公式 | 形态 |
|---|---|---|
| `ring` | 半径 `R = 1/(2·sin(π/n))`，`θ_i = 2πi/n` | 均匀圆周 |
| `line` | `(0, ±rank)`，`rank = i/2+1` | `… 2 0 L 1 3 …` |
| `column` | `(-(i+1), 0)` | 纵队紧随 |
| `v_shape` | `(-rank, ±rank)`（45°） | 雁阵 |
| `wedge` | `(-rank, ±0.5·rank)`（≈27°） | 楔形 |
| `custom` | 外部 `setOffsets`；不足补 ring、超出截断 | 由 TUI 输入 |

### 5.3 Custom 阵型数据链路

```
formation_tui (ncurses 20×20)
   │ 用户在网格选位 / 保存 / 加载
   ▼
sunray_msgs::FormationOffsets.offsets[]
   │
   ▼ /sunray/formation_offsets
UavSwarmNode::formationOffsetsCb
   │
   ▼ CustomPolicy::setOffsets
后续 computeTarget 查表即可
```

---

## 6. ORCA 引擎

### 6.1 分层

```
       sunray_msgs::OrcaSetup/OrcaCmd   ← ROS 边界
                │     │
                ▼     │
       orca_swarm::OrcaEngine           ← 逻辑封装
                │     │
                ▼     │
       RVO::RVOSimulator                ← RVO2 内核
       └─ Agent / Obstacle / KdTree
```

### 6.2 主流程 `OrcaEngine::step()`

```
step(now):
  out.goal_pos ← goal_pos_                          # 回填目标
  for i in agents: valid[i] = (odom fresh ≤ 1s)
  agent_state_ready = all(valid)

  if !start_flag_ or state==INIT or !ready:
      state = INIT; linear = 0; return false

  arrived_goal |= reachedGoal(idx)                  # 0.15m 半径

  for i: 写 sim_.setAgentPosition/Velocity；无效 agent → (1e4,1e4)
  sim_->computeVel()                                # ORCA 核心

  if state == STOP:    linear=0; return true
  if arrived_goal:
      state = ARRIVED
      linear = sim.getAgentVelCMD(idx)              # 到位仍输出（维持围栏排斥）
      return true

  state = RUN
  linear = sim.getAgentVelCMD(idx)
  return false
```

### 6.3 指令语义 (`OrcaSetup.cmd`)

| 命令 | 行为 |
|---|---|
| `GOAL` | 仅更新目标，不立即 RUN |
| `GOAL_RUN` | 更新目标并进入 RUN |
| `RUN` | 开始跟随当前目标 |
| `STOP` | 冻结速度输出（围栏避障仍运行） |

防抖：若已 `ARRIVED` 且新目标仍在 0.15m 到达半径内，保持 `ARRIVED`；否则置 `arrived_goal_=false` 重新起跑。

### 6.4 围栏障碍（`ObstacleBuilder`）

由 4 个厚度 0.2m 的矩形墙构成闭合地理围栏：

```
 (min_x, max_y+0.2) ┌──────── obstacle2 ────────┐ (max_x, max_y+0.2)
                    │                           │
          obstacle3 │         活动区域          │ obstacle1
                    │       [min_x,max_x]       │
                    │       [min_y,max_y]       │
                    └──────── obstacle4 ────────┘
```

`sim_->addObstacle(...) → processObstacles()`，由 KdTree 加速查询。

### 6.5 ORCA 参数 (`OrcaParams`)

| 参数 | launch 默认 | 说明 |
|---|---|---|
| `neighbor_dist` | 3.0 | 邻居搜索半径 |
| `time_horizon` | 2.0 | 对邻居的时窗 |
| `time_horizon_obst` | 2.0 | 对障碍的时窗 |
| `radius` | 0.3 | agent 半径 |
| `max_speed` | 2.0 | 最大速度 |
| `time_step` | 0.2 | 仿真步长 |

---

## 7. 控制映射 (`ControlCommandMapper`)

`OrcaCmd → sunray_msgs::UAVControlCMD` 翻译表：

| `OrcaCmd.state` | 条件 | 下发指令 |
|---|---|---|
| `STOP` | — | `HOVER` |
| `ARRIVED` | `|linear_xy| > 0.05` | `MOVE_VELOCITY`（含围栏排斥速度） |
| `ARRIVED` | 速度≈0 | `MOVE_POINT` → goal |
| `RUN` | — | `MOVE_VELOCITY`（xy = orca.linear；`vz = clamp(1.5·(goal_z−cur_z), 0.5)`） |
| 起飞/降落/悬停 | 节点直接调用 `publishTakeoff/Land/Hover/PosTarget` |

> UGV 分支当前仅打印 `WARN("not wired to a stable control interface yet")`，接口预留但未启用。

---

## 8. ROS 接口汇总

### 8.1 订阅

| 话题 | 类型 | 说明 |
|---|---|---|
| `/{name}{id}/sunray/localization/local_odom` | `nav_msgs/Odometry` | 每个 agent 的位姿/速度（自身 + 全体） |
| `/{name}{id}/sunray/fsm/state` | `sunray_msgs/UAVControlFSMState` | UAV 底层 FSM（识别起飞完成） |
| `/{name}{id}/orca/setup` | `sunray_msgs/OrcaSetup` | agent 间互相广播目标，驱动镜像仿真 |
| `/sunray/swarm/uav_swarm_cmd` (UGV: `ugv_swarm_cmd`) | `sunray_msgs/UAVSwarmCMD` | 顶层集群指令（起降/阵型/返航） |
| `/sunray/leader_goal` | `geometry_msgs/PoseStamped` | 外部指定 Leader 目标（优先于真实 Leader odom） |
| `/sunray/formation_offsets` | `sunray_msgs/FormationOffsets` | Custom 阵型偏移量表 |

### 8.2 发布

| 话题 | 类型 | 说明 |
|---|---|---|
| `/{name}{id}/orca/setup` | `sunray_msgs/OrcaSetup` | 下发 ORCA 目标（兼作 agent 间通告） |
| `/{name}{id}/sunray/uav_control_cmd` | `sunray_msgs/UAVControlCMD` | 最终控制指令 |

### 8.3 命名约定

- UAV：`agent_name="uav"`，`leader_id ≤ 100`，话题前缀 `/uav{id}`
- UGV Leader 混编：`leader_id > 100`，实际索引 `leader_id - 100`，话题前缀 `/ugv{id}`

---

## 9. 指令语义矩阵

`UAVSwarmCMD.swarm_cmd` → 请求状态变化：

| 指令 | 前置状态限制 | 动作 |
|---|---|---|
| `TAKEOFF` | 仅 `INIT` | 缓存 home → 请求 `TAKEOFF` |
| `LAND` | 非 `INIT/TAKEOFF` | 请求 `LAND` |
| `HOVER` | 非 `INIT/TAKEOFF/LAND` | 缓存当前位姿为 hold → `HOVER` |
| `SWARM_RETURN` | 需 home | 请求 `ORCA_RETURN_HOME` |
| `SWARM_FORMATION` | 非 `INIT/TAKEOFF/LAND/RETURN` | 更新 spacing/policy/leader_goal → `FORMATION` |

`formation` 字段枚举：`1=ring, 2=line, 3=column, 4=v_shape, 5=wedge, 6=custom, EXPAND/CONTRACT = 按比例缩放 spacing`。

---

## 10. 启动流程

### 10.1 Launch 拓扑

```
swarm_sim.launch (agent_num ≤ 6)
   ├─ <param name="agent_num"/>                 # 暴露给 formation_tui
   └─ for i in 1..agent_num:
         include Agent_Swarm/launch/agent_swarm.launch  → uav_swarm_node / ugv_swarm_node

formation_tui.launch    → formation_tui         (custom 阵型编辑 + TUI 遥控)
formation_switch.launch → uav_command_pub       (stdin 菜单发指令)
```

### 10.2 Agent 节点初始化顺序（`UavSwarmNode` 构造）

```
读参 (agent_id/num, leader_id, formation_policy, spacing, fixed_altitude,
      orca_params, geo_fence, …)
  │
  ▼
create policy  (ring/line/column/v_shape/wedge/custom)
  │
  ▼
leader_tracker_.init → 订阅 leader odom
state_cache_.init    → 订阅全体 odom
goal_dispatcher_.init→ 建 /orca/setup publisher
orca_engine_.init    → 构造 RVOSimulator、加 agents、加围栏
control_mapper_.init → 建 /sunray/uav_control_cmd publisher
  │
  ▼
订阅全体 /{name}{id}/orca/setup（跳过自己）
创建 goal_timer / control_timer / state_pub_timer
```

---

## 11. 关键不变式与注意事项

- **镜像仿真**：每个 agent 各自跑一份完整 `RVOSimulator`；其他 agent 的位姿来自 odom，其他 agent 的目标来自其广播的 `/orca/setup`。**所有 agent 必须订阅到全体 odom**，否则过期 agent 会被放到 `(1e4, 1e4)` 以避免幽灵障碍。
- **新鲜度阈值**：`OrcaEngine` 内部硬编码 1s 过期；`leader_timeout`/`orca_timeout` 作用于上层 FSM 的 HOVER 兜底。
- **到达判定**：ORCA 层 0.15m；节点层 `isActiveGoalReached` 叠加 `goal_z_tolerance` 再切 `HOVER`。
- **UGV 控制未接线**：`agent_type != 0` 时 `ControlCommandMapper` 只 warn，UGV 节点可计算目标但无终端输出。
- **自定义阵型座位数**：TUI 侧要求等于 `agent_num - 1`，否则 `CustomPolicy` 截断或用 ring 补齐。
- **fixed_altitude 覆盖**：`use_fixed_altitude=true` 时 follower/返航目标 z 统一为 `fixed_altitude`；当 `/sunray/leader_goal` 到来，会用其 z 更新 `fixed_altitude_`。
- **leader_id > 100**：用于 UGV 带队 UAV 的异构编队，相关模块均按 `leader_id-100` 取索引。

---

## 12. 扩展点

| 扩展需求 | 落点 |
|---|---|
| 新增阵型 | 继承 `OffsetBasedPolicy` 实现 `generateOffsets`，在 `FormationPolicyFactory::create` 注册 |
| 接入真实 UGV 控制 | 在 `ControlCommandMapper` 的 UGV 分支填充 publisher；或新增 `UgvControlCommandMapper` |
| 替换避障内核 | 抽象 `OrcaEngine` 为接口，替换为 MPC/Buffered Voronoi；保持 `OrcaSetup/OrcaCmd` 协议 |
| 动态障碍 | 在 `ObstacleBuilder::apply` 前追加；或新增 `DynamicObstacleSource` 订阅传感器 |
| 多 Leader / 分组 | 扩展 `FormationContext` 增加组 id；`AgentStateCache` 按组过滤 |

---

## 13. 构建与运行

```bash
# 按本仓库的非标准 catkin 流程构建（详见 tools/build_scripts）
catkin_make --source swarm/ --build build/sunray_swarm

# 6 机仿真
roslaunch sunray_swarm swarm_sim.launch agent_num:=6

# 键盘指令
roslaunch sunray_swarm formation_switch.launch

# TUI 自定义阵型
rosparam set /agent_num 6
roslaunch sunray_swarm formation_tui.launch
```
