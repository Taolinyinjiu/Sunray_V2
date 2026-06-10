# Sunray Planning 模块说明

`planning` 目录负责把“外部目标点/任务指令”转换为无人机控制模块能执行的轨迹控制指令。对新开发者来说，可以先把它理解成三层：

- `sunray_planner_msgs`：规划器适配消息，主要承接 EGO / Diff 等外部规划器输出。
- `sunray_planner_tools`：辅助工具，例如 RViz 目标点桥接、点云坐标转换。
- `sunray_planning`：Sunray 规划状态机，负责连接 Sunray 控制指令、外部规划器和 `uav_control`。
- `source_planners`：第三方规划器源码区，包含 EGO、Diff、FUEL、SUPER 等算法包。当前 Sunray 状态机只适配了 EGO 和 Diff。

## 目录结构

```text
planning/
  sunray_planner_msgs/       # EGO / Diff 规划器输出消息定义
  sunray_planner_tools/      # RViz goal 桥接、点云转换等工具节点
  sunray_planning/           # Sunray 规划状态机和 planner 适配层
  source_planners/           # 第三方规划器源码，尽量按上游结构保留
```

### sunray_planner_msgs

这个包定义 Sunray 适配层使用的 planner 输出消息：

- `EgoPositionCommand.msg`：EGO planner 输出的位置、速度、加速度、yaw 等轨迹指令。
- `DiffPositionCommand.msg`：Diff planner 输出，字段类似 EGO，额外包含 jerk。
- `DiffGoalSet.msg`：Diff planner 目标点集合相关消息。

注意：这些消息不是最终控制指令。最终发给控制模块的是 `sunray_msgs/UAVControlCMD`。

### sunray_planner_tools

包含两个实用工具：

- `rviz_goal_bridge`：订阅 RViz 的 `geometry_msgs/PoseStamped` 目标点，转换为 `sunray_msgs/UAVPlanningCMD`，发布到 `${agent_key}/sunray/uav_planning/planning_cmd`。
- `point_cloud_transform`：订阅点云和里程计，根据最近一次 odom 位姿把点云转换到指定 frame 后发布。

### sunray_planning

核心节点是 `sunray_planning_node`。它做的事情是：

1. 订阅 `UAVPlanningCMD`，接收起飞、降落、悬停、返航、局部目标点、全局目标点等规划命令。
2. 根据 `planner_type` 创建对应适配器，目前支持 `ego` 和 `diff`。
3. 把目标点发给第三方 planner。
4. 订阅第三方 planner 的 `position_cmd` 输出。
5. 把 planner 输出转换成 `UAVControlCMD::MOVE_TRAJECTORY`，发布给 `sunray_uav_control`。
6. 发布 `UAVPlanningState`，供监控和上层系统查看规划状态。

### source_planners

这里放的是第三方算法源码：

- `ego_planner_swarm`：EGO planner 相关包，当前由 `EgoPlanner` 适配。
- `diff_planner`：Diff planner 相关包，当前由 `DiffPlanner` 适配。
- `FUEL`：探索规划相关源码，目前 Sunray 状态机中保留枚举但未接入适配器。
- `SUPER`：SUPER planner 相关源码，目前 Sunray 状态机中保留枚举但未接入适配器。

建议开发 Sunray 适配逻辑时优先改 `sunray_planning` 和 `sunray_planner_tools`，不要直接改第三方算法核心，除非确认是算法包自身 bug。

## 核心数据流

典型目标点规划流程如下：

```text
RViz / 上层任务
  -> sunray_msgs/UAVPlanningCMD
  -> sunray_planning_node
  -> EGO / Diff planner target_point
  -> EGO / Diff planner position_cmd
  -> sunray_planning_node
  -> sunray_msgs/UAVControlCMD(MOVE_TRAJECTORY)
  -> sunray_uav_control
```

如果使用 RViz 点目标：

```text
/move_base_simple/goal
  -> rviz_goal_bridge
  -> /uav1/sunray/uav_planning/planning_cmd
```

如果直接写程序发规划指令，则可以跳过 `rviz_goal_bridge`，直接发布 `sunray_msgs/UAVPlanningCMD`。

## 常用话题

默认以 `uav1` 为例：

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 输入 | `/uav1/sunray/uav_planning/planning_cmd` | `sunray_msgs/UAVPlanningCMD` | 上层给规划模块的任务命令 |
| 输入 | `/uav1/sunray/uav_control/control_state` | `sunray_msgs/UAVControlState` | 控制状态机状态，用于判断是否允许规划 |
| 输出 | `/uav1/sunray/uav_control/control_cmd` | `sunray_msgs/UAVControlCMD` | 发给控制模块的轨迹/特殊控制命令 |
| 输出 | `/uav1/sunray/uav_planning/planning_state` | `sunray_msgs/UAVPlanningState` | 规划模块状态 |
| EGO 输入 | `/uav1/sunray/planning/ego_planner/target_point` | `geometry_msgs/PoseStamped` | 转发给 EGO planner 的目标点 |
| EGO 输出 | `/uav1/sunray/planning/ego_planner/position_cmd` | `sunray_planner_msgs/EgoPositionCommand` | EGO planner 轨迹输出 |
| Diff 输入 | `/uav1/sunray/planning/diff_planner/target_point` | `geometry_msgs/PoseStamped` | 转发给 Diff planner 的目标点 |
| Diff 输出 | `/uav1/sunray/planning/diff_planner/position_cmd` | `sunray_planner_msgs/DiffPositionCommand` | Diff planner 轨迹输出 |

## 启动方式

推荐统一从 `sunray_planning.launch` 启动：

```bash
source ~/Sunray_v2/devel/setup.bash
roslaunch sunray_planning sunray_planning.launch planner_type:=ego
```

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `planner_type` | `ego` | 选择 `ego` 或 `diff` |
| `agent_name` | `uav` | 智能体名前缀 |
| `agent_id` | `1` | 智能体编号 |
| `use_private_agent_key` | `false` | 是否使用私有参数构造 agent key |
| `odom_topic` | `${agent_key}/sunray/localization/local_odom` | 传给底层 planner 的里程计话题 |
| `cloud_topic` | `${agent_key}/sunray/localization/global_points` | 传给底层 planner 的点云话题 |
| `front_vis` | `false` | Diff planner 的前向视野裁剪参数 |
| `enable_rviz` | `false` | 是否启动 planner 自带 RViz |
| `enable_rviz_bridge` | `true` | 是否启动 RViz goal 到 planning_cmd 的桥接 |
| `log_save` | `false` | 是否保存规划状态机日志 |

示例：EGO planner，使用自定义点云话题：

```bash
roslaunch sunray_planning sunray_planning.launch \
  planner_type:=ego \
  cloud_topic:=/uav1/sunray/localization/global_points
```

示例：Diff planner，开启前向视野裁剪：

```bash
roslaunch sunray_planning sunray_planning.launch \
  planner_type:=diff \
  front_vis:=true
```

## 仿真调试顺序

规划模块本身不强依赖严格启动顺序，但入门调试建议按下面顺序：

1. 启动仿真或真机基础系统。
2. 启动定位模块，确认 `/uav1/sunray/localization/local_odom` 正常。
3. 启动控制模块，确认无人机能起飞并进入 `HOVER`。
4. 启动规划模块。
5. 用 RViz `2D Nav Goal` 或自定义节点发布 `UAVPlanningCMD`。

参考命令：

```bash
roslaunch localization_fusion localization_fusion.launch
roslaunch sunray_uav_control uav_control.launch
roslaunch sunray_planning sunray_planning.launch planner_type:=ego
```

如果使用 RViz 点目标，需要保证目标点 frame 是下面三者之一：

- `world`
- `${agent_key}/sunray_global`
- `${agent_key}/sunray_local`

`rviz_goal_bridge` 会根据 frame 判断生成 `PLAN_GLOBAL_GOAL` 或 `PLAN_LOCAL_GOAL`。当 RViz 目标是 `world` frame 时，由 `world_goal_plan_frame` 决定转换到 global 还是 local，默认是 `global`。

## 状态机行为

`sunray_planning_node` 内部有一层轻量状态机：

- `INIT`：planner 未就绪。
- `READY`：等待规划命令。
- `PLANNING`：目标已发送，正在接收并转发 planner 输出。
- `ARRIVED`：planner 报告轨迹完成，向控制模块发一次 `HOVER`。
- `HOVER`：规划失败、超时或收到悬停命令后保持悬停。
- `TAKEOFF` / `LAND` / `RETURN`：特殊命令会直接转发给控制模块。
- `EMERGENCY_KILL`：保留状态，用于急停语义。

默认情况下，规划状态机会跟随 `sunray_uav_control` 的状态。只有控制模块处于 `HOVER` 或 `MOVE` 时，新的目标点才会被接受。若控制状态超时，规划命令会被拒绝。

## 如何接入新的 planner

如果要接入 FUEL、SUPER 或新的自研 planner，建议按这个流程做：

1. 在 `planner_datatypes.hpp` 中增加 planner 类型枚举和字符串转换。
2. 新建一个继承 `PlannerInterface` 的适配器，例如 `FuelPlanner`。
3. 在适配器中完成三件事：
   - 发布目标点给底层 planner。
   - 订阅底层 planner 的 position command。
   - 将底层 planner 状态转换为 `PlannerSnapshot`。
4. 在 `PlanningFSM::init()` 的 switch 中创建新适配器。
5. 在 `sunray_planning.launch` 中 include 对应底层 planner 的 launch。
6. 尽量保持底层 planner 源码不改，只在适配层做话题、frame、消息格式转换。

适配器最关键的接口是：

```cpp
bool send_goal(const PlanningTarget& target);
bool get_planner_positioncmd(PlannerPositionCommand& cmd);
PlannerSnapshot get_planner_state() const;
```

只要能把底层 planner 输出稳定转换成 `PlannerPositionCommand`，Sunray 控制模块就可以复用同一条控制链路。

## 常见问题

### 目标点发了但无人机不动

优先检查：

```bash
rostopic echo /uav1/sunray/uav_planning/planning_cmd
rostopic echo /uav1/sunray/uav_planning/planning_state
rostopic echo /uav1/sunray/uav_control/control_state
rostopic echo /uav1/sunray/uav_control/control_cmd
```

如果 `control_state` 不是 `HOVER` 或 `MOVE`，规划状态机默认会拒绝目标点。

### RViz 点目标没有生效

检查 RViz 发布的 frame：

```bash
rostopic echo /move_base_simple/goal/header
```

只支持 `world`、`${agent_key}/sunray_global`、`${agent_key}/sunray_local`。如果使用 `world`，还需要 TF 能把 `world` 转到目标规划 frame。

### planner 没有输出 position_cmd

检查底层 planner 的输入：

```bash
rostopic echo /uav1/sunray/planning/ego_planner/target_point
rostopic echo /uav1/sunray/planning/ego_planner/position_cmd
```

同时确认 odom 和点云话题已经传给底层 planner，并且两者 frame 语义一致。

### 只支持第一个 waypoint

当前 `sunray_planning` 收到多 waypoint 时只转发第一个 waypoint。多航点任务需要后续扩展队列和到点切换逻辑。
