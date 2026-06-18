<!-- title: sunray_planning -->

<section id="sunray-planning">

## sunray_planning

`sunray_planning` 是 Sunray 自有规划状态机包，负责把上层 `UAVPlanningCMD`、底层 EGO/Diff planner 输出和 `sunray_uav_control` 的 `UAVControlCMD` 串起来。

### 核心职责

1. 订阅 `sunray_msgs/UAVPlanningCMD`，接收起飞、降落、悬停、返航、局部目标点、全局目标点等规划命令。
2. 根据 `planner_type` 创建 planner 适配器，目前支持 `ego` 和 `diff`。
3. 把目标点发送给底层 planner。
4. 订阅底层 planner 的 `position_cmd` 输出。
5. 把 planner 输出转换成 `UAVControlCMD::MOVE_TRAJECTORY`，发布给 `sunray_uav_control`。
6. 发布 `sunray_msgs/UAVPlanningState`，供上层和监控工具查看规划状态。

### 目录结构

```text
planning/sunray_planner/sunray_planning/
├── include/
│   ├── planner_datatypes.hpp
│   ├── planner_interface/
│   │   ├── diff_planner.hpp
│   │   ├── ego_planner.hpp
│   │   └── planner_interface.hpp
│   ├── planner_position_cmd.hpp
│   └── planning_fsm.hpp
├── launch/
│   └── sunray_planning.launch
└── src/
    ├── planner_interface/
    │   ├── diff_planner.cpp
    │   └── ego_planner.cpp
    ├── sunray_planning_common.hpp
    ├── sunray_planning_fsm.cpp
    ├── sunray_planning_logs.cpp
    └── sunray_planning_node.cpp
```

### 重要文件

| 文件 | 作用 |
| --- | --- |
| `src/sunray_planning_node.cpp` | ROS 节点入口。 |
| `include/planning_fsm.hpp`、`src/sunray_planning_fsm.cpp` | 规划状态机，处理 planning_cmd、控制状态、planner 输出和控制命令发布。 |
| `src/sunray_planning_logs.cpp` | 规划状态日志输出。 |
| `include/planner_interface/planner_interface.hpp` | planner 适配器统一接口。 |
| `src/planner_interface/ego_planner.cpp` | EGO planner 适配器。 |
| `src/planner_interface/diff_planner.cpp` | Diff planner 适配器。 |
| `include/planner_datatypes.hpp` | planner 类型、状态和字符串转换。 |
| `include/planner_position_cmd.hpp` | 统一 planner 输出结构。 |

### 启动方式

```bash
roslaunch sunray_planning sunray_planning.launch planner_type:=ego
roslaunch sunray_planning sunray_planning.launch planner_type:=diff
```

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `planner_type` | `ego` | 选择 `ego` 或 `diff`。`FUEL`、`SUPER` 当前未接入适配器。 |
| `agent_name` | `uav` | 智能体名前缀。 |
| `agent_id` | `1` | 智能体编号。 |
| `use_private_agent_key` | `false` | 是否使用私有参数构造 agent key。 |
| `log_save` | `false` | 是否保存规划状态机日志。 |
| `odom_topic` | `${agent_key}/sunray/localization/local_odom` | 传给底层 planner 的里程计话题。 |
| `cloud_topic` | `${agent_key}/sunray/localization/global_points` | 传给底层 planner 的点云话题。 |
| `front_vis` | `false` | Diff planner 的前向视野裁剪。 |
| `enable_rviz` | `false` | 是否启动底层 planner 自带 RViz。 |
| `enable_rviz_bridge` | `true` | 是否启动 RViz goal 到 planning_cmd 的桥接。 |

### 输入输出话题

以 `uav1` 为例：

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/uav1/sunray/uav_planning/planning_cmd` | `sunray_msgs/UAVPlanningCMD` | 上层规划命令。 |
| 订阅 | `/uav1/sunray/uav_control/control_state` | `sunray_msgs/UAVControlState` | 控制状态，用于判断是否允许规划。 |
| 发布 | `/uav1/sunray/uav_control/control_cmd` | `sunray_msgs/UAVControlCMD` | 发给 UAV 控制模块的轨迹或特殊控制命令。 |
| 发布 | `/uav1/sunray/uav_planning/planning_state` | `sunray_msgs/UAVPlanningState` | 规划模块状态。 |
| 发布 | `/uav1/sunray/planning/ego_planner/target_point` | `geometry_msgs/PoseStamped` | EGO 目标点。 |
| 订阅 | `/uav1/sunray/planning/ego_planner/position_cmd` | `sunray_planner_msgs/EgoPositionCommand` | EGO 轨迹输出。 |
| 发布 | `/uav1/sunray/planning/diff_planner/target_point` | `geometry_msgs/PoseStamped` | Diff 目标点。 |
| 订阅 | `/uav1/sunray/planning/diff_planner/position_cmd` | `sunray_planner_msgs/DiffPositionCommand` | Diff 轨迹输出。 |

### 状态机行为

规划状态机包括：

| 状态 | 说明 |
| --- | --- |
| `INIT` | planner 未就绪。 |
| `READY` | 等待规划命令。 |
| `PLANNING` | 目标已发送，正在接收并转发 planner 输出。 |
| `ARRIVED` | planner 报告轨迹完成，向控制模块发一次 `HOVER`。 |
| `HOVER` | 规划失败、超时或收到悬停命令后保持悬停。 |
| `TAKEOFF` / `LAND` / `RETURN` | 特殊命令直接转发给控制模块。 |
| `EMERGENCY_KILL` | 保留急停语义。 |

默认情况下，规划状态机会跟随 `sunray_uav_control` 的状态。只有控制模块处于 `HOVER` 或 `MOVE` 时，新的目标点才会被接受。若控制状态超时，规划命令会被拒绝。

### 临时使用说明原文

> 本文档只是临时使用，在planning模块完善后应该被移除

1. 首先sunray_planning并不需要遵循严格的启动顺序，我们可以在任意的时间阶段启动planning模块，但通常我会在起飞前启动或者悬停阶段启动
2. sunray_planning模块有一个launch文件，用于启动与规划相关的所有子模块，子模块一共有三个
 - planner本体：目前仅适配了ego与diff,diff可以认为是ego_v2
 - sunray_planning状态机，用于planner本体与sunray控制模块交互，但目前仅实现了uav的控制模块，后续应该加入ugv部分，如果uav与ugv的控制命令能统一，则不需要额外添加，而是优化为对agent的控制命令
 - rviz_goal_bridge 这个工具将rviz的2D nav goal话题数据转换为sunray_planning_cmd发布到surnay_planning模块

当我们要进行一次实机仿真时，执行顺序为
1. 启动仿真环节
roslaunch sunray_sim sunray_sim.launch
2. 启动定位模块
roslaunch localization_fusion localization_fusion.launch
3. 启动sunray_planning模块
roslaunch sunray_planning sunray_planning.launch planner_type:=ego cloud_topic:=/uav1/global_points
> 这里可以选择ego或者diff,但这里存在编译上的小问题，由于ego与diff都是高飞老师团队设计的，他们似乎在代码中复用了faster_planner的部分内容，这导致当使用build.sh编译时，如果同时勾选两者，能够顺利通过，但是启动时diff_planner会出现无法连接动态库的问题
> 解决方式为 ： 先单独编译diff_planner 再编译ego_planner,因为我对ego_planner出现这个问题进行了修复，这样就可以同时使用两者

4. 启动控制节点
roslaunch sunray_uav_control uav_control.launch
> sunray_sim 是轻量动力学仿真，和 Gazebo/PX4 SITL 不完全等价。做规划验证时建议优先使用 PX4 原生控制链路，确认闭环稳定后再调整更复杂的控制器。
5. 启动控制终端（这一步只是为了起飞，用别的方式也行）
rosrun uav_control_tools uav_terminal_control_node

在这一步后，应该检查无人机是否顺利起飞并进入hover状态， 如果进入了hover状态就可以通过rviz实现给目标点
> 1.rviz的目标点捕获使用的是move_simple_goal话题
> 2.rviz的目标点捕获工具似乎，是硬编码了uav1，这点需要注意下

由于主要做的单机仿真，因此代码中可能有部分是 uav1。使用 sunray_sim 适配 planner 时，可以先使用 uav1 进行测试，这些小问题正在被清理。
目前初步的想法是，使用agent_name与agent_id，不再区分uav与ugv，这样可以轻量化localization与planning模块

</section>
