<!-- title: EGO 源码结构 -->

<section id="ego-planner-source">

## EGO 源码结构

`planning/third_party_planner_examples/ego_planner_example/ego-planner-swarm` 是当前仓库中保留的 EGO planner 示例源码。这个目录的定位不是 Sunray 自研规划器，而是“第三方开源规划器源码 + Sunray 接口示例”。

本页重点围绕真实实验 LIO 入口展开：

```text
planning/third_party_planner_examples/ego_planner_example/ego-planner-swarm/plan_manage/launch/exp/run_exp_single_lio.launch
```

这条启动链路代表了 EGO planner 在 Sunray 中最典型的用法：

```text
Sunray 定位 local_odom + 激光点云
  -> EGO 内部 grid_map 建局部占据地图
  -> EGOReplanFSM 状态机接收目标点并触发重规划
  -> EGOPlannerManager 调用 A* / B-spline 优化生成轨迹
  -> traj_server 将 B-spline 采样成 position_cmd
  -> sunray_ego_adapter 转换成 UAVControlCMD
  -> sunray_uav_control 执行 MOVE_TRAJECTORY
```

对二次开发者来说，先把这条链路跑通，再修改算法或接口，是最稳妥的学习路径。

## 启动入口

`run_exp_single_lio.launch` 做三件事：

1. 定义当前无人机、地图、定位、点云和轨迹约束参数。
2. include `ego_planner/launch/include/advanced_param.xml`，真正启动 EGO planner 内部节点。
3. include `sunray_ego_adapter/launch/Ego2Sunray.launch`，把 EGO 输出转换为 Sunray 控制指令。

入口结构如下：

```text
run_exp_single_lio.launch
  -> ego_planner/launch/include/advanced_param.xml
       -> ego_planner_node
       -> traj_server
  -> rviz
  -> sunray_ego_adapter/launch/Ego2Sunray.launch
       -> Ego2Sunray_node
```

常用启动命令：

```bash
roslaunch ego_planner run_exp_single_lio.launch agent_name:=uav agent_id:=1
```

如果由 Sunray 统一规划模块启动，通常会通过 `sunray_planning` 间接 include 该 launch，并传入 `agent_name`、`agent_id`、`odom_topic`、`cloud_topic`、`enable_rviz` 等参数。

## 参数入口

`run_exp_single_lio.launch` 中最重要的参数如下。

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `agent_name` | `uav` | Sunray 机器人名称前缀。 |
| `agent_id` | `1` | Sunray 机器人编号，与 `agent_name` 拼成 `/uav1`。 |
| `planner_prefix` | `/sunray/uav_planning_example/ego_planner` | EGO 规划话题后缀，最终与 `/uav1` 拼成完整话题前缀。 |
| `drone_id` | `0` | EGO 内部无人机编号，从 0 开始。单机一般保持 0。 |
| `odom_topic` | `/uav1/sunray/localization/local_odom` | 里程计输入，来自 Sunray 定位模块。 |
| `cloud_topic` | `/velodyne_points` | LIO 点云输入，供 EGO 内部 `grid_map` 建图。 |
| `max_vel` | `0.5` | 轨迹最大速度约束。 |
| `max_acc` | `3.0` | 轨迹最大加速度约束。 |
| `max_jerk` | `10.0` | 轨迹最大 jerk 约束。 |
| `planning_horizon` | `7.5` | 局部规划空间前视距离。 |
| `planning_horizon_time` | `1.0` | 局部规划时间前视窗口。 |
| `flight_type` | `1` | 目标输入模式，`1` 表示 RViz 选点，`2` 表示预设航点。 |
| `fix_agent_height` | `true` | 是否让 FSM 和 grid_map 使用固定高度。 |
| `agent_height` | `1.0` | FSM 和 grid_map 使用的固定高度。 |
| `fix_height` | `true` | 是否强制 planner manager 输出固定 z 轨迹。 |
| `fixed_height` | `1.0` | 优化后轨迹的固定高度。 |
| `virtual_ceil_height` | `-1.0` | 虚拟天花板高度，小于等于 `-0.5` 表示关闭。 |
| `enable_rviz` | `true` | 是否启动 RViz。 |

注意 `planner_prefix` 会被入口 launch 拼成完整规划前缀：

```text
/<agent_name><agent_id><planner_prefix>
```

以默认值为例，完整前缀是：

```text
/uav1/sunray/uav_planning_example/ego_planner
```

因此 `traj_server` 最终发布的 EGO 控制采样话题是：

```text
/uav1/sunray/uav_planning_example/ego_planner/position_cmd
```

如果你希望与 Sunray 主线规划文档中常见的 `/uav1/sunray/planning/ego_planner` 保持一致，可以启动时显式覆盖：

```bash
roslaunch ego_planner run_exp_single_lio.launch \
  agent_name:=uav \
  agent_id:=1 \
  planner_prefix:=/sunray/planning/ego_planner
```

`Ego2Sunray.launch` 必须使用同一个 `planner_prefix`，否则 adapter 订阅不到 `position_cmd`。

## 完整数据流

以 `agent_name=uav`、`agent_id=1`、`planner_prefix=/sunray/planning/ego_planner` 为例，完整数据流如下：

```text
/uav1/sunray/localization/local_odom
  -> ego_planner_node: ~odom_world
  -> EGOReplanFSM::odometryCallback()
  -> 当前位姿/速度状态

/velodyne_points
  -> ego_planner_node: ~grid_map/cloud
  -> GridMap::cloudCallback()
  -> 局部占据地图 / 膨胀占据地图

/move_base_simple/goal
  -> ego_planner_node: ~move_base_simple/goal
  -> EGOReplanFSM::waypointCallback()
  -> 全局目标点

EGOReplanFSM
  -> EGOPlannerManager::planGlobalTraj()
  -> EGOPlannerManager::reboundReplan()
  -> BsplineOptimizer + AStar + GridMap
  -> traj_utils/Bspline

/uav1/sunray/planning/ego_planner/drone_0_planning/bspline
  -> traj_server
  -> /uav1/sunray/planning/ego_planner/position_cmd
  -> Ego2Sunray_node
  -> /uav1/sunray/uav_control/control_cmd
```

其中 EGO planner 自己并不直接发布 `sunray_msgs/UAVControlCMD`。真正把第三方规划器输出接入 Sunray 控制系统的是：

```text
planning/third_party_planner_examples/ego_planner_example/sunray_ego_adapter
```

## 运行节点

`advanced_param.xml` 会启动两个 EGO 节点。

| 节点 | 源码 | 作用 |
| --- | --- | --- |
| `drone_0_ego_planner_node` | `plan_manage/src/ego_planner_node.cpp` | EGO 主节点，内部创建 `EGOReplanFSM`，负责状态机、建图、目标管理、重规划和轨迹发布。 |
| `drone_0_traj_server` | `plan_manage/src/traj_server.cpp` | 订阅 B-spline 轨迹，按 100 Hz 采样出位置、速度、加速度和 yaw。 |
| `Ego2Sunray_1` | `sunray_ego_adapter/src/Ego2Sunray.cpp` | 订阅 EGO 的 `position_cmd`，转换成 `sunray_msgs/UAVControlCMD`。 |

### ego_planner_node

`ego_planner_node.cpp` 很短，它只做入口初始化：

```text
main()
  -> ros::init(..., "ego_planner_node")
  -> ros::NodeHandle nh("~")
  -> EGOReplanFSM rebo_replan
  -> rebo_replan.init(nh)
  -> ros::spin()
```

真正的逻辑在：

```text
plan_manage/src/ego_replan_fsm.cpp
plan_manage/src/planner_manager.cpp
plan_env/src/grid_map.cpp
bspline_opt/src/bspline_optimizer.cpp
path_searching/src/...
```

### traj_server

`traj_server.cpp` 的输入是 `traj_utils/Bspline`，输出是 `planner_msgs/EgoPositionCommand`。

它的核心逻辑是：

```text
bsplineCallback()
  -> 读取 B-spline 控制点和 knots
  -> 构造 position trajectory
  -> 求一阶导得到 velocity trajectory
  -> 求二阶导得到 acceleration trajectory

cmdCallback() 100Hz
  -> 按当前时间 t_cur 在轨迹上采样 pos/vel/acc
  -> 根据前视点计算 yaw/yaw_dot
  -> 发布 EgoPositionCommand
```

如果轨迹执行完，`traj_server` 会保持最后一个位置点，并把速度、加速度置零，相当于悬停在终点。

## EGO 内部源码分层

源码目录结构：

```text
ego-planner-swarm/
├── bspline_opt
├── drone_detect
├── path_searching
├── plan_env
├── plan_manage
├── rosmsg_tcp_bridge
└── traj_utils
```

| 子包 | 作用 | 二次开发时主要看哪里 |
| --- | --- | --- |
| `plan_manage` | EGO 主流程，包含状态机、planner manager、traj server 和 launch。 | 先看这里。目标输入、状态机切换、重规划触发、B-spline 发布都在这里。 |
| `plan_env` | 局部占据栅格地图，负责接收 odom、点云或深度图。 | 修改传感器输入、地图尺寸、障碍物膨胀、占据概率时看这里。 |
| `path_searching` | 搜索模块，主要给优化器提供初始路径或避障引导。 | 调整搜索策略、启发函数、网格搜索行为时看这里。 |
| `bspline_opt` | B-spline 轨迹优化核心。 | 调整优化代价、避障距离、速度/加速度约束、平滑性权重时看这里。 |
| `traj_utils` | B-spline 消息、轨迹容器和 RViz 可视化工具。 | 修改轨迹消息、显示 marker、轨迹数据结构时看这里。 |
| `drone_detect` | 多机检测相关工具。 | 当前 Sunray 单机 LIO 链路不是重点。 |
| `rosmsg_tcp_bridge` | ROS 消息 TCP 桥。 | 当前 Sunray 单机 LIO 链路通常不用改。 |

## plan_manage 主流程

`plan_manage` 是最重要的入口包。

```text
plan_manage/
├── launch/
│   ├── exp/run_exp_single_lio.launch
│   └── include/advanced_param.xml
├── src/
│   ├── ego_planner_node.cpp
│   ├── ego_replan_fsm.cpp
│   ├── planner_manager.cpp
│   └── traj_server.cpp
└── include/plan_manage/
    ├── ego_replan_fsm.h
    ├── planner_manager.h
    └── topic_utils.h
```

### EGOReplanFSM

核心文件：

```text
plan_manage/include/plan_manage/ego_replan_fsm.h
plan_manage/src/ego_replan_fsm.cpp
```

它负责“什么时候规划、什么时候重规划、什么时候急停”。主要状态如下：

| 状态 | 含义 |
| --- | --- |
| `INIT` | 等待 odom。 |
| `WAIT_TARGET` | 已有 odom，等待目标点或触发信号。 |
| `SEQUENTIAL_START` | 多机按编号顺序启动，单机 `drone_id=0` 会直接进入规划。 |
| `GEN_NEW_TRAJ` | 根据全局目标生成第一条局部轨迹。 |
| `EXEC_TRAJ` | 正在执行轨迹，并根据时间、距离和碰撞检查决定是否重规划。 |
| `REPLAN_TRAJ` | 从当前轨迹状态继续重规划。 |
| `EMERGENCY_STOP` | 检测到紧急风险时生成停止轨迹。 |

关键回调：

| 函数 | 输入 | 作用 |
| --- | --- | --- |
| `odometryCallback()` | `nav_msgs/Odometry` | 更新当前位置、速度和姿态。若 `fix_agent_height=true`，会把 z 固定为 `agent_height`。 |
| `waypointCallback()` | `geometry_msgs/PoseStamped` | 接收 RViz 目标点，生成全局目标。当前手动模式会把目标 z 固定为 `1.0`。 |
| `triggerCallback()` | `geometry_msgs/PoseStamped` | 真实实验或预设航点模式下用于触发开始。 |
| `swarmTrajsCallback()` | `traj_utils/MultiBsplines` | 接收上一架无人机的多机轨迹。 |
| `BroadcastBsplineCallback()` | `traj_utils/Bspline` | 接收其他无人机广播轨迹，用于多机避碰。 |
| `execFSMCallback()` | Timer 100 Hz | 主状态机循环。 |
| `checkCollisionCallback()` | Timer 20 Hz | 检查当前轨迹是否撞障碍或撞其他无人机。 |

`execFSMCallback()` 是理解流程的关键。它会根据状态调用：

```text
planFromGlobalTraj()
  -> callReboundReplan()

planFromCurrentTraj()
  -> callReboundReplan()

callEmergencyStop()
  -> EGOPlannerManager::EmergencyStop()
```

### EGOPlannerManager

核心文件：

```text
plan_manage/include/plan_manage/planner_manager.h
plan_manage/src/planner_manager.cpp
```

它负责把“起点、速度、加速度、局部目标点、局部地图”交给规划算法，最终生成 B-spline 轨迹。

初始化入口：

```text
EGOPlannerManager::initPlanModules()
  -> 读取 manager 参数
  -> 创建 GridMap
  -> GridMap::initMap()
  -> 创建 BsplineOptimizer
  -> BsplineOptimizer::setParam()
  -> BsplineOptimizer::setEnvironment(grid_map, obj_predictor)
  -> 创建 AStar
  -> AStar::initGridMap()
```

主要规划入口：

```text
EGOPlannerManager::planGlobalTraj()
  -> 根据起点和终点生成全局 min-snap 参考轨迹

EGOPlannerManager::reboundReplan()
  -> 生成初始控制点
  -> BsplineOptimizer::initControlPoints()
  -> BsplineOptimizer::BsplineOptimizeTrajRebound()
  -> 必要时 refineTrajAlgo() 重新分配时间
  -> 可选 fix_height 固定 z
  -> updateTrajInfo()
```

`reboundReplan()` 可以理解为 EGO 局部规划的核心过程：

```text
当前状态 start_pt/start_vel/start_acc
  + 局部目标 local_target_pt/local_target_vel
  + GridMap 障碍物
  + B-spline 约束和优化权重
  -> 优化后的 UniformBspline
```

如果 `use_distinctive_trajs=true`，优化器会生成多条候选初始轨迹，然后选择代价更低且优化成功的一条。这个参数在复杂障碍环境中更容易绕开局部失败，但会增加计算量。

### topic_utils

核心文件：

```text
plan_manage/include/plan_manage/topic_utils.h
```

它提供两个小工具：

| 函数 | 作用 |
| --- | --- |
| `loadTopicPrefix()` | 从私有参数 `planner_prefix` 读取并规范化话题前缀。 |
| `makePrefixedTopic()` | 把 `planner_prefix` 和具体话题拼成绝对话题。 |

Sunray 接入 EGO 时，很多 topic 都依赖这个前缀。如果遇到话题名字不一致，优先检查 `planner_prefix` 是否在 `run_exp_single_lio.launch`、`advanced_param.xml`、`Ego2Sunray.launch` 三处保持一致。

## plan_env 地图输入

`plan_env` 的核心文件是：

```text
plan_env/include/plan_env/grid_map.h
plan_env/src/grid_map.cpp
```

在 LIO 启动链路中，`GridMap` 主要订阅：

```text
~grid_map/odom   -> remap 到 odom_topic
~grid_map/cloud  -> remap 到 cloud_topic
```

对应代码：

```text
GridMap::initMap()
  -> indep_odom_sub_ 订阅 "grid_map/odom"
  -> indep_cloud_sub_ 订阅 "grid_map/cloud"

GridMap::odomCallback()
  -> 记录当前机体/传感器位置
  -> 可选固定 z 高度

GridMap::cloudCallback()
  -> 读取 PointCloud2
  -> 只处理局部更新范围内的点
  -> 写入 occupancy_buffer
  -> 按 obstacles_inflation 生成膨胀障碍
```

地图输出主要用于 RViz 和规划器内部碰撞查询：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `{planner_prefix}/drone_0_ego_planner_node/grid_map/occupancy` | `sensor_msgs/PointCloud2` | 原始占据栅格可视化。 |
| `{planner_prefix}/drone_0_ego_planner_node/grid_map/occupancy_inflate` | `sensor_msgs/PointCloud2` | 膨胀后的障碍物可视化，也是规划避障更接近使用的地图。 |

LIO 模式下最常见的问题是 `odom_topic` 与 `cloud_topic` 不在同一个坐标系。EGO 代码默认直接用点云点坐标和 odom 位姿做局部地图更新，如果点云仍在雷达坐标系，而 odom 在世界坐标系，障碍物会出现在错误位置。

## 轨迹消息

EGO 内部轨迹主要使用 `traj_utils/Bspline`：

```text
traj_utils/msg/Bspline.msg
```

字段含义：

| 字段 | 说明 |
| --- | --- |
| `drone_id` | 多机编号。 |
| `order` | B-spline 阶数，当前代码主要使用 3。 |
| `traj_id` | 轨迹编号，用于区分新旧轨迹。 |
| `start_time` | 轨迹开始时间。 |
| `knots` | B-spline knot 序列。 |
| `pos_pts` | B-spline 位置控制点。 |
| `yaw_pts` | yaw 控制点，当前 `traj_server` 中基本未使用。 |
| `yaw_dt` | yaw 轨迹时间间隔，当前链路中不是重点。 |

`EGOReplanFSM::callReboundReplan()` 成功后会把 `local_data_.position_traj_` 转成 `traj_utils/Bspline`，发布给 `traj_server`：

```text
{planner_prefix}/drone_0_planning/bspline
```

多机相关消息：

```text
traj_utils/msg/MultiBsplines.msg
```

字段含义：

| 字段 | 说明 |
| --- | --- |
| `drone_id_from` | 这组轨迹来自哪一架无人机。 |
| `traj` | 多架无人机的 B-spline 轨迹数组。 |

单机实验可以先忽略 `MultiBsplines`，但多机避碰需要保证所有无人机时间同步，并且 `drone_id` 从 0 连续编号。

## position_cmd 输出

`traj_server` 会把 `traj_utils/Bspline` 采样成：

```text
planner_msgs/EgoPositionCommand
```

消息定义在：

```text
planning/third_party_planner_examples/planner_msgs/msg/EgoPositionCommand.msg
```

关键字段：

| 字段 | 说明 |
| --- | --- |
| `position` | 当前时刻期望位置。 |
| `velocity` | 当前时刻期望速度，由 B-spline 一阶导得到。 |
| `acceleration` | 当前时刻期望加速度，由 B-spline 二阶导得到。 |
| `yaw` | 根据轨迹前视方向计算的期望 yaw。 |
| `yaw_dot` | yaw 变化率。 |
| `trajectory_id` | 当前轨迹编号。 |
| `trajectory_flag` | 轨迹状态，正常输出为 `TRAJECTORY_STATUS_READY`。 |

`traj_server` 的发布频率是 100 Hz：

```text
ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);
```

这很重要：Sunray 控制模块中的轨迹命令需要持续发布。`Ego2Sunray_node` 正是把这个持续发布的 `position_cmd` 转成持续发布的 `UAVControlCMD::MOVE_TRAJECTORY`。

## Sunray 适配

适配包路径：

```text
planning/third_party_planner_examples/ego_planner_example/sunray_ego_adapter
```

核心文件：

```text
sunray_ego_adapter/launch/Ego2Sunray.launch
sunray_ego_adapter/src/Ego2Sunray.cpp
```

`Ego2Sunray.launch` 默认拼接：

```text
input_topic  = /<agent_name><agent_id><planner_prefix>/position_cmd
output_topic = /<agent_name><agent_id>/sunray/uav_control/control_cmd
```

`Ego2Sunray.cpp` 转换关系如下：

| EGO 字段 | Sunray 字段 |
| --- | --- |
| `msg->header` | `control_cmd.header` |
| 固定为规划来源 | `cmd_source = UAVControlCMD::PLANNING` |
| 固定为轨迹控制 | `control_cmd = UAVControlCMD::MOVE_TRAJECTORY` |
| `position` | `desired_pos` |
| `velocity` | `desired_vel` |
| `acceleration` | `desired_acc` |
| `yaw` | `desired_yaw` |
| `yaw_dot` | `desired_yaw_rate` |

`use_yaw_rate=false` 时，控制器使用 `SET_YAW`，即跟踪 `desired_yaw`。`use_yaw_rate=true` 时，控制器使用 `SET_YAWRATE`，即跟踪 `desired_yaw_rate`。

## 主要话题清单

以下以 `planner_prefix=/sunray/planning/ego_planner` 为例。

| 方向 | 话题 | 类型 | 来源/去向 | 说明 |
| --- | --- | --- | --- | --- |
| 输入 | `/uav1/sunray/localization/local_odom` | `nav_msgs/Odometry` | localization_fusion -> EGO | 当前位姿和速度。 |
| 输入 | `/velodyne_points` | `sensor_msgs/PointCloud2` | LiDAR/LIO -> EGO | 当前点云，用于局部建图。 |
| 输入 | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz -> EGO | 手动目标点。 |
| 输出 | `/uav1/sunray/planning/ego_planner/drone_0_planning/bspline` | `traj_utils/Bspline` | EGO FSM -> traj_server | 优化后的 B-spline 轨迹。 |
| 输出 | `/uav1/sunray/planning/ego_planner/position_cmd` | `planner_msgs/EgoPositionCommand` | traj_server -> adapter | 100 Hz 位置/速度/加速度/yaw 指令。 |
| 输出 | `/uav1/sunray/uav_control/control_cmd` | `sunray_msgs/UAVControlCMD` | adapter -> UAV control | Sunray 控制模块最终输入。 |
| 可视化 | `/uav1/sunray/planning/ego_planner/drone_0_ego_planner_node/goal_point` | `visualization_msgs/Marker` | EGO -> RViz | 当前目标点。 |
| 可视化 | `/uav1/sunray/planning/ego_planner/drone_0_ego_planner_node/global_list` | `visualization_msgs/Marker` | EGO -> RViz | 全局参考路径。 |
| 可视化 | `/uav1/sunray/planning/ego_planner/drone_0_ego_planner_node/init_list` | `visualization_msgs/Marker` | EGO -> RViz | 初始轨迹。 |
| 可视化 | `/uav1/sunray/planning/ego_planner/drone_0_ego_planner_node/optimal_list` | `visualization_msgs/Marker` | EGO -> RViz | 优化后轨迹。 |
| 可视化 | `/uav1/sunray/planning/ego_planner/drone_0_ego_planner_node/a_star_list` | `visualization_msgs/Marker` | EGO -> RViz | A* 搜索路径。 |
| 可视化 | `/uav1/sunray/planning/ego_planner/drone_0_ego_planner_node/grid_map/occupancy` | `sensor_msgs/PointCloud2` | GridMap -> RViz | 原始占据地图。 |
| 可视化 | `/uav1/sunray/planning/ego_planner/drone_0_ego_planner_node/grid_map/occupancy_inflate` | `sensor_msgs/PointCloud2` | GridMap -> RViz | 膨胀占据地图。 |
| 多机 | `/uav1/sunray/planning/ego_planner/broadcast_bspline` | `traj_utils/Bspline` | EGO <-> EGO | 多机轨迹广播。 |
| 多机 | `/uav1/sunray/planning/ego_planner/drone_0_planning/swarm_trajs` | `traj_utils/MultiBsplines` | EGO <-> EGO | 多机顺序启动轨迹。 |

如果启动时使用默认 `planner_prefix=/sunray/uav_planning_example/ego_planner`，上表中的 `/sunray/planning/ego_planner` 需要替换成 `/sunray/uav_planning_example/ego_planner`。

## 参数如何影响代码

`advanced_param.xml` 中的参数基本都进入 `EGOReplanFSM`、`GridMap`、`EGOPlannerManager` 和 `BsplineOptimizer`。

| 参数组 | 使用代码 | 作用 |
| --- | --- | --- |
| `fsm/*` | `EGOReplanFSM::init()` | 目标输入模式、重规划阈值、规划前视距离、真实实验触发、固定高度。 |
| `grid_map/*` | `GridMap::initMap()` | 地图分辨率、地图范围、点云更新范围、障碍膨胀、占据概率、深度相机参数。 |
| `manager/*` | `EGOPlannerManager::initPlanModules()` | 速度/加速度/jerk 约束、控制点间距、规划前视距离、多机编号、固定高度。 |
| `optimization/*` | `BsplineOptimizer::setParam()` | 平滑、碰撞、可行性、贴合、多机避碰等优化权重。 |
| `bspline/*` | `BsplineOptimizer` / `UniformBspline` | B-spline 速度和加速度限制。 |
| `prediction/*` | `ObjPredictor` | 动态对象预测，当前单机 LIO 链路通常不是重点。 |
| `traj_server/*` | `traj_server.cpp` | yaw 前视时间等输出采样行为。 |

常见调参方向：

| 现象 | 优先检查/修改 |
| --- | --- |
| 轨迹贴障碍太近 | 增大 `obstacles_inflation`、`optimization/dist0` 或检查点云坐标系。 |
| 轨迹太慢 | 增大 `max_vel`，同时确认控制器和机体能力支持。 |
| 轨迹太急或跟踪差 | 降低 `max_acc`、`max_jerk`，或增大 `control_points_distance`。 |
| 规划频繁失败 | 检查 `cloud_topic`、地图范围、障碍膨胀是否过大，必要时关闭或调整 `use_distinctive_trajs`。 |
| RViz 选点后无反应 | 检查 `flight_type=1`、`/move_base_simple/goal`、`odom_topic` 是否正常。 |
| 有轨迹但飞机不动 | 检查 `position_cmd` 是否发布、`Ego2Sunray` 是否订阅同一 `planner_prefix`、`control_cmd` 是否到达控制模块。 |

## 二次开发切入点

如果只是把 EGO 接到 Sunray，优先改 adapter，不要先改 EGO 核心算法。

### 修改 Sunray 输出接口

改这里：

```text
planning/third_party_planner_examples/ego_planner_example/sunray_ego_adapter/src/Ego2Sunray.cpp
```

适合修改：

- `UAVControlCMD` 字段映射。
- yaw 使用 `SET_YAW` 还是 `SET_YAWRATE`。
- 是否附加限幅、状态检查或调试输出。
- 是否根据 `trajectory_flag` 决定继续发布或停止发布。

### 修改 EGO 目标输入

改这里：

```text
plan_manage/src/ego_replan_fsm.cpp
```

重点函数：

| 函数 | 可改内容 |
| --- | --- |
| `waypointCallback()` | RViz 目标点处理、目标高度、目标过滤。 |
| `readGivenWps()` | 预设航点读取。 |
| `planNextWaypoint()` | 全局目标到全局参考轨迹的生成。 |
| `getLocalTarget()` | 从全局轨迹截取局部目标的策略。 |

当前手动目标模式中，`waypointCallback()` 会把目标 z 写成 `1.0`。如果你希望 RViz 目标点的 z 生效，需要关注 `target_type_ == CMD_TARGET` 或修改这里的固定高度逻辑。

### 修改建图输入

改这里：

```text
plan_env/src/grid_map.cpp
```

重点函数：

| 函数 | 可改内容 |
| --- | --- |
| `initMap()` | 订阅模式、地图参数、发布话题。 |
| `odomCallback()` | odom 高度、机体系/传感器系位置处理。 |
| `cloudCallback()` | 点云过滤、局部地图更新、障碍物膨胀。 |
| `raycastProcess()` | 深度图模式下的射线清空和占据更新。 |

LIO 场景通常走 `cloudCallback()`，深度相机场景才重点关注 `projectDepthImage()` 和 `raycastProcess()`。

### 修改轨迹优化

改这里：

```text
bspline_opt/src/bspline_optimizer.cpp
bspline_opt/include/bspline_opt/bspline_optimizer.h
```

适合修改：

- 平滑代价。
- 障碍物距离代价。
- 速度、加速度可行性代价。
- 多机避碰代价。
- `distinctiveTrajs()` 候选轨迹生成策略。

### 修改输出采样

改这里：

```text
plan_manage/src/traj_server.cpp
```

适合修改：

- `cmd_timer` 发布频率。
- 轨迹结束后的悬停行为。
- yaw/yaw_dot 计算方式。
- 输出 `EgoPositionCommand` 的字段。

## 调试顺序

建议按下面顺序排查，不要一上来改算法。

1. 确认定位：

```bash
rostopic hz /uav1/sunray/localization/local_odom
rostopic echo /uav1/sunray/localization/local_odom
```

2. 确认点云：

```bash
rostopic hz /velodyne_points
rostopic echo /velodyne_points/header
```

3. 确认 RViz 目标点：

```bash
rostopic echo /move_base_simple/goal
```

4. 确认 EGO 输出 B-spline：

```bash
rostopic echo /uav1/sunray/planning/ego_planner/drone_0_planning/bspline
```

5. 确认 `traj_server` 输出：

```bash
rostopic hz /uav1/sunray/planning/ego_planner/position_cmd
rostopic echo /uav1/sunray/planning/ego_planner/position_cmd
```

6. 确认 Sunray adapter 输出：

```bash
rostopic hz /uav1/sunray/uav_control/control_cmd
rostopic echo /uav1/sunray/uav_control/control_cmd
```

7. 确认控制模块收到 `MOVE_TRAJECTORY`：

```bash
rostopic echo /uav1/sunray/uav_control/control_state
```

如果第 4 步没有输出，问题通常在 EGO 内部输入、目标、建图或规划参数。  
如果第 4 步有输出但第 5 步没有输出，重点看 `traj_server` 是否启动、`planner_prefix` 是否一致。  
如果第 5 步有输出但第 6 步没有输出，重点看 `Ego2Sunray.launch` 的 `input_topic`。  
如果第 6 步有输出但飞机不动，重点回到 `sunray_uav_control` 检查控制模式、解锁、offboard 和控制器状态。

</section>
