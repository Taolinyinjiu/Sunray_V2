# sunray_swarm_control

`sunray_swarm_control` 是 Sunray 的集群编队控制包。它位于定位模块和单机控制模块之间，负责接收集群指令、生成每个智能体的阵型目标点、调用 ORCA 计算避碰速度，并分别向 `sunray_uav_control` 和 `sunray_ugv_control` 发布单机控制指令。

本包当前同时支持 UAV 和 UGV。两类平台共用 `formation` 阵型生成库和 `ORCA` 避障库，但各自有独立的 ROS 状态机节点、监控节点、RViz 可视化节点和指令发布工具。

## 1. 模块边界

本包负责的内容：

- 接收外部集群指令，例如起飞、降落、悬停、返航、静态阵型、动态阵型。
- 根据 `sunray_msgs/Formation` 计算每个 `agent_id` 对应的目标位置和目标 yaw。
- 读取本机和邻居的 `local_odom`，判断输入是否就绪。
- 调用 ORCA 在 XY 平面计算避碰后的速度指令。
- 将集群控制结果转换为 `UAVControlCMD` 或 `UGVControlCMD`。
- 发布 `UAVSwarmState` 或 `UGVSwarmState`，供 monitor、RViz 和 Qt 面板使用。

本包不负责的内容：

- 不负责单机底层姿态、位置、速度闭环控制。
- 不负责定位融合。
- 不负责通信链路本身，只假设每个智能体的 `local_odom` 能被当前节点订阅到。
- 不直接控制电机、PX4、底盘驱动器或仿真动力学。

## 2. 总体架构

```text
外部指令源
  terminal / formation_tui / Qt panel / 其他上层模块
        |
        v
/sunray/swarm/uav_swarm_cmd 或 /sunray/swarm/ugv_swarm_cmd
        |
        v
swarm_control_uav_node 或 swarm_control_ugv_node
        |
        +--> 读取本机 local_odom
        +--> 读取邻居 local_odom
        +--> formation 计算目标点
        +--> ORCA 计算 XY 避碰速度
        |
        v
/{agent_name}{agent_id}/sunray/uav_control_cmd
或
/{agent_name}{agent_id}/sunray/ugv_control/control_cmd
        |
        v
sunray_uav_control 或 sunray_ugv_control
```

状态监控链路：

```text
swarm_control_uav_node / swarm_control_ugv_node
        |
        v
/sunray/swarm/uav_swarm_state 或 /sunray/swarm/ugv_swarm_state
        |
        +--> swarm_control_uav_monitor_node / swarm_control_ugv_monitor_node
        +--> rviz_visualization_uav_node / rviz_visualization_ugv_node
        +--> swarm_control_panel_node
```

## 3. 目录结构

```text
swarm/
├── ORCA/
│   ├── ORCA.h / ORCA.cpp          # 新版 ORCA 封装入口，业务代码只应调用这一层
│   ├── orca_lib/                  # 原生 RVO2/ORCA 源码，不建议业务开发时直接修改
│   └── test/orca_test.cpp         # ORCA + formation 压力测试节点
├── formation/
│   ├── formation.h
│   └── formation.cpp              # 阵型目标点生成库，不发布/订阅 ROS 话题
├── swarm_control_uav/
│   ├── swarm_control_uav.h
│   ├── swarm_control_uav.cpp      # UAV 集群控制状态机
│   └── swarm_control_uav_node.cpp # UAV ROS main 入口
├── swarm_control_ugv/
│   ├── swarm_control_ugv.h
│   ├── swarm_control_ugv.cpp      # UGV 集群控制状态机
│   └── swarm_control_ugv_node.cpp # UGV ROS main 入口
├── utils/
│   ├── uav_swarm_cmd_pub_terminal.cpp
│   ├── ugv_swarm_cmd_pub_terminal.cpp
│   ├── formation_tui.cpp
│   ├── swarm_control_uav_monitor_node.cpp
│   ├── swarm_control_ugv_monitor_node.cpp
│   ├── rviz_visualization_uav_node.cpp
│   ├── rviz_visualization_ugv_node.cpp
│   ├── swarm_control_panel_node.cpp # Qt 集群控制面板
│   └── meshes/
├── launch/                          # 仿真启动、工具启动、测试启动
├── rviz/                            # RViz 配置
├── example/
├── CMakeLists.txt
└── package.xml
```

## 4. 全局命名约定

`agent_name` 表示平台名前缀，例如 `uav` 或 `ugv`。

`agent_id` 表示本机编号，从 1 开始。

`swarm_num` 表示当前集群规模。

单机话题前缀统一写作：

```text
/{agent_name}{agent_id}
```

例如：

```text
/uav1
/uav2
/ugv1
/ugv6
```

集群指令消息中的 `agent_id` 是目标智能体 ID。该字段等于 `99` 时表示广播，所有智能体都会响应该命令。

## 5. 节点说明

本章按 ROS Wiki 风格描述每个节点的功能、订阅话题、发布话题和参数。

### 5.1 `swarm_control_uav_node`

功能：

`swarm_control_uav_node` 是单架 UAV 的集群控制节点。每个无人机启动一个该节点。节点接收 `UAVSwarmCMD`，根据本机 ID 计算目标点，调用 ORCA 得到 XY 避碰速度，并通过 `UAVControlCMD.fixed_height` 交给 `sunray_uav_control` 锁定 Z 轴高度。

Subscribed Topics：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/{agent_name}{agent_id}/sunray/localization/local_odom` | `nav_msgs/Odometry` | 本机局部里程计，来自 `localization_fusion` |
| `/{agent_name}{agent_id}/sunray/uav_control/control_state` | `sunray_msgs/UAVControlState` | 本机 UAV 控制状态机状态，来自 `sunray_uav_control` |
| `/{agent_name}{peer_id}/sunray/localization/local_odom` | `nav_msgs/Odometry` | 邻居里程计，`peer_id` 范围为 `[1, swarm_num]` 且不等于本机 ID |
| `/sunray/swarm/uav_swarm_cmd` | `sunray_msgs/UAVSwarmCMD` | UAV 集群控制指令 |

Published Topics：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/{agent_name}{agent_id}/sunray/uav_control_cmd` | `sunray_msgs/UAVControlCMD` | 下发给单机 UAV 控制器的控制指令 |
| `/sunray/swarm/uav_swarm_state` | `sunray_msgs/UAVSwarmState` | 本机集群控制状态，供监控、RViz、Qt 面板使用 |

Parameters：

| 参数 | 类型 | 默认值 | 单位 | 说明 |
| --- | --- | --- | --- | --- |
| `agent_name` | `string` | `uav` | - | 智能体名前缀 |
| `agent_id` | `int` | `1` | - | 本机 ID，从 1 开始 |
| `swarm_num` | `int` | `1` | - | 集群数量 |
| `control_loop_hz` | `double` | `50.0` | Hz | 集群控制主循环频率 |
| `peer_odom_timeout` | `double` | `0.1` | s | 邻居 odom 超时时间 |
| `dynamic_prepare_wait_time` | `double` | `2.0` | s | 动态阵型全体到达 t=0 初始点后的等待时间 |
| `goal_xy_tolerance` | `double` | `0.1` | m | XY 平面到达阈值 |
| `goal_z_tolerance` | `double` | `0.1` | m | 高度到达阈值 |
| `goal_yaw_tolerance` | `double` | `0.1` | rad | yaw 到达阈值 |
| `field/x_min` | `double` | `-50.0` | m | 阵型目标点 X 下界 |
| `field/x_max` | `double` | `50.0` | m | 阵型目标点 X 上界 |
| `field/y_min` | `double` | `-50.0` | m | 阵型目标点 Y 下界 |
| `field/y_max` | `double` | `50.0` | m | 阵型目标点 Y 上界 |
| `field/z_min` | `double` | `0.0` | m | 阵型目标点 Z 下界 |
| `field/z_max` | `double` | `3.0` | m | 阵型目标点 Z 上界 |
| `orca/neighbor_dist` | `double` | `2.0` | m | ORCA 邻居搜索距离 |
| `orca/time_horizon` | `double` | `2.0` | s | ORCA 预测时间窗口 |
| `orca/radius` | `double` | `0.2` | m | 智能体等效避碰半径 |
| `orca/max_speed` | `double` | `1.0` | m/s | ORCA 输出最大 XY 平面速度 |
| `orca/max_neighbors` | `int` | `-1` | - | ORCA 最大邻居数，`-1` 表示自动使用全集群 |
| `static_obstacles/enabled` | `bool` | `false` | - | 是否启用圆形静态障碍物 |
| `static_obstacles/x` | `double[]` | `[]` | m | 圆形障碍物圆心 X |
| `static_obstacles/y` | `double[]` | `[]` | m | 圆形障碍物圆心 Y |
| `static_obstacles/radius` | `double[]` | `[]` | m | 圆形障碍物半径 |

### 5.2 `swarm_control_ugv_node`

功能：

`swarm_control_ugv_node` 是单辆 UGV 的集群控制节点。每辆无人车启动一个该节点。节点接收 `UGVSwarmCMD`，计算目标点，调用 ORCA 得到世界系 XY 速度，最终发布 `UGVControlCMD` 给 `sunray_ugv_control`。

Subscribed Topics：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/{agent_name}{agent_id}/sunray/localization/local_odom` | `nav_msgs/Odometry` | 本机局部里程计，来自 `localization_fusion` |
| `/{agent_name}{agent_id}/sunray/ugv_control/ugv_control_fsm_state` | `sunray_msgs/UGVControlFSMState` | 本机 UGV 控制状态机状态 |
| `/{agent_name}{peer_id}/sunray/localization/local_odom` | `nav_msgs/Odometry` | 邻居里程计，`peer_id` 范围为 `[1, swarm_num]` 且不等于本机 ID |
| `/sunray/swarm/ugv_swarm_cmd` | `sunray_msgs/UGVSwarmCMD` | UGV 集群控制指令 |

Published Topics：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/{agent_name}{agent_id}/sunray/ugv_control/control_cmd` | `sunray_msgs/UGVControlCMD` | 下发给单机 UGV 控制器的控制指令 |
| `/sunray/swarm/ugv_swarm_state` | `sunray_msgs/UGVSwarmState` | 本机集群控制状态，供监控、RViz、Qt 面板使用 |

Parameters：

| 参数 | 类型 | 默认值 | 单位 | 说明 |
| --- | --- | --- | --- | --- |
| `agent_name` | `string` | `ugv` | - | 智能体名前缀 |
| `agent_id` | `int` | `1` | - | 本机 ID，从 1 开始 |
| `swarm_num` | `int` | `1` | - | 集群数量 |
| `control_loop_hz` | `double` | `50.0` | Hz | 集群控制主循环频率 |
| `peer_odom_timeout` | `double` | `0.1` | s | 邻居 odom 超时时间 |
| `dynamic_prepare_wait_time` | `double` | `2.0` | s | 动态阵型全体到达 t=0 初始点后的等待时间 |
| `goal_xy_tolerance` | `double` | `0.1` | m | XY 平面到达阈值 |
| `goal_yaw_tolerance` | `double` | `0.1` | rad | yaw 到达阈值 |
| `field/x_min` | `double` | `-50.0` | m | 阵型目标点 X 下界 |
| `field/x_max` | `double` | `50.0` | m | 阵型目标点 X 上界 |
| `field/y_min` | `double` | `-50.0` | m | 阵型目标点 Y 下界 |
| `field/y_max` | `double` | `50.0` | m | 阵型目标点 Y 上界 |
| `field/z_min` | `double` | `-1.0` | m | 阵型目标点 Z 下界 |
| `field/z_max` | `double` | `1.0` | m | 阵型目标点 Z 上界 |
| `orca/neighbor_dist` | `double` | `2.0` | m | ORCA 邻居搜索距离 |
| `orca/time_horizon` | `double` | `2.0` | s | ORCA 预测时间窗口 |
| `orca/radius` | `double` | `0.3` | m | 无人车等效避碰半径 |
| `orca/max_speed` | `double` | `0.8` | m/s | ORCA 输出最大 XY 平面速度 |
| `orca/max_neighbors` | `int` | `-1` | - | ORCA 最大邻居数，`-1` 表示自动使用全集群 |
| `static_obstacles/enabled` | `bool` | `false` | - | 是否启用圆形静态障碍物 |
| `static_obstacles/x` | `double[]` | `[]` | m | 圆形障碍物圆心 X |
| `static_obstacles/y` | `double[]` | `[]` | m | 圆形障碍物圆心 Y |
| `static_obstacles/radius` | `double[]` | `[]` | m | 圆形障碍物半径 |

### 5.3 `swarm_control_uav_monitor_node`

功能：

集中订阅所有 UAV 的 `UAVSwarmState`，在一个终端中刷新显示 UAV 集群控制状态，避免多节点同时打印导致终端刷屏。面板按“基本状态、集群位姿、集群输入、控制输出”分段显示，并在话题名后标注“订阅/发布”方向。

Subscribed Topics：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/sunray/swarm/uav_swarm_state` | `sunray_msgs/UAVSwarmState` | UAV 集群状态 |

Parameters：

| 参数 | 类型 | 默认值 | 单位 | 说明 |
| --- | --- | --- | --- | --- |
| `swarm_state_topic` | `string` | `/sunray/swarm/uav_swarm_state` | - | 订阅的 UAV 集群状态话题 |
| `agent_name` | `string` | `uav` | - | 显示时使用的智能体名前缀 |
| `display_hz` | `double` | `1.0` | Hz | 终端刷新频率 |
| `stale_timeout` | `double` | `1.0` | s | 状态超时判断阈值 |
| `clear_screen` | `bool` | `true` | - | 是否每次刷新前清屏 |

### 5.4 `swarm_control_ugv_monitor_node`

功能：

集中订阅所有 UGV 的 `UGVSwarmState`，在一个终端中刷新显示 UGV 集群控制状态。面板按“基本状态、集群位姿、集群输入、控制输出”分段显示，并在话题名后标注“订阅/发布”方向。该节点会按照当前车辆 ID 排除自身，显示邻居 odom 话题范围，例如 `/ugv[1-2,4-6]/sunray/localization/local_odom`。

Subscribed Topics：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/sunray/swarm/ugv_swarm_state` | `sunray_msgs/UGVSwarmState` | UGV 集群状态 |

Parameters：

| 参数 | 类型 | 默认值 | 单位 | 说明 |
| --- | --- | --- | --- | --- |
| `swarm_state_topic` | `string` | `/sunray/swarm/ugv_swarm_state` | - | 订阅的 UGV 集群状态话题 |
| `agent_name` | `string` | `ugv` | - | 显示时使用的智能体名前缀 |
| `display_hz` | `double` | `1.0` | Hz | 终端刷新频率 |
| `stale_timeout` | `double` | `1.0` | s | 状态超时判断阈值 |
| `clear_screen` | `bool` | `true` | - | 是否每次刷新前清屏 |

### 5.5 `rviz_visualization_uav_node`

功能：

订阅 `UAVSwarmState`，将 UAV 集群状态转换为 `visualization_msgs/MarkerArray`，用于 RViz 中显示无人机模型、速度、ID、任务文字、轨迹、目标点、静态障碍物和 world 原点坐标系。

Subscribed Topics：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/sunray/swarm/uav_swarm_state` | `sunray_msgs/UAVSwarmState` | UAV 集群状态 |

Published Topics：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/sunray/swarm/uav_rviz_markers` | `visualization_msgs/MarkerArray` | RViz marker 数组 |

Parameters：

| 参数 | 类型 | 默认值 | 单位 | 说明 |
| --- | --- | --- | --- | --- |
| `swarm_state_topic` | `string` | `/sunray/swarm/uav_swarm_state` | - | 订阅的 UAV 集群状态话题 |
| `agent_name` | `string` | `uav` | - | RViz 文本和 marker namespace 使用的前缀 |
| `marker_topic` | `string` | `/sunray/swarm/uav_rviz_markers` | - | 发布的 MarkerArray 话题 |
| `frame_id` | `string` | `world` | - | marker 坐标系 |
| `mesh_resource` | `string` | `package://sunray_swarm_control/utils/meshes/uav.dae` | - | UAV mesh 路径 |
| `publish_hz` | `double` | `10.0` | Hz | marker 发布频率 |
| `stale_timeout` | `double` | `1.0` | s | 状态超时后删除该 agent marker |
| `trail_size` | `int` | `50` | 帧 | 轨迹最多保留的历史点数量 |
| `mesh_scale` | `double` | `1.0` | - | mesh 缩放 |
| `velocity_scale` | `double` | `1.0` | - | 速度箭头缩放 |
| `text_height` | `double` | `0.35` | m | 文本 marker 高度 |
| `static_obstacles/enabled` | `bool` | `false` | - | 是否显示静态障碍物 |
| `static_obstacles/x` | `double[]` | `[]` | m | 障碍物圆心 X |
| `static_obstacles/y` | `double[]` | `[]` | m | 障碍物圆心 Y |
| `static_obstacles/radius` | `double[]` | `[]` | m | 障碍物半径 |
| `static_obstacle_height` | `double` | `0.10` | m | 障碍物圆柱高度 |
| `static_obstacle_alpha` | `double` | `0.42` | - | 障碍物透明度 |

### 5.6 `rviz_visualization_ugv_node`

功能：

订阅 `UGVSwarmState`，将 UGV 集群状态转换为 `visualization_msgs/MarkerArray`，用于 RViz 中显示无人车模型、速度、ID、任务文字、轨迹、目标点、静态障碍物和 world 原点坐标系。

Subscribed Topics：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/sunray/swarm/ugv_swarm_state` | `sunray_msgs/UGVSwarmState` | UGV 集群状态 |

Published Topics：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/sunray/swarm/ugv_rviz_markers` | `visualization_msgs/MarkerArray` | RViz marker 数组 |

Parameters：

| 参数 | 类型 | 默认值 | 单位 | 说明 |
| --- | --- | --- | --- | --- |
| `swarm_state_topic` | `string` | `/sunray/swarm/ugv_swarm_state` | - | 订阅的 UGV 集群状态话题 |
| `agent_name` | `string` | `ugv` | - | RViz 文本和 marker namespace 使用的前缀 |
| `marker_topic` | `string` | `/sunray/swarm/ugv_rviz_markers` | - | 发布的 MarkerArray 话题 |
| `frame_id` | `string` | `world` | - | marker 坐标系 |
| `mesh_resource` | `string` | `package://sunray_swarm_control/utils/meshes/ugv.dae` | - | UGV mesh 路径 |
| `publish_hz` | `double` | `10.0` | Hz | marker 发布频率 |
| `stale_timeout` | `double` | `1.0` | s | 状态超时后删除该 agent marker |
| `trail_size` | `int` | `50` | 帧 | 轨迹最多保留的历史点数量 |
| `mesh_scale` | `double` | `1.0` | - | mesh 缩放 |
| `velocity_scale` | `double` | `1.0` | - | 速度箭头缩放 |
| `text_height` | `double` | `0.28` | m | 文本 marker 高度 |
| `static_obstacles/enabled` | `bool` | `false` | - | 是否显示静态障碍物 |
| `static_obstacles/x` | `double[]` | `[]` | m | 障碍物圆心 X |
| `static_obstacles/y` | `double[]` | `[]` | m | 障碍物圆心 Y |
| `static_obstacles/radius` | `double[]` | `[]` | m | 障碍物半径 |
| `static_obstacle_height` | `double` | `0.08` | m | 障碍物圆柱高度 |
| `static_obstacle_alpha` | `double` | `0.42` | - | 障碍物透明度 |

### 5.7 `uav_swarm_cmd_pub_terminal`

功能：

行式终端交互工具，用于发布 `UAVSwarmCMD`。默认目标 ID 为 `99`，即广播到所有 UAV。

Published Topics：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/sunray/swarm/uav_swarm_cmd` | `sunray_msgs/UAVSwarmCMD` | UAV 集群控制指令 |

Parameters：

| 参数 | 类型 | 默认值 | 单位 | 说明 |
| --- | --- | --- | --- | --- |
| `swarm_cmd_topic` | `string` | `/sunray/swarm/uav_swarm_cmd` | - | 发布的集群指令话题 |
| `default_target_agent_id` | `int` | `99` | - | 默认目标 ID，`99` 表示广播 |
| `default_formation_type` | `int` | `1` | - | 默认阵型类型 |
| `default_leader_x` | `double` | `0.0` | m | 默认虚拟 leader X |
| `default_leader_y` | `double` | `0.0` | m | 默认虚拟 leader Y |
| `default_leader_z` | `double` | `1.5` | m | 默认虚拟 leader Z |
| `default_leader_yaw` | `double` | `0.0` | rad | 默认虚拟 leader yaw |
| `default_dynamic_time` | `double` | `10.0` | s | 默认动态阵型持续时间 |
| `default_static_line_spacing` | `double` | `1.5` | m | 默认静态直线阵型间距 |
| `default_static_line_angle` | `double` | `0.0` | deg | 默认静态直线阵型角度 |
| `default_static_polygon_spacing` | `double` | `2.0` | m | 默认静态多边形边长 |
| `default_dynamic_ring_radius` | `double` | `2.0` | m | 默认动态圆环半径 |
| `default_dynamic_ring_move_speed` | `double` | `0.5` | m/s | 默认动态圆环移动速度 |
| `default_dynamic_polygon_spacing` | `double` | `2.0` | m | 默认动态多边形边长 |
| `default_dynamic_polygon_move_speed` | `double` | `0.5` | m/s | 默认动态多边形移动速度 |
| `default_dynamic_lemniscate_x_radius` | `double` | `3.0` | m | 默认 8 字轨迹 X 尺度 |
| `default_dynamic_lemniscate_y_radius` | `double` | `2.0` | m | 默认 8 字轨迹 Y 尺度 |
| `default_dynamic_lemniscate_move_speed` | `double` | `0.5` | m/s | 默认 8 字轨迹移动速度 |

### 5.8 `ugv_swarm_cmd_pub_terminal`

功能：

行式终端交互工具，用于发布 `UGVSwarmCMD`。默认目标 ID 为 `99`，即广播到所有 UGV。

Published Topics：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/sunray/swarm/ugv_swarm_cmd` | `sunray_msgs/UGVSwarmCMD` | UGV 集群控制指令 |

Parameters：

| 参数 | 类型 | 默认值 | 单位 | 说明 |
| --- | --- | --- | --- | --- |
| `swarm_cmd_topic` | `string` | `/sunray/swarm/ugv_swarm_cmd` | - | 发布的集群指令话题 |
| `default_target_agent_id` | `int` | `99` | - | 默认目标 ID，`99` 表示广播 |
| `default_formation_type` | `int` | `1` | - | 默认阵型类型 |
| `default_leader_x` | `double` | `0.0` | m | 默认虚拟 leader X |
| `default_leader_y` | `double` | `0.0` | m | 默认虚拟 leader Y |
| `default_leader_z` | `double` | `0.0` | m | 默认虚拟 leader Z |
| `default_leader_yaw` | `double` | `0.0` | rad | 默认虚拟 leader yaw |
| `default_dynamic_time` | `double` | `10.0` | s | 默认动态阵型持续时间 |
| `default_static_line_spacing` | `double` | `1.5` | m | 默认静态直线阵型间距 |
| `default_static_line_angle` | `double` | `0.0` | deg | 默认静态直线阵型角度 |
| `default_static_polygon_spacing` | `double` | `2.0` | m | 默认静态多边形边长 |
| `default_dynamic_ring_radius` | `double` | `2.0` | m | 默认动态圆环半径 |
| `default_dynamic_ring_move_speed` | `double` | `0.4` | m/s | 默认动态圆环移动速度 |
| `default_dynamic_polygon_spacing` | `double` | `2.0` | m | 默认动态多边形边长 |
| `default_dynamic_polygon_move_speed` | `double` | `0.4` | m/s | 默认动态多边形移动速度 |
| `default_dynamic_lemniscate_x_radius` | `double` | `3.0` | m | 默认 8 字轨迹 X 尺度 |
| `default_dynamic_lemniscate_y_radius` | `double` | `2.0` | m | 默认 8 字轨迹 Y 尺度 |
| `default_dynamic_lemniscate_move_speed` | `double` | `0.4` | m/s | 默认 8 字轨迹移动速度 |

### 5.9 `formation_tui`

功能：

基于 ncurses 的终端阵型编辑工具，主要用于快速编辑阵型参数、选择自定义 offset 并发布 `UAVSwarmCMD`。当前默认面向 UAV 指令话题，可通过参数切换发布目标话题。

Published Topics：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/sunray/swarm/uav_swarm_cmd` | `sunray_msgs/UAVSwarmCMD` | 默认发布的 UAV 集群控制指令 |

Parameters：

| 参数 | 类型 | 默认值 | 单位 | 说明 |
| --- | --- | --- | --- | --- |
| `swarm_cmd_topic` | `string` | `/sunray/swarm/uav_swarm_cmd` | - | 发布的集群指令话题 |
| `default_target_agent_id` | `int` | `99` | - | 默认目标 ID，`99` 表示广播 |
| `swarm_num` | `int` | `3` | - | 默认自定义阵型数量 |
| `grid_resolution` | `double` | `1.0` | m | TUI 网格分辨率 |
| `default_leader_x/y/z/yaw` | `double` | 见代码默认值 | m/rad | 默认虚拟 leader 位姿 |
| `default_dynamic_time` | `double` | `10.0` | s | 默认动态阵型持续时间 |
| `default_static_line_spacing` | `double` | `1.5` | m | 默认静态直线阵型间距 |
| `default_static_line_angle` | `double` | `0.0` | deg | 默认静态直线阵型角度 |
| `default_static_polygon_spacing` | `double` | `2.0` | m | 默认静态多边形边长 |
| `default_dynamic_ring_radius` | `double` | `2.0` | m | 默认动态圆环半径 |
| `default_dynamic_ring_move_speed` | `double` | `0.5` | m/s | 默认动态圆环移动速度 |
| `default_dynamic_polygon_spacing` | `double` | `2.0` | m | 默认动态多边形边长 |
| `default_dynamic_polygon_move_speed` | `double` | `0.5` | m/s | 默认动态多边形移动速度 |
| `default_dynamic_lemniscate_x_radius` | `double` | `3.0` | m | 默认 8 字轨迹 X 尺度 |
| `default_dynamic_lemniscate_y_radius` | `double` | `2.0` | m | 默认 8 字轨迹 Y 尺度 |
| `default_dynamic_lemniscate_move_speed` | `double` | `0.5` | m/s | 默认 8 字轨迹移动速度 |

### 5.10 `swarm_control_panel_node`

功能：

Qt Widgets 集群控制面板。它可以发布 UAV/UGV 集群指令，并订阅 UAV/UGV 集群状态，在界面中显示在线状态、FSM、目标点和关键状态。适合替代 terminal 工具作为主要调试入口。

Subscribed Topics：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/sunray/swarm/uav_swarm_state` | `sunray_msgs/UAVSwarmState` | UAV 集群状态 |
| `/sunray/swarm/ugv_swarm_state` | `sunray_msgs/UGVSwarmState` | UGV 集群状态 |

Published Topics：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/sunray/swarm/uav_swarm_cmd` | `sunray_msgs/UAVSwarmCMD` | UAV 集群控制指令 |
| `/sunray/swarm/ugv_swarm_cmd` | `sunray_msgs/UGVSwarmCMD` | UGV 集群控制指令 |

Parameters：

| 参数 | 类型 | 默认值 | 单位 | 说明 |
| --- | --- | --- | --- | --- |
| `uav_swarm_cmd_topic` | `string` | `/sunray/swarm/uav_swarm_cmd` | - | UAV 指令发布话题 |
| `ugv_swarm_cmd_topic` | `string` | `/sunray/swarm/ugv_swarm_cmd` | - | UGV 指令发布话题 |
| `uav_swarm_state_topic` | `string` | `/sunray/swarm/uav_swarm_state` | - | UAV 状态订阅话题 |
| `ugv_swarm_state_topic` | `string` | `/sunray/swarm/ugv_swarm_state` | - | UGV 状态订阅话题 |
| `state_timeout` | `double` | `1.0` | s | 状态超时判断阈值 |

### 5.11 `orca_test`

功能：

ORCA + formation 压力测试节点。该节点不参与业务话题通信，只读取参数、构造多组阵型切换测试、调用 formation 和 ORCA，并在终端输出测试结果。它用于验证 ORCA 在集群阵型切换、目标互换、密集队形等情况下是否容易出现卡死或碰撞风险。

Parameters：

| 参数 | 类型 | 默认值 | 单位 | 说明 |
| --- | --- | --- | --- | --- |
| `orca_test/transition_case_count` | `int` | `100` | 组 | 随机/组合阵型切换测试数量 |
| `orca_test/print_failed_detail` | `bool/int` | `1` | - | 是否打印失败用例细节 |
| `orca/*` | 多种 | 见 YAML | - | 与仿真节点保持一致的 ORCA 参数 |
| `field/*` | 多种 | 见 YAML | - | 与仿真节点保持一致的场地参数 |

## 6. 自定义消息

本包主要使用 `common/sunray_msgs/msg` 中的自定义消息。

### 6.1 `sunray_msgs/UAVSwarmCMD`

用途：

UAV 集群控制指令，通常由 terminal、TUI、Qt 面板或其他上层模块发布。

关键字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 消息时间戳 |
| `cmd_source` | `uint8` | 命令来源，`GROUND_STATION=1`，`TERMINAL=2` |
| `agent_id` | `uint8` | 目标智能体 ID，`99` 表示广播 |
| `swarm_cmd` | `uint8` | 集群命令 |
| `formation_cmd` | `sunray_msgs/Formation` | 阵型参数，仅 `SWARM_FORMATION` 时使用 |

命令枚举：

| 枚举 | 含义 |
| --- | --- |
| `SWARM_TAKEOFF=1` | UAV 原地起飞 |
| `SWARM_LAND=2` | UAV 原地降落 |
| `SWARM_HOVER=3` | UAV 当前点悬停 |
| `SWARM_RETURN=4` | UAV 返回记录的返航点 |
| `SWARM_FORMATION=5` | 执行 formation 指定的阵型控制 |

### 6.2 `sunray_msgs/UGVSwarmCMD`

用途：

UGV 集群控制指令，通常由 terminal、Qt 面板或其他上层模块发布。

关键字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 消息时间戳 |
| `cmd_source` | `uint8` | 命令来源，`GROUND_STATION=1`，`TERMINAL=2` |
| `agent_id` | `uint8` | 目标智能体 ID，`99` 表示广播 |
| `swarm_cmd` | `uint8` | 集群命令 |
| `formation_cmd` | `sunray_msgs/Formation` | 阵型参数，仅 `SWARM_FORMATION` 时使用 |

命令枚举：

| 枚举 | 含义 |
| --- | --- |
| `SWARM_HOLD=1` | UGV 当前点保持 |
| `SWARM_RETURN=2` | UGV 返回记录的返航点 |
| `SWARM_FORMATION=3` | 执行 formation 指定的阵型控制 |

### 6.3 `sunray_msgs/Formation`

用途：

描述阵型目标点生成所需的所有参数。该消息由集群指令携带，传入 `formation` 类后生成每个 agent 的目标 `x/y/z/yaw`。

消息源码结构：

```text
std_msgs/Header header

uint8 STATIC_KEEP_FORMATION=0
uint8 STATIC_FORMATION_LINE=1
uint8 STATIC_FORMATION_POLYGON=2
uint8 STATIC_FORMATION_RANDOM=3
uint8 STATIC_FORMATION_CUSTOM=9

uint8 DYNAMIC_FORMATION_RING=11
uint8 DYNAMIC_FORMATION_POLYGON=12
uint8 DYNAMIC_FORMATION_LEMNISCATE=13

uint8 formation_type
geometry_msgs/Point leader_pos
float32 leader_yaw
float32 dynamic_time

float32 static_line_spacing
float32 static_line_angle
float32 static_polygon_spacing
geometry_msgs/Point[] custom_offsets_pos
float32[] custom_offsets_yaw

float32 dynamic_ring_radius
float32 dynamic_ring_move_speed
float32 dynamic_polygon_spacing
float32 dynamic_polygon_move_speed
float32 dynamic_lemniscate_x_radius
float32 dynamic_lemniscate_y_radius
float32 dynamic_lemniscate_move_speed
```

公共约定：

| 名称 | 含义 |
| --- | --- |
| `leader_pos` | 虚拟 leader 的世界系位置，也是阵型参考点 |
| `leader_yaw` | 虚拟 leader 的 yaw；静态阵型中作为目标 yaw 参考，部分动态阵型中作为初始相位 |
| `orca_radius_` | `orca/radius`，智能体等效避碰半径 |
| `minSafeDistance()` | 最小安全距离，当前为 `3 * orca_radius_` |
| `maxSafeDistance()` | 最大安全距离，当前为 `10 * orca_radius_` |
| `isSpacingInSafeRange()` | 判断间距是否位于 `[minSafeDistance(), maxSafeDistance()]` |
| `orca_max_speed_` | `orca/max_speed`，ORCA 输出最大平面速度 |
| 场地限制 | 最终目标点必须位于 `field/x_min~x_max`、`field/y_min~y_max`、`field/z_min~z_max` 内 |
| 障碍物限制 | 若启用圆形静态障碍物，目标点不能进入 `obstacle_radius + orca_radius_` 范围 |

#### 6.3.1 `STATIC_KEEP_FORMATION`

示意图：

```text
抓拍时刻：          执行阵型后：

  A2      A3          A2'     A3'
      C                   L
 A1      A4          A1'     A4'

C = 抓拍得到的几何中心
L = 新的虚拟 leader_pos
```

| 项 | 说明 |
| --- | --- |
| 枚举 | `STATIC_KEEP_FORMATION=0` |
| 相关参数 | `leader_pos`、`leader_yaw` |
| 几何含义 | 抓拍当前全集群相对位置关系，然后把这组 offset 整体平移到新的虚拟 leader 附近 |
| 安全限制 | 不检查智能体之间的距离，只检查最终目标点是否在场地内、是否避开圆形静态障碍物 |
| 速度限制 | 无独立速度参数，移动过程由 ORCA 的 `orca_max_speed_` 限制 |

#### 6.3.2 `STATIC_FORMATION_LINE`

示意图：

```text
angle = 0 deg:

A1 ---- A2 ---- L ---- A3 ---- A4
        <--- spacing --->

angle = 90 deg 时，整条直线沿世界系 Y 轴展开。
```

| 项 | 说明 |
| --- | --- |
| 枚举 | `STATIC_FORMATION_LINE=1` |
| 相关参数 | `static_line_spacing`、`static_line_angle`、`leader_pos`、`leader_yaw` |
| 几何含义 | 所有智能体沿一条直线等间距分布，虚拟 leader 位于队列几何中心 |
| 角度定义 | `static_line_angle` 单位为 deg，表示直线方向与世界系 X 轴夹角 |
| 安全限制 | `1 <= swarm_num <= 10`，`isSpacingInSafeRange(static_line_spacing)` 必须为 true |
| 速度限制 | 无独立速度参数，移动过程由 ORCA 的 `orca_max_speed_` 限制 |

#### 6.3.3 `STATIC_FORMATION_POLYGON`

示意图：

```text
N = 6:

      A1
   A6    A2
      L
   A5    A3
      A4

相邻顶点距离 = static_polygon_spacing
```

| 项 | 说明 |
| --- | --- |
| 枚举 | `STATIC_FORMATION_POLYGON=2` |
| 相关参数 | `static_polygon_spacing`、`leader_pos`、`leader_yaw` |
| 几何含义 | `swarm_num=N` 时，所有智能体位于正 N 边形顶点，虚拟 leader 位于几何中心 |
| 安全限制 | `3 <= swarm_num <= 10`，`isSpacingInSafeRange(static_polygon_spacing)` 必须为 true |
| 速度限制 | 无独立速度参数，移动过程由 ORCA 的 `orca_max_speed_` 限制 |

#### 6.3.4 `STATIC_FORMATION_RANDOM`

示意图：

```text
场地安全范围内随机生成：

+-----------------------+
|   A2          A5      |
|        A1             |
|             L         |
| A4              A3    |
+-----------------------+
```

| 项 | 说明 |
| --- | --- |
| 枚举 | `STATIC_FORMATION_RANDOM=3` |
| 相关参数 | `leader_pos`、`leader_yaw`、消息时间戳 |
| 几何含义 | 在场地安全范围内随机生成每个 agent 的最终目标点，再转换成相对虚拟 leader 的 offset |
| 安全限制 | `1 <= swarm_num <= 10`，任意两个随机目标点的三维距离必须通过 `isSpacingInSafeRange()` |
| 场地限制 | 随机采样范围会从场地边界向内收缩 `2 * orca_radius_`，并限制在虚拟 leader 附近，避免随机阵型过度分散 |
| 速度限制 | 无独立速度参数，移动过程由 ORCA 的 `orca_max_speed_` 限制 |

#### 6.3.5 `STATIC_FORMATION_CUSTOM`

示意图：

```text
用户直接指定 offset:

leader_pos + custom_offsets_pos[0] -> A1
leader_pos + custom_offsets_pos[1] -> A2
leader_pos + custom_offsets_pos[2] -> A3
```

| 项 | 说明 |
| --- | --- |
| 枚举 | `STATIC_FORMATION_CUSTOM=9` |
| 相关参数 | `custom_offsets_pos`、`custom_offsets_yaw`、`leader_pos`、`leader_yaw` |
| 几何含义 | 外部直接指定每个智能体相对虚拟 leader 的位置和 yaw 偏移 |
| 数组规则 | `custom_offsets_pos[i]` 和 `custom_offsets_yaw[i]` 对应 `agent_id=i+1` |
| 安全限制 | 两个数组长度必须都等于 `swarm_num`；任意两个自定义目标点的三维距离必须通过 `isSpacingInSafeRange()`；最终目标点还必须在场地内、避开圆形静态障碍物 |
| 速度限制 | 无独立速度参数，移动过程由 ORCA 的 `orca_max_speed_` 限制 |

#### 6.3.6 `DYNAMIC_FORMATION_RING`

示意图：

```text
        A1 ->
    A4       A2
         L
    A3       A5

所有智能体在圆周上均匀分布，并沿切向运动。
```

| 项 | 说明 |
| --- | --- |
| 枚举 | `DYNAMIC_FORMATION_RING=11` |
| 相关参数 | `dynamic_ring_radius`、`dynamic_ring_move_speed`、`dynamic_time`、`leader_pos`、`leader_yaw` |
| 几何含义 | 虚拟 leader 为圆心，智能体均匀分布在圆周上，并按切向速度绕圆运动 |
| 半径限制 | `swarm_num >= 2`，`dynamic_ring_radius > 0`，相邻弦长 `2 * dynamic_ring_radius * sin(pi / swarm_num)` 必须通过 `isSpacingInSafeRange()` |
| 场地限制 | 整个圆轨迹必须能放进场地 XY 安全范围，边界安全余量为 `2 * orca_radius_` |
| 速度限制 | `0.05m/s <= abs(dynamic_ring_move_speed) <= orca_max_speed_`，`dynamic_time > 0` |

#### 6.3.7 `DYNAMIC_FORMATION_POLYGON`

示意图：

```text
N = 4:

A1 ----> A2
^        |
|   L    v
A4 <---- A3

所有智能体沿正 N 边形边线循环运动。
```

| 项 | 说明 |
| --- | --- |
| 枚举 | `DYNAMIC_FORMATION_POLYGON=12` |
| 相关参数 | `dynamic_polygon_spacing`、`dynamic_polygon_move_speed`、`dynamic_time`、`leader_pos`、`leader_yaw` |
| 几何含义 | 所有智能体沿正 N 边形周长运动，不同智能体在周长参数上错开一个边长 |
| 间距限制 | `3 <= swarm_num <= 10`，`isSpacingInSafeRange(dynamic_polygon_spacing)` 必须为 true |
| 场地限制 | 正 N 边形外接圆必须能放进场地 XY 安全范围，边界安全余量为 `2 * orca_radius_` |
| 速度限制 | `0.05m/s <= abs(dynamic_polygon_move_speed) <= orca_max_speed_`，`dynamic_time > 0` |

#### 6.3.8 `DYNAMIC_FORMATION_LEMNISCATE`

示意图：

```text
        ↗ A1 ↘
     ↗         ↘
   A4     L     A2
     ↘         ↗
        ↘ A3 ↗

所有智能体沿同一条 8 字轨迹运动，初始相位分布在同一侧半瓣内部，避免偶数机在中心自交点重叠。
```

| 项 | 说明 |
| --- | --- |
| 枚举 | `DYNAMIC_FORMATION_LEMNISCATE=13` |
| 相关参数 | `dynamic_lemniscate_x_radius`、`dynamic_lemniscate_y_radius`、`dynamic_lemniscate_move_speed`、`dynamic_time`、`leader_pos`、`leader_yaw` |
| 几何含义 | 虚拟 leader 为 8 字轨迹中心，所有智能体沿同一条 Lemniscate 轨迹运动；初始相位使用单侧半瓣队列，任意两机初始相位差小于 `pi` |
| 尺寸限制 | `1 <= swarm_num <= 10`，`dynamic_lemniscate_x_radius` 和 `dynamic_lemniscate_y_radius` 都必须通过 `isSpacingInSafeRange()` |
| 场地限制 | 8 字轨迹必须能放进场地 XY 安全范围，边界安全余量为 `2 * orca_radius_` |
| 速度限制 | `0.05m/s <= abs(dynamic_lemniscate_move_speed) <= orca_max_speed_`，`dynamic_time > 0` |

### 6.4 `sunray_msgs/UAVSwarmState`

用途：

UAV 集群控制节点周期发布的状态消息。monitor、RViz 和 Qt 面板只应该依赖该消息中已经发布出来的字段，不应该直接读取控制节点内部变量。

关键字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `agent_id` | `uint8` | 本机 ID |
| `swarm_num` | `uint32` | 集群数量 |
| `self_odom_ready` | `bool` | 本机 odom 是否可用 |
| `peers_odom_ready` | `bool` | 邻居 odom 是否全部可用且未超时 |
| `ready_peer_num` | `uint32` | 当前有效邻居数量 |
| `self_odom` | `nav_msgs/Odometry` | 本机 odom，RViz 中的 mesh、速度、轨迹均来自该字段 |
| `swarm_cmd` | `sunray_msgs/UAVSwarmCMD` | 当前已接收并正在执行的集群指令 |
| `fsm_state` | `uint8` | 当前集群控制状态 |
| `target_valid` | `bool` | 是否存在有效目标点 |
| `target_pos` | `geometry_msgs/Point` | 当前集群控制目标点 |
| `target_yaw` | `float32` | 当前集群控制目标 yaw |
| `uav_cmd` | `sunray_msgs/UAVControlCMD` | 当前发布给 UAV 控制器的指令 |

状态枚举：

| 枚举 | 含义 |
| --- | --- |
| `INIT=0` | UAV 集群控制初始化状态 |
| `TAKEOFF=1` | 已发送起飞指令，等待底层进入 hover |
| `LAND=2` | 已发送降落指令，等待底层回到 init |
| `RETURN_HOME=3` | 正在返航 |
| `ARRIVED=4` | 已到达目标点，持续发布悬停点 |
| `SWARM_STATIC_FORMATION=5` | 正在执行静态阵型 |
| `SWARM_DYNAMIC_FORMATION=6` | 正在追踪动态阵型 |
| `SWARM_DYNAMIC_FORMATION_PREPARE=7` | 动态阵型准备阶段，先抵达 t=0 初始目标点 |

### 6.5 `sunray_msgs/UGVSwarmState`

用途：

UGV 集群控制节点周期发布的状态消息。它与 `UAVSwarmState` 结构类似，但输出控制指令字段为 `ugv_cmd`。

关键字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `agent_id` | `uint8` | 本机 ID |
| `swarm_num` | `uint32` | 集群数量 |
| `self_odom_ready` | `bool` | 本机 odom 是否可用 |
| `peers_odom_ready` | `bool` | 邻居 odom 是否全部可用且未超时 |
| `ready_peer_num` | `uint32` | 当前有效邻居数量 |
| `self_odom` | `nav_msgs/Odometry` | 本机 odom，RViz 中的 mesh、速度、轨迹均来自该字段 |
| `swarm_cmd` | `sunray_msgs/UGVSwarmCMD` | 当前已接收并正在执行的集群指令 |
| `fsm_state` | `uint8` | 当前集群控制状态 |
| `target_valid` | `bool` | 是否存在有效目标点 |
| `target_pos` | `geometry_msgs/Point` | 当前集群控制目标点 |
| `target_yaw` | `float32` | 当前集群控制目标 yaw |
| `ugv_cmd` | `sunray_msgs/UGVControlCMD` | 当前发布给 UGV 控制器的指令 |

状态枚举：

| 枚举 | 含义 |
| --- | --- |
| `INIT=0` | UGV 集群控制初始化状态 |
| `RETURN_HOME=1` | 正在返航 |
| `ARRIVED=2` | 已到达目标点，持续保持 |
| `SWARM_STATIC_FORMATION=3` | 正在执行静态阵型 |
| `SWARM_DYNAMIC_FORMATION=4` | 正在追踪动态阵型 |
| `SWARM_DYNAMIC_FORMATION_PREPARE=5` | 动态阵型准备阶段，先抵达 t=0 初始目标点 |

## 7. 状态机说明

### 7.1 UAV 状态机

UAV 集群状态由 `UAVSwarmState.fsm_state` 表示。

```text
INIT
  接收 SWARM_TAKEOFF
  发送 UAVControlCMD::TAKEOFF
  -> TAKEOFF

TAKEOFF
  等待 UAVControlState::HOVER
  记录返航点
  -> ARRIVED

ARRIVED
  持续发布 MOVE_POINT 到 hover_point
  可接收 HOVER / RETURN / FORMATION / LAND

RETURN_HOME
  goal = return_point
  ORCA 速度控制返航
  到达后 -> ARRIVED

SWARM_STATIC_FORMATION
  formation(t=0) 生成固定目标点
  ORCA 速度控制到目标点
  到达后 -> ARRIVED

SWARM_DYNAMIC_FORMATION_PREPARE
  formation(t=0) 生成动态阵型初始点
  所有 agent 根据 odom 判断均到达 t=0 初始点
  等待 dynamic_prepare_wait_time
  -> SWARM_DYNAMIC_FORMATION

SWARM_DYNAMIC_FORMATION
  formation(elapsed_time) 持续刷新动态目标点
  ORCA 速度控制追踪
  elapsed_time >= dynamic_time 后 -> ARRIVED

LAND
  发送 UAVControlCMD::LAND
  等待底层回到 FSM_INIT
  -> INIT
```

### 7.2 UGV 状态机

UGV 集群状态由 `UGVSwarmState.fsm_state` 表示。

```text
INIT
  等待本机 odom 和 UGV 控制 FSM 状态

ARRIVED
  持续发布 UGVControlCMD::HOLD
  可接收 HOLD / RETURN / FORMATION

RETURN_HOME
  goal = return_point
  ORCA 速度控制返航
  到达后 -> ARRIVED

SWARM_STATIC_FORMATION
  formation(t=0) 生成固定目标点
  ORCA 速度控制到目标点
  到达后 -> ARRIVED

SWARM_DYNAMIC_FORMATION_PREPARE
  formation(t=0) 生成动态阵型初始点
  所有 agent 根据 odom 判断均到达 t=0 初始点
  等待 dynamic_prepare_wait_time
  -> SWARM_DYNAMIC_FORMATION

SWARM_DYNAMIC_FORMATION
  formation(elapsed_time) 持续刷新动态目标点
  ORCA 速度控制追踪
  elapsed_time >= dynamic_time 后 -> ARRIVED
```

### 7.3 动态阵型 PREPARE 逻辑

动态阵型不会在收到命令后立刻开始跑轨迹。当前实现会先让每个智能体抵达动态轨迹在 `t=0` 时刻对应的初始点。

每个节点会用相同的 `Formation` 指令计算所有 agent 的 `t=0` 初始目标点，再用 `odom_caches_` 中的全体 odom 判断是否全部到达。全体到达后，节点等待 `dynamic_prepare_wait_time` 秒，再记录 `dynamic_formation_start_time_`，进入正式动态追踪。

该方案不额外引入 `formation_ready` 消息字段，依赖的是已有的全体 odom 输入。

## 8. Formation 设计

`formation` 是纯 C++ 算法类，不发布也不订阅 ROS 话题。它的统一入口是：

```cpp
bool GetFormationGoal(const sunray_msgs::Formation &formation_cmd,
                      int agent_id,
                      double formation_time,
                      double &target_x,
                      double &target_y,
                      double &target_z,
                      double &target_yaw);
```

设计要点：

- `agent_id` 从 1 开始，内部会转换为 `agent_index = agent_id - 1`。
- `leader_pos` 是虚拟 leader 在世界系下的位置。
- 当前实现中的 offset 直接定义在世界系下，不会再按 `leader_yaw` 旋转。
- 静态阵型通常传入 `formation_time=0`。
- 动态阵型使用 `formation_time` 计算当前相位。
- 目标点会经过场地边界检查。
- 如果启用了圆形静态障碍物，目标点不能落入 `obstacle_radius + orca_radius` 的安全区内。

新增阵型的一般步骤：

1. 在 `common/sunray_msgs/msg/Formation.msg` 中增加枚举和参数字段。
2. 在 `formation/formation.h` 中声明对应的 offset 计算函数。
3. 在 `formation/formation.cpp` 的 `GetFormationGoal()` switch 中接入新阵型。
4. 在 terminal、TUI、Qt panel 中补充参数编辑和显示。
5. 在 monitor/RViz 的阵型名称转换函数中补充显示名称。

## 9. ORCA 设计

`ORCA/orca_lib` 是原生 RVO2/ORCA 代码，业务开发时不建议直接修改。

`ORCA/ORCA.h` 和 `ORCA/ORCA.cpp` 是本包对 ORCA 的封装入口。该类不依赖 ROS，公开接口只使用基础数值类型。

典型调用流程：

```cpp
orca_.init(agent_num, neighbor_dist, time_horizon, radius, max_speed, time_step, max_neighbors);

for (int i = 0; i < agent_num; ++i)
{
    orca_.setAgentState(i, x, y, vx, vy);
}

orca_.setAgentGoal(self_idx, goal_x, goal_y);
orca_.GetOrcaVelCmd(self_idx, cmd_vx, cmd_vy);
```

当前 ORCA 只输出 XY 平面速度：

- UAV 的 Z 方向速度由 `swarm_control_uav_node` 使用高度误差 P 控制生成。
- UAV 和 UGV 的 yaw 指令来自当前目标点 `target_yaw`。
- 静态圆形障碍物通过 `addCircleObstacle()` 加入，内部会近似成多边形障碍物。

修改 ORCA 行为时，应优先修改 `ORCA/ORCA.h` 和 `ORCA/ORCA.cpp`。除非明确要调整原生算法，否则不要修改 `ORCA/orca_lib`。

## 10. RViz 可视化说明

UAV 和 UGV 的 RViz 可视化节点都订阅 `SwarmState`，发布 `visualization_msgs/MarkerArray`。默认坐标系为 `world`。

### 10.1 Agent 颜色

每个 agent 根据 `agent_id` 使用固定颜色，10 个颜色循环一次。

| agent 序号 | 颜色含义 |
| --- | --- |
| 1 | 蓝色 |
| 2 | 绿色 |
| 3 | 橙色 |
| 4 | 红色 |
| 5 | 紫色 |
| 6 | 黄色 |
| 7 | 青色 |
| 8 | 粉色 |
| 9 | 黄绿色 |
| 10 | 灰色 |

这些颜色用于速度箭头和轨迹线。mesh 是否明显呈现该颜色取决于 DAE 文件的 embedded material。

### 10.2 world 原点 marker

数据来源：

| Marker | 颜色 | 来源 | 含义 |
| --- | --- | --- | --- |
| 原点球 | 白色 | 固定点 `(0,0,0)` | world 坐标系原点 |
| X 轴箭头 | 红色 | 固定方向 `(1,0,0)` | world X 正方向 |
| Y 轴箭头 | 绿色 | 固定方向 `(0,1,0)` | world Y 正方向 |
| Z 轴箭头 | 蓝色 | 固定方向 `(0,0,1)` | world Z 正方向，UAV 可视化中显示 |
| 原点文字 | 白色 | 固定文本 | 标注 `world origin` |

UGV 可视化是平面场景，默认只显示 X/Y 方向箭头。UAV 可视化会额外显示 Z 方向箭头。

### 10.3 静态障碍物 marker

数据来源：

```text
static_obstacles/enabled
static_obstacles/x
static_obstacles/y
static_obstacles/radius
```

显示含义：

| Marker | 颜色 | 数据来源 | 含义 |
| --- | --- | --- | --- |
| 圆柱 | 半透明橙红色 | `x/y/radius` | 圆形静态障碍物占据区域 |
| 文字 | 橙色 | `radius` | `obsN r=...`，显示障碍物编号和半径 |

这些障碍物同时会被 swarm control 写入 ORCA，用于速度避障；也会被 formation 用于拒绝落入障碍物安全区的目标点。

### 10.4 智能体 mesh

数据来源：

```text
UAVSwarmState.self_odom.pose.pose
UGVSwarmState.self_odom.pose.pose
```

显示含义：

| Marker | 类型 | 数据来源 | 含义 |
| --- | --- | --- | --- |
| UAV mesh | `MESH_RESOURCE` | `self_odom.pose.pose` | 无人机当前位置和姿态 |
| UGV mesh | `MESH_RESOURCE` | `self_odom.pose.pose` | 无人车当前位置和姿态 |

mesh 路径由 `mesh_resource` 参数指定：

```text
package://sunray_swarm_control/utils/meshes/uav.dae
package://sunray_swarm_control/utils/meshes/ugv.dae
```

如果 `self_odom_ready=false` 或状态超过 `stale_timeout`，该 agent 的 marker 会被删除。

### 10.5 当前速度箭头

数据来源：

```text
SwarmState.self_odom.twist.twist.linear
```

显示含义：

| Marker | 颜色 | 起点 | 终点 | 含义 |
| --- | --- | --- | --- | --- |
| 速度箭头 | agent 专属颜色 | `self_odom.pose.pose.position` | 位置 + `linear_velocity * velocity_scale` | 当前实际/估计速度方向和大小 |

UAV 速度箭头直接使用 `linear.x/y/z`。UGV 的 `self_odom.twist.twist.linear.x/y` 按车体系速度处理，RViz 节点会结合当前 yaw 旋转到 `world` 坐标系后显示，避免车辆转向后速度箭头与实际移动方向不一致。

### 10.6 ID 文字

数据来源：

```text
SwarmState.agent_id
agent_name 参数
SwarmState.self_odom.pose.pose.position
```

显示含义：

| Marker | 颜色 | 文本 | 含义 |
| --- | --- | --- | --- |
| ID 文字 | 白色 | `uav1`、`uav2`、`ugv1`、`ugv6` 等 | 当前 marker 对应的智能体编号 |

### 10.7 任务状态文字

数据来源：

```text
SwarmState.fsm_state
SwarmState.swarm_cmd.swarm_cmd
SwarmState.swarm_cmd.formation_cmd.formation_type
```

显示含义：

| Marker | 颜色 | 文本格式 | 含义 |
| --- | --- | --- | --- |
| 任务文字 | 青色 | `FSM / CMD / FORMATION` | 当前集群状态、当前指令、当前阵型 |

示例：

```text
DYNAMIC_PREPARE / FORMATION / DYNAMIC_RING
STATIC_FORMATION / FORMATION / STATIC_LINE
ARRIVED / HOVER
```

### 10.8 轨迹线

数据来源：

```text
SwarmState.self_odom.pose.pose.position
```

显示含义：

| Marker | 颜色 | 类型 | 含义 |
| --- | --- | --- | --- |
| 轨迹线 | agent 专属颜色 | `LINE_STRIP` 细线 | 最近 `trail_size` 帧历史位置 |

轨迹线宽度较细，用于减少多机轨迹同时显示时的遮挡。只有当历史点数量不少于 2 个时才会显示轨迹线，避免 RViz 报 `At least two points are required for a LINE_STRIP marker`。

### 10.9 目标点球

数据来源：

```text
SwarmState.target_valid
SwarmState.target_pos
```

显示含义：

| Marker | 颜色 | 位置 | 含义 |
| --- | --- | --- | --- |
| 目标点球 | 黄橙色 | `target_pos` | 当前 swarm control 正在追踪的目标点 |

目标点球用于标注目标位置，尺寸较小，避免遮挡 agent mesh 和轨迹。UGV 目标点球会在显示时略微抬高 Z 位置，便于在地面上观察。

### 10.10 目标点文字

数据来源：

```text
SwarmState.target_valid
SwarmState.target_pos
SwarmState.agent_id
agent_name 参数
```

显示含义：

| Marker | 颜色 | 文本 | 含义 |
| --- | --- | --- | --- |
| 目标点文字 | 黄色 | `goal uav1`、`goal ugv3` 等 | 当前 agent 的目标点标注 |

当 `target_valid=false` 时，目标点球和目标点文字都会被删除。

## 11. 常用启动方式

UAV 集群控制仿真：

```bash
roslaunch sunray_swarm_control swarm_control_uav_sim.launch agent_name:=uav swarm_num:=6
```

UGV 集群控制仿真：

```bash
roslaunch sunray_swarm_control swarm_control_ugv_sim.launch agent_name:=ugv swarm_num:=6
```

只启动控制节点，不启动 RViz 和 monitor：

```bash
roslaunch sunray_swarm_control swarm_control_uav_sim.launch agent_name:=uav swarm_num:=6 enable_rviz:=false enable_monitor:=false
roslaunch sunray_swarm_control swarm_control_ugv_sim.launch agent_name:=ugv swarm_num:=6 enable_rviz:=false enable_monitor:=false
```

Qt 集群控制面板：

```bash
roslaunch sunray_swarm_control swarm_control_panel.launch
```

终端指令发布工具：

```bash
roslaunch sunray_swarm_control uav_swarm_cmd_pub_terminal.launch
roslaunch sunray_swarm_control ugv_swarm_cmd_pub_terminal.launch
```

ORCA 测试：

```bash
roslaunch sunray_swarm_control orca_test.launch
roslaunch sunray_swarm_control orca_test.launch transition_case_count:=200
```

## 12. 二次开发指南

### 12.1 新增阵型

推荐修改范围：

| 文件 | 修改内容 |
| --- | --- |
| `common/sunray_msgs/msg/Formation.msg` | 增加阵型枚举和必要参数 |
| `swarm/formation/formation.h` | 声明新的 offset 计算函数 |
| `swarm/formation/formation.cpp` | 实现目标点计算，并在 `GetFormationGoal()` 中接入 |
| `swarm/utils/uav_swarm_cmd_pub_terminal.cpp` | 增加终端参数输入和显示 |
| `swarm/utils/ugv_swarm_cmd_pub_terminal.cpp` | 增加终端参数输入和显示 |
| `swarm/utils/swarm_control_panel_node.cpp` | 增加 Qt 面板参数编辑 |
| `swarm/utils/*monitor*.cpp` | 增加阵型名称显示 |
| `swarm/utils/rviz_visualization_*_node.cpp` | 增加阵型名称显示 |

### 12.2 修改 ORCA 策略

推荐修改范围：

| 文件 | 修改内容 |
| --- | --- |
| `swarm/ORCA/ORCA.h` | 调整或新增上层 API |
| `swarm/ORCA/ORCA.cpp` | 实现新的 preferred velocity、死锁处理或障碍物处理策略 |
| `swarm/ORCA/test/orca_test.cpp` | 增加压力测试用例 |

不要优先修改 `swarm/ORCA/orca_lib`。该目录是原生 RVO2/ORCA 内核，除非明确要改底层算法，否则应保持稳定。

### 12.3 新增平台

如果后续增加新平台，例如地面机器人、船、机械臂底盘等，可以参考当前 UAV/UGV 分层方式：

| 层级 | 建议做法 |
| --- | --- |
| 纯算法 | 继续复用 `formation` 和 `ORCA` |
| 消息 | 新增平台自己的 `SwarmCMD` 和 `SwarmState`，或抽象公共消息 |
| 状态机 | 参考 `swarm_control_uav` 或 `swarm_control_ugv` 新建平台目录 |
| 可视化 | 新增对应 `rviz_visualization_xxx_node` |
| 控制面板 | 在 Qt panel 中增加平台 tab 或平台选择项 |

### 12.4 新增状态量

如果某个过程量需要被 monitor、RViz 或 Qt 面板显示，应优先加入 `UAVSwarmState.msg` 或 `UGVSwarmState.msg`。不要让显示节点直接读取控制节点内部变量。

推荐流程：

```text
修改 SwarmState.msg
  -> 编译 common 重新生成消息
  -> 在 swarm_control_uav/ugv 中填充字段
  -> 在 monitor/RViz/panel 中读取显示
```

## 13. 调试建议

指令没有响应：

```text
先看 swarm monitor
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

ORCA 输出不明显或车辆不动：

```text
确认 goal_point 是否有效
确认 odom_caches_ 中所有 agent 的 odom 都已收到
确认 orca/max_speed 是否过小
确认 orca/radius 是否过大导致可行空间太小
确认静态障碍物是否把目标点或路径完全堵住
```

RViz 看不到模型：

```text
确认 rviz_visualization 节点已启动
确认 MarkerArray 话题已加入 RViz
确认 Fixed Frame 为 world
确认 self_odom_ready=true
确认 mesh_resource 路径有效
```

## 14. 编译说明

如果只修改 `swarm` 内部 `.cpp/.h/.launch/.yaml/.rviz/.md` 文件，通常只需要编译 `swarm`：

```bash
source /opt/ros/noetic/setup.bash
source /home/amov/Sunray_v2/devel/setup.bash
cd /home/amov/Sunray_v2
catkin_make --source swarm --build build/swarm -j4
```

如果修改了 `common/sunray_msgs/msg` 中的消息定义，需要先编译 `common`，再编译 `swarm`：

```bash
source /opt/ros/noetic/setup.bash
cd /home/amov/Sunray_v2
catkin_make --source common --build build/common -j4
source /home/amov/Sunray_v2/devel/setup.bash
catkin_make --source swarm --build build/swarm -j4
```

launch 解析检查：

```bash
source /opt/ros/noetic/setup.bash
source /home/amov/Sunray_v2/devel/setup.bash
roslaunch --files sunray_swarm_control swarm_control_uav_sim.launch swarm_num:=3 agent_name:=uav
roslaunch --files sunray_swarm_control swarm_control_ugv_sim.launch swarm_num:=3 agent_name:=ugv
```

## 15. 当前实现注意事项

旧版 `orca_engine` 相关入口已经不作为当前主链路使用。当前主链路是：

```text
swarm_control_uav_node
swarm_control_ugv_node
formation_lib
orca_api_lib
```

`CMakeLists.txt` 中保留了旧版 `orca_engine` 的条件编译兼容逻辑。如果旧文件不存在，会跳过旧节点编译。

UAV 和 UGV 的集群控制节点当前都是分布式节点。仿真中通过一个 launch 同时启动多个节点，真机中可以在每台机器上单独启动对应 `agent_id` 的节点。
