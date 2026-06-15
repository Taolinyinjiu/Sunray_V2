<!-- title: 集群总览与架构 -->

<section id="swarm-overview">

## 集群总览与架构

`swarm` 是 Sunray 的集群编队控制模块，ROS 包名为 `sunray_swarm_control`。它位于定位模块和单机控制模块之间，负责把“集群级命令”拆解成每个智能体的单机控制命令。

它当前同时支持 UAV 和 UGV。两类平台共用：

- `formation` 阵型目标点生成库。
- `ORCA` 避障库。
- `sunray_msgs/Formation` 阵型描述消息。

但 UAV 和 UGV 拥有独立的：

- ROS 状态机节点。
- 监控节点。
- RViz 可视化节点。
- 指令发布工具。
- 单机控制命令输出类型。

### 模块边界

集群模块负责：

- 接收外部集群指令，例如起飞、降落、悬停、返航、静态阵型、动态阵型。
- 根据 `sunray_msgs/Formation` 计算每个 `agent_id` 对应的目标位置和目标 yaw。
- 读取本机和邻居的 `local_odom`，判断输入是否就绪。
- 调用 ORCA 在 XY 平面计算避碰后的速度指令。
- 将集群控制结果转换为 `UAVControlCMD` 或 `UGVControlCMD`。
- 发布 `UAVSwarmState` 或 `UGVSwarmState`，供 monitor、RViz 和 Qt 面板使用。

集群模块不负责：

- 不负责单机底层姿态、位置、速度闭环控制。
- 不负责定位融合。
- 不负责通信链路本身，只假设每个智能体的 `local_odom` 能被当前节点订阅到。
- 不直接控制电机、PX4、底盘驱动器或仿真动力学。

### 总体架构与数据流

UAV 数据流：

```text
terminal / formation_tui / Qt panel / 上层任务
  -> /sunray/swarm/uav_swarm_cmd
  -> swarm_control_uav_node
  -> formation 计算本机目标点
  -> ORCA 计算 XY 避碰速度
  -> /uavX/sunray/uav_control/control_cmd
  -> sunray_uav_control
  -> MAVROS / PX4
```

UGV 数据流：

```text
terminal / Qt panel / 上层任务
  -> /sunray/swarm/ugv_swarm_cmd
  -> swarm_control_ugv_node
  -> formation 计算本车目标点
  -> ORCA 计算 XY 避碰速度
  -> /ugvX/sunray/ugv_control/control_cmd
  -> sunray_ugv_control
  -> 底盘驱动
```

状态监控链路：

```text
swarm_control_uav_node / swarm_control_ugv_node
  -> /sunray/swarm/uav_swarm_state 或 /sunray/swarm/ugv_swarm_state
  -> monitor 节点
  -> RViz marker 节点
  -> swarm_control_panel_node
```

### 目录结构

```text
swarm/
├── ORCA/
│   ├── ORCA.h / ORCA.cpp
│   ├── orca_lib/
│   └── test/orca_test.cpp
├── formation/
│   ├── formation.h
│   └── formation.cpp
├── swarm_control_uav/
│   ├── swarm_control_uav.h
│   ├── swarm_control_uav.cpp
│   └── swarm_control_uav_node.cpp
├── swarm_control_ugv/
│   ├── swarm_control_ugv.h
│   ├── swarm_control_ugv.cpp
│   └── swarm_control_ugv_node.cpp
├── utils/
│   ├── uav_swarm_cmd_pub_terminal.cpp
│   ├── ugv_swarm_cmd_pub_terminal.cpp
│   ├── formation_tui.cpp
│   ├── swarm_control_uav_monitor_node.cpp
│   ├── swarm_control_ugv_monitor_node.cpp
│   ├── rviz_visualization_uav_node.cpp
│   ├── rviz_visualization_ugv_node.cpp
│   ├── swarm_control_panel_node.cpp
│   └── meshes/
├── launch/
├── rviz/
└── example/
```

### 全局命名约定

`agent_name` 表示平台名前缀，例如 `uav` 或 `ugv`。

`agent_id` 表示本机编号，从 `1` 开始。

`swarm_num` 表示当前集群规模。

单机话题前缀统一为：

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

集群指令消息里的 `agent_id` 是目标智能体 ID。当 `agent_id=99` 时表示广播，所有智能体都会响应这条命令。

### 核心话题

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 输入 | `/sunray/swarm/uav_swarm_cmd` | `sunray_msgs/UAVSwarmCMD` | UAV 集群命令。 |
| 输出 | `/sunray/swarm/uav_swarm_state` | `sunray_msgs/UAVSwarmState` | UAV 集群状态。 |
| 输入 | `/sunray/swarm/ugv_swarm_cmd` | `sunray_msgs/UGVSwarmCMD` | UGV 集群命令。 |
| 输出 | `/sunray/swarm/ugv_swarm_state` | `sunray_msgs/UGVSwarmState` | UGV 集群状态。 |
| 输入 | `/{agent}/sunray/localization/local_odom` | `nav_msgs/Odometry` | 本机和邻居 odom。 |
| 输入 | `/{agent}/sunray/uav_control/control_state` | `sunray_msgs/UAVControlState` | UAV 单机控制状态。 |
| 输入 | `/{agent}/sunray/ugv_control/control_state` | `sunray_msgs/UGVControlState` | UGV 单机控制状态。 |
| 输出 | `/{agent}/sunray/uav_control/control_cmd` | `sunray_msgs/UAVControlCMD` | 发给 UAV 单机控制器。 |
| 输出 | `/{agent}/sunray/ugv_control/control_cmd` | `sunray_msgs/UGVControlCMD` | 发给 UGV 单机控制器。 |

### 状态机说明

UAV 状态由 `UAVSwarmState.fsm_state` 表示：

```text
INIT
  -> TAKEOFF
  -> ARRIVED
  -> RETURN_HOME
  -> SWARM_STATIC_FORMATION
  -> SWARM_DYNAMIC_FORMATION_PREPARE
  -> SWARM_DYNAMIC_FORMATION
  -> LAND
```

UGV 状态由 `UGVSwarmState.fsm_state` 表示：

```text
INIT
  -> ARRIVED
  -> RETURN_HOME
  -> SWARM_STATIC_FORMATION
  -> SWARM_DYNAMIC_FORMATION_PREPARE
  -> SWARM_DYNAMIC_FORMATION
```

动态阵型不会收到命令后立刻开始跑轨迹。当前实现会先让所有智能体抵达动态轨迹 `t=0` 的初始点，等待 `dynamic_prepare_wait_time` 秒后，再进入正式动态追踪。

### UAV 状态机详细流程

UAV 集群状态机比 UGV 多了起飞和降落，因为 UAV 必须等待单机控制器进入可飞状态后才能执行阵型。

```text
INIT
  接收 SWARM_TAKEOFF
  发布 UAVControlCMD::TAKEOFF
  -> TAKEOFF

TAKEOFF
  等待 UAVControlState::HOVER
  记录 return_point / hover_point
  -> ARRIVED

ARRIVED
  持续发布 MOVE_POINT 到 hover_point
  可接收 HOVER / RETURN / FORMATION / LAND

RETURN_HOME
  goal = return_point
  formation 不参与，ORCA 根据 return_point 生成避碰速度
  到达后 -> ARRIVED

SWARM_STATIC_FORMATION
  formation(t=0) 生成固定目标点
  ORCA 根据本机目标和邻居状态计算速度
  到达后 -> ARRIVED

SWARM_DYNAMIC_FORMATION_PREPARE
  formation(t=0) 生成动态阵型初始点
  等待所有 agent 都到达初始点
  等待 dynamic_prepare_wait_time
  -> SWARM_DYNAMIC_FORMATION

SWARM_DYNAMIC_FORMATION
  formation(elapsed_time) 持续刷新动态目标点
  ORCA 计算避碰速度
  elapsed_time >= dynamic_time 后 -> ARRIVED

LAND
  发布 UAVControlCMD::LAND
  等待底层回到 INIT
  -> INIT
```

UAV 的集群节点不会自己做姿态控制。它输出给 `sunray_uav_control` 的仍然是 Sunray 标准 UAV 控制指令。这样做的好处是：单机控制器仍然负责定位健康检查、PX4 模式、MAVROS setpoint、起降流程和底层安全保护，集群模块只关心队形目标和避碰速度。

### UGV 状态机详细流程

UGV 没有起飞和降落流程，因此状态机更短：

```text
INIT
  等待本机 odom 和 UGV 控制状态
  满足条件后进入可接收命令状态

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
  所有 agent 到达后等待 dynamic_prepare_wait_time
  -> SWARM_DYNAMIC_FORMATION

SWARM_DYNAMIC_FORMATION
  formation(elapsed_time) 持续刷新动态目标点
  ORCA 速度控制追踪
  elapsed_time >= dynamic_time 后 -> ARRIVED
```

UGV 输出给 `sunray_ugv_control` 的命令最终会变成底盘 `cmd_vel`。差速底盘和麦克纳姆底盘的能力差异仍由 UGV 单机控制模块处理，集群模块不直接接触底盘串口或仿真动力学。

### 动态阵型 PREPARE 逻辑

动态阵型分成 PREPARE 和正式运行两段，这是为了避免收到动态命令后各机立即沿轨迹运动，导致初始相位混乱。

流程如下：

```text
收到 DYNAMIC_FORMATION_* 命令
  -> 所有节点用同一条 Formation 命令计算 t=0 目标点
  -> 每个 agent 先用 ORCA 运动到自己的 t=0 初始点
  -> 每个节点根据 odom_caches_ 判断全体是否到达
  -> 全体到达后等待 dynamic_prepare_wait_time
  -> 记录 dynamic_formation_start_time_
  -> 进入 SWARM_DYNAMIC_FORMATION
```

这里没有额外引入 `formation_ready` 消息，而是复用所有 agent 的 odom 状态做判断。优点是消息结构简单；缺点是对 `peer_odom_timeout`、到达阈值和定位质量比较敏感。

如果动态阵型一直停在 PREPARE，优先检查：

- 是否所有 agent 都启动了 swarm control 节点。
- `swarm_num` 是否和实际数量一致。
- 所有 `/uavX/sunray/localization/local_odom` 或 `/ugvX/sunray/localization/local_odom` 是否持续更新。
- `goal_xy_tolerance`、`goal_z_tolerance`、`goal_yaw_tolerance` 是否过严。
- 某个 agent 的目标点是否被场地边界或障碍物约束拒绝。

### 二次开发指南

| 需求 | 优先修改位置 |
| --- | --- |
| 增加新阵型 | `Formation.msg`、`formation/formation.*`、终端/TUI/Qt/RViz 显示。 |
| 调整避障距离或速度 | YAML 中的 `orca/*` 参数。 |
| 改 ORCA 策略 | `ORCA/ORCA.h`、`ORCA/ORCA.cpp`。 |
| 新增集群命令 | `UAVSwarmCMD` / `UGVSwarmCMD`、UAV/UGV 状态机、工具 UI。 |
| 新增状态显示字段 | `UAVSwarmState` / `UGVSwarmState`，再同步 monitor/RViz/panel。 |

### 当前实现注意事项

当前主链路是：

```text
swarm_control_uav_node
swarm_control_ugv_node
formation_lib
orca_api_lib
```

`orca_engine` 相关历史入口不作为当前主链路使用。`CMakeLists.txt` 中仍保留兼容逻辑，如果旧文件不存在会跳过旧节点编译。

UAV 和 UGV 集群控制节点当前都是分布式节点。仿真中通过一个 launch 同时启动多个节点；真机中可以在每台机器上单独启动对应 `agent_id` 的节点。无论哪种方式，所有节点都需要能看到其他成员的 `local_odom`。

### 与其他模块的关系

集群模块不是孤立运行的。它依赖定位模块和单机控制模块已经正常工作。

| 上游/下游 | 关系 | 集群模块依赖什么 |
| --- | --- | --- |
| `localization_fusion` | 上游 | 每个 agent 的 `local_odom` 必须持续更新。 |
| `sunray_uav_control` | 下游 | UAV 集群输出 `UAVControlCMD`，由单机控制器执行。 |
| `sunray_ugv_control` | 下游 | UGV 集群输出 `UGVControlCMD`，由单机控制器执行。 |
| `sunray_msgs` | 接口桥梁 | `Formation`、`UAVSwarmCMD`、`UGVSwarmCMD`、`UAVSwarmState`、`UGVSwarmState`。 |
| `uav_control_tools` / `ugv_control_tools` / monitor | 工具 | 面板或终端工具负责发布命令和观察状态。 |
| `communication/yunlink_ros_bridge` | 可选外部链路 | 地面站或外部系统可以通过桥接发送集群命令。 |

如果集群命令发出后没有动作，不要一开始就改 `formation` 或 ORCA。推荐排查顺序：

```text
local_odom 是否正常
  -> 单机控制状态是否正常
  -> swarm_state 是否更新
  -> swarm_cmd 是否收到
  -> target_valid 是否为 true
  -> uav_cmd / ugv_cmd 是否有输出
  -> 单机控制器是否执行输出
```

### 真机部署理解

仿真 launch 会在同一台电脑上启动多台 agent 的 swarm 节点；真机部署时，常见方式是在每台机载电脑上启动自己的 swarm 节点：

```text
uav1 机载电脑：
  agent_name:=uav agent_id:=1 swarm_num:=6

uav2 机载电脑：
  agent_name:=uav agent_id:=2 swarm_num:=6

...
```

每个节点都需要能订阅到其他成员的 odom。这个要求可以通过同一个 ROS master、多机网络、话题桥接或其他通信系统满足。集群模块本身不解决跨机器通信，它只使用 ROS 话题。

真机部署时需要重点确认：

- 所有机器的 ROS master 配置一致。
- 每台机器的 `agent_id` 唯一。
- 所有机器的 `swarm_num` 一致。
- `local_odom` frame 语义一致。
- 所有机器的时间同步足够稳定。
- 网络延迟不会让邻居 odom 超过 `peer_odom_timeout`。

### 坐标系理解

集群模块主要在 world/local odom 的平面坐标中计算目标点和避障速度。它不主动维护 TF 树，而是相信 `localization_fusion` 和单机控制模块给出的 odom 语义。

对 UAV 来说：

- XY 由 formation 和 ORCA 生成。
- Z 由 formation 目标高度和 UAV 控制器共同处理。
- yaw 由 formation 目标 yaw 传递给单机控制器。

对 UGV 来说：

- XY 由 formation 和 ORCA 生成。
- Z 通常不参与控制，只用于消息和可视化兼容。
- yaw 目标会传给 UGV 控制器，具体执行能力取决于底盘类型。

多机集群最常见的问题是“每台机器的 local frame 不一致”。如果 `/uav1/sunray/localization/local_odom` 和 `/uav2/sunray/localization/local_odom` 不在同一个坐标语义下，formation 生成的目标点就没有实际意义。此时应先处理定位全局一致性，而不是调 ORCA。

</section>
