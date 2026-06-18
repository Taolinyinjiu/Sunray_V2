<!-- title: 节点接口与启动 -->

<section id="swarm-nodes-launch">

## 节点接口与启动

本页按节点说明集群模块的输入输出、参数和常用启动方式。新手可以先从仿真 launch 启动，再用终端工具或 Qt 面板发送命令。

### swarm_control_uav_node

`swarm_control_uav_node` 是单架 UAV 的集群控制节点。每架无人机启动一个该节点。

它接收 `UAVSwarmCMD`，根据本机 `agent_id` 计算目标点，调用 ORCA 得到 XY 避碰速度，并通过 `UAVControlCMD` 交给 `sunray_uav_control` 执行。

订阅：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/{agent_name}{agent_id}/sunray/localization/local_odom` | `nav_msgs/Odometry` | 本机局部里程计。 |
| `/{agent_name}{agent_id}/sunray/uav_control/control_state` | `sunray_msgs/UAVControlState` | 本机 UAV 控制状态。 |
| `/{agent_name}{peer_id}/sunray/localization/local_odom` | `nav_msgs/Odometry` | 邻居里程计，`peer_id` 为其他成员。 |
| `/sunray/swarm/uav_swarm_cmd` | `sunray_msgs/UAVSwarmCMD` | UAV 集群控制指令。 |

发布：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/{agent_name}{agent_id}/sunray/uav_control/control_cmd` | `sunray_msgs/UAVControlCMD` | 下发给单机 UAV 控制器。 |
| `/sunray/swarm/uav_swarm_state` | `sunray_msgs/UAVSwarmState` | 本机集群状态。 |

关键参数：

| 参数 | 默认值 | 单位 | 说明 |
| --- | ---: | --- | --- |
| `agent_name` | `uav` | - | 智能体名前缀。 |
| `agent_id` | `1` | - | 本机 ID。 |
| `swarm_num` | `1` | - | 集群数量。 |
| `control_loop_hz` | `50.0` | Hz | 集群控制主循环频率。 |
| `peer_odom_timeout` | `0.1` | s | 邻居 odom 超时阈值。 |
| `dynamic_prepare_wait_time` | `2.0` | s | 动态阵型开始前等待时间。 |
| `goal_xy_tolerance` | `0.1` | m | XY 到达阈值。 |
| `goal_z_tolerance` | `0.1` | m | 高度到达阈值。 |
| `goal_yaw_tolerance` | `0.1` | rad | yaw 到达阈值。 |
| `orca/neighbor_dist` | `2.0` | m | ORCA 邻居搜索距离。 |
| `orca/time_horizon` | `2.0` | s | ORCA 预测时间窗口。 |
| `orca/radius` | `0.2` | m | UAV 等效避碰半径。 |
| `orca/max_speed` | `1.0` | m/s | ORCA 最大 XY 速度。 |
| `orca/max_neighbors` | `-1` | - | 最大邻居数，`-1` 表示全集群。 |

完整参数组还包括场地限制和静态障碍物：

| 参数 | 典型默认值 | 单位 | 说明 |
| --- | ---: | --- | --- |
| `field/x_min` | `-50.0` | m | 阵型目标点 X 下界。 |
| `field/x_max` | `50.0` | m | 阵型目标点 X 上界。 |
| `field/y_min` | `-50.0` | m | 阵型目标点 Y 下界。 |
| `field/y_max` | `50.0` | m | 阵型目标点 Y 上界。 |
| `field/z_min` | `0.0` | m | UAV 阵型目标点 Z 下界。 |
| `field/z_max` | `3.0` | m | UAV 阵型目标点 Z 上界。 |
| `static_obstacles/enabled` | `false` | - | 是否启用圆形静态障碍物。 |
| `static_obstacles/x` | `[]` | m | 圆形障碍物圆心 X 数组。 |
| `static_obstacles/y` | `[]` | m | 圆形障碍物圆心 Y 数组。 |
| `static_obstacles/radius` | `[]` | m | 圆形障碍物半径数组。 |

UAV 节点内部会把 ORCA 输出的 XY 速度和高度目标组合成 UAV 控制命令。高度目标主要来自 formation 生成的 `target_z`，yaw 目标来自 formation 生成的 `target_yaw`。

### swarm_control_ugv_node

`swarm_control_ugv_node` 是单辆 UGV 的集群控制节点。每辆无人车启动一个该节点。

它接收 `UGVSwarmCMD`，计算目标点，调用 ORCA 得到世界系 XY 速度，并发布 `UGVControlCMD` 给 `sunray_ugv_control`。

订阅：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/{agent_name}{agent_id}/sunray/localization/local_odom` | `nav_msgs/Odometry` | 本车局部里程计。 |
| `/{agent_name}{agent_id}/sunray/ugv_control/control_state` | `sunray_msgs/UGVControlState` | 本车 UGV 控制状态。 |
| `/{agent_name}{peer_id}/sunray/localization/local_odom` | `nav_msgs/Odometry` | 邻居里程计。 |
| `/sunray/swarm/ugv_swarm_cmd` | `sunray_msgs/UGVSwarmCMD` | UGV 集群控制指令。 |

发布：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/{agent_name}{agent_id}/sunray/ugv_control/control_cmd` | `sunray_msgs/UGVControlCMD` | 下发给单机 UGV 控制器。 |
| `/sunray/swarm/ugv_swarm_state` | `sunray_msgs/UGVSwarmState` | 本车集群状态。 |

UGV 参数与 UAV 基本一致，区别是：

| 参数 | 默认值 | 单位 | 说明 |
| --- | ---: | --- | --- |
| `agent_name` | `ugv` | - | 智能体名前缀。 |
| `goal_yaw_tolerance` | `0.1` | rad | UGV yaw 到达阈值。 |
| `orca/radius` | `0.3` | m | UGV 等效避碰半径。 |
| `orca/max_speed` | `0.8` | m/s | UGV ORCA 最大平面速度。 |
| `field/z_min` / `field/z_max` | `-1.0 / 1.0` | m | UGV 场景通常近似平面。 |

UGV 节点输出的是 `UGVControlCMD`。对于麦克纳姆底盘，单机控制器可以执行世界系速度或车体系速度；对于差速底盘，单机控制器会限制不可执行的横向速度。因此如果集群中使用差速底盘，应重点验证 UGV 单机控制器的底盘类型配置。

### monitor 节点

`swarm_control_uav_monitor_node` 和 `swarm_control_ugv_monitor_node` 用于集中显示集群状态，避免多个控制节点在终端中混合刷屏。

订阅：

| 节点 | 话题 | 类型 |
| --- | --- | --- |
| `swarm_control_uav_monitor_node` | `/sunray/swarm/uav_swarm_state` | `UAVSwarmState` |
| `swarm_control_ugv_monitor_node` | `/sunray/swarm/ugv_swarm_state` | `UGVSwarmState` |

参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `swarm_state_topic` | 对应 state 话题 | 订阅状态话题。 |
| `agent_name` | `uav` / `ugv` | 显示名前缀。 |
| `display_hz` | `1.0` | 终端刷新频率。 |
| `stale_timeout` | `1.0` | 状态超时阈值。 |
| `clear_screen` | `true` | 是否刷新前清屏。 |

### RViz 可视化说明

`rviz_visualization_uav_node` 和 `rviz_visualization_ugv_node` 把 SwarmState 转换为 `visualization_msgs/MarkerArray`。

| 节点 | 订阅 | 发布 |
| --- | --- | --- |
| `rviz_visualization_uav_node` | `/sunray/swarm/uav_swarm_state` | `/sunray/swarm/uav_rviz_markers` |
| `rviz_visualization_ugv_node` | `/sunray/swarm/ugv_swarm_state` | `/sunray/swarm/ugv_rviz_markers` |

RViz 中显示：

- Agent 颜色：每个 agent 按 ID 使用固定颜色，便于区分多机。
- world 原点和坐标轴：用于确认 RViz Fixed Frame 和集群坐标系是否一致。
- 静态障碍物：显示 YAML 中配置的圆形障碍物，同时对应 formation 和 ORCA 使用的障碍物。
- 智能体 mesh：显示 UAV/UGV 模型位置和姿态。
- 当前速度箭头：显示当前 odom 中的速度方向和大小。
- ID 文字：显示 `uav1`、`ugv3` 等编号。
- 任务状态文字：显示 FSM、CMD 和 FORMATION 状态。
- 最近轨迹线：显示一段历史运动轨迹。
- 目标点球：显示当前 agent 的目标位置。
- 目标点文字：显示目标点对应的 agent。

#### RViz marker 详细含义

world 原点 marker：

| Marker | 颜色 | 来源 | 含义 |
| --- | --- | --- | --- |
| 原点球 | 白色 | 固定点 `(0,0,0)` | world 坐标系原点。 |
| X 轴箭头 | 红色 | 固定方向 `(1,0,0)` | world X 正方向。 |
| Y 轴箭头 | 绿色 | 固定方向 `(0,1,0)` | world Y 正方向。 |
| Z 轴箭头 | 蓝色 | 固定方向 `(0,0,1)` | world Z 正方向，UAV 可视化显示。 |
| 原点文字 | 白色 | 固定文本 | 标注 `world origin`。 |

智能体 marker：

| Marker | 类型 | 数据来源 | 含义 |
| --- | --- | --- | --- |
| UAV mesh | `MESH_RESOURCE` | `UAVSwarmState.self_odom.pose.pose` | 无人机当前位置和姿态。 |
| UGV mesh | `MESH_RESOURCE` | `UGVSwarmState.self_odom.pose.pose` | 无人车当前位置和姿态。 |
| 速度箭头 | `ARROW` | `self_odom.twist.twist.linear` | 当前速度方向和大小。 |
| ID 文字 | `TEXT_VIEW_FACING` | `agent_id`、`agent_name` | 显示 `uav1`、`ugv3` 等编号。 |
| 任务文字 | `TEXT_VIEW_FACING` | `fsm_state`、`swarm_cmd`、`formation_type` | 显示当前任务状态。 |
| 轨迹线 | `LINE_STRIP` | 最近 `trail_size` 帧位置 | 显示历史轨迹。 |
| 目标点球 | `SPHERE` | `target_pos` | 当前 agent 目标点。 |
| 目标点文字 | `TEXT_VIEW_FACING` | `target_pos`、`agent_id` | 显示 `goal uav1` 等目标标签。 |

状态文字示例：

```text
DYNAMIC_PREPARE / FORMATION / DYNAMIC_RING
STATIC_FORMATION / FORMATION / STATIC_LINE
ARRIVED / HOVER
```

agent 颜色按 ID 循环，默认 10 种颜色：

| agent 序号 | 颜色 |
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

UGV 速度箭头会结合当前 yaw 把车体系速度旋转到 world 坐标系显示，避免车辆转向后箭头方向和实际移动方向不一致。

静态障碍物 marker：

| Marker | 颜色 | 数据来源 | 含义 |
| --- | --- | --- | --- |
| 圆柱 | 半透明橙红色 | `static_obstacles/x/y/radius` | 圆形静态障碍物占据区域。 |
| 文字 | 橙色 | 障碍物编号和半径 | `obsN r=...`。 |

这些障碍物同时会被 swarm control 写入 ORCA，用于速度避障；也会被 formation 用于拒绝落入障碍物安全区的目标点。

关键参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `marker_topic` | `/sunray/swarm/*_rviz_markers` | MarkerArray 输出话题。 |
| `frame_id` | `world` | RViz 固定坐标系。 |
| `mesh_resource` | `package://sunray_swarm_control/utils/meshes/*.dae` | 模型路径。 |
| `publish_hz` | `10.0` | marker 发布频率。 |
| `trail_size` | `50` | 轨迹历史点数量。 |
| `static_obstacles/*` | YAML 配置 | 静态障碍物显示和 ORCA/formation 共用。 |

### 指令发布工具

`uav_swarm_cmd_pub_terminal` 发布 UAV 集群命令：

```bash
roslaunch sunray_swarm_control uav_swarm_cmd_pub_terminal.launch
```

`ugv_swarm_cmd_pub_terminal` 发布 UGV 集群命令：

```bash
roslaunch sunray_swarm_control ugv_swarm_cmd_pub_terminal.launch
```

它们默认 `default_target_agent_id=99`，也就是广播到所有 agent。常用默认参数包括虚拟 leader 位置、阵型类型、动态阵型持续时间、直线间距、圆环半径、8 字轨迹尺度等。

`formation_tui` 是基于 ncurses 的阵型编辑工具，主要用于快速编辑阵型参数和自定义 offsets。当前默认面向 UAV 指令话题，也可通过参数切换发布目标话题。

### Qt 集群控制面板

启动：

```bash
roslaunch sunray_swarm_control swarm_control_panel.launch
```

订阅：

| 话题 | 类型 |
| --- | --- |
| `/sunray/swarm/uav_swarm_state` | `UAVSwarmState` |
| `/sunray/swarm/ugv_swarm_state` | `UGVSwarmState` |

发布：

| 话题 | 类型 |
| --- | --- |
| `/sunray/swarm/uav_swarm_cmd` | `UAVSwarmCMD` |
| `/sunray/swarm/ugv_swarm_cmd` | `UGVSwarmCMD` |

面板适合作为主要调试入口，可以同时查看 UAV/UGV 在线状态、FSM、目标点和关键状态，并发布起飞、降落、返航、编队等命令。

### 常用启动方式

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
roslaunch sunray_swarm_control swarm_control_uav_sim.launch \
  agent_name:=uav swarm_num:=6 enable_rviz:=false enable_monitor:=false

roslaunch sunray_swarm_control swarm_control_ugv_sim.launch \
  agent_name:=ugv swarm_num:=6 enable_rviz:=false enable_monitor:=false
```

ORCA 压力测试：

```bash
roslaunch sunray_swarm_control orca_test.launch
roslaunch sunray_swarm_control orca_test.launch transition_case_count:=200
```

### YAML 配置

UAV 仿真参数：

```text
swarm/launch/swarm_control_uav_sim.yaml
```

UGV 仿真参数：

```text
swarm/launch/swarm_control_ugv_sim.yaml
```

核心配置：

| 分组 | 说明 |
| --- | --- |
| `control_loop_hz` | 集群状态机运行频率。 |
| `peer_odom_timeout` | 邻居 odom 超时阈值。 |
| `dynamic_prepare_wait_time` | 动态阵型初始点全体到达后的等待时间。 |
| `goal_*_tolerance` | 到达判定阈值。 |
| `field/*` | 场地边界，formation 会用它拒绝越界目标。 |
| `orca/*` | ORCA 邻居距离、预测时间、半径、最大速度。 |
| `static_obstacles/*` | 圆形静态障碍物，同时用于 ORCA、formation 和 RViz。 |

### 启动文件参数

`swarm_control_uav_sim.launch` 和 `swarm_control_ugv_sim.launch` 都显式展开到 10 个节点，实际启动数量由 `swarm_num` 控制。

UAV 仿真 launch 参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `swarm_num` | `3` | 启动的 UAV 数量，最大按当前 launch 展开到 10。 |
| `agent_name` | `uav` | UAV 名前缀。 |
| `param_file` | `swarm_control_uav_sim.yaml` | 公共参数文件。 |
| `node_output` | `screen` | 节点输出方式。 |
| `enable_rviz_visualization` | `true` | 是否启动 marker 发布节点。 |
| `rviz_frame_id` | `world` | RViz marker frame。 |
| `rviz_marker_topic` | `/sunray/swarm/uav_rviz_markers` | MarkerArray 话题。 |
| `rviz_mesh_resource` | `package://sunray_swarm_control/utils/meshes/uav.dae` | UAV mesh。 |
| `enable_rviz` | `true` | 是否启动 RViz。 |
| `enable_monitor` | `true` | 是否启动 monitor。 |
| `monitor_clear_screen` | `true` | monitor 是否清屏刷新。 |

UGV 仿真 launch 参数类似，默认 `agent_name=ugv`，marker 话题为 `/sunray/swarm/ugv_rviz_markers`，mesh 为 `ugv.dae`。

终端命令发布 launch 中的默认 Formation 参数：

| 参数 | UAV 默认 | UGV 默认 | 说明 |
| --- | ---: | ---: | --- |
| `default_target_agent_id` | `99` | `99` | 广播目标。 |
| `default_formation_type` | `1` | `1` | 默认静态直线。 |
| `default_leader_x` | `0.0` | `0.0` | leader X。 |
| `default_leader_y` | `0.0` | `0.0` | leader Y。 |
| `default_leader_z` | `1.5` | `0.0` | leader Z。 |
| `default_dynamic_time` | `10.0` | `10.0` | 动态阵型持续时间。 |
| `default_static_line_spacing` | `1.5` | `1.5` | 静态直线间距。 |
| `default_static_polygon_spacing` | `2.0` | `2.0` | 静态多边形边长。 |
| `default_dynamic_ring_radius` | `2.0` | `2.0` | 动态圆环半径。 |
| `default_dynamic_ring_move_speed` | `0.5` | `0.4` | 动态圆环速度。 |
| `default_dynamic_lemniscate_x_radius` | `3.0` | `3.0` | 8 字 X 尺度。 |
| `default_dynamic_lemniscate_y_radius` | `2.0` | `2.0` | 8 字 Y 尺度。 |

### 运行前检查

启动集群控制前，建议先确认单机链路已经正常：

```bash
rostopic echo /uav1/sunray/localization/local_odom
rostopic echo /uav1/sunray/uav_control/control_state
rostopic echo /ugv1/sunray/localization/local_odom
rostopic echo /ugv1/sunray/ugv_control/control_state
```

启动集群后，检查：

```bash
rostopic echo /sunray/swarm/uav_swarm_state
rostopic echo /sunray/swarm/ugv_swarm_state
rostopic echo /sunray/swarm/uav_swarm_cmd
rostopic echo /sunray/swarm/ugv_swarm_cmd
```

如果 `self_odom_ready=false`，说明本机定位没有输入。如果 `peers_odom_ready=false`，说明至少一个邻居 odom 没有收到或超时。

### 从零启动示例

下面给出一个典型“6 架 UAV 仿真集群”的启动顺序。具体仿真器和定位源可以替换，但顺序建议保持：先仿真/定位，再单机控制，再集群控制，最后发送集群命令。

1. 启动仿真器或动力学：

```bash
roslaunch <sim_package> <multi_uav_sim.launch>
```

2. 启动每架 UAV 的定位融合：

```bash
roslaunch localization_fusion localization_fusion_swarm.launch \
  source_id:=5 \
  agent_name:=uav \
  agent_num:=6
```

3. 启动每架 UAV 的单机控制：

```bash
roslaunch sunray_uav_control uav_control_swarm.launch \
  agent_name:=uav \
  agent_num:=6
```

4. 启动 UAV 集群控制：

```bash
roslaunch sunray_swarm_control swarm_control_uav_sim.launch \
  agent_name:=uav \
  swarm_num:=6
```

5. 启动 Qt 集群控制面板：

```bash
roslaunch sunray_swarm_control swarm_control_panel.launch
```

或启动终端命令发布工具：

```bash
roslaunch sunray_swarm_control uav_swarm_cmd_pub_terminal.launch
```

UGV 集群类似：

```bash
roslaunch localization_fusion localization_fusion_swarm.launch \
  source_id:=5 \
  agent_name:=ugv \
  agent_num:=6

roslaunch sunray_ugv_control ugv_control_swarm.launch \
  agent_name:=ugv \
  agent_num:=6 \
  airframe:=mecanum_default

roslaunch sunray_swarm_control swarm_control_ugv_sim.launch \
  agent_name:=ugv \
  swarm_num:=6

roslaunch sunray_swarm_control ugv_swarm_cmd_pub_terminal.launch
```

### 单独启动某个 agent

真机或分布式仿真中，可以只启动本机对应的 swarm 节点。当前仓库提供的 sim launch 是批量启动形式；如果要单独启动，可以参考 launch 中的 node 配置，直接启动节点并加载 YAML：

```bash
rosrun sunray_swarm_control swarm_control_uav_node \
  _agent_name:=uav \
  _agent_id:=1 \
  _swarm_num:=6
```

实际项目中更建议写一个真机专用 launch：

```xml
<launch>
  <arg name="agent_name" default="uav" />
  <arg name="agent_id" default="1" />
  <arg name="swarm_num" default="6" />
  <arg name="param_file" default="$(find sunray_swarm_control)/launch/swarm_control_uav_sim.yaml" />

  <node pkg="sunray_swarm_control"
        type="swarm_control_uav_node"
        name="$(arg agent_name)$(arg agent_id)_swarm_control"
        output="screen">
    <rosparam command="load" file="$(arg param_file)" />
    <param name="agent_name" value="$(arg agent_name)" />
    <param name="agent_id" type="int" value="$(arg agent_id)" />
    <param name="swarm_num" type="int" value="$(arg swarm_num)" />
  </node>
</launch>
```

UGV 只需把节点类型换成 `swarm_control_ugv_node`，参数文件换成 `swarm_control_ugv_sim.yaml`。

### 话题命名常见误区

集群命令话题是全局统一的：

```text
/sunray/swarm/uav_swarm_cmd
/sunray/swarm/ugv_swarm_cmd
```

单机控制话题是每个 agent 独立的：

```text
/uav1/sunray/uav_control/control_cmd
/uav2/sunray/uav_control/control_cmd
/ugv1/sunray/ugv_control/control_cmd
/ugv2/sunray/ugv_control/control_cmd
```

集群状态话题当前也是全局统一的：

```text
/sunray/swarm/uav_swarm_state
/sunray/swarm/ugv_swarm_state
```

每个 agent 都会往同一个 state 话题发布自己的状态，monitor/RViz/Qt panel 通过 `agent_id` 区分它们。

如果你在二次开发时新增工具节点，建议遵守这个命名结构，不要为每个 agent 新建不同的 swarm command 话题，否则广播、面板和桥接都会变复杂。

### launch 解析检查

修改 launch 后，可以先不启动节点，只检查 roslaunch 能否解析：

```bash
roslaunch --files sunray_swarm_control swarm_control_uav_sim.launch \
  swarm_num:=3 agent_name:=uav

roslaunch --files sunray_swarm_control swarm_control_ugv_sim.launch \
  swarm_num:=3 agent_name:=ugv
```

如果这里失败，优先检查 package 名、launch 文件名、参数语法和 YAML 路径。

### 编译说明和验证

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

推荐验证顺序：

| 步骤 | 命令/检查 |
| --- | --- |
| 编译 | `catkin_make --source swarm --build build/swarm -j4` |
| launch 解析 | `roslaunch --files ...` |
| 节点启动 | 启动 UAV/UGV sim launch，观察是否报参数错误。 |
| 状态话题 | `rostopic echo /sunray/swarm/uav_swarm_state` |
| 命令话题 | 终端工具或 Qt 面板发命令。 |
| RViz | 检查 marker、目标点、轨迹和障碍物是否显示。 |
| ORCA 测试 | `roslaunch sunray_swarm_control orca_test.launch` |

</section>
