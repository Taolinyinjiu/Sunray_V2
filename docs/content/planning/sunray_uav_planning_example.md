<!-- title: 无人机规划示例 -->

<section id="examples-uav-planning">

## 无人机规划示例

路径：

```text
planning/sunray_uav_planning_example
```

`sunray_uav_planning_example` 用于验证外部规划器与 Sunray 无人机控制链路的集成。当前示例以 `ego_planner` 为规划器：`ego_planner` 根据定位和点云生成 `position_cmd`，`ego_example` 中的 `Ego2Sunray` 节点再把该指令转换为 `sunray_msgs/UAVControlCMD`，发送给 `sunray_uav_control` 执行。

当前主要覆盖已经测试过的 Gazebo 仿真流程。

### 总体数据流

```text
/uav1/sunray/localization/local_odom
        + /uav1/global_points
        -> ego_planner_node
        -> traj_server
        -> /uav1/sunray/planning/ego_planner/position_cmd
        -> Ego2Sunray
        -> /uav1/sunray/uav_control/control_cmd
        -> sunray_uav_control
```

关键包和文件：

| 路径 | 说明 |
| --- | --- |
| `ego_example/src/Ego2Sunray.cpp` | 把 EGO `position_cmd` 转为 `UAVControlCMD`。 |
| `ego_example/launch/Ego2Sunray.launch` | 桥接节点启动文件。 |
| `source_planners/ego-planner-swarm` | EGO planner 源码。 |
| `source_planners/ego-planner-swarm/plan_manage/launch/sim/run_gazebo_lio.launch` | 当前 Gazebo LIO 推荐入口。 |
| `source_planners/planner_msgs` | 示例规划器内部消息包，不要和 `planning/sunray_planner_msgs` 混淆。 |

### Gazebo 仿真机制

仿真环境使用：

```bash
roslaunch sunray_simulator sunray_sim_uav_planning.launch
```

该 launch 会完成两件事：

- 启动 Gazebo，并加载 `planning_test.world`。
- 启动一架带 Mid360/Livox 点云传感器的 Sunray 无人机模型，默认命名空间为 `/uav1`。

`sunray_sim_uav_planning.launch` 还应包含一个点云坐标转换工具：

```xml
<!-- 将 Mid360 传感器坐标系下的点云转换到 world 坐标系。 -->
<include file="$(find sunray_planner_tools)/launch/point_cloud_transform.launch">
    <arg name="input_point_topic" value="/uav$(arg uav_id)/livox/lidar" />
    <arg name="output_point_topic" value="/uav$(arg uav_id)/global_points" />
    <arg name="odom_topic" value="/uav$(arg uav_id)/sunray/gazebo_pose" />
    <arg name="frame_id" value="world" />
    <arg name="child_frame_id" value="uav$(arg uav_id)/base_link" />
</include>
```

如果当前仿真 launch 中还没有这段 include，可以单独启动：

```bash
roslaunch sunray_planner_tools point_cloud_transform.launch \
  input_point_topic:=/uav1/livox/lidar \
  output_point_topic:=/uav1/global_points \
  odom_topic:=/uav1/sunray/gazebo_pose \
  frame_id:=world \
  child_frame_id:=uav1/base_link
```

该工具将传感器坐标系下的点云：

```text
/uav1/livox/lidar
```

结合 Gazebo 位姿：

```text
/uav1/sunray/gazebo_pose
```

转换到 `world` 坐标系，并发布为：

```text
/uav1/global_points
```

Gazebo LIO 版本的 EGO planner 默认使用 `/uav1/global_points` 作为点云输入，因此该转换节点是当前仿真规划链路中的必要环节。

### Ego2Sunray 工作机理

`ego_example` 是本 planning example 的配套桥接工具，不是 Sunray 主线规划工具。核心节点为：

```text
ego_example/src/Ego2Sunray.cpp
```

启动文件为：

```text
ego_example/launch/Ego2Sunray.launch
```

`Ego2Sunray` 的工作流程如下：

1. 订阅 `ego_planner` 的 `traj_server` 输出：

   ```text
   /<agent_name><agent_id><planner_prefix>/position_cmd
   ```

   默认单机仿真中通常为：

   ```text
   /uav1/sunray/planning/ego_planner/position_cmd
   ```

2. 接收 `planner_msgs/EgoPositionCommand`，读取其中的期望位置、速度、加速度、偏航角和偏航角速度。

3. 转换为 `sunray_msgs/UAVControlCMD`：

| `planner_msgs/EgoPositionCommand` | `sunray_msgs/UAVControlCMD` |
| --- | --- |
| `position` | `desired_pos` |
| `velocity` | `desired_vel` |
| `acceleration` | `desired_acc` |
| `yaw` | `desired_yaw` |
| `yaw_dot` | `desired_yaw_rate` |
| 固定逻辑 | `cmd_source=PLANNING`，`control_cmd=MOVE_TRAJECTORY` |

4. 发布到 Sunray 控制模块：

   ```text
   /<agent_name><agent_id>/sunray/uav_control/control_cmd
   ```

   默认单机仿真中为：

   ```text
   /uav1/sunray/uav_control/control_cmd
   ```

`Ego2Sunray.launch` 只从调用它的 launch 中读取 `agent_name`、`agent_id` 和 `planner_prefix`，并在 launch 内拼接话题。它不读取全局 `/agent_name`、`/agent_id`，也不依赖 Sunray 主线工具中的 `agent_key` 机制。

### EGO Planner 启动方式

EGO planner 源码放在：

```text
source_planners/ego-planner-swarm
```

常用入口 launch 位于：

```text
source_planners/ego-planner-swarm/plan_manage/launch
```

当前 Gazebo 仿真建议使用 LIO 入口：

```bash
roslaunch ego_planner run_gazebo_lio.launch agent_name:=uav agent_id:=1
```

`run_gazebo_lio.launch` 会将 `agent_name`、`agent_id` 和 `planner_prefix` 拼成完整规划话题前缀，例如：

```text
/uav1/sunray/planning/ego_planner
```

随后它会 include：

```text
ego_planner/launch/include/advanced_param.xml
```

`advanced_param.xml` 启动两个核心节点：

| 节点 | 作用 |
| --- | --- |
| `ego_planner_node` | 负责建图、搜索和轨迹优化。 |
| `traj_server` | 将优化出的 B-spline 轨迹采样为 `position_cmd`。 |

`run_gazebo_lio.launch` 还会 include：

```text
ego_example/launch/Ego2Sunray.launch
```

因此在启动 planner 本体后，`position_cmd -> UAVControlCMD` 的转换节点会自动启动。

### Gazebo 仿真流程

以下流程以单机 `/uav1` Gazebo 仿真为例。

1. 启动仿真环境：

```bash
roslaunch sunray_simulator sunray_sim_uav_planning.launch
```

该步骤启动 Gazebo、无人机模型和点云坐标转换工具。确认 `/uav1/global_points` 有数据后，再继续启动规划链路。

如果需要单独启动点云转换：

```bash
roslaunch sunray_planner_tools point_cloud_transform.launch \
  input_point_topic:=/uav1/livox/lidar \
  output_point_topic:=/uav1/global_points \
  odom_topic:=/uav1/sunray/gazebo_pose \
  frame_id:=world \
  child_frame_id:=uav1/base_link
```

2. 启动定位融合：

```bash
roslaunch localization_fusion localization_fusion.launch source_id:=3 agent_name:=uav agent_id:=1
```

`source_id:=3` 对应 Gazebo 定位源。定位融合会输出：

```text
/uav1/sunray/localization/local_odom
```

该话题会作为 EGO planner 的里程计输入。

3. 启动 Sunray 无人机控制：

```bash
roslaunch sunray_uav_control uav_control.launch agent_name:=uav agent_id:=1
```

控制模块订阅：

```text
/uav1/sunray/uav_control/control_cmd
```

后续起飞、降落和规划轨迹都会通过该控制指令话题进入控制模块。

4. 启动 uav_control_tools 发送起飞/降落指令：

```bash
roslaunch uav_control_tools uav_control_panel.launch uav_id:=1
```

使用该工具先发送起飞指令，使无人机进入可执行轨迹控制的状态。任务结束后也可以通过该工具发送降落指令。

5. 启动 planner 本体：

```bash
roslaunch ego_planner run_gazebo_lio.launch agent_name:=uav agent_id:=1
```

启动后，在 RViz 中通过 `2D Nav Goal` 或对应目标点工具发送目标点。EGO planner 会生成轨迹，`Ego2Sunray` 会持续将轨迹采样指令转换为 Sunray 控制指令。

### 常用检查话题

按顺序检查：

```bash
rostopic echo /uav1/global_points
rostopic echo /uav1/sunray/localization/local_odom
rostopic echo /uav1/sunray/uav_control/control_state
rostopic echo /uav1/sunray/planning/ego_planner/position_cmd
rostopic echo /uav1/sunray/uav_control/control_cmd
```

如果 `/uav1/global_points` 没有数据，优先检查 Gazebo 点云和 `point_cloud_transform`。

如果 `position_cmd` 有数据但 `control_cmd` 没有数据，检查 `Ego2Sunray` 是否启动、`agent_name/agent_id/planner_prefix` 是否一致。

如果 `control_cmd` 有 `MOVE_TRAJECTORY` 但无人机不动，检查 `sunray_uav_control` 是否已起飞并处于可执行 MOVE/轨迹的状态。

### 注意事项

- 当前说明只覆盖已经测试过的 Gazebo 仿真链路。
- `run_gazebo_lio.launch` 默认点云输入为 `/uav1/global_points`，该话题来自 `sunray_sim_uav_planning.launch` 中的点云坐标转换工具。
- `Ego2Sunray` 是 example 级桥接工具，只负责消息格式转换和话题衔接，不承担状态机、起飞、降落或安全保护逻辑。
- 启动 planner 前应先完成起飞，否则控制模块可能无法执行 `MOVE_TRAJECTORY` 指令。
- `planner_msgs` 是示例规划器内部消息，不要和 `planning/sunray_planner_msgs` 混淆。

</section>
