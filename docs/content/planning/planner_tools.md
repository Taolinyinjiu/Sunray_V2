<!-- title: planner_tools -->

<section id="planner-tools">

## planner_tools

`planner_tools` 是规划模块的辅助工具包，当前包含 RViz 目标点桥接和点云坐标转换两个节点。它们不实现 planner 算法，只负责把外部输入整理成规划器需要的消息和坐标系。

### 目录结构

```text
planning/third_party_planner_examples/planner_tools/
├── launch/
│   ├── point_cloud_transform.launch
│   └── rviz_goal_bridge.launch
└── src/
    ├── point_cloud_transform.cpp
    └── rviz_goal_bridge.cpp
```

### rviz_goal_bridge

启动：

```bash
roslaunch planner_tools rviz_goal_bridge.launch agent_name:=uav agent_id:=1
```

数据流：

```text
/move_base_simple/goal
  -> rviz_goal_bridge
  -> /uav1/sunray/uav_planning/planning_cmd
```

输入输出：

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz 2D Nav Goal / 2D Goal Pose。 |
| 订阅 | `/uav1/sunray/localization/local_odom` | `nav_msgs/Odometry` | 当 `use_agent_height=true` 时读取当前高度。 |
| 发布 | `/uav1/sunray/uav_planning/planning_cmd` | `sunray_msgs/UAVPlanningCMD` | 转换后的 Sunray 规划命令。 |

launch 参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `agent_name` | `uav` | 智能体名前缀。 |
| `agent_id` | `1` | 智能体编号。 |
| `use_private_agent_key` | `false` | 是否使用私有 agent key。 |
| `rviz_topic` | `/move_base_simple/goal` | RViz 目标点输入话题。 |
| `use_agent_height` | `true` | 是否使用无人机当前高度作为目标高度。 |
| `fixed_goal_height` | `0.6` | 不使用当前高度时的固定目标高度。 |
| `world_goal_plan_frame` | `global` | RViz 目标点 frame 为 `world` 时，转换为 global 还是 local 目标。 |

支持的输入 frame：

```text
world
${agent_key}/sunray_global
${agent_key}/sunray_local
```

当输入是 `world` frame 时，由 `world_goal_plan_frame` 决定生成 `PLAN_GLOBAL_GOAL` 还是 `PLAN_LOCAL_GOAL`。

### point_cloud_transform

启动：

```bash
roslaunch planner_tools point_cloud_transform.launch   input_point_topic:=/uav1/livox/lidar   output_point_topic:=/uav1/global_points   odom_topic:=/uav1/sunray/gazebo_pose
```

数据流：

```text
点云 + odom
  -> point_cloud_transform
  -> 指定 frame 下的点云
```

输入输出：

| 方向 | 话题 | 类型 | 默认值 |
| --- | --- | --- | --- |
| 订阅 | `input_point_topic` | `sensor_msgs/PointCloud2` | `/uav1/livox/lidar` |
| 订阅 | `odom_topic` | `nav_msgs/Odometry` | `/uav1/sunray/gazebo_pose` |
| 发布 | `output_point_topic` | `sensor_msgs/PointCloud2` | `/uav1/global_points` |

参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `frame_id` | `world` | 输出点云 frame。 |
| `child_frame_id` | `uav1/base_link` | odom 对应的机体 frame。 |

### 二次开发注意事项

- `rviz_goal_bridge` 当前 `planning_cmd_topic` 在代码中由 `${agent_key}/sunray/uav_planning/planning_cmd` 拼接，launch 里不能直接覆盖；如果后续需要多套桥接节点并行运行，建议把它改成可配置参数。
- `point_cloud_transform` 当前使用最近一次 odom 生成 transform，点云先到或 odom 长时间不更新时可能发布错误点云，后续建议增加 odom 超时和时间差检查。
- 如果底层 planner 要求点云在 world frame，务必确认输入点云和 odom 的坐标系语义一致。

</section>
