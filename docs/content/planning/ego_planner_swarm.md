<!-- title: ego_planner_swarm -->

<section id="ego-planner-swarm">

## ego_planner_swarm

`planning/source_planners/ego_planner_swarm` 是 EGO planner 源码区。Sunray 当前通过 `sunray_planning` 中的 `EgoPlanner` adapter 接入它。

### 目录结构

```text
planning/source_planners/ego_planner_swarm/
├── bspline_opt
├── drone_detect
├── path_searching
├── plan_env
├── plan_manage
├── rosmsg_tcp_bridge
└── traj_utils
```

| 子包 | 作用 |
| --- | --- |
| `plan_manage` | EGO planner 主状态机、traj server 和启动文件。 |
| `plan_env` | 占据栅格地图、传感器输入和地图管理。 |
| `path_searching` | kinodynamic/A* 等搜索模块。 |
| `bspline_opt` | B 样条轨迹优化。 |
| `traj_utils` | 轨迹消息和工具。 |
| `drone_detect` | 多机检测相关工具。 |
| `rosmsg_tcp_bridge` | ROS 消息 TCP 桥。 |

### Sunray 接入关系

```text
sunray_planning
  -> /uav1/sunray/planning/ego_planner/target_point
  -> EGO planner
  -> /uav1/sunray/planning/ego_planner/position_cmd
  -> sunray_planning
  -> UAVControlCMD(MOVE_TRAJECTORY)
```

`EgoPlanner` adapter 代码：

```text
planning/sunray_planning/src/planner_interface/ego_planner.cpp
planning/sunray_planning/include/planner_interface/ego_planner.hpp
```

### 常用启动

推荐从 Sunray 统一 launch 启动：

```bash
roslaunch sunray_planning sunray_planning.launch planner_type:=ego
```

该 launch 会 include EGO planner 的：

```text
$(find ego_planner)/launch/exp/run_exp_single_lio.launch
```

并传入：

```text
agent_name
agent_id
use_private_agent_key
odom_topic
cloud_topic
enable_rviz
```

### Sunray 相关 launch

仓库中还有 EGO 的 Sunray 适配 launch：

```text
planning/source_planners/ego_planner_swarm/plan_manage/launch_sunray/run_in_sim.launch
planning/source_planners/ego_planner_swarm/plan_manage/launch_sunray/rviz.launch
planning/source_planners/ego_planner_swarm/plan_manage/launch_sunray/sunray_ego_exp.launch
planning/source_planners/ego_planner_swarm/plan_manage/launch_sunray/sunray_ego_sim_with_rviz.launch
planning/source_planners/ego_planner_swarm/plan_manage/launch_sunray/sunray_map_generator.launch
```

### 调试重点

- `odom_topic` 和 `cloud_topic` 必须与定位/点云模块一致。
- planner 内部通常假设 odom 和点云在同一坐标系下，frame 不一致会导致轨迹异常。
- 先确认 `/uav1/sunray/planning/ego_planner/target_point` 能收到目标点。
- 再确认 `/uav1/sunray/planning/ego_planner/position_cmd` 持续输出。
- 最后检查 `/uav1/sunray/uav_control/control_cmd` 是否变成 `MOVE_TRAJECTORY`。

### 修改建议

普通 Sunray 适配优先改 `sunray_planning` 的 adapter，不建议直接改 EGO 核心算法。只有确认是算法包内部 bug 或参数问题时，再进入 `plan_env`、`path_searching`、`bspline_opt` 等子包。

</section>
