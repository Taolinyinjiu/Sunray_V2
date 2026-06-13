<!-- title: diff_planner -->

<section id="diff-planner">

## diff_planner

`planning/source_planners/diff_planner` 是 Diff planner 源码区。当前 Sunray 把它视为 EGO 的另一个 planner 适配目标，通过 `sunray_planning` 中的 `DiffPlanner` adapter 接入。

### 目录结构

```text
planning/source_planners/diff_planner/
├── drone_detect
├── path_searching
├── plan_env
├── plan_manage
├── swarm_bridge
├── traj_opt
└── traj_utils
```

| 子包 | 作用 |
| --- | --- |
| `plan_manage` | Diff planner 主流程和启动文件。 |
| `plan_env` | 环境地图、传感器输入。 |
| `path_searching` | 路径搜索。 |
| `traj_opt` | 轨迹优化。 |
| `traj_utils` | 轨迹消息和工具。 |
| `swarm_bridge` | 集群通信桥，包含 TCP/UDP launch。 |
| `drone_detect` | 多机检测相关工具。 |

### Sunray 接入关系

```text
sunray_planning
  -> /uav1/sunray/planning/diff_planner/target_point
  -> Diff planner
  -> /uav1/sunray/planning/diff_planner/position_cmd
  -> sunray_planning
  -> UAVControlCMD(MOVE_TRAJECTORY)
```

`DiffPlanner` adapter 代码：

```text
planning/sunray_planning/src/planner_interface/diff_planner.cpp
planning/sunray_planning/include/planner_interface/diff_planner.hpp
```

### 常用启动

```bash
roslaunch sunray_planning sunray_planning.launch planner_type:=diff
```

开启前向视野裁剪：

```bash
roslaunch sunray_planning sunray_planning.launch planner_type:=diff front_vis:=true
```

`sunray_planning.launch` 会 include：

```text
$(find diff_planner)/launch/exp/run_exp_single_lio.launch
```

并传入：

```text
odom_topic
cloud_topic
front_vis
```

当前 launch 中 `enable_rviz` 是用 `<param>` 传入 include 区块。维护时建议检查被 include 的 launch 是否声明了同名参数；如果目的是把参数传给子 launch，通常应使用 `<arg>`。

### 编译注意事项

原 `sunray_planning/如何使用sunray_planning.md` 中记录过一个临时问题：EGO 和 Diff 同时编译/启动时，Diff planner 可能出现动态库连接问题。历史解决方式是先单独编译 Diff planner，再编译 Ego planner。当前是否仍存在，需要以当前工作空间编译结果为准。

### 调试重点

```bash
rostopic echo /uav1/sunray/planning/diff_planner/target_point
rostopic echo /uav1/sunray/planning/diff_planner/position_cmd
rostopic echo /uav1/sunray/uav_planning/planning_state
rostopic echo /uav1/sunray/uav_control/control_cmd
```

Diff planner 对点云视野、odom/点云 frame 一致性也很敏感。如果 planner 没有输出，先检查输入点云、里程计和目标点，而不是先改控制器。

</section>
