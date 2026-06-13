<!-- title: sunray_planner_msgs -->

<section id="sunray-planner-msgs">

## sunray_planner_msgs

`sunray_planner_msgs` 是规划适配层的消息包，主要承接 EGO / Diff 等外部 planner 输出。它不是最终发给控制模块的控制消息，最终控制命令仍然是 `common/sunray_msgs/msg/UAVControlCMD.msg`。

### 目录结构

```text
planning/sunray_planner_msgs/
├── msg/
│   ├── DiffGoalSet.msg
│   ├── DiffPositionCommand.msg
│   └── EgoPositionCommand.msg
├── CMakeLists.txt
└── package.xml
```

### 消息说明

| 消息 | 作用 | 主要用途 |
| --- | --- | --- |
| `EgoPositionCommand.msg` | EGO planner 输出的位置、速度、加速度、yaw 等轨迹指令。 | 被 `sunray_planning` 的 `EgoPlanner` 适配器订阅。 |
| `DiffPositionCommand.msg` | Diff planner 输出，字段类似 EGO，额外包含 jerk。 | 被 `sunray_planning` 的 `DiffPlanner` 适配器订阅。 |
| `DiffGoalSet.msg` | Diff planner 目标点集合相关消息。 | 给 Diff planner 内部或适配逻辑使用。 |

### 和 Sunray 控制消息的关系

```text
EgoPositionCommand / DiffPositionCommand
  -> sunray_planning adapter
  -> PlannerPositionCommand
  -> sunray_msgs/UAVControlCMD(MOVE_TRAJECTORY)
  -> sunray_uav_control
```

二次开发时，如果接入新的 planner，优先在 `sunray_planning` 中新增 adapter，把新 planner 的输出转成 `PlannerPositionCommand`。只有当已有消息完全无法表达新 planner 输出时，才建议新增 `sunray_planner_msgs` 消息。

### 修改注意事项

- 修改消息后需要重新编译依赖它的包。
- 消息字段变更会影响 `sunray_planning`、底层 planner adapter 和示例程序。
- 如果只是对外发控制命令，不需要使用这个包，直接使用 `sunray_msgs/UAVControlCMD`。

</section>
