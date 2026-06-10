# Planning TODO

这个文件记录阅读 `planning` 目录后建议优先处理的代码改进项，重点面向 Sunray 自有适配层。第三方规划器源码建议尽量少改，除非明确是上游算法包问题。

## 1. 统一话题和 frame 参数化

当前仍有一些话题和 frame 在代码中直接拼接或硬编码，例如：

- `EgoPlanner` / `DiffPlanner` 中固定使用 `${agent_key}/sunray/planning/.../target_point` 和 `position_cmd`。
- `EgoPlanner::send_goal()` 和 `DiffPlanner::send_goal()` 固定使用 `world` 作为目标点 frame。
- `rviz_goal_bridge` 中 `planning_cmd_topic` 由代码拼接，launch 中不能直接覆盖。
- `point_cloud_transform.launch` 默认话题仍写死为 `/uav1/...`。

建议所有对外话题和关键 frame 都支持 launch 参数，并保留 `${agent_key}` 模板替换能力。

## 2. 修正 launch 参数传递细节

`sunray_planning.launch` 中 Diff planner include 里使用了：

```xml
<param name="enable_rviz" value="$(arg enable_rviz)" />
```

这里更像是想把参数传给被 include 的 launch，建议改为 `<arg>`，并检查 diff planner 的 `run_exp_single_lio.launch` 是否确实声明了该参数。

## 3. 多 waypoint 任务支持

当前 `PlanningFSM::planning_cmd_callback()` 收到多个 waypoint 时只转发第一个，并打印 warning。

建议补齐：

- waypoint 队列管理。
- 当前 waypoint 到达判定。
- 到达后自动发送下一个 waypoint。
- `hold_time` 生效逻辑。
- `UAVPlanningState` 中暴露当前 waypoint index 和总数。

## 4. planner 状态语义统一

EGO 和 Diff 当前都通过 `trajectory_flag` 推断 planner 状态，但状态语义比较粗：

- Diff planner 没有独立 state timeout 检查。
- EGO / Diff 的 `SUCCESS` 依赖 `TRAJECTORY_STATUS_COMPLETED`，不同 planner 对完成状态的发布时机可能不一致。
- `TRAJECTROY_STATUS_ABORT` 拼写错误来自消息定义，后续需要考虑兼容迁移。

建议定义一套 Sunray 内部 planner 状态规范，并在每个 adapter 中显式映射。

## 5. 增加输入消息校验

规划命令目前主要检查 waypoint 是否为空。建议继续补充：

- waypoint 坐标是否有限值。
- yaw 是否有限值。
- `plan_cmd_source` 是否为空。
- local/global 目标是否和 header frame 一致。
- 目标高度是否在合理范围。

这些检查应在拒绝目标时给出清晰日志，方便现场调试。

## 6. 点云转换工具需要更强保护

`point_cloud_transform` 当前在收到点云时直接使用最近一次 odom 生成的 `transform`。如果点云先于 odom 到达，或 odom 长时间未更新，可能发布错误点云。

建议增加：

- 是否已收到 odom 的标志。
- odom 超时阈值。
- 点云和 odom 时间差检查。
- 支持使用 TF buffer 按时间戳查询变换，而不是只缓存最近一次 odom。

## 7. RViz goal bridge 可扩展性

`rviz_goal_bridge` 当前固定发布到 `${agent_key}/sunray/uav_planning/planning_cmd`，并默认使用当前无人机高度作为目标高度。

建议：

- `planning_cmd_topic` 支持参数覆盖。
- `tf_lookup_timeout_sec` 和 `default_hold_time` 暴露到 launch。
- 增加目标点预览 marker，便于确认转换后的 local/global 目标。
- 对 unsupported frame 的日志增加建议 frame 名称和当前 TF 状态。

## 8. FUEL / SUPER 适配状态明确化

`PlannerType` 已包含 `FUEL` 和 `SUPER`，但 `PlanningFSM::init()` 会直接抛出“不支持”异常。

建议二选一：

- 如果短期不支持，在 README 和 launch 参数说明中明确仅支持 `ego` / `diff`。
- 如果计划支持，先建立空 adapter 骨架和对应 topic contract，再逐步接入。

## 9. 第三方源码和 Sunray wrapper 边界

`source_planners` 中保留了多个完整第三方工作区。建议明确维护规则：

- 第三方核心算法尽量不直接修改。
- Sunray 适配逻辑放在 `sunray_planning` 或单独 wrapper 包。
- 对确实修改过的第三方源码，建立 patch 说明或变更记录。

这样后续升级 EGO / Diff / FUEL / SUPER 时，成本会更低。

## 10. 文档和示例需要持续同步

建议补充最小可运行示例：

- 发布一个 `PLAN_LOCAL_GOAL` 的 Python/C++ 示例。
- RViz goal bridge 的 frame 配置示例。
- EGO 和 Diff 的典型启动命令。
- 常见话题检查命令。

这些内容能显著降低新开发者第一次跑通规划链路的成本。
