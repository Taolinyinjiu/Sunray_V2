<!-- title: sunray_diff_adapter -->

<section id="sunray-diff-adapter">

## sunray_diff_adapter

`planning/third_party_planner_examples/diff_planner_example/sunray_diff_adapter` 是 Diff planner 到 Sunray 控制接口的适配包目录。

### 功能定位

当前 Diff 的主要接入逻辑在 `planning/sunray_planner/sunray_planning` 的 `DiffPlanner` adapter 中，`sunray_diff_adapter` 保留为独立适配节点目录。后续如果希望让 Diff 示例脱离 `sunray_planning` 状态机单独运行，可以在该包内实现消息桥接节点。

### 建议职责

`sunray_diff_adapter` 后续建议只处理接口转换，不直接修改 Diff planner 核心算法：

1. 订阅 Diff planner 的目标轨迹或 `position_cmd` 输出。
2. 转换为 `sunray_msgs/UAVControlCMD` 或 Sunray 统一规划中间消息。
3. 对齐 `agent_name`、`agent_id`、frame 和话题前缀。
4. 保持起飞、降落、状态机和安全保护仍由 Sunray 控制/规划主链路处理。

### 相关源码

当前可参考的 Diff 接入代码：

```text
planning/sunray_planner/sunray_planning/src/planner_interface/diff_planner.cpp
planning/sunray_planner/sunray_planning/include/planner_interface/diff_planner.hpp
```

Diff planner 源码架构见同级页面“Diff规划器源码架构”。

</section>
