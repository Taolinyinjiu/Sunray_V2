<!-- title: 集群控制示例 -->

<section id="examples-swarm-control">

## 集群控制示例

路径：

```text
examples/sunray_swarm_control_example
```

当前该目录只有 `.gitkeep`，说明它是预留目录，还没有可运行示例节点或 launch。

### 后续建议补充内容

后续如果完善集群示例，建议按下面结构组织：

```text
examples/sunray_swarm_control_example/
├── README.md
├── launch/
│   ├── uav_swarm_takeoff_land.launch
│   ├── uav_swarm_formation.launch
│   └── ugv_swarm_formation.launch
└── src/
    ├── uav_swarm_takeoff_land.cpp
    ├── uav_swarm_formation.cpp
    └── ugv_swarm_formation.cpp
```

示例应重点演示：

- 如何发布 `sunray_msgs/UAVSwarmCMD`。
- 如何发布 `sunray_msgs/UGVSwarmCMD`。
- 如何填写 `Formation`。
- 如何等待 `UAVSwarmState` 或 `UGVSwarmState`。
- 集群命令中 `agent_id=99` 表示全体响应的语义。

### 二次开发提醒

在示例补齐前，学习集群接口应先看：

- `common/sunray_msgs` 中的 `Formation`、`UAVSwarmCMD`、`UGVSwarmCMD`。
- `swarm` 模块文档。
- 控制示例中单机命令的发布方式。

</section>
