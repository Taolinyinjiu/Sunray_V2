<!-- title: 示例程序总览 -->

<section id="examples-overview">

## 示例程序总览

`examples` 目录是新手二次开发最推荐复制的入口。它里面的代码一般不是底层算法，而是“上层任务节点”：订阅 Sunray 状态话题，发布 `sunray_msgs` 中的控制或规划命令。

### 示例分类

| 示例目录 | 类型 | 学习重点 |
| --- | --- | --- |
| `examples/sunray_uav_control_example` | 无人机控制示例 | 如何发布 `UAVControlCMD`，如何区分起降/位置/速度/轨迹命令。 |
| `examples/sunray_ugv_control_example` | 无人车控制示例 | 如何发布 `UGVControlCMD`，如何处理点位、车体系速度和差速/麦克纳姆差异。 |
| `examples/sunray_uav_planning_example` | 无人机规划示例 | 如何把外部规划器输出转换为 Sunray UAV 控制命令。 |
| `examples/sunray_swarm_control_example` | 集群控制示例 | 当前仅保留目录占位，后续可补充 UAV/UGV 集群任务示例。 |

### 推荐学习顺序

1. 先看无人机或无人车控制示例，理解最小任务节点怎么写。
2. 再看控制模块文档，理解 `control_cmd` 到底层控制器的关系。
3. 需要规划时看无人机规划示例，理解 `position_cmd -> UAVControlCMD` 的桥接方式。
4. 需要集群时先看 `sunray_msgs` 中的 swarm 消息，再补充集群示例。

### 通用开发原则

- 起飞、降落、悬停、点位这类事件型命令通常发一次。
- 速度、轨迹这类连续参考命令必须持续发布。
- 示例节点不要绕过 Sunray 控制层直接发 MAVROS 或底盘驱动。
- 新任务优先复制最接近的示例，再改目标点、速度、状态判断和任务流程。

</section>
