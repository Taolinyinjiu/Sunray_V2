<section id="sunray-mocap">

## sunray_mocap

`sunray_mocap` 是动捕适配包，用来把 VRPN 发布的 `PoseStamped` 和 `TwistStamped` 同步成 `nav_msgs/Odometry`。它通常用于室内动捕实验，并作为 `localization_fusion` 的 `source_id=1` 输入。

### 数据流

以 `uav1` 为例：

```text
/vrpn_client_node/uav1/pose
/vrpn_client_node/uav1/twist
  -> sunray_mocap/mocap_odom_node
  -> /uav1/sunray/odometry
  -> localization_fusion source_id=1
  -> /uav1/sunray/localization/local_odom
```

### 代码结构

```text
localization/sunray_mocap/
├── launch/
│   └── mocap_odom.launch
└── src/
    └── mocap_odom_node.cpp
```

`mocap_odom_node.cpp` 的主要逻辑：

1. 读取参数 `uav_id`，默认 `1`。
2. 订阅 `/vrpn_client_node/uav{uav_id}/pose`。
3. 订阅 `/vrpn_client_node/uav{uav_id}/twist`。
4. 使用 `message_filters::Synchronizer` 和 `ApproximateTime` 做近似同步。
5. 同步成功后发布 `/uav{uav_id}/sunray/odometry`。

### 输入输出

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/vrpn_client_node/uav1/pose` | `geometry_msgs/PoseStamped` | 动捕系统输出的位置和姿态。 |
| 订阅 | `/vrpn_client_node/uav1/twist` | `geometry_msgs/TwistStamped` | 动捕系统输出的线速度和角速度。 |
| 发布 | `/uav1/sunray/odometry` | `nav_msgs/Odometry` | 同步后的动捕 odom，供 `localization_fusion` 使用。 |

发布的 odom 当前使用：

```text
header.frame_id: world
child_frame_id: base_link
```

接入 `localization_fusion` 后，会被重新规范化成：

```text
uav1/sunray_local -> uav1/base_link
```

### 启动方式

单独启动适配节点：

```bash
roslaunch sunray_mocap mocap_odom.launch
```

通过 `localization_fusion` 启动完整动捕定位链路：

```bash
roslaunch localization_fusion localization_fusion.launch source_id:=1 agent_name:=uav agent_id:=1
```

`source_id=1` 时，`localization_fusion.launch` 会 include：

```text
localization/localization_fusion/source_launch/start_mocap.launch
```

`start_mocap.launch` 会继续启动：

```text
vrpn_client_ros/launch/sample.launch
sunray_mocap/launch/mocap_odom.launch
```

默认 VRPN 服务器地址：

```text
192.168.20.15
```

如果现场动捕服务器地址不同，可以直接启动 source launch：

```bash
roslaunch localization_fusion start_mocap.launch vrpn_server:=192.168.20.15
```

注意：当前 `localization_fusion.launch` 没有直接声明并转发 `vrpn_server` 参数到 `start_mocap.launch`。如果希望一条命令同时修改 VRPN 地址并启动 fusion，建议后续给 `localization_fusion.launch` 补齐这个参数转发；目前更稳妥的方式是先单独启动 VRPN/动捕适配，再启动 `localization_fusion`。

### 和 localization_fusion 的配置关系

`localization_sources.yaml` 中的 MOCAP 配置：

```yaml
MOCAP:
  source_id: 1
  odometry_topic: "${agent_key}/sunray/odometry"
  relocalization_topic: ""
  timeout_s: 0.5
  source_frame_to_base:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
```

如果 `agent_name=uav`、`agent_id=1`，`${agent_key}/sunray/odometry` 会变成：

```text
/uav1/sunray/odometry
```

这正好对应 `sunray_mocap` 的输出。

### 常见检查命令

先检查 VRPN 是否有原始数据：

```bash
rostopic hz /vrpn_client_node/uav1/pose
rostopic hz /vrpn_client_node/uav1/twist
```

再检查适配后的 odom：

```bash
rostopic hz /uav1/sunray/odometry
rostopic echo /uav1/sunray/odometry
```

最后检查 fusion 输出：

```bash
rostopic echo /uav1/sunray/localization/odom_state
rostopic hz /uav1/sunray/localization/local_odom
```

### 二次开发注意事项

- 动捕刚体名称必须和 `/vrpn_client_node/uav1` 中的 `uav1` 对上。
- `mocap_odom_node` 当前用的是全局参数 `uav_id`，不是私有参数；如果你希望 launch 传参更清晰，可以后续改成私有参数并更新 launch。
- 如果动捕刚体原点不是机体中心，不建议在控制器里补偿，应该在 `localization_sources.yaml` 的 `source_frame_to_base` 中配置外参，或者在动捕适配节点里输出机体中心位姿。
- 动捕坐标系方向必须和 Sunray/ROS ENU 约定一致；如果飞行方向反了，优先检查动捕坐标轴定义和刚体姿态标定。

</section>
