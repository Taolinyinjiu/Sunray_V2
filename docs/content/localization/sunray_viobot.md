<section id="sunray-viobot">

## sunray_viobot

`sunray_viobot` 是 VIOBOT/Baton mini 适配包。它订阅设备输出的里程计，对姿态做固定旋转修正，然后发布 Sunray 约定的 `nav_msgs/Odometry`。这个包的定位是“设备适配层”，不是 VIO 算法本体。

### 数据流

默认单机数据流：

```text
/baton/stereo3/odometry
  -> sunray_viobot/viobot_odom_node
  -> /uav1/sunray/odometry
  -> localization_fusion
  -> /uav1/sunray/localization/local_odom
```

### 代码结构

```text
localization/sunray_viobot/
├── launch/
│   └── viobot_odom.launch
└── src/
    └── viobot_odom_node.cpp
```

`viobot_odom_node.cpp` 的主要逻辑：

1. 读取私有参数 `odom_sub_topic`，默认 `/baton/stereo3/odometry`。
2. 读取私有参数 `odom_pub_topic`，默认 `sunray/odometry`。
3. 订阅 VIOBOT 原始 odom。
4. 对姿态做固定旋转修正：绕 Z 轴 `+90 deg`，绕 Y 轴 `-90 deg`。
5. 发布修正后的 odom。

### 输入输出

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/baton/stereo3/odometry` | `nav_msgs/Odometry` | VIOBOT/Baton mini 输出的原始 odom。 |
| 发布 | `sunray/odometry` | `nav_msgs/Odometry` | 修正后的 odom；在 launch namespace 下会变成 `/uav1/sunray/odometry`。 |

### 启动方式

```bash
roslaunch sunray_viobot viobot_odom.launch agent_name:=uav agent_id:=1
```

`viobot_odom.launch` 会把节点放在：

```text
/uav1
```

命名空间下，因此默认输出实际是：

```text
/uav1/sunray/odometry
```

如果设备原始话题不同：

```bash
roslaunch sunray_viobot viobot_odom.launch \
  agent_name:=uav \
  agent_id:=1 \
  odom_sub_topic:=/your_viobot/odometry \
  odom_pub_topic:=sunray/odometry
```

### 和 localization_fusion 的配置关系

当前 `localization_sources.yaml` 中 `VIOBOT` 的 `odometry_topic` 为空，表示这个源位目前是预留状态：

```yaml
VIOBOT:
  source_id: 0
  odometry_topic: ""
```

如果要直接通过 `source_id=0` 使用 VIOBOT，需要把它改成：

```yaml
odometry_topic: "${agent_key}/sunray/odometry"
```

然后启动：

```bash
roslaunch sunray_viobot viobot_odom.launch agent_name:=uav agent_id:=1
roslaunch localization_fusion localization_fusion.launch source_id:=0 agent_name:=uav agent_id:=1
```

也可以暂时复用其他 `source_id` 的配置，但不建议长期这样做，因为监控界面显示的定位源名称会和真实来源不一致。

### 常见检查命令

检查设备输出：

```bash
rostopic hz /baton/stereo3/odometry
rostopic echo /baton/stereo3/odometry
```

检查适配输出：

```bash
rostopic hz /uav1/sunray/odometry
rostopic echo /uav1/sunray/odometry
```

检查 fusion 输出：

```bash
rostopic echo /uav1/sunray/localization/odom_state
```

### 二次开发注意事项

- 当前姿态修正是代码中的固定旋转，适合已有设备安装方式。如果设备安装方向变化，需要重新确认这两个旋转是否仍然正确。
- `odom_pub_topic` 默认是相对话题，配合 launch namespace 使用。改成绝对话题时要确认不会破坏多机隔离。
- 如果 VIOBOT 输出的是相机坐标系而不是机体中心，建议通过 `source_frame_to_base` 或适配节点补齐外参。
- VIO 对光照、纹理、快速转动比较敏感，控制前必须先看 `odom_state.odometry_valid` 和 `odometry_update_hz`。

</section>
