<section id="localization-extension-debug">

## 扩展与排错

这一节面向想接入新定位源、改配置或排查定位问题的开发者。核心原则是：新定位源先转换成 `nav_msgs/Odometry`，再交给 `localization_fusion`，不要让控制、规划、集群分别适配同一个私有定位话题。

### 接入新定位源

推荐步骤：

1. 写一个适配节点，把新算法或设备输出转换为 `nav_msgs/Odometry`。
2. 确认 `header.frame_id` 表示定位源所在的局部或全局参考系，`child_frame_id` 表示机体或传感器 frame。
3. 确认单位为米、弧度，四元数归一化，时间戳来自同一 ROS 时间体系。
4. 在 `localization_fusion/config/localization_sources.yaml` 中新增一项，并分配新的 `source_id`。
5. 填写 `odometry_topic`、可选 `relocalization_topic`、`timeout_s` 和 `source_frame_to_base`。
6. 如需一键启动，在 `localization_fusion/source_launch/` 下新增 source launch。
7. 在 `localization_fusion/launch/localization_fusion.launch` 中按 `source_id` include 新 launch。
8. 如需状态枚举同步，更新 `sunray_msgs/OdomState.msg`。

新增配置示例：

```yaml
My_New_Source:
  source_id: 7
  odometry_topic: "${agent_key}/my_localization/odom"
  relocalization_topic: ""
  timeout_s: 0.5
  source_frame_to_base:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
```

启动：

```bash
roslaunch localization_fusion localization_fusion.launch source_id:=7 agent_name:=uav agent_id:=1
```

### 适配节点最小模板

如果你的算法输出不是 `nav_msgs/Odometry`，建议单独写适配节点。适配节点只做三件事：

```text
订阅算法私有消息
  -> 坐标系/单位/外参整理
  -> 发布 nav_msgs/Odometry
```

输出 odom 最少要保证：

| 字段 | 要求 |
| --- | --- |
| `header.stamp` | 使用当前消息对应的真实时间，不要一直填 `ros::Time::now()` 掩盖延迟问题。 |
| `header.frame_id` | 表示 odom 所在参考系。 |
| `child_frame_id` | 表示机体或传感器 frame。 |
| `pose.pose.position` | 单位为米。 |
| `pose.pose.orientation` | 四元数归一化。 |
| `twist.twist.linear` | 如果算法能提供速度，建议填。控制器调试会更方便。 |

### 判断定位是否可用于控制

启动定位后，先看：

```bash
rostopic echo /uav1/sunray/localization/odom_state
```

重点字段：

| 字段 | 判断 |
| --- | --- |
| `odometry_valid` | 必须为 `true`，否则控制不应解锁或执行任务。 |
| `odometry_update_hz` | 应稳定高于控制所需最低频率。 |
| `source_id` | 确认当前使用的定位源和预期一致。 |
| `source_topic` | 确认 fusion 订阅的是你真正发布的 odom。 |
| `local_odom` | 看位置、姿态、速度是否合理。 |
| `global_odom` | 有重定位时看全局坐标是否合理。 |
| frame 名和 TF | 检查是否符合 `world -> sunray_global -> sunray_local -> base_link`。 |

### 常用调试命令

查看定位源是否有数据：

```bash
rostopic hz /uav1/sunray/localization/local_odom
rostopic echo /uav1/sunray/localization/odom_state
```

查看 TF：

```bash
rosrun tf view_frames
rosrun tf tf_echo uav1/sunray_local uav1/base_link
```

查看定位源输入：

```bash
rostopic hz /uav1/sunray/odometry
rostopic hz /vins_estimator/imu_propagate
rostopic hz /sunray/odometry
```

根据使用的 `source_id` 选择对应话题即可。

### odometry_valid=false 的排查顺序

如果 `odom_state.odometry_valid=false`，优先检查：

1. `source_id` 是否选对。
2. `localization_sources.yaml` 中的 `odometry_topic` 是否有数据。
3. `agent_name/agent_id` 是否与上游话题一致。
4. `timeout_s` 是否过小。
5. 输入 odom 的时间戳是否异常。
6. 是否有多个节点发布同一个输入话题，导致数据跳变。

### 常见问题表

| 问题 | 可能原因 | 检查方式 |
| --- | --- | --- |
| 控制节点一直等待定位 | `local_odom` 没数据或 `odometry_valid=false`。 | `rostopic echo /uav1/sunray/localization/odom_state`。 |
| 飞机方向反了 | ENU/NED 搞反、动捕坐标轴错、相机/雷达外参错。 | `rviz` 看 TF，`tf_echo` 看姿态。 |
| 起飞后位置突变 | 定位源初始化跳变、重定位输入更新了 `global_to_local_tf`。 | 同时 echo `local_odom` 和 `global_odom`。 |
| 速度很抖 | 输入 odom 频率低、时间戳不均匀、速度由差分得到且未滤波。 | `rostopic hz`、录 bag 回放分析。 |
| 多机话题串了 | 上游使用绝对话题，缺少 `/uavX` namespace。 | `rostopic list | grep odom`。 |
| RViz 看不到 Marker | 没启动 RViz 可视化节点，或 fixed frame 选错。 | 检查 `/uav1/sunray/localization/rviz_markers` 和 RViz Fixed Frame。 |

### 配置修改建议

- 只换定位源话题时，优先改 `localization_sources.yaml`，不要改 C++。
- 只换无人机编号时，优先改 launch 的 `agent_name` 和 `agent_id`。
- 只换传感器安装位置时，优先改 `source_frame_to_base` 或定位源自身外参。
- 需要多机时，避免使用全局绝对话题，例如 `/sunray/odometry`；更推荐带 `/uav1`、`/uav2` 前缀。
- 需要给 PX4 输入外部视觉时，先验证 `local_odom` 稳定，再打开 `enable_vision_pose`。

### 和控制模块的接口边界

定位模块对控制模块提供的是：

```text
/uav1/sunray/localization/local_odom
/uav1/sunray/localization/global_odom
/uav1/sunray/localization/odom_state
```

控制模块不应该关心当前定位来自动捕、VINS、FAST-LIO 还是仿真。这样控制逻辑可以稳定复用，定位源切换只发生在 launch 和 YAML 配置层。

</section>
