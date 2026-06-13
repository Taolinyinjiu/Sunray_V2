<!-- title: livox_ros_driver2 -->

<section id="driver-livox-ros-driver2">

## livox_ros_driver2

`drivers/livox_ros_driver2` 是 Livox 3D LiDAR 的 ROS 驱动，Sunray 中主要用于 MID360/HAP 等雷达点云输入。FAST-LIO、建图、避障和记录 rosbag 都可能依赖它。

### 常用启动

MID360 单雷达：

```bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

MID360 多雷达：

```bash
roslaunch livox_ros_driver2 msg_MID360s.launch
```

可视化版本：

```bash
roslaunch livox_ros_driver2 rviz_MID360.launch
```

### 关键配置

常见配置文件位于：

```text
drivers/livox_ros_driver2/config/
```

`msg_MID360s.launch` 中常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `xfer_format` | `1` | 数据传输格式。 |
| `multi_topic` | `0` | 多雷达时是否分话题输出。 |
| `data_src` | `0` | 数据来源，通常为实时设备。 |
| `publish_freq` | `10.0` | 点云发布频率。 |
| `msg_frame_id` | `livox_frame` | 点云 frame。 |
| `rosbag_enable` | `false` | 是否同时录包。 |

### 输出

驱动会发布 Livox 点云和 IMU 相关话题。具体话题名和消息类型受 launch/config 中 `xfer_format`、`multi_topic`、设备型号影响。

接 FAST-LIO 时重点确认：

- 点云话题是否和 FAST-LIO 配置一致。
- IMU 话题是否存在且频率正常。
- `frame_id` 是否和外参/TF 约定一致。

### 二次开发边界

- 换雷达型号或多雷达时，优先改 `config/*.json` 和 launch 参数。
- 不建议在业务节点里直接依赖 Livox 私有消息，除非算法确实需要原始字段。
- 要给 Sunray 上层提供定位结果，应通过 FAST-LIO/EKF/localization_fusion 输出 `Odometry`，不要让控制器直接订阅点云。

</section>
