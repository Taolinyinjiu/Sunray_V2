<section id="ekf-odometry">

## ekf_odometry

`ekf_odometry` 用 IMU 对外部 odom 做预测，并在 odom 到来时更新，用来得到更高频、更平滑的里程计估计。它主要服务于 FAST-LIO 这类“odom 频率低于 IMU”的链路。

### 当前仓库中的实际链路

`ekf_odometry.launch` 当前启动：

```text
ekf_odometry_node
mocap_odom_node
```

其中 `transform_odom_pointCloud` 节点在 launch 中是注释状态。也就是说，默认启动后：

```text
/livox/imu + /Odometry
  -> ekf_odometry_node
  -> /ekf_odometry
```

而 `localization_sources.yaml` 的 `Fast_LIO_EKF` 当前订阅的是：

```text
/sunray/odometry
```

这个 `/sunray/odometry` 默认来自 `fast_lio` 包里的 `transform_odom_pointCloud`，不是来自 `/ekf_odometry`。如果你希望 `localization_fusion` 使用 EKF 输出，需要明确改配置或启用相应转换节点。

### 代码结构

```text
localization/ekf_odometry/
├── include/
│   ├── ekf_filter.h
│   └── imu_process.h
├── launch/
│   └── ekf_odometry.launch
└── src/
    ├── ekf_filter.cpp
    ├── ekf_odometry_node.cpp
    ├── imu_process.cpp
    ├── mocap_odom_node.cpp
    └── transform_odom_pointCloud.cpp
```

重点文件：

| 文件 | 作用 |
| --- | --- |
| `src/ekf_odometry_node.cpp` | EKF 主节点，订阅 IMU 和 odom，发布 `/ekf_odometry`。 |
| `include/ekf_filter.h`、`src/ekf_filter.cpp` | EKF 状态预测和更新实现。 |
| `include/imu_process.h`、`src/imu_process.cpp` | IMU 初始化、重力、bias 等处理。 |
| `src/transform_odom_pointCloud.cpp` | 可选转换节点，订阅 `/ekf_odometry` 和 `/PointCloud`，发布 `/sunray/odometry` 和 `/sunray/pointCloud`。当前 launch 未启用。 |
| `src/mocap_odom_node.cpp` | 旧的动捕辅助节点，输出 `/uavX/sunray/mocap_odometry` 等话题。 |

### 输入输出

`ekf_odometry_node` 默认参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `imu_topic` | `/livox/imu` | IMU 输入。 |
| `odom_topic` | `/Odometry` | 外部 odom 输入，通常来自 FAST-LIO。 |

`ekf_odometry_node` 话题：

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/livox/imu` | `sensor_msgs/Imu` | 高频预测输入。 |
| 订阅 | `/Odometry` | `nav_msgs/Odometry` | 低频或中频里程计更新。 |
| 发布 | `/ekf_odometry` | `nav_msgs/Odometry` | EKF 输出。 |

节点内部循环频率为 `300 Hz`。代码中会丢弃延迟过大的 odom，当前判断阈值约为 `0.15 s`。

### 启动方式

单独启动：

```bash
roslaunch ekf_odometry ekf_odometry.launch
```

通过 FAST-LIO 链路启动：

```bash
roslaunch localization_fusion localization_fusion.launch source_id:=6 agent_name:=uav agent_id:=1
```

### 如何让 fusion 使用 EKF 输出

如果你确认 `/ekf_odometry` 比 `/sunray/odometry` 更适合作为控制输入，可以在 `localization_sources.yaml` 中把 `Fast_LIO_EKF` 改成：

```yaml
Fast_LIO_EKF:
  source_id: 6
  odometry_topic: "/ekf_odometry"
```

如果你还需要点云也跟随 EKF odom 做转换，可以考虑启用 `ekf_odometry/launch/ekf_odometry.launch` 中注释掉的：

```xml
<node pkg="ekf_odometry" type="transform_odom_pointCloud" name="transform_odom" output="$(arg log_type)" />
```

启用前需要确认不会和 `fast_lio` 包里的 `transform_odom_pointCloud` 同时发布 `/sunray/odometry`，否则两个节点抢同一个话题会让下游很难排错。

### 常见检查命令

```bash
rostopic hz /livox/imu
rostopic hz /Odometry
rostopic hz /ekf_odometry
rostopic echo /ekf_odometry
```

如果要检查 fusion 当前到底用了哪个输入：

```bash
rostopic echo /uav1/sunray/localization/odom_state
```

重点看 `odom_state` 里的外部定位源订阅话题和 `odometry_update_hz`。

### 二次开发注意事项

- 这个包适合处理“IMU 高频、odom 较低频”的场景，不应该用来掩盖明显错误的外参或时间戳。
- 如果 EKF 输出跳变，先检查 `/livox/imu` 和 `/Odometry` 时间戳是否同一时钟体系。
- 当前 launch 里启动了一个 `mocap_odom_node`，这和 FAST-LIO EKF 主链路关系不强，后续可以整理为更清晰的 launch。
- 对控制来说，最终使用哪个 odom 不是由 `ekf_odometry` 决定，而是由 `localization_fusion/config/localization_sources.yaml` 决定。

</section>
