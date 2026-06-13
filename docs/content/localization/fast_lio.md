<section id="fast-lio">

## fast_lio

`fast_lio` 是 FAST-LIO/FAST-LIO2 激光惯性里程计包，用 LiDAR 点云和 IMU 估计机体运动。Sunray 在保留原算法结构的基础上，增加了 `transform_odom_pointCloud` 节点，把 FAST-LIO 的输出转换成 Sunray 使用的话题。

### 典型用途

`fast_lio` 适合：

- 使用 Livox MID360、Livox Avia、Horizon、Velodyne、Ouster 等 LiDAR 的定位建图。
- 需要在无 GPS 或弱 GPS 环境中获取局部 odom。
- 给 `localization_fusion` 的 `source_id=6` 提供定位源。
- 给 `open3d_loc` 提供当前点云和局部 odom，用于全局重定位。

### 数据流

以 MID360 为例：

```text
/livox/lidar
/livox/imu
  -> fastlio_mapping
  -> /Odometry
  -> /PointCloud
  -> transform_odom_pointCloud
  -> /sunray/odometry
  -> /sunray/pointCloud
  -> localization_fusion source_id=6
```

注意：当前 `localization_sources.yaml` 中 `Fast_LIO_EKF` 的输入是 `/sunray/odometry`。这和旧文档里可能看到的 `/sunray/ekf_odometry` 不一致，应以当前配置文件为准。

### 代码结构

```text
localization/fast_lio/
├── config/
│   ├── mid360.yaml
│   ├── avia.yaml
│   ├── horizon.yaml
│   ├── marsim.yaml
│   ├── ouster64.yaml
│   └── velodyne.yaml
├── launch/
│   ├── mapping_mid360.launch
│   ├── mapping_avia.launch
│   ├── mapping_horizon.launch
│   ├── mapping_marsim.launch
│   ├── mapping_ouster64.launch
│   └── mapping_velodyne.launch
└── src/
    ├── laserMapping.cpp
    └── transform_odom_pointCloud.cpp
```

重点文件：

| 文件 | 作用 | 新手是否建议修改 |
| --- | --- | --- |
| `launch/mapping_mid360.launch` | MID360 常用启动入口。 | 可以改启动参数。 |
| `config/mid360.yaml` | MID360 的话题、LiDAR 类型、外参、滤波和发布配置。 | 可以按硬件标定改。 |
| `src/laserMapping.cpp` | FAST-LIO 主算法。 | 不建议小白直接改。 |
| `src/transform_odom_pointCloud.cpp` | Sunray 输出转换节点。 | 接 Sunray 时需要理解。 |

### 输入输出

`fastlio_mapping` 主要输入：

| 方向 | 话题 | 类型 | 来源 |
| --- | --- | --- | --- |
| 订阅 | `/livox/lidar` | 点云消息 | `mid360.yaml` 的 `common/lid_topic`。 |
| 订阅 | `/livox/imu` | `sensor_msgs/Imu` | `mid360.yaml` 的 `common/imu_topic`。 |

`fastlio_mapping` 主要输出：

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 发布 | `/Odometry` | `nav_msgs/Odometry` | FAST-LIO 原始里程计。 |
| 发布 | `/PointCloud` | `sensor_msgs/PointCloud2` | Sunray 转换节点使用的点云。 |
| 发布 | `/cloud_registered` | `sensor_msgs/PointCloud2` | 注册后的点云。 |
| 发布 | `/cloud_registered_body` | `sensor_msgs/PointCloud2` | body frame 点云。 |
| 发布 | `/Laser_map` | `sensor_msgs/PointCloud2` | 局部/全局地图点云。 |
| 发布 | `/path` | `nav_msgs/Path` | 轨迹。 |

`transform_odom_pointCloud` 主要输入输出：

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/livox/imu` | `sensor_msgs/Imu` | 用前 200 帧 IMU 估计初始倾斜修正。 |
| 订阅 | `/Odometry` | `nav_msgs/Odometry` | FAST-LIO 原始 odom。 |
| 订阅 | `/PointCloud` | `sensor_msgs/PointCloud2` | FAST-LIO 原始点云输出。 |
| 发布 | `/sunray/odometry` | `nav_msgs/Odometry` | 修正后的 Sunray odom。 |
| 发布 | `/sunray/pointCloud` | `sensor_msgs/PointCloud2` | 修正后的 Sunray 点云。 |

### MID360 启动方式

单独启动 FAST-LIO：

```bash
roslaunch fast_lio mapping_mid360.launch
```

启动 MID360 驱动 + FAST-LIO + EKF + localization_fusion：

```bash
roslaunch localization_fusion localization_fusion.launch source_id:=6 agent_name:=uav agent_id:=1
```

`source_id=6` 时会 include：

```text
localization/localization_fusion/source_launch/start_fastlio_ekf.launch
```

这个 launch 当前启动：

```text
livox_ros_driver2/launch_ROS1/msg_MID360.launch
fast_lio/launch/mapping_mid360.launch
ekf_odometry/launch/ekf_odometry.launch
localization_fusion_node
```

### mid360.yaml 关键参数

```yaml
common:
  lid_topic: "/livox/lidar"
  imu_topic: "/livox/imu"
  time_sync_en: false
  time_offset_lidar_to_imu: 0.0
```

| 参数 | 作用 |
| --- | --- |
| `lid_topic` | LiDAR 点云输入话题。驱动话题不同就改这里。 |
| `imu_topic` | IMU 输入话题。 |
| `time_sync_en` | 原算法的时间同步选项；只有外部时间同步不可用时才考虑打开。 |
| `time_offset_lidar_to_imu` | LiDAR 和 IMU 的时间偏移；不知道准确值时保持 `0.0`。 |

```yaml
preprocess:
  lidar_type: 1
  scan_line: 4
  blind: 0.5
```

| 参数 | 作用 |
| --- | --- |
| `lidar_type` | LiDAR 类型。`1` 表示 Livox，`2` 表示 Velodyne，`3` 表示 Ouster。 |
| `scan_line` | 扫描线数或 Livox 配置相关参数。 |
| `blind` | 近距离盲区过滤，单位通常为米。太小可能引入近场噪声，太大可能丢失近处结构。 |

```yaml
mapping:
  acc_cov: 0.1
  gyr_cov: 0.1
  b_acc_cov: 0.0001
  b_gyr_cov: 0.0001
  fov_degree: 360
  det_range: 100.0
  extrinsic_est_en: false
  extrinsic_T: [-0.011, -0.02329, 0.04412]
  extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]
```

| 参数 | 作用 |
| --- | --- |
| `acc_cov`、`gyr_cov` | IMU 加速度/角速度噪声协方差。 |
| `b_acc_cov`、`b_gyr_cov` | IMU bias 随机游走参数。 |
| `fov_degree` | LiDAR 视场角。 |
| `det_range` | 有效检测距离。 |
| `extrinsic_est_en` | 是否在线估计 LiDAR-IMU 外参。实机已标定时一般关闭。 |
| `extrinsic_T`、`extrinsic_R` | LiDAR 到 IMU 的外参。硬件安装变化时必须重新确认。 |

### 接 localization_fusion

`localization_sources.yaml` 中 `source_id=6`：

```yaml
Fast_LIO_EKF:
  source_id: 6
  odometry_topic: "/sunray/odometry"
  relocalization_topic: ""
  timeout_s: 0.5
```

因此只要 `/sunray/odometry` 正常发布，`localization_fusion` 就会输出：

```text
/uav1/sunray/localization/local_odom
/uav1/sunray/localization/global_odom
/uav1/sunray/localization/odom_state
```

### 常见检查命令

检查传感器：

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
```

检查 FAST-LIO：

```bash
rostopic hz /Odometry
rostopic hz /PointCloud
```

检查 Sunray 转换输出：

```bash
rostopic hz /sunray/odometry
rostopic hz /sunray/pointCloud
```

检查 fusion：

```bash
rostopic echo /uav1/sunray/localization/odom_state
```

### 二次开发注意事项

- FAST-LIO 对时间同步、外参、IMU 方向非常敏感。定位漂移或飞行方向异常时，优先检查 `mid360.yaml` 中的话题、时间偏移和外参。
- `/sunray/odometry` 当前是全局绝对话题，不带 `/uav1` 前缀。多机实机同时使用多套 LiDAR 时需要重新设计命名空间。
- `transform_odom_pointCloud` 会用一段 IMU 数据估计初始倾斜，启动后前几秒不要立刻判断输出异常。
- 小白用户通常只需要改 `config/*.yaml` 和 launch，不建议直接修改 `laserMapping.cpp` 的算法细节。

</section>
