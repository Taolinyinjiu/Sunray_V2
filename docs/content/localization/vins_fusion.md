<section id="vins-fusion">

## vins-fusion

`vins-fusion` 是原版 VINS-Fusion 视觉/视觉惯性里程计代码。它主要用相机和可选 IMU 估计运动，Sunray 当前把它作为一种可选定位源，通过 `localization_fusion` 的 `source_id=2` 接入。

这个包更接近第三方算法工程。小白开发者通常只需要理解启动文件、配置文件、输入输出话题，以及如何把 VINS 输出接到 `localization_fusion`。

### 目录结构

```text
localization/vins-fusion/
├── camera_models/
├── config/
│   ├── realsense_d435i/
│   └── sunray_150/
├── global_fusion/
├── loop_fusion/
└── vins_estimator/
    ├── launch/
    └── src/
```

重点路径：

| 路径 | 作用 |
| --- | --- |
| `config/sunray_150/sunray_nx_150.yaml` | Sunray 150 双目/IMU 配置。 |
| `config/sunray_150/left.yaml`、`right.yaml` | 左右相机内参。 |
| `vins_estimator/launch/vins_sunray_150.launch` | Sunray 150 启动入口。 |
| `vins_estimator/src/rosNodeTest.cpp` | VINS ROS 节点入口。 |
| `vins_estimator/src/utility/visualization.cpp` | VINS 输出话题定义。 |
| `vins_estimator/src/estimator/` | VINS 估计器核心逻辑。 |

### 启动方式

Sunray 150 配置：

```bash
roslaunch vins vins_sunray_150.launch
```

打开 VINS 自带 RViz：

```bash
roslaunch vins vins_sunray_150.launch rviz_enable:=true
```

指定自定义配置文件：

```bash
roslaunch vins vins_sunray_150.launch config_path:=/path/to/your_vins_config.yaml
```

通用 Realsense D435i 配置入口：

```bash
roslaunch vins vins_start.launch
```

### Sunray 150 配置重点

`config/sunray_150/sunray_nx_150.yaml` 中常见关键项：

```yaml
imu: 1
num_of_cam: 2
imu_topic: "/uav1/mavros/imu/data_raw"
image0_topic: "/camera/infra1/image_rect_raw"
image1_topic: "/camera/infra2/image_rect_raw"
cam0_calib: "left.yaml"
cam1_calib: "right.yaml"
image_width: 640
image_height: 480
```

| 参数 | 作用 |
| --- | --- |
| `imu` | 是否使用 IMU。`1` 表示视觉惯性，`0` 表示纯视觉。 |
| `num_of_cam` | 相机数量。Sunray 150 配置为双目。 |
| `imu_topic` | IMU 输入话题。当前配置使用 `/uav1/mavros/imu/data_raw`。 |
| `image0_topic`、`image1_topic` | 左右相机图像话题。 |
| `cam0_calib`、`cam1_calib` | 相机内参文件。 |

外参和标定相关：

```yaml
estimate_extrinsic: 1
body_T_cam0: ...
body_T_cam1: ...
```

| 参数 | 作用 |
| --- | --- |
| `estimate_extrinsic` | `0` 表示信任配置中的相机-IMU 外参；`1` 表示以配置为初值继续优化。 |
| `body_T_cam0`、`body_T_cam1` | body/IMU 到相机的外参。相机安装变化时必须重新标定。 |

特征和优化相关：

| 参数 | 作用 |
| --- | --- |
| `max_cnt` | 最大跟踪特征数量。 |
| `min_dist` | 特征之间的最小像素距离。 |
| `freq` | 特征跟踪结果发布频率。 |
| `F_threshold` | RANSAC 阈值。 |
| `show_track` | 是否发布跟踪图像。 |
| `flow_back` | 是否使用前后向光流一致性检查。 |
| `max_solver_time` | 优化器单次求解时间上限。 |
| `max_num_iterations` | 优化迭代次数上限。 |
| `keyframe_parallax` | 关键帧选择视差阈值。 |

IMU 噪声相关：

| 参数 | 作用 |
| --- | --- |
| `acc_n`、`gyr_n` | 加速度计和陀螺仪测量噪声。 |
| `acc_w`、`gyr_w` | 加速度计和陀螺仪 bias 随机游走噪声。 |
| `g_norm` | 重力大小。 |
| `estimate_td` | 是否在线估计相机与 IMU 时间偏移。 |
| `td` | 初始时间偏移。 |

### VINS 输出话题

`vins_estimator/src/utility/visualization.cpp` 中定义了主要输出。节点名通常是 `vins_estimator`，因此相对话题会变成 `/vins_estimator/...`。

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/vins_estimator/imu_propagate` | `nav_msgs/Odometry` | 高频 IMU 传播 odom，Sunray `source_id=2` 默认订阅这个话题。 |
| `/vins_estimator/odometry` | `nav_msgs/Odometry` | VINS 优化后的 odom。 |
| `/vins_estimator/path` | `nav_msgs/Path` | VINS 轨迹。 |
| `/vins_estimator/point_cloud` | `sensor_msgs/PointCloud` | 当前特征点云。 |
| `/vins_estimator/margin_cloud` | `sensor_msgs/PointCloud` | 边缘化点云。 |
| `/vins_estimator/key_poses` | `visualization_msgs/Marker` | 关键帧位置。 |
| `/vins_estimator/camera_pose` | `nav_msgs/Odometry` | 相机位姿。 |
| `/vins_estimator/camera_pose_visual` | `visualization_msgs/MarkerArray` | 相机位姿可视化。 |
| `/vins_estimator/keyframe_pose` | `nav_msgs/Odometry` | 关键帧位姿。 |
| `/vins_estimator/keyframe_point` | `sensor_msgs/PointCloud` | 关键帧点云。 |
| `/vins_estimator/extrinsic` | `nav_msgs/Odometry` | 估计出的外参。 |
| `/vins_estimator/image_track` | `sensor_msgs/Image` | 特征跟踪图像。 |

### 接 localization_fusion

`localization_sources.yaml` 当前配置：

```yaml
VINS:
  source_id: 2
  odometry_topic: "/vins_estimator/imu_propagate"
  relocalization_topic: ""
  timeout_s: 0.5
```

启动顺序建议：

```bash
roslaunch vins vins_sunray_150.launch
roslaunch localization_fusion localization_fusion.launch source_id:=2 agent_name:=uav agent_id:=1
```

然后检查：

```bash
rostopic hz /vins_estimator/imu_propagate
rostopic echo /uav1/sunray/localization/odom_state
```

### 常见问题

| 现象 | 优先检查 |
| --- | --- |
| `/vins_estimator/imu_propagate` 没数据 | 图像话题、IMU 话题、时间戳是否正常。 |
| VINS 初始化失败 | 相机是否运动充分、纹理是否足够、IMU 方向和噪声参数是否合理。 |
| 姿态方向不对 | 相机-IMU 外参、ENU/NED 坐标约定、`body_T_cam*`。 |
| fusion 显示 invalid | `source_id=2` 是否启动、`/vins_estimator/imu_propagate` 频率是否超过 `timeout_s`。 |

### 二次开发注意事项

- VINS 对相机内参、双目外参、相机-IMU 外参和时间同步非常敏感，改硬件后必须重新标定。
- VINS 输出 frame 默认为 `world`，接入 Sunray 后会由 `localization_fusion` 重新组织成 `uav1/sunray_local -> uav1/base_link`。
- 如果你更想使用 `/vins_estimator/odometry` 而不是 `/vins_estimator/imu_propagate`，可以修改 `localization_sources.yaml` 的 `VINS.odometry_topic`。
- 小白用户优先改配置文件和 launch，不建议直接改 `estimator/`、`factor/` 下的算法代码。

</section>
