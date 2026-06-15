<section id="open3d-loc">

## open3d_loc

`open3d_loc` 是基于 Open3D 的点云地图全局定位/重定位包。它使用当前点云和已有 PLY 地图做配准，估计 `base_link`、`odom` 与 `map` 之间的关系，适合需要全局地图对齐、定位漂移修正或重定位的场景。

它不是基础飞行必需包。小白用户如果只是想让飞机在动捕、Gazebo 或 FAST-LIO 下飞起来，可以先跳过它；当你需要“局部里程计对齐到已有地图”时再看。

### 数据流

典型链路：

```text
FAST-LIO odom + 当前点云 + PLY 地图 + 初始位姿
  -> open3d_loc/global_localization_node
  -> /localization_3d
  -> /odom2map
  -> /baselink2map
```

如果要接入 `localization_fusion` 的重定位机制，可以写一个适配节点，把 `open3d_loc` 的全局定位结果整理成 `nav_msgs/Odometry`，然后填入 `localization_sources.yaml` 的 `relocalization_topic`。

### 代码结构

```text
localization/third_party_localization/open3d_loc/
├── config/
│   └── loc_param_g1.yaml
├── launch/
│   ├── localization_3d_g1.launch
│   └── open3d_loc_g1.launch
├── rviz_cfg/
└── src/
    ├── global_localization.cpp
    ├── open3d_conversions/
    └── open3d_registration/
```

重点文件：

| 文件 | 作用 |
| --- | --- |
| `src/global_localization.cpp` | 主节点，负责订阅 odom/点云/初始位姿，执行全局点云配准并发布定位结果。 |
| `src/open3d_registration/open3d_registration.cpp` | Open3D 配准相关封装。 |
| `config/loc_param_g1.yaml` | 初始位姿、Kalman 参数等。 |
| `launch/open3d_loc_g1.launch` | 单独启动全局定位节点。 |
| `launch/localization_3d_g1.launch` | 组合 FAST-LIO、Open3D 定位和 RViz 的启动入口。 |

### 输入输出

`global_localization_node` 主要输入：

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/Odometry_loc` | `nav_msgs/Odometry` | 当前局部里程计。代码启动时会等待该话题。 |
| 订阅 | `/cloud_registered_1` | `sensor_msgs/PointCloud2` | 当前点云。代码启动时会等待该话题。 |
| 订阅 | `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz 或外部工具给出的初始位姿。 |

主要输出：

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 发布 | `/baselink2map` | `nav_msgs/Odometry` | `base_link` 在 map 下的估计。 |
| 发布 | `/baselink2map_kalman` | `nav_msgs/Odometry` | Kalman 滤波后的 `base_link -> map`。 |
| 发布 | `/motionlink2map` | `nav_msgs/Odometry` | motion_link 在 map 下的估计。 |
| 发布 | `/odom2map` | `nav_msgs/Odometry` | odom 到 map 的关系。 |
| 发布 | `/odom2map_kalman` | `nav_msgs/Odometry` | Kalman 滤波后的 odom 到 map 关系。 |
| 发布 | `/localization_3d` | `geometry_msgs/PoseStamped` | 3D 定位结果。 |
| 发布 | `/localization_3d_confidence` | `std_msgs/Float32` | 定位置信度。 |
| 发布 | `/localization_3d_delay_ms` | `std_msgs/Float32` | 定位耗时，单位毫秒。 |
| 发布 | `/map` | `sensor_msgs/PointCloud2` | 地图点云。 |
| 发布 | `/submap` | `sensor_msgs/PointCloud2` | 当前子地图。 |
| 发布 | `/scan` | `sensor_msgs/PointCloud2` | 当前扫描。 |
| 发布 | `/scan2map` | `sensor_msgs/PointCloud2` | 配准到 map 后的扫描。 |

### 启动方式

单独启动 Open3D 全局定位：

```bash
roslaunch open3d_loc open3d_loc_g1.launch
```

组合启动定位链路：

```bash
roslaunch open3d_loc localization_3d_g1.launch
```

启动前需要确认地图路径存在。`open3d_loc_g1.launch` 当前默认：

```text
$(find open3d_loc)/../../data/map.ply
```

也就是仓库中的 `localization/data/map.ply`。

### open3d_loc_g1.launch 关键参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `path_map` | `$(find open3d_loc)/../../data/map.ply` | PLY 地图路径。 |
| `pcd_queue_maxsize` | `10` | 点云队列长度。 |
| `voxelsize_coarse` | `0.15` | 粗配准体素大小。 |
| `voxelsize_fine` | `0.1` | 精配准体素大小。 |
| `threshold_fitness` | `0.5` | 定位配准阈值，超过该值才更新 `odom2map`。 |
| `threshold_fitness_init` | `0.5` | 初始化配准阈值。 |
| `loc_frequence` | `2.5` | 全局定位频率。 |
| `save_scan` | `0` | 是否保存当前扫描。 |
| `hidden_removal` | `0` | 是否启用隐藏点移除。 |
| `maxpoints_source` | `80000` | 源点云最大点数。 |
| `maxpoints_target` | `400000` | 目标地图最大点数。 |
| `filter_odom2map` | `0` | 是否对 `odom2map` 做滤波。 |
| `kalman_processVar2` | `0.001` | `odom2map` 滤波过程噪声。 |
| `kalman_estimatedMeasVar2` | `0.02` | `odom2map` 滤波测量噪声。 |
| `confidence_loc_th` | `0.7` | 定位置信度阈值。 |
| `dis_updatemap` | `3.5` | 更新当前子地图的距离阈值。 |

### TF 与 frame

`open3d_loc_g1.launch` 会发布几个静态 TF：

```text
odom -> camera_init
base_link -> imu_link
motion_link -> base_link
```

这些 TF 是当前 G1 链路的假设。如果你的机器人 frame 名不同，或者 FAST-LIO 输出 frame 不同，需要同步修改 launch、输入话题和下游使用方式。

### 常见检查命令

检查输入：

```bash
rostopic hz /Odometry_loc
rostopic hz /cloud_registered_1
```

检查输出：

```bash
rostopic echo /localization_3d
rostopic echo /localization_3d_confidence
rostopic echo /odom2map
rostopic echo /baselink2map
```

在 RViz 中可以查看：

```text
/map
/submap
/scan
/scan2map
```

### 二次开发注意事项

- `open3d_loc` 的输出目前不是直接接入 `localization_fusion` 的标准重定位输入。要进入统一链路，建议写一个小适配节点，把 `/baselink2map` 或 `/localization_3d` 转成 `nav_msgs/Odometry`，再配置为 `relocalization_topic`。
- 点云配准质量强依赖地图质量、初始位姿、体素大小和阈值。不要只看是否有输出，还要看 `/localization_3d_confidence`。
- `path_map` 指向的 PLY 地图文件必须和当前点云坐标系一致，否则会出现配准成功但全局位置错误的情况。
- 如果定位结果周期性跳变，优先检查 `threshold_fitness`、`confidence_loc_th`、初始位姿和输入 odom 是否稳定。

</section>
