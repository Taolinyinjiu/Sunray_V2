<section id="vins-fusion-gpu">

## vins-fusion-gpu

`vins-fusion-gpu` 是带 GPU 加速选项的 VINS-Fusion 版本。它和 `vins-fusion` 的整体接口类似，但配置中增加了 GPU 相关参数，并且输出中额外包含右相机位姿、校正位姿等话题。

### 目录结构

```text
localization/vins-fusion-gpu/
├── camera_models/
├── config/
│   ├── euroc/
│   ├── kitti_odom/
│   ├── kitti_raw/
│   ├── realsense_d435i/
│   ├── simulation/
│   └── sunray_150/
├── third_party/
│   └── vision_opencv/
└── vins_estimator/
    ├── launch/
    └── src/
```

重点路径：

| 路径 | 作用 |
| --- | --- |
| `config/sunray_150/sunray_nx_150.yaml` | Sunray 150 GPU VINS 配置。 |
| `vins_estimator/launch/vins_sunray_150.launch` | Sunray 150 启动入口。 |
| `vins_estimator/src/utility/visualization.cpp` | 输出话题定义。 |
| `third_party/vision_opencv/` | GPU 版本自带的 OpenCV/ROS vision 依赖，不属于 Sunray 业务接口。 |

### 启动方式

```bash
roslaunch vins vins_sunray_150.launch
```

打开 RViz：

```bash
roslaunch vins vins_sunray_150.launch rviz_enable:=true
```

如果工作空间里同时存在 CPU 和 GPU 两套 `vins` 包，要特别注意 ROS 包解析路径。两个目录的 package 名都可能是 `vins`，实际启动的是哪个版本取决于工作空间编译和 `ROS_PACKAGE_PATH`。

### GPU 配置重点

`config/sunray_150/sunray_nx_150.yaml` 中 GPU 相关参数：

```yaml
use_gpu: 1
use_gpu_acc_flow: 1
```

| 参数 | 作用 |
| --- | --- |
| `use_gpu` | 是否启用 GPU 路径。 |
| `use_gpu_acc_flow` | 是否使用 GPU 加速光流。 |

其他输入配置和 CPU 版本类似：

```yaml
imu_topic: "/uav1/mavros/imu/data_raw"
image0_topic: "/camera/infra1/image_rect_raw"
image1_topic: "/camera/infra2/image_rect_raw"
cam0_calib: "left.yaml"
cam1_calib: "right.yaml"
```

外参：

```yaml
estimate_extrinsic: 1
body_T_cam0: ...
body_T_cam1: ...
```

GPU 版本配置中 `estimate_td` 当前为 `0`，表示默认不在线估计相机和 IMU 时间偏移。硬件时间同步不稳定时需要谨慎评估这个设置。

### 输出话题

GPU 版本同样发布：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/vins_estimator/imu_propagate` | `nav_msgs/Odometry` | 高频 IMU 传播 odom，可接 `localization_fusion source_id=2`。 |
| `/vins_estimator/odometry` | `nav_msgs/Odometry` | VINS 优化后的 odom。 |
| `/vins_estimator/path` | `nav_msgs/Path` | 轨迹。 |
| `/vins_estimator/point_cloud` | `sensor_msgs/PointCloud` | 特征点云。 |
| `/vins_estimator/margin_cloud` | `sensor_msgs/PointCloud` | 边缘化点云。 |
| `/vins_estimator/camera_pose` | `nav_msgs/Odometry` | 左相机或主相机位姿。 |
| `/vins_estimator/camera_pose_visual` | `visualization_msgs/MarkerArray` | 相机位姿可视化。 |
| `/vins_estimator/keyframe_pose` | `nav_msgs/Odometry` | 关键帧位姿。 |
| `/vins_estimator/keyframe_point` | `sensor_msgs/PointCloud` | 关键帧点云。 |
| `/vins_estimator/extrinsic` | `nav_msgs/Odometry` | 外参估计。 |

GPU 版本额外可能发布：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/vins_estimator/camera_pose_right` | `nav_msgs/Odometry` | 右相机位姿。 |
| `/vins_estimator/rectify_pose_left` | `geometry_msgs/PoseStamped` | 左相机校正位姿。 |
| `/vins_estimator/rectify_pose_right` | `geometry_msgs/PoseStamped` | 右相机校正位姿。 |

### 接 localization_fusion

GPU VINS 和 CPU VINS 接入方式相同，因为 `localization_fusion` 只关心最终 odom 话题：

```bash
roslaunch vins vins_sunray_150.launch
roslaunch localization_fusion localization_fusion.launch source_id:=2 agent_name:=uav agent_id:=1
```

`source_id=2` 默认订阅：

```text
/vins_estimator/imu_propagate
```

如果 GPU 版本实际输出命名空间不同，需要同步修改：

```text
localization/localization_fusion/config/localization_sources.yaml
```

### 什么时候选 GPU 版本

适合使用 GPU 版本的情况：

- 图像分辨率或帧率较高，CPU 版本实时性不足。
- 平台有可用 CUDA/GPU 环境。
- 已经确认 GPU 版本依赖可以稳定编译和运行。

不建议一开始就使用 GPU 版本的情况：

- 还没有完成相机/IMU 标定。
- CPU 版本都无法稳定初始化。
- 平台 GPU 驱动、CUDA、OpenCV 版本不稳定。

### 二次开发注意事项

- GPU 版本和 CPU 版本接口相似，但依赖更复杂；定位异常时先排除输入数据和标定问题，再排查 GPU 加速路径。
- 不要同时启动 CPU 和 GPU 两个 VINS 节点并发布同名 `/vins_estimator/imu_propagate`，否则 `localization_fusion` 无法区分数据来源。
- `third_party/vision_opencv` 是依赖代码，不建议作为 Sunray 业务功能修改。
- 如果要把 GPU VINS 做成独立机型配置，建议在 `config/` 下新增独立文件夹，不要直接覆盖 `sunray_150` 的已验证配置。

</section>
