<!-- title: realsense2_camera -->

<section id="driver-realsense2-camera">

## realsense2_camera

`drivers/realsense2_camera` 是 RealSense ROS1 相机驱动，依赖 `drivers/librealsense`。它用于输出深度图、彩色图、红外图、点云和可选 IMU 数据。

### 启动方式

```bash
roslaunch realsense2_camera rs_camera.launch
```

启用彩色图和点云示例：

```bash
roslaunch realsense2_camera rs_camera.launch enable_color:=true enable_pointcloud:=true
```

### 常用参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `camera` | `camera` | 话题命名空间。 |
| `serial_no` | 空 | 指定相机序列号，多相机时建议填写。 |
| `enable_depth` | `true` | 是否发布深度图。 |
| `enable_color` | `false` | 是否发布彩色图。 |
| `enable_infra1/2` | `true` | 是否发布红外图。 |
| `enable_pointcloud` | `false` | 是否发布点云。 |
| `enable_gyro/accel` | `false` | 是否发布 IMU。 |
| `align_depth` | `true` | 是否对齐深度。 |
| `publish_tf` | `false` | 是否由驱动发布 TF。 |

### 输出

典型话题在 `/camera/...` 命名空间下，例如：

```text
/camera/depth/image_rect_raw
/camera/color/image_raw
/camera/infra1/image_rect_raw
/camera/depth/color/points
```

具体话题取决于启用的流。

### 二次开发边界

- VINS、感知、避障需要图像或深度时，优先通过 launch 参数启用对应流。
- 多相机一定要配置 `serial_no` 和不同 `camera` 命名空间。
- 上层算法要订阅标准 `sensor_msgs/Image`、`sensor_msgs/PointCloud2`，不要依赖驱动内部类。

</section>
