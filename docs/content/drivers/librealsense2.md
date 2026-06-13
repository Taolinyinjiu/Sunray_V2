<!-- title: librealsense2 -->

<section id="driver-librealsense2">

## librealsense2

`drivers/librealsense` 在 ROS 中的包名是 `librealsense2`，它是 Intel RealSense 的底层 SDK。Sunray 中通常不直接调用它，而是由 `realsense2_camera` 作为 ROS 驱动使用。

### 作用

```text
RealSense USB 设备
  -> librealsense2
  -> realsense2_camera
  -> ROS 图像/深度/点云话题
```

### 什么时候需要关注

- RealSense 设备无法识别。
- 固件版本、SDK 版本和 ROS 驱动版本不匹配。
- 深度流、彩色流或 IMU 流无法打开。
- Jetson/Ubuntu 内核补丁、udev 权限或 USB 带宽问题。

### 二次开发边界

- 普通视觉任务不要修改 `librealsense` 源码。
- 优先通过 `realsense2_camera` 的 launch 参数控制分辨率、帧率、流开关和点云。
- 只有在底层设备兼容性、固件支持或 SDK bug 层面，才进入这里排查。

</section>
