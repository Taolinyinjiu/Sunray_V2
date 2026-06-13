<!-- title: web_cam -->

<section id="driver-web-cam">

## web_cam

`drivers/web_cam` 是 USB 摄像头 ROS 驱动，用于把 V4L 摄像头发布成 `sensor_msgs/Image`。它常作为 ArUco 检测、NPU 检测或简单视觉任务的输入。

### 启动方式

```bash
roslaunch web_cam web_cam.launch
```

打开图像预览：

```bash
roslaunch web_cam web_cam.launch image_view:=true
```

### 配置文件

```text
drivers/web_cam/config/web_cam.yml
drivers/web_cam/config/head_camera.yaml
```

常见配置项包括设备号、分辨率、帧率、像素格式、camera info 等。实际字段以配置文件为准。

### 输出

默认图像话题：

```text
/web_cam/image_raw
```

这个话题也是 `sunray_perception` 中 ArUco/NPU 检测 launch 的默认输入。

### 二次开发边界

- 换摄像头优先改配置文件中的设备路径和分辨率。
- 如果相机被系统识别成不同 `/dev/videoX`，建议用 udev 规则固定设备名。
- 感知算法应订阅图像话题，不要直接访问摄像头设备文件。

</section>
