<!-- title: sunray_perception -->

<section id="perception-sunray-perception">

## sunray_perception

`perception/sunray_perception` 是 Sunray 的视觉感知包，当前包含 ArUco 检测、NPU 目标检测和目标跟踪。它的定位是“看见什么”和“目标在图像中的位置”，不直接决定无人机或无人车怎么运动。

### 包结构

```text
perception/sunray_perception/
├── config/
│   ├── camera/
│   ├── detection/
│   └── tracking/
├── launch/
├── msg/
├── models/
└── src/
```

### 节点与启动方式

ArUco 检测：

```bash
roslaunch sunray_perception aruco_detection.launch image_topic:=/web_cam/image_raw
```

NPU 检测：

```bash
roslaunch sunray_perception npu_detection.launch image_topic:=/web_cam/image_raw
```

单 ArUco 跟踪：

```bash
roslaunch sunray_perception single_aruco_tracking.launch
```

ArUco board 跟踪：

```bash
roslaunch sunray_perception aruco_board_tracking.launch
```

NPU 跟踪：

```bash
roslaunch sunray_perception npu_tracking.launch
```

### 消息

`Detection.msg` 表示单个检测目标：

| 字段 | 说明 |
| --- | --- |
| `detector_type` | 检测器类型，例如 `aruco` 或 `npu`。 |
| `class_id` / `class_name` | 类别 ID 和类别名。 |
| `score` | 置信度。 |
| `corners_x/y` | 角点坐标，ArUco 常用。 |
| `center_x/y` | 目标中心像素。 |
| `width` / `height` | 目标框宽高。 |

`DetectionArray.msg` 是检测数组，适合一帧图像中存在多个目标。

`Tracking.msg` 表示跟踪估计结果：

| 字段 | 说明 |
| --- | --- |
| `height` / `width` | 图像尺寸。 |
| `fov_x` / `fov_y` | 相机视场角。 |
| `mean_px/py/pz` | 目标平均位置估计。 |
| `mean_roll/pitch/yaw` | 姿态估计。 |

### 配置文件

检测配置：

```text
config/detection/aruco_detector.yaml
config/detection/npu_detector.yaml
```

跟踪配置：

```text
config/tracking/single_aruco_tracking.yaml
config/tracking/aruco_board_tracking.yaml
config/tracking/npu_tracking.yaml
```

相机配置：

```text
config/camera/usb_camera.yaml
config/camera/gimbal.yaml
```

### 二次开发建议

- 图像输入优先来自 `web_cam`、`realsense2_camera` 或 `sunray_gimbal`。
- 感知节点只发布检测/跟踪结果；自动降落、目标跟随、避障决策应写在任务节点或规划控制层。
- 新增检测模型时，优先扩展 `config/detection/npu_detector.yaml` 和 `models/`，再确认后端 `npu_backend_*` 是否支持。
- 如果要把感知结果用于飞行控制，应先转换为 `sunray_msgs/UAVControlCMD` 或规划命令，不要让感知节点直接发布 MAVROS setpoint。

</section>
