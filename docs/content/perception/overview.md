<!-- title: 视觉感知总览 -->

<section id="perception-overview">

## 视觉感知总览

`perception/sunray_perception` 是 Sunray 的视觉感知功能包，当前包含 ArUco 检测、NPU 目标检测、单 ArUco 位姿跟踪、ArUco board 位姿跟踪，以及预留的 NPU tracking 接口。

它的定位是“看见什么”和“目标相对相机在哪里”，不直接控制无人机运动。自动降落、目标跟随、搜索任务等上层逻辑应订阅感知输出，再转换成 `sunray_msgs/UAVControlCMD`、规划命令或任务状态。

### 数据流

检测链路：

```text
web_cam / realsense2_camera / sunray_gimbal
  -> sensor_msgs/Image
  -> detection_node
  -> /sunray_perception/detections
  -> /sunray_perception/detection_image
```

跟踪链路：

```text
/sunray_perception/detections
  + camera config
  + tracking config
  -> tracking_node
  -> /sunray_perception/tracking
```

### 包结构

```text
perception/sunray_perception/
├── aruco/
│   ├── generate_aruco_layout.py
│   ├── layout.csv
│   └── out/layout_offsets.csv
├── config/
│   ├── camera/
│   ├── detection/
│   └── tracking/
├── include/sunray_perception/
├── launch/
├── models/
├── msg/
└── src/
```

### 节点与启动

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

### 话题

| 话题 | 类型 | 发布者 | 说明 |
| --- | --- | --- | --- |
| `/sunray_perception/detections` | `sunray_perception/DetectionArray` | `detection_node` | 当前帧检测结果数组。 |
| `/sunray_perception/detection_image` | `sensor_msgs/Image` | `detection_node` | 调试图像，有订阅者时才发布。 |
| `/sunray_perception/tracking` | `sunray_perception/Tracking` | `tracking_node` | 位姿/跟踪估计结果。 |

### 消息

`Detection.msg` 表示单个检测目标：

| 字段 | 说明 |
| --- | --- |
| `detector_type` | 检测器类型，例如 `aruco` 或 `npu`。 |
| `class_id` / `class_name` | 类别 ID 和类别名。 |
| `score` | 置信度。ArUco 固定为 `1.0`。 |
| `corners_x/y` | 角点坐标。ArUco 是 marker 四角；NPU 是框的四角。 |
| `center_x/y` | 目标中心像素。 |
| `width` / `height` | 目标框宽高。 |

`DetectionArray.msg` 是检测数组：

```text
std_msgs/Header header
sunray_perception/Detection[] detections
```

`Tracking.msg` 表示跟踪估计：

| 字段 | 说明 |
| --- | --- |
| `height` / `width` | 图像尺寸。 |
| `fov_x` / `fov_y` | 相机视场角，由相机内参计算。 |
| `mean_px/py/pz` | 目标相对相机的平移估计。 |
| `mean_roll/pitch/yaw` | 目标姿态估计。 |

### 配置文件

相机配置：

```text
config/camera/usb_camera.yaml
config/camera/gimbal.yaml
```

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

模型文件：

```text
models/yolov26.rknn
models/yolov26.plan
```

### 二次开发建议

- 换摄像头时，先保证图像话题正常，再改 `image_topic` 和相机内参。
- 做 ArUco 位姿估计时，`markerLength`、相机内参和畸变参数必须准确。
- 做 NPU 检测时，`backend_type`、模型格式、输入尺寸和 class_names 必须匹配。
- 感知结果用于飞控前，应先经过任务层判断和限幅，不要在感知节点里直接发布 MAVROS setpoint。

</section>
