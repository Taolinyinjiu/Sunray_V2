<!-- title: 功能代码分析 -->

<section id="perception-code-analysis">

## 功能代码分析

本页按代码路径解释 `sunray_perception` 的主要实现。新手二次开发时，建议先理解抽象接口，再进入具体检测器或跟踪器。

### detection_node.cpp

入口文件：

```text
src/detection_node.cpp
```

节点启动时读取私有参数：

| 参数 | 说明 |
| --- | --- |
| `~detector_type` | 检测器类型，当前支持 `aruco`、`npu`。 |
| `~image_topic` | 输入图像话题。 |
| `~config_path` | 检测器配置 YAML。 |
| `~model_path` | NPU 模型路径；ArUco 不需要。 |

初始化流程：

```text
读取参数
  -> createDetector(detector_type)
  -> detector->load(config_path, model_path)
  -> 订阅 image_topic
  -> 发布 /sunray_perception/detections
  -> 发布 /sunray_perception/detection_image
```

回调流程：

```text
sensor_msgs/Image
  -> cv_bridge 转 BGR8
  -> detector->detect()
  -> 发布 DetectionArray
  -> 如果 debug 图像有订阅者，发布 detection_image
```

这里有一个重要设计：`detection_node` 不关心具体算法，只依赖 `Detector` 抽象类。新增检测器时，不需要重写节点主流程。

### Detector 抽象接口

接口文件：

```text
include/sunray_perception/detection/detectors.h
```

核心接口：

```cpp
class Detector {
public:
  virtual bool load(const std::string& config_path,
                    const std::string& model_path,
                    std::string* error) = 0;

  virtual bool detect(const cv::Mat& image,
                      const std_msgs::Header& header,
                      DetectionArray* detections,
                      cv::Mat* debug_image) = 0;
};
```

二次开发含义：

- `load()` 负责读取 YAML、模型、阈值、类别名等静态资源。
- `detect()` 负责处理单帧图像，输出结构化检测结果和调试图。
- `createDetector()` 是工厂函数，新增 detector 类型必须在这里注册。

### ArucoDetector

实现位置：

```text
src/detection/detectors.cpp
```

`ArucoDetector` 使用 OpenCV ArUco：

```text
读取 dictionaryId
  -> 创建 cv::aruco dictionary
  -> 读取 DetectorParameters
  -> detectMarkers()
  -> drawDetectedMarkers()
  -> 填充 DetectionArray
```

输出规则：

| Detection 字段 | ArUco 填充方式 |
| --- | --- |
| `detector_type` | `aruco` |
| `class_id` | marker id |
| `class_name` | `aruco_<id>` |
| `score` | `1.0` |
| `corners_x/y` | OpenCV 检测到的四个角点 |
| `center_x/y` | 四角平均值 |
| `width/height` | 角点外接框尺寸 |

常改配置：

```text
config/detection/aruco_detector.yaml
```

重点字段：

- `dictionaryId`：ArUco 字典。
- `adaptiveThresh*`：自适应阈值。
- `cornerRefinement*`：角点细化。
- `minMarkerPerimeterRate` / `maxMarkerPerimeterRate`：marker 尺寸范围。
- `detectInvertedMarker`：是否检测反色 marker。

### NpuDetector

`NpuDetector` 用于 YOLO26 风格模型推理，当前支持两个后端：

| `backend_type` | 后端文件 | 典型平台 |
| --- | --- | --- |
| `rk3588` | `npu_backend_rknn.cpp` | RK3588/RKNN |
| `orin_nx` | `npu_backend_tensorrt.cpp` | NVIDIA Orin NX/TensorRT |

加载流程：

```text
检查 model_path
  -> 读取 npu_detector.yaml
  -> 检查 backend_type / input_width / input_height / threshold
  -> 读取 class_names
  -> createNpuBackend()
  -> backend->load(model_path, input_spec)
```

推理流程：

```text
BGR 图像
  -> letterbox resize 到模型输入尺寸
  -> BGR 转 RGB
  -> 按 NCHW/NHWC 和 FLOAT32/UINT8 打包
  -> backend->infer()
  -> decodeYolo26()
  -> NMS
  -> 填充 DetectionArray
  -> 绘制 debug_image
```

`decodeYolo26()` 假设输出形状类似：

```text
(1, nc + 4, 8400)
```

如果候选数不是 8400，代码会按实际 shape 解码并打印节流 warning。

常改配置：

```text
config/detection/npu_detector.yaml
```

重点字段：

| 字段 | 说明 |
| --- | --- |
| `backend_type` | `rk3588` 或 `orin_nx`。 |
| `input_width/height` | 模型输入尺寸。 |
| `confidence_threshold` | 置信度阈值。 |
| `nms_iou_threshold` | NMS IoU 阈值。 |
| `max_detections` | 最多输出目标数。 |
| `class_names` | 类别 ID 到类别名映射。 |

### tracking_node.cpp

入口文件：

```text
src/tracking_node.cpp
```

节点读取：

| 参数 | 说明 |
| --- | --- |
| `~tracker_type` | 当前支持 `single_aruco`、`aruco_board`、`npu`。 |
| `~camera_config_path` | 相机内参配置。 |
| `~tracking_config_path` | 跟踪器配置。 |

初始化流程：

```text
loadCameraConfig()
  -> createTracker(tracker_type)
  -> tracker->load()
  -> 订阅 /sunray_perception/detections
  -> 发布 /sunray_perception/tracking
```

回调流程：

```text
DetectionArray
  -> tracker->track()
  -> Tracking
  -> /sunray_perception/tracking
```

如果当前帧没有有效目标，节点不会发布 tracking，并用节流 warning 提示。

### CameraConfig

实现位置：

```text
include/sunray_perception/common/config_utils.h
src/common/config_utils.cpp
```

相机配置必须包含：

| 字段 | 说明 |
| --- | --- |
| `image_width` | 图像宽度。 |
| `image_height` | 图像高度。 |
| `camera_matrix` | 3x3 相机内参矩阵。 |
| `distortion` | 畸变系数数组。 |

代码会根据内参计算：

```text
fov_x = 2 * atan(width / (2 * fx))
fov_y = 2 * atan(height / (2 * fy))
```

跟踪结果中的 `fov_x/fov_y` 就来自这里。

### SingleArucoTracker

配置：

```text
config/tracking/single_aruco_tracking.yaml
```

字段：

| 字段 | 说明 |
| --- | --- |
| `dictionaryId` | ArUco 字典。 |
| `markerId` | 要跟踪的 marker ID。 |
| `markerLength` | marker 实际边长，单位 m。 |

处理流程：

```text
从 DetectionArray 中查找 detector_type=aruco 且 class_id=markerId 的目标
  -> 读取四个角点
  -> 构造 marker 四个 3D 角点
  -> cv::solvePnP()
  -> Rodrigues 转旋转矩阵
  -> rotationMatrixToRpy()
  -> 填充 Tracking
```

这个跟踪器适合单 marker 定位或精降类任务。

### ArucoBoardTracker

配置：

```text
config/tracking/aruco_board_tracking.yaml
aruco/out/layout_offsets.csv
```

字段：

| 字段 | 说明 |
| --- | --- |
| `dictionaryId` | ArUco 字典。 |
| `layoutOffsetsPath` | board 布局 CSV。 |
| `smallMarkerLength` | 小 marker 边长。 |
| `sizeUnitScale` | 不同尺寸 marker 相对小 marker 的比例。 |

处理流程：

```text
读取 layout_offsets.csv
  -> 为每个 marker id 构造 3D 角点
  -> 当前帧中匹配多个 ArUco 检测
  -> 汇总 object_points 和 image_points
  -> solvePnP()
  -> 姿态轴修正
  -> 填充 Tracking
```

相比单 marker，board tracking 利用多个 marker，目标更大，部分 marker 被遮挡时也更容易保留有效位姿。

### NpuTracker

当前 `NpuTracker` 是占位实现：

```text
load() 返回 true
track() 返回 false
```

启动 `npu_tracking.launch` 后不会发布有效 tracking，只会打印节流 warning。后续如果要实现 NPU tracking，建议先定义清楚：

- 跟踪的目标类别。
- 多目标时选择哪个目标。
- 是否需要 3D 深度。
- 没有深度时如何从 2D 框推测相对方向。
- 是否要输出速度、ID 或目标生命周期。

### 新增检测器

推荐步骤：

1. 新建类继承 `Detector`。
2. 在 `load()` 中读取配置和模型。
3. 在 `detect()` 中输出 `DetectionArray` 和 debug 图。
4. 在 `createDetector()` 中注册新 `detector_type`。
5. 新增 `config/detection/*.yaml`。
6. 新增 launch，传入 `detector_type`、`image_topic`、`config_path`、`model_path`。
7. 在文档和示例中说明输出字段语义。

### 新增跟踪器

推荐步骤：

1. 新建类继承 `Tracker`。
2. 在 `load()` 中读取相机和跟踪配置。
3. 在 `track()` 中从 `DetectionArray` 选目标并填充 `Tracking`。
4. 在 `createTracker()` 中注册新 `tracker_type`。
5. 新增 `config/tracking/*.yaml`。
6. 新增 launch。

### 常见问题

| 现象 | 优先检查 |
| --- | --- |
| 没有 detections | 图像话题、`detector_type`、配置路径、模型路径。 |
| ArUco 检测不稳定 | 光照、分辨率、字典、marker 尺寸、阈值参数。 |
| NPU 无输出 | 后端类型、模型格式、输入尺寸、阈值过高。 |
| tracking 不发布 | 是否有 detections、marker ID 是否匹配、相机内参是否正确。 |
| 位姿方向不对 | 相机坐标系、board 布局、轴修正、PnP 角点顺序。 |

</section>
