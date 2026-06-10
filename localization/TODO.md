# Localization TODO

这个文件记录 `localization` 目录后续可优化的方向。README 只保留系统说明和使用说明，具体改造项集中维护在这里。

## 1. 统一参数化话题和 frame

部分节点仍使用硬编码话题或 frame，例如：

- `fast_lio/src/transform_odom_pointCloud.cpp` 固定订阅 `/livox/imu`、`/Odometry`、`/PointCloud`。
- `open3d_loc` 固定使用 `/Odometry_loc`、`/cloud_registered_1`、`map/odom/base_link` 等。

建议统一改为 ROS 参数，并在 launch 文件中显式配置，减少不同机型和多机环境下的改代码需求。

## 2. 统一命名和枚举拼写

历史拼写问题已统一处理，定位源枚举、全局定位类名和 IMU 处理函数名已改为规范命名。

## 3. 加强时间同步和缓冲管理

`ekf_odometry` 当前用两个 `deque` 手动匹配 IMU 与 odom，缺少队列长度上限、时间戳异常处理和插值。建议：

- 给 IMU/odom buffer 增加最大长度。
- 对倒退时间戳、过大 dt、空队列状态做保护。
- 对 odom 更新时刻做 IMU 插值或更严格的时间同步。
- 将噪声参数、初始化帧数、发布话题全部参数化。

## 6. 改善 CMake 和依赖配置

当前可优化项：

- `open3d_loc/CMakeLists.txt` 中 `Open3D_DIR` 是绝对路径 `/home/liar/open3d141/...`，应改为环境变量、CMake 参数或系统查找。
- `fast_lio/CMakeLists.txt` 同时设置 Debug 和 `-O3`，构建类型语义不清。
- 多处重复 `-std=c++14` 或 `-std=c++0x`。
- 建议统一 CMake 最低版本、C++ 标准和 install 规则。

## 8. 增加测试和回放验证

建议增加以下最小测试：

- YAML 配置解析测试。
- `source_frame_to_base` 外参转换单元测试。
- `global_to_local_tf` 重定位计算测试。
- rosbag 回放下的 `odom_state.odometry_valid`、输出频率和 TF 连通性检查。

## 9. 改善多机命名空间一致性

当前部分包按 `/uavX/...` 输出，部分包使用全局 `/sunray/...`。建议统一遵循：

```text
/{agent_name}{agent_id}/sunray/...
```

对于必须全局唯一的话题，应在 README 和 launch 参数中明确说明。

## 10. 文档化实机标定流程

定位质量高度依赖外参和时间同步。建议补充：

- LiDAR-IMU 外参标定流程。
- VIO 相机-IMU 外参和时间偏移标定流程。
- 动捕坐标系与机体系对齐流程。
- PX4 EKF2 外部视觉参数配置建议。
