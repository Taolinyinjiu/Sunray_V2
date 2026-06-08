# Localization TODO

这个文件记录 `localization` 目录后续可优化的方向。README 只保留系统说明和使用说明，具体改造项集中维护在这里。

## 1. 统一参数化话题和 frame

部分节点仍使用硬编码话题或 frame，例如：

- `sunray_viobot` 固定订阅 `/baton/stereo3/odometry`。
- `ekf_odometry` 固定发布 `/sunray/ekf_odometry`。
- `fast_lio/src/transform_odom_pointCloud.cpp` 固定订阅 `/livox/imu`、`/Odometry`、`/PointCloud`。
- `open3d_loc` 固定使用 `/Odometry_loc`、`/cloud_registered_1`、`map/odom/base_link` 等。

建议统一改为 ROS 参数，并在 launch 文件中显式配置，减少不同机型和多机环境下的改代码需求。

## 2. 统一命名和枚举拼写

当前存在一些历史拼写问题：

- `FASTLIO_EFK` 应考虑统一为 `FASTLIO_EKF`。
- `GloabalLocalization` 应考虑改为 `GlobalLocalization`。
- `ProcssIMU` 应考虑改为 `ProcessIMU`。

如果这些名称已被外部使用，建议先做兼容别名，再逐步迁移。

## 3. 加强时间同步和缓冲管理

`ekf_odometry` 当前用两个 `deque` 手动匹配 IMU 与 odom，缺少队列长度上限、时间戳异常处理和插值。建议：

- 给 IMU/odom buffer 增加最大长度。
- 对倒退时间戳、过大 dt、空队列状态做保护。
- 对 odom 更新时刻做 IMU 插值或更严格的时间同步。
- 将噪声参数、初始化帧数、发布话题全部参数化。

## 4. 明确协方差语义

多个节点发布 `nav_msgs/Odometry` 时没有填充 pose/twist covariance。对飞控、融合、诊断工具来说，协方差是判断定位可信度的重要信息。建议：

- 动捕、VIO、FAST-LIO、EKF 输出都按来源填写合理协方差。
- `localization_fusion` 转换外参时同步旋转协方差。
- `OdomState` 中可增加定位质量、延迟、重定位置信度等字段。

## 5. 降低第三方算法包和 Sunray 封装耦合

`fast_lio`、`vins-fusion`、`open3d_loc` 中既有第三方算法代码，也有 Sunray 适配代码。建议：

- 尽量不直接改第三方核心算法。
- 把 Sunray 适配节点放在单独 wrapper 包或明确的 `sunray_*` 节点中。
- 保留上游算法 README，另写 Sunray 使用说明。

## 6. 改善 CMake 和依赖配置

当前可优化项：

- `open3d_loc/CMakeLists.txt` 中 `Open3D_DIR` 是绝对路径 `/home/liar/open3d141/...`，应改为环境变量、CMake 参数或系统查找。
- `fast_lio/CMakeLists.txt` 同时设置 Debug 和 `-O3`，构建类型语义不清。
- 多处重复 `-std=c++14` 或 `-std=c++0x`。
- 建议统一 CMake 最低版本、C++ 标准和 install 规则。

## 7. 完善 launch 覆盖

`localization_fusion.launch` 中部分 `source_id` 目前只有占位注释，没有 include 实际启动源：

- `source_id=0` VIOBOT
- `source_id=2` VINS
- `source_id=3` GAZEBO
- `source_id=4` GAZEBO_ARUCO
- `source_id=5` PENGYU_SIM

建议补齐对应 `source_launch/start_*.launch`，让每个 source 都能一键启动或明确声明由外部系统提供。

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
