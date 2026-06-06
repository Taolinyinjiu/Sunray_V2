# Sunray Localization

`localization` 目录是 Sunray 的定位子系统集合，用来把仿真、动捕、视觉惯性、激光惯性、全局点云重定位等不同定位来源，转换成控制和上层任务可统一使用的里程计、TF 和状态信息。

它不是单一算法包，而是一组 ROS 包：

- 前端定位算法：`fast_lio`、`vins-fusion`、`vins-fusion-gpu`
- 传感器或系统适配：`sunray_mocap`、`sunray_viobot`
- 高频融合与补偿：`ekf_odometry`
- 全局点云定位：`open3d_loc`
- Sunray 统一输出层：`localization_fusion`

## 总体架构

推荐把 `localization_fusion` 理解为定位系统的出口层。不同定位源先发布各自的 `nav_msgs/Odometry`，然后由 `localization_fusion` 统一转换成 Sunray 约定的话题和坐标系。

典型数据流：

```text
传感器/仿真器/算法
  -> 原始定位源 odom
  -> localization_fusion
  -> /uav1/sunray/localization/local_odom
  -> /uav1/sunray/localization/global_odom
  -> /uav1/sunray/localization/odom_state
  -> TF: world -> uav1/sunray_global -> uav1/sunray_local -> uav1/base_link
```

`localization_fusion` 做的主要事情：

- 按 `source_id` 从 `localization_fusion/config/localization_sources.yaml` 选择定位源。
- 订阅定位源的 `odometry_topic`。
- 按 `source_frame_to_base` 外参把传感器坐标系位姿转换到机体中心 `base_link`。
- 发布局部里程计 `local_odom`。
- 如果配置了重定位话题，则根据重定位结果更新 `sunray_global -> sunray_local`。
- 发布全局里程计 `global_odom`、TF 和 `sunray_msgs/OdomState` 状态快照。
- 按 `timeout_s` 判断定位源是否超时。

## 包结构

| 包 | 作用 | 主要输入 | 主要输出 |
| --- | --- | --- | --- |
| `localization_fusion` | 统一定位出口，负责坐标系规范化、状态监控和多定位源配置 | 各定位源 `nav_msgs/Odometry`，可选重定位 `nav_msgs/Odometry` | `${agent_key}/sunray/localization/local_odom`、`global_odom`、`odom_state`、TF |
| `sunray_mocap` | 动捕适配器，把 VRPN 的 pose/twist 同步成 odom | `/vrpn_client_node/uavX/pose`、`/vrpn_client_node/uavX/twist` | `/uavX/sunray/odometry` |
| `sunray_viobot` | VIOBOT/Baton mini 适配器，修正姿态方向后输出 odom | `/baton/stereo3/odometry` | `/uavX/sunray/odometry` |
| `fast_lio` | FAST-LIO/FAST-LIO2 激光惯性里程计，支持 Livox、Velodyne、Ouster 和仿真点云 | LiDAR 点云、IMU | `/Odometry`、`/PointCloud`，以及 Sunray 封装输出 `/sunray/odometry`、`/sunray/pointCloud` |
| `ekf_odometry` | 用 IMU 对低频 odom 做预测更新，生成更高频里程计 | `/livox/imu`、`/sunray/odometry` | `/sunray/ekf_odometry` |
| `open3d_loc` | 基于 Open3D 的点云地图全局定位/重定位 | FAST-LIO odom、当前点云、初始位姿、PLY 地图 | `/localization_3d`、`/odom2map`、`/baselink2map`、点云可视化话题 |
| `vins-fusion` | 原版 VINS-Fusion 视觉/视觉惯性里程计 | 相机、IMU | VINS 自身 odom/pose 话题 |
| `vins-fusion-gpu` | 带 GPU 加速的 VINS-Fusion | 相机、IMU | VINS 自身 odom/pose 话题 |

## 坐标系约定

`localization_fusion` 统一维护以下坐标系：

```text
world
  -> {agent_key}/sunray_global
  -> {agent_key}/sunray_local
  -> {agent_key}/base_link
```

含义：

- `world`：最外层世界坐标系，当前默认与 `{agent_key}/sunray_global` 零变换。
- `{agent_key}/sunray_global`：全局坐标系，用于重定位或多机全局一致性。
- `{agent_key}/sunray_local`：局部里程计坐标系，直接来自当前定位源。
- `{agent_key}/base_link`：无人机机体中心坐标系。

当没有重定位输入时，`global_to_local_tf` 默认为单位变换，因此 `global_odom` 与 `local_odom` 数值基本一致，只是 frame 名不同。

当有重定位输入时，`localization_fusion` 根据：

```text
T_global_local = T_global_base * inverse(T_local_base)
```

更新 `{agent_key}/sunray_global -> {agent_key}/sunray_local`，从而让局部里程计对齐到全局地图或全局定位结果。

## 统一输出

以 `agent_name=uav`、`agent_id=1` 为例：

```text
/uav1/sunray/localization/local_odom
/uav1/sunray/localization/global_odom
/uav1/sunray/localization/odom_state
/uav1/sunray/localization/rviz_markers
```

`odom_state` 类型为 `sunray_msgs/OdomState`，包含：

- 当前定位源枚举值。
- 外部定位源订阅话题。
- 当前定位是否有效。
- 外部 odom 更新频率。
- local/global odom 快照。
- world/global/local/base_link 的 frame 名和 TF 快照。

## 定位源配置

统一配置文件：

```text
localization/localization_fusion/config/localization_sources.yaml
```

当前定义的定位源：

| source_id | 名称 | 典型输入话题 | 说明 |
| --- | --- | --- | --- |
| `0` | `VIOBOT` | 当前为空 | 预留 VIOBOT 输入配置 |
| `1` | `MOCAP` | `${agent_key}/sunray/odometry` | 动捕适配后的 odom |
| `2` | `VINS` | `/vins_estimator/imu_propagate` | VINS 输出 |
| `3` | `GAZEBO` | `${agent_key}/sunray/gazebo_pose` | Gazebo 仿真真值/位姿 |
| `4` | `GAZEBO_ARUCO` | `${agent_key}/sunray_odom_in` | Gazebo + ArUco 重定位 |
| `5` | `PENGYU_SIM` | `${agent_key}/pengyu_sim/odom` | RViz/Pengyu 仿真器 |
| `6` | `Fast_LIO_EKF` | `/sunray/ekf_odometry` | MID360 + FAST-LIO + EKF |

每个定位源配置包含：

```yaml
odometry_topic: "${agent_key}/sunray/example/odom"
relocalization_topic: ""
timeout_s: 0.5
source_frame_to_base:
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
```

`source_frame_to_base` 是定位源输出坐标系到机体中心的外参。如果定位源已经输出机体中心位姿，可以保持单位矩阵。

## 常用启动方式

### Gazebo 或仿真定位

```bash
roslaunch localization_fusion localization_fusion.launch source_id:=3 agent_name:=uav agent_id:=1
```

### 动捕定位

```bash
roslaunch localization_fusion localization_fusion.launch source_id:=1 agent_name:=uav agent_id:=1
```

该 launch 会包含：

```text
vrpn_client_ros/sample.launch
sunray_mocap/mocap_odom.launch
localization_fusion_node
localization_fusion_monitor_node
rviz_visualization_localization_fusion_node
```

### MID360 + FAST-LIO + EKF

```bash
roslaunch localization_fusion localization_fusion.launch source_id:=6 agent_name:=uav agent_id:=1
```

该 launch 会包含：

```text
livox_ros_driver2/launch_ROS1/msg_MID360s.launch
fast_lio/launch/mapping_mid360.launch
ekf_odometry/launch/ekf_odometry.launch
localization_fusion_node
```

### 多机仿真

```bash
roslaunch localization_fusion localization_fusion_swarm.launch source_id:=5 agent_name:=uav agent_num:=6
```

`localization_fusion_swarm.launch` 会递归启动 `uav1` 到 `uavN` 的 `localization_fusion`，并只在 `agent_id=1` 时启动集群监视器。

## 各子包说明

### localization_fusion

这是定位子系统最重要的 Sunray 封装层。

核心文件：

```text
localization_fusion/src/localization_fusion.cpp
localization_fusion/include/localization_fusion.hpp
localization_fusion/include/localization_fusion_utils.hpp
localization_fusion/config/localization_sources.yaml
```

主要节点：

- `localization_fusion_node`：统一输出 local/global odom、TF 和 OdomState。
- `localization_fusion_monitor_node`：终端监控当前定位源、有效性、频率和输出状态。
- `rviz_visualization_localization_fusion_node`：根据 OdomState 发布 RViz Marker。

### sunray_mocap

动捕适配包。它订阅 VRPN 的 pose 与 twist，并使用 `message_filters::ApproximateTime` 做近似同步，最后发布标准 `nav_msgs/Odometry`。

默认话题：

```text
输入:
  /vrpn_client_node/uav1/pose
  /vrpn_client_node/uav1/twist
输出:
  /uav1/sunray/odometry
```

### sunray_viobot

VIOBOT/Baton mini 适配包。它订阅 `/baton/stereo3/odometry`，对姿态做固定旋转修正后发布 `/uavX/sunray/odometry`。

当前代码中订阅话题是硬编码的：

```text
/baton/stereo3/odometry
```

### fast_lio

FAST-LIO/FAST-LIO2 激光惯性里程计包。它保留了原算法结构，同时增加了 `transform_odom_pointCloud` 节点，用于把 FAST-LIO 的输出转换成 Sunray 使用的话题。

常用 MID360 启动文件：

```text
fast_lio/launch/mapping_mid360.launch
fast_lio/config/mid360.yaml
```

其中 `mid360.yaml` 默认使用：

```text
LiDAR: /livox/lidar
IMU:   /livox/imu
```

### ekf_odometry

`ekf_odometry` 用 IMU 对外部 odom 做预测，并在 odom 到来时更新。当前默认链路是：

```text
/livox/imu + /sunray/odometry -> /sunray/ekf_odometry
```

它主要服务于 `source_id=6` 的 `Fast_LIO_EKF` 定位源，用来把 FAST-LIO 输出和 IMU 高频数据组合成更高频、更平滑的里程计输出。

### open3d_loc

`open3d_loc` 用 Open3D 对当前点云和已有 PLY 地图做配准，估计 `odom -> map` 或 `base_link -> map` 的全局关系。它适合需要地图重定位、全局漂移修正的场景。

主要输入：

```text
/Odometry_loc
/cloud_registered_1
/initialpose
```

主要输出：

```text
/localization_3d
/localization_3d_confidence
/localization_3d_delay_ms
/odom2map
/baselink2map
/map
/submap
/scan
/scan2map
```

### vins-fusion 与 vins-fusion-gpu

这两个目录基本是第三方 VINS-Fusion 算法代码：

- `vins-fusion`：CPU 版本。
- `vins-fusion-gpu`：带 GPU 加速选项的版本。

Sunray 当前主要把 VINS 作为一种可选定位源，通过 `localization_sources.yaml` 中 `source_id=2` 接入 `/vins_estimator/imu_propagate`。

## 扩展新定位源

新增定位源建议按以下步骤：

1. 让新算法或适配节点输出 `nav_msgs/Odometry`。
2. 在 `localization_fusion/config/localization_sources.yaml` 中新增一项，分配新的 `source_id`。
3. 填写 `odometry_topic`、可选 `relocalization_topic`、`timeout_s` 和 `source_frame_to_base`。
4. 如需一键启动，在 `localization_fusion/source_launch/` 下新增 source launch。
5. 在 `localization_fusion/launch/localization_fusion.launch` 中按 `source_id` include 新 launch。
6. 如需状态枚举同步，更新 `sunray_msgs/OdomState.msg`。

## 调试建议

查看定位源是否有数据：

```bash
rostopic hz /uav1/sunray/localization/local_odom
rostopic echo /uav1/sunray/localization/odom_state
```

查看 TF：

```bash
rosrun tf view_frames
rosrun tf tf_echo uav1/sunray_local uav1/base_link
```

查看当前 source 配置：

```bash
roslaunch localization_fusion localization_fusion.launch source_id:=3 agent_name:=uav agent_id:=1
```

如果 `odom_state.odometry_valid=false`，优先检查：

- `source_id` 是否选对。
- `localization_sources.yaml` 中的 `odometry_topic` 是否有数据。
- `agent_name/agent_id` 是否与上游话题一致。
- `timeout_s` 是否过小。
- 输入 odom 的时间戳是否异常。

## 当前代码优化方向

### 1. 统一参数化话题和 frame

部分节点仍使用硬编码话题或 frame，例如：

- `sunray_viobot` 固定订阅 `/baton/stereo3/odometry`。
- `ekf_odometry` 固定发布 `/sunray/ekf_odometry`。
- `fast_lio/src/transform_odom_pointCloud.cpp` 固定订阅 `/livox/imu`、`/Odometry`、`/PointCloud`。
- `open3d_loc` 固定使用 `/Odometry_loc`、`/cloud_registered_1`、`map/odom/base_link` 等。

建议统一改为 ROS 参数，并在 launch 文件中显式配置，减少不同机型和多机环境下的改代码需求。

### 2. 统一命名和枚举拼写

当前存在一些历史拼写问题：

- `FASTLIO_EFK` 应考虑统一为 `FASTLIO_EKF`。
- `GloabalLocalization` 应考虑改为 `GlobalLocalization`。
- `ProcssIMU` 应考虑改为 `ProcessIMU`。

如果这些名称已被外部使用，建议先做兼容别名，再逐步迁移。

### 3. 加强时间同步和缓冲管理

`ekf_odometry` 当前用两个 `deque` 手动匹配 IMU 与 odom，缺少队列长度上限、时间戳异常处理和插值。建议：

- 给 IMU/odom buffer 增加最大长度。
- 对倒退时间戳、过大 dt、空队列状态做保护。
- 对 odom 更新时刻做 IMU 插值或更严格的时间同步。
- 将噪声参数、初始化帧数、发布话题全部参数化。

### 4. 明确协方差语义

多个节点发布 `nav_msgs/Odometry` 时没有填充 pose/twist covariance。对飞控、融合、诊断工具来说，协方差是判断定位可信度的重要信息。建议：

- 动捕、VIO、FAST-LIO、EKF 输出都按来源填写合理协方差。
- `localization_fusion` 转换外参时同步旋转协方差。
- `OdomState` 中可增加定位质量、延迟、重定位置信度等字段。

### 5. 降低第三方算法包和 Sunray 封装耦合

`fast_lio`、`vins-fusion`、`open3d_loc` 中既有第三方算法代码，也有 Sunray 适配代码。建议：

- 尽量不直接改第三方核心算法。
- 把 Sunray 适配节点放在单独 wrapper 包或明确的 `sunray_*` 节点中。
- 保留上游算法 README，另写 Sunray 使用说明。

### 6. 改善 CMake 和依赖配置

当前可优化项：

- `open3d_loc/CMakeLists.txt` 中 `Open3D_DIR` 是绝对路径 `/home/liar/open3d141/...`，应改为环境变量、CMake 参数或系统查找。
- `fast_lio/CMakeLists.txt` 同时设置 Debug 和 `-O3`，构建类型语义不清。
- 多处重复 `-std=c++14` 或 `-std=c++0x`。
- 建议统一 CMake 最低版本、C++ 标准和 install 规则。

### 7. 完善 launch 覆盖

`localization_fusion.launch` 中部分 `source_id` 目前只有占位注释，没有 include 实际启动源：

- `source_id=0` VIOBOT
- `source_id=2` VINS
- `source_id=3` GAZEBO
- `source_id=4` GAZEBO_ARUCO
- `source_id=5` PENGYU_SIM

建议补齐对应 `source_launch/start_*.launch`，让每个 source 都能一键启动或明确声明由外部系统提供。

### 8. 增加测试和回放验证

建议增加以下最小测试：

- YAML 配置解析测试。
- `source_frame_to_base` 外参转换单元测试。
- `global_to_local_tf` 重定位计算测试。
- rosbag 回放下的 `odom_state.odometry_valid`、输出频率和 TF 连通性检查。

### 9. 改善多机命名空间一致性

当前部分包按 `/uavX/...` 输出，部分包使用全局 `/sunray/...`。建议统一遵循：

```text
/{agent_name}{agent_id}/sunray/...
```

对于必须全局唯一的话题，应在 README 和 launch 参数中明确说明。

### 10. 文档化实机标定流程

定位质量高度依赖外参和时间同步。建议补充：

- LiDAR-IMU 外参标定流程。
- VIO 相机-IMU 外参和时间偏移标定流程。
- 动捕坐标系与机体系对齐流程。
- PX4 EKF2 外部视觉参数配置建议。
