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

这是定位子系统最重要的 Sunray 封装层，也是推荐给控制、规划、监控模块使用的统一定位出口。

核心文件：

```text
localization_fusion/src/localization_fusion.cpp
localization_fusion/include/localization_fusion.hpp
localization_fusion/include/localization_fusion_utils.hpp
localization_fusion/config/localization_sources.yaml
```

主要节点：

- `localization_fusion_node`：统一输出 local/global odom、TF 和 OdomState。
- `localization_fusion_monitor_node`：终端打印监控节点，订阅一个或多个无人机的 OdomState，周期性打印定位源、输入话题、输出话题、定位有效性、频率、frame 和 odom 快照。
- `rviz_visualization_localization_fusion_node`：RViz 可视化节点，只订阅 OdomState，发布 MarkerArray 展示坐标轴、local/global odom 轨迹和定位状态文字。

`localization_fusion.launch` 默认会同时启动这三个节点。可以通过参数关闭监控或 RViz：

```bash
roslaunch localization_fusion localization_fusion.launch \
  source_id:=3 \
  agent_name:=uav \
  agent_id:=1 \
  enable_monitor:=true \
  enable_rviz_visualization:=true
```

#### 主节点：localization_fusion_node

`localization_fusion_node` 的职责是把某个外部定位源转换成 Sunray 标准输出。

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `source_id` | `3` | 选择 `localization_sources.yaml` 中的定位源 |
| `config_yamlfile_path` | `$(find localization_fusion)/config/localization_sources.yaml` | 定位源配置文件 |
| `agent_name` | `uav` | 智能体名前缀 |
| `agent_id` | `1` | 智能体编号 |
| `use_private_agent_key` | `true` | 是否使用 launch 私有参数生成 agent key |

主节点的话题关系：

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | 配置中的 `odometry_topic` | `nav_msgs/Odometry` | 当前定位源的局部里程计输入 |
| 订阅 | 配置中的 `relocalization_topic` | `nav_msgs/Odometry` | 可选重定位输入；为空时不订阅 |
| 发布 | `/${agent_name}${agent_id}/sunray/localization/local_odom` | `nav_msgs/Odometry` | 标准局部里程计，frame 为 `{agent}/sunray_local -> {agent}/base_link` |
| 发布 | `/${agent_name}${agent_id}/sunray/localization/global_odom` | `nav_msgs/Odometry` | 标准全局里程计，frame 为 `{agent}/sunray_global -> {agent}/base_link` |
| 发布 | `/${agent_name}${agent_id}/sunray/localization/odom_state` | `sunray_msgs/OdomState` | 定位状态快照，包含输入/输出话题、有效性、频率、odom 和 TF |
| 发布 TF | `world -> {agent}/sunray_global` | `geometry_msgs/TransformStamped` | 静态 TF，默认单位变换 |
| 发布 TF | `{agent}/sunray_global -> {agent}/sunray_local` | `geometry_msgs/TransformStamped` | 重定位后更新；无重定位时为单位变换 |
| 发布 TF | `{agent}/sunray_local -> {agent}/base_link` | `geometry_msgs/TransformStamped` | 根据当前 local odom 更新 |

以 `uav1` 为例，主节点默认输出：

```text
/uav1/sunray/localization/local_odom
/uav1/sunray/localization/global_odom
/uav1/sunray/localization/odom_state
```

#### 终端打印节点：localization_fusion_monitor_node

`localization_fusion_monitor_node` 是定位状态的终端面板。它不参与定位计算，只订阅 `odom_state` 并打印状态。

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `agent_name` | `uav` | 智能体名前缀 |
| `agent_num` | `1` | 监控无人机数量；多机模式会订阅 `uav1` 到 `uavN` |
| `source_id` | `3` | 用于解析和显示当前定位源配置 |
| `config_yamlfile_path` | `localization_sources.yaml` | 定位源配置文件 |
| `print_hz` | `5.0` | 终端刷新频率 |
| `stale_timeout` | `1.0` | 超过该时间未收到 OdomState 时标记为陈旧 |

单机默认订阅：

```text
/uav1/sunray/localization/odom_state
```

多机模式下会订阅：

```text
/uav1/sunray/localization/odom_state
/uav2/sunray/localization/odom_state
...
/uavN/sunray/localization/odom_state
```

#### RViz 节点：rviz_visualization_localization_fusion_node

`rviz_visualization_localization_fusion_node` 是定位可视化节点。它只依赖 `OdomState`，不额外订阅 `local_odom` 或 `global_odom`。

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `agent_name` | `uav` | 智能体名前缀 |
| `agent_id` | `1` | 智能体编号 |
| `frame_id` | `world` | Marker 使用的参考坐标系 |
| `state_topic` | `/uav1/sunray/localization/odom_state` | OdomState 输入话题 |
| `marker_topic` | `/uav1/sunray/localization/rviz_markers` | MarkerArray 输出话题 |
| `world_axis_length` | 代码默认值 | world 坐标轴长度 |
| `tf_axis_length` | 代码默认值 | TF 坐标轴长度 |
| `trail_length` | 代码默认值 | local/global odom 轨迹保留长度 |

默认话题关系：

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/uav1/sunray/localization/odom_state` | `sunray_msgs/OdomState` | 定位状态快照 |
| 发布 | `/uav1/sunray/localization/rviz_markers` | `visualization_msgs/MarkerArray` | RViz Marker，包含坐标轴、轨迹和状态文字 |

在 RViz 中添加 `MarkerArray`，话题选择：

```text
/uav1/sunray/localization/rviz_markers
```

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

## 后续优化

代码优化和维护建议单独整理在 [TODO.md](TODO.md)。
