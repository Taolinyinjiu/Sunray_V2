<!-- title: 定位模块 -->

<section id="localization-overview">

## 定位总览

`localization` 目录是 Sunray 的定位子系统集合，用来把仿真、动捕、视觉惯性、激光惯性、全局点云重定位等不同定位来源，转换成控制和上层任务可统一使用的里程计、TF 和状态信息。

它不是单一算法包，而是一组 ROS 包：

| 类型 | 功能包 | 说明 |
| --- | --- | --- |
| 前端定位算法 | `fast_lio`、`vins-fusion`、`vins-fusion-gpu` | 直接使用传感器数据估计自身运动。 |
| 传感器或系统适配 | `sunray_mocap`、`sunray_viobot` | 把外部系统输出转换成 Sunray 更容易复用的 odom。 |
| 高频融合与补偿 | `ekf_odometry` | 使用 IMU 对低频 odom 做预测和更新。 |
| 全局点云定位 | `open3d_loc` | 使用已有点云地图进行全局定位或重定位。 |
| Sunray 统一输出层 | `localization_fusion` | 统一定位出口，向控制、规划、集群、可视化输出标准话题和 TF。 |

对二次开发者来说，最重要的原则是：控制和规划尽量只依赖 `localization_fusion` 的标准输出，不直接耦合某一个定位算法的私有话题。这样以后从 Gazebo 切到动捕、从动捕切到 FAST-LIO，控制接口不需要跟着重写。

### 目录分类

当前 `localization` 目录按定位链路角色分成三类：

```text
localization/
├── localization_fusion/           # Sunray 统一定位出口
├── localization_sources/          # Sunray 维护的定位源适配和补偿
│   ├── sunray_mocap/
│   ├── sunray_viobot/
│   └── ekf_odometry/
└── third_party_localization/      # 第三方定位算法或基于第三方算法改造的包
    ├── fast_lio/
    ├── open3d_loc/
    ├── vins-fusion/
    └── vins-fusion-gpu/
```

- `localization_fusion` 单独保留在一级目录，因为它是控制、规划和上层任务推荐依赖的统一出口层。
- `localization_sources` 放 Sunray 自己维护的定位源适配、同步、补偿代码。这些包通常服务于 `localization_fusion`，把外部系统或低频 odom 转成可统一消费的定位输入。
- `third_party_localization` 放第三方定位算法源码或基于第三方算法修改的包。这样后续升级第三方算法、替换算法版本或编写 Sunray 适配层时，边界更清楚。

### 总体架构

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

### 包结构

| 包 | 作用 | 主要输入 | 主要输出 |
| --- | --- | --- | --- |
| `localization_fusion` | 统一定位出口，负责坐标系规范化、状态监控和多定位源配置 | 各定位源 `nav_msgs/Odometry`，可选重定位 `nav_msgs/Odometry` | `${agent_key}/sunray/localization/local_odom`、`global_odom`、`odom_state`、TF |
| `sunray_mocap` | 动捕适配器，把 VRPN 的 pose/twist 同步成 odom | `/vrpn_client_node/uavX/pose`、`/vrpn_client_node/uavX/twist` | `/uavX/sunray/odometry` |
| `sunray_viobot` | VIOBOT/Baton mini 适配器，修正姿态方向后输出 odom | `/baton/stereo3/odometry` | `/uavX/sunray/odometry` |
| `fast_lio` | FAST-LIO/FAST-LIO2 激光惯性里程计，支持 Livox、Velodyne、Ouster 和仿真点云 | LiDAR 点云、IMU | `/Odometry`、`/PointCloud`，以及 Sunray 封装输出 `/sunray/odometry`、`/sunray/pointCloud` |
| `ekf_odometry` | 用 IMU 对低频 odom 做预测更新，生成更高频里程计 | `/livox/imu`、`/Odometry` | `/ekf_odometry`；可选转换节点可输出 `/sunray/odometry`、`/sunray/pointCloud` |
| `open3d_loc` | 基于 Open3D 的点云地图全局定位/重定位 | FAST-LIO odom、当前点云、初始位姿、PLY 地图 | `/localization_3d`、`/odom2map`、`/baselink2map`、点云可视化话题 |
| `vins-fusion` | 原版 VINS-Fusion 视觉/视觉惯性里程计 | 相机、IMU | VINS 自身 odom/pose 话题 |
| `vins-fusion-gpu` | 带 GPU 加速的 VINS-Fusion | 相机、IMU | VINS 自身 odom/pose 话题 |

### 各子包说明

定位模块按功能拆成多个页面。阅读时可以按下面的关系查找：

| 子包/主题 | 当前页面 | 建议重点 |
| --- | --- | --- |
| `localization_fusion` | `localization_fusion` | 主节点、monitor、RViz、统一输出、定位源配置、重定位 TF。 |
| `sunray_mocap` | `sunray_mocap` | VRPN pose/twist 如何同步成 Sunray odom。 |
| `sunray_viobot` | `sunray_viobot` | VIOBOT/Baton mini 输出如何转换成 Sunray odom。 |
| `fast_lio` | `fast_lio` | FAST-LIO/FAST-LIO2 的输入、输出、配置和 Sunray 封装输出。 |
| `ekf_odometry` | `ekf_odometry` | IMU 对低频 odom 的高频预测补偿。 |
| `open3d_loc` | `open3d_loc` | 点云地图全局定位、重定位和 map/local 坐标关系。 |
| `vins-fusion` | `vins-fusion` | 原版 VINS-Fusion 的使用边界和接入方式。 |
| `vins-fusion-gpu` | `vins-fusion-gpu` | GPU 加速版本的使用边界和接入方式。 |
| 扩展新定位源 | `扩展与排错` | 新增 `source_id`、配置 odom 话题、外参、timeout 和调试步骤。 |
| 调试建议 | `扩展与排错` | 定位无效、TF 不对、频率异常、重定位跳变等问题排查。 |
| 后续优化 | `TODO` 章节 | 定位相关待优化项统一迁移到附录后的 TODO 章节。 |

### 坐标系约定

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

### 统一输出

以 `agent_name=uav`、`agent_id=1` 为例：

```text
/uav1/sunray/localization/local_odom
/uav1/sunray/localization/global_odom
/uav1/sunray/localization/odom_state
/uav1/sunray/localization/rviz_markers
```

其中 `odom_state` 类型为 `sunray_msgs/OdomState`，适合作为定位健康状态的统一入口。它包含：

- 当前定位源枚举值。
- 外部定位源订阅话题。
- 当前定位是否有效。
- 外部 odom 更新频率。
- local/global odom 快照。
- world/global/local/base_link 的 frame 名和 TF 快照。

### 定位源配置

统一配置文件：

```text
localization/localization_fusion/config/localization_sources.yaml
```

当前定义的定位源：

| source_id | 名称 | 当前输入话题 | 说明 |
| --- | --- | --- | --- |
| `0` | `VIOBOT` | 当前为空 | 预留 VIOBOT 输入配置；如要直接走 fusion，需要补齐 `odometry_topic`。 |
| `1` | `MOCAP` | `${agent_key}/sunray/odometry` | 动捕适配后的 odom。 |
| `2` | `VINS` | `/vins_estimator/imu_propagate` | VINS 输出。 |
| `3` | `GAZEBO` | `${agent_key}/sunray/gazebo_pose` | Gazebo 仿真真值/位姿。 |
| `4` | `GAZEBO_ARUCO` | `${agent_key}/sunray_odom_in` | Gazebo + ArUco 重定位；重定位话题为 `/aruco_test`。 |
| `5` | `SUNRAY_SIM` | `${agent_key}/sunray_sim/odom` | sunray_sim 轻量仿真器。 |
| `6` | `Fast_LIO_EKF` | `/sunray/odometry` | MID360 + FAST-LIO 链路当前给 fusion 的输入；默认配置不是 `/sunray/ekf_odometry`。 |

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

参数含义：

| 参数 | 作用 | 修改时机 |
| --- | --- | --- |
| `odometry_topic` | 当前定位源输出的 `nav_msgs/Odometry` 话题。 | 换算法、换仿真器、换适配节点时修改。 |
| `relocalization_topic` | 可选重定位话题，类型也应为 `nav_msgs/Odometry`。语义是 `base_link` 在 `sunray_global` 下的位姿。 | 接入 ArUco、点云地图重定位、全局定位时填写。 |
| `timeout_s` | 超过这个时间没有收到输入 odom，`odom_state.odometry_valid` 会变为 `false`。 | 定位频率较低或网络延迟较大时适当调大。 |
| `source_frame_to_base` | 定位源输出坐标系到机体中心 `base_link` 的外参矩阵。 | 定位源输出的是相机、雷达、动捕刚体标记点而不是机体中心时修改。 |

如果定位源已经输出机体中心位姿，可以保持单位矩阵。

### 常用启动方式

Gazebo 或仿真定位：

```bash
roslaunch localization_fusion localization_fusion.launch source_id:=3 agent_name:=uav agent_id:=1
```

动捕定位：

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

MID360 + FAST-LIO + EKF：

```bash
roslaunch localization_fusion localization_fusion.launch source_id:=6 agent_name:=uav agent_id:=1
```

该 launch 会包含：

```text
livox_ros_driver2/launch_ROS1/msg_MID360.launch
fast_lio/launch/mapping_mid360.launch
ekf_odometry/launch/ekf_odometry.launch
localization_fusion_node
```

多机仿真：

```bash
roslaunch localization_fusion localization_fusion_swarm.launch source_id:=5 agent_name:=uav agent_num:=6
```

`localization_fusion_swarm.launch` 会递归启动 `uav1` 到 `uavN` 的 `localization_fusion`，并只在 `agent_id=1` 时启动集群监视器。

### 新手学习顺序

建议按这个顺序读定位模块：

1. 先理解本页的统一输入输出和坐标系。
2. 再读 `localization_fusion`，确认 Sunray 标准定位出口是怎么工作的。
3. 根据自己使用的定位来源，选择读 `sunray_mocap`、`sunray_viobot`、`fast_lio`、`vins-fusion` 或仿真相关配置。
4. 如果需要重定位或地图对齐，再读 `open3d_loc`。
5. 如果要自己接新算法，最后读“扩展与排错”。

</section>
