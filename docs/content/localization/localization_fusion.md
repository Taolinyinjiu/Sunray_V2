<section id="localization-fusion">

## localization_fusion

`localization_fusion` 是定位子系统最重要的 Sunray 封装层，也是推荐给控制、规划、监控模块使用的统一定位出口。它不负责实现 FAST-LIO、VINS、动捕或仿真算法本身，而是把这些定位源的 `nav_msgs/Odometry` 变成 Sunray 统一话题、统一 frame 和统一状态。

### 目录结构

```text
localization/localization_fusion/
├── config/
│   └── localization_sources.yaml
├── include/
│   ├── localization_fusion.hpp
│   └── localization_fusion_utils.hpp
├── launch/
│   ├── localization_fusion.launch
│   └── localization_fusion_swarm.launch
├── source_launch/
│   ├── start_fastlio_ekf.launch
│   └── start_mocap.launch
└── src/
    ├── localization_fusion.cpp
    ├── localization_fusion_node.cpp
    └── tools/
        ├── local_odom_to_vision_pose_node.cpp
        ├── localization_fusion_monitor_node.cpp
        └── rviz_visualization_localization_fusion_node.cpp
```

### 核心节点

| 节点 | 代码文件 | 作用 |
| --- | --- | --- |
| `localization_fusion_node` | `src/localization_fusion_node.cpp`、`src/localization_fusion.cpp` | 主节点。订阅外部 odom，发布标准 local/global odom、TF 和 `OdomState`。 |
| `localization_fusion_monitor_node` | `src/tools/localization_fusion_monitor_node.cpp` | 终端监控节点。订阅一个或多个无人机的 `OdomState`，周期性打印状态。 |
| `rviz_visualization_localization_fusion_node` | `src/tools/rviz_visualization_localization_fusion_node.cpp` | RViz Marker 节点。只依赖 `OdomState`，发布坐标轴、轨迹和状态文字。 |
| `local_odom_to_vision_pose_node` | `src/tools/local_odom_to_vision_pose_node.cpp` | 可选桥接节点。把 `local_odom` 转成 MAVROS `vision_pose/pose`。 |

本页按“主节点：localization_fusion_node”、“终端打印节点：localization_fusion_monitor_node”和“RViz 节点：rviz_visualization_localization_fusion_node”展开，把启动参数、输入输出和代码流程放在同一个页面里，方便从接口读到实现。

### 单机启动

最常用启动命令：

```bash
roslaunch localization_fusion localization_fusion.launch source_id:=3 agent_name:=uav agent_id:=1
```

`source_id` 决定当前使用哪个定位源。比如：

```bash
# Gazebo 仿真定位
roslaunch localization_fusion localization_fusion.launch source_id:=3 agent_name:=uav agent_id:=1

# 动捕定位
roslaunch localization_fusion localization_fusion.launch source_id:=1 agent_name:=uav agent_id:=1

# MID360 + FAST-LIO 链路
roslaunch localization_fusion localization_fusion.launch source_id:=6 agent_name:=uav agent_id:=1
```

`localization_fusion.launch` 主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `source_id` | `3` | 选择 `localization_sources.yaml` 中的定位源。 |
| `config_yamlfile_path` | `$(find localization_fusion)/config/localization_sources.yaml` | 定位源配置文件路径。 |
| `agent_name` | `uav` | 智能体名前缀。 |
| `agent_id` | `1` | 智能体编号。 |
| `use_private_agent_key` | `true` | 为 `true` 时使用 launch 中传入的 `agent_name` 和 `agent_id` 生成 `agent_key`，例如 `uav1`。 |
| `enable_monitor` | `true` | 是否启动终端监控节点。集群场景通常关闭单机 monitor，改用 swarm monitor。 |
| `enable_vision_pose` | `false` | 是否把 `local_odom` 转发为 MAVROS vision pose。 |
| `enable_rviz_visualization` | `true` | 是否启动 RViz Marker 发布节点。 |
| `rviz_frame_id` | `world` | Marker 使用的参考坐标系。 |
| `localization_state_topic` | `/uav1/sunray/localization/odom_state` | RViz 节点订阅的状态话题。 |
| `localization_marker_topic` | `/uav1/sunray/localization/rviz_markers` | RViz Marker 输出话题。 |

### 多机启动

多机仿真可以使用：

```bash
roslaunch localization_fusion localization_fusion_swarm.launch source_id:=5 agent_name:=uav agent_num:=6
```

这个 launch 会递归启动：

```text
uav1_localization_fusion_node
uav2_localization_fusion_node
...
uav6_localization_fusion_node
```

每个无人机单独输出：

```text
/uav1/sunray/localization/local_odom
/uav2/sunray/localization/local_odom
...
```

`localization_fusion_swarm.launch` 会把每个单机 launch 的 `enable_monitor` 设为 `false`，并且只在 `agent_id=1` 时启动一个 `localization_fusion_monitor_swarm`，用于集中监控 `uav1` 到 `uavN`。

### 主节点：localization_fusion_node

`localization_fusion_node` 是定位融合主节点。它的职责不是估计位姿，而是把当前选定定位源输出的 `nav_msgs/Odometry` 规范化成 Sunray 统一坐标系、统一话题和统一状态。

### 主节点输入输出

以 `agent_name=uav`、`agent_id=1` 为例：

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | 配置中的 `odometry_topic` | `nav_msgs/Odometry` | 当前定位源的局部里程计输入。 |
| 订阅 | 配置中的 `relocalization_topic` | `nav_msgs/Odometry` | 可选重定位输入；为空时不订阅。 |
| 发布 | `/uav1/sunray/localization/local_odom` | `nav_msgs/Odometry` | 标准局部里程计，frame 为 `uav1/sunray_local -> uav1/base_link`。 |
| 发布 | `/uav1/sunray/localization/global_odom` | `nav_msgs/Odometry` | 标准全局里程计，frame 为 `uav1/sunray_global -> uav1/base_link`。 |
| 发布 | `/uav1/sunray/localization/odom_state` | `sunray_msgs/OdomState` | 定位状态快照，包含输入/输出话题、有效性、频率、odom 和 TF。 |
| 发布 TF | `world -> uav1/sunray_global` | `geometry_msgs/TransformStamped` | 当前默认单位变换。 |
| 发布 TF | `uav1/sunray_global -> uav1/sunray_local` | `geometry_msgs/TransformStamped` | 重定位后更新；无重定位时为单位变换。 |
| 发布 TF | `uav1/sunray_local -> uav1/base_link` | `geometry_msgs/TransformStamped` | 根据当前 local odom 更新。 |

### source 配置文件

配置文件是：

```text
localization/localization_fusion/config/localization_sources.yaml
```

`sources_list` 下每一项都是一个定位源。主节点启动时根据 `source_id` 找到对应项，读取：

| 字段 | 代码如何使用 |
| --- | --- |
| `source_id` | 与 launch 参数 `source_id` 匹配，决定选中哪一项。 |
| `odometry_topic` | `localization_fusion_node` 订阅这个话题，输入必须是 `nav_msgs/Odometry`。 |
| `relocalization_topic` | 非空时订阅这个话题，用它更新 `sunray_global -> sunray_local`。 |
| `timeout_s` | 健康定时器用它判断定位是否超时。 |
| `source_frame_to_base` | 输入 odom 先乘这个外参，再发布成机体中心 `base_link` 的位姿。 |

`${agent_key}` 是配置文件中的占位符。假设 `agent_name=uav`、`agent_id=1`，则 `${agent_key}` 会被替换成 `uav1`。例如：

```yaml
odometry_topic: "${agent_key}/sunray/odometry"
```

实际订阅：

```text
/uav1/sunray/odometry
```

### 主节点代码流程

`src/localization_fusion_node.cpp` 只负责创建对象并进入 `ros::spin()`，主要逻辑在 `src/localization_fusion.cpp`。

初始化阶段：

1. 读取私有参数 `source_id`、`config_yamlfile_path`、`use_private_agent_key`。
2. 根据 `agent_name` 和 `agent_id` 生成 `agent_key`，例如 `uav1`。
3. 读取 `localization_sources.yaml`。
4. 通过 `source_id` 找到定位源配置。
5. 替换 `odometry_topic` 和 `relocalization_topic` 中的 `${agent_key}`。
6. 创建输入订阅器和标准输出发布器。
7. 启动健康检查定时器。

收到定位源 odom 时：

1. `odometry_callback` 接收外部 `nav_msgs/Odometry`。
2. `transform_source_odom_to_local` 根据 `source_frame_to_base` 转换位姿。
3. 输出 `local_odom`，frame 为 `{agent_key}/sunray_local -> {agent_key}/base_link`。
4. 更新输入频率统计。
5. 更新 `local_to_base_tf`。
6. 根据当前 `global_to_local_tf` 生成 `global_odom`。

收到重定位 odom 时：

1. `relocalization_callback` 接收 `base_link` 在 `sunray_global` 下的位姿。
2. 如果当前已经有有效 local odom，计算：

   ```text
   T_global_local = T_global_base * inverse(T_local_base)
   ```

3. 更新 `global_to_local_tf`。

健康定时器每 10 Hz 执行：

1. 检查当前时间和最近一次 odom 时间的差值。
2. 若超过 `timeout_s`，`odometry_valid=false`。
3. 定位有效时发布 TF。
4. 始终发布 `OdomState`，方便上层知道定位是否有效。

### source_frame_to_base 怎么理解

定位源输出的 odom 不一定表示机体中心。比如：

- 动捕刚体原点可能贴在飞机上方。
- VIO 输出可能是相机坐标系。
- 雷达惯导输出可能是 LiDAR 或 IMU 坐标系。

`source_frame_to_base` 用来描述“定位源输出坐标系到机体中心”的固定外参。代码中对位姿执行：

```text
T_local_base = T_local_source * T_source_base
```

速度也会按外参旋转部分转换。对于小白开发者，最稳妥的做法是先让适配节点直接发布 `base_link` 位姿，如果做不到，再在这里填外参。

### 终端打印节点：localization_fusion_monitor_node

`localization_fusion_monitor_node` 是终端打印工具，不参与定位计算。

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `agent_name` | `uav` | 智能体名前缀。 |
| `agent_num` | `1` | 监控无人机数量；多机模式会订阅 `uav1` 到 `uavN`。 |
| `source_id` | `3` | 用于解析并显示当前定位源配置。 |
| `config_yamlfile_path` | `localization_sources.yaml` | 定位源配置文件。 |
| `print_hz` | `5.0` | 终端刷新频率。 |
| `stale_timeout` | `1.0` | 超过该时间未收到 `OdomState` 时标记为陈旧。 |

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

### RViz 节点：rviz_visualization_localization_fusion_node

`rviz_visualization_localization_fusion_node` 只依赖 `OdomState`，不额外订阅 `local_odom` 或 `global_odom`。

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `agent_name` | `uav` | 智能体名前缀。 |
| `agent_id` | `1` | 智能体编号。 |
| `frame_id` | `world` | Marker 使用的参考坐标系。 |
| `state_topic` | `/uav1/sunray/localization/odom_state` | `OdomState` 输入话题。 |
| `marker_topic` | `/uav1/sunray/localization/rviz_markers` | `MarkerArray` 输出话题。 |
| `world_axis_length` | 代码默认值 | world 坐标轴长度。 |
| `tf_axis_length` | 代码默认值 | TF 坐标轴长度。 |
| `trail_length` | 代码默认值 | local/global odom 轨迹保留长度。 |

在 RViz 中添加 `MarkerArray`，话题选择：

```text
/uav1/sunray/localization/rviz_markers
```

### MAVROS vision_pose 桥接

如果希望 PX4/MAVROS 使用 Sunray 的外部定位，可以打开：

```bash
roslaunch localization_fusion localization_fusion.launch \
  source_id:=1 \
  agent_name:=uav \
  agent_id:=1 \
  enable_vision_pose:=true
```

默认桥接关系：

```text
/uav1/sunray/localization/local_odom
  -> /uav1/mavros/vision_pose/pose
```

注意这个节点只做消息类型和话题桥接，PX4 侧的 EKF2 外部视觉参数、MAVROS 坐标系约定、时间同步仍需要单独配置和验证。

### 二次开发入口

常见修改位置：

| 需求 | 修改位置 |
| --- | --- |
| 新增一个定位源 | `config/localization_sources.yaml`，必要时加 `source_launch/*.launch` 和主 launch 中的 include。 |
| 调整输入 odom 话题 | `localization_sources.yaml` 的 `odometry_topic`。 |
| 调整超时判定 | `localization_sources.yaml` 的 `timeout_s`。 |
| 调整传感器到机体中心外参 | `localization_sources.yaml` 的 `source_frame_to_base`。 |
| 加更多状态字段 | `sunray_msgs/OdomState.msg` 和 `localization_fusion.cpp` 的状态填充逻辑。 |
| 修改终端显示 | `src/tools/localization_fusion_monitor_node.cpp`。 |
| 修改 RViz 轨迹或文字 | `src/tools/rviz_visualization_localization_fusion_node.cpp`。 |

二次开发时不要直接让控制器订阅 `/Odometry`、`/vins_estimator/imu_propagate` 或动捕原始话题。推荐先接入 `localization_fusion`，再由控制器订阅 `/uav1/sunray/localization/local_odom` 或 `global_odom`。

</section>
