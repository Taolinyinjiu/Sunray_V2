<!-- title: sunray_sim 一体化仿真包 -->

<section id="simulation-sunray-sim">

## sunray_sim 一体化仿真包

`/home/amov/Sunray_v2/sunray_sim` 是一个轻量化、纯数值、借助 RViz 可视化的单节点 ROS 仿真包。它在一个 node 中完成 PCD 全局地图发布、MID360 风格局部点云仿真、UAV MAVROS setpoint 到电机转速的控制转换、四旋翼动力学、UGV 平面动力学、IMU、fake MAVROS 输出和 RViz 可视化。

启动入口只有一个：

```bash
cd /home/amov/Sunray_v2
source devel/setup.bash
roslaunch sunray_sim sunray_sim.launch
```

默认配置启用 UAV：`uav.agent_name=uav`、`uav.agent_ids=[1]`，程序运行时会组合出实际无人机命名空间 `/uav1`。如果启用 UGV：`ugv.enable=true`、`ugv.agent_name=ugv`、`ugv.agent_ids=[1]`，同一个 node 会创建 `/ugv1` 的无人车级输出。下面文档中的 `~` 表示“带智能体前缀的相对话题”，例如内部仿真话题统一写成：

```text
~/sunray_sim/...
```

fake MAVROS 话题统一使用：

```text
~/mavros/...
```

命名语义：全局地图是环境级公共输出，默认发布到 `/sunray_sim/global_cloud`，不带智能体前缀；odom、IMU、局部点云和可视化是智能体级输出，使用 `~` 表示带 `/uav1` 或 `/ugv1` 这类前缀的相对话题。fake MAVROS 只给 UAV 链路使用。

### 模块关系图

```text
                             PCD 地图文件
                                  |
                                  v
                         +-----------------+
                         | GlobalMapServer |
                         +-----------------+
                                  |
                                  +--> /sunray_sim/global_cloud
                                  |
                                  v
                         +----------------------+
                         | LocalMid360Simulator |
                         +----------------------+
                            ^       |       |
                            |       |       +--> ~/sunray_sim/cloud_sensor_frame
                            |       +----------> ~/sunray_sim/cloud_world_frame
                            |       +----------> ~/sunray_sim/collision
                            |
       +--------------------+--------------------+
       |                                         |
       | UAV 链路                                | UGV 链路
       |                                         |
 ~/mavros/setpoint_raw/local                     | ~/sunray/ugv_control/cmd_vel
 ~/mavros/setpoint_raw/attitude                  |          |
       |                                         |          v
       v                                         |  +----------+
+----------------+                               |  | UgvPlant |
| Px4MavrosSim  |                               |  +----------+
+----------------+                               |     |       |
       |       ^                                 |     |       +--> ~/sunray_sim/imu
       |       |                                 |     +----------> ~/sunray_sim/odom
       |       +------ ~/sunray_sim/odom
       |       +------ ~/sunray_sim/imu
       |       +------ ~/sunray_sim/navsat
       |
       +--> ~/sunray_sim/cmd_RPM
       +--> ~/mavros/state
       +--> ~/mavros/local_position/odom
       +--> ~/mavros/imu/data
       |
       v
+----------+
| UavPlant |
+----------+
   |       |       |
   |       |       +--> ~/sunray_sim/navsat
   |       +----------> ~/sunray_sim/imu
   +------------------> ~/sunray_sim/odom

~/sunray_sim/odom
~/sunray_sim/cloud_world_frame
~/sunray_sim/collision
UAV 额外订阅：~/sunray_sim/cmd_RPM、~/mavros/state
       |
       v
+---------------+
| SimVisualizer |
+---------------+
       |
       +--> ~/sunray_sim/visualization
```

### 配置文件

配置目录：

```text
/home/amov/Sunray_v2/sunray_sim/config
```

配置分成两层：

```text
config/
├── uav_single.yaml       # 单 UAV，默认场景
├── uav_swarm6.yaml       # 6 机 UAV 集群
├── ugv_single.yaml       # 单 UGV
├── ugv_swarm6.yaml       # 6 车 UGV 集群
├── uav_ugv_swarm6.yaml   # 6 机 + 6 车混合集群
└── modules/              # 高级模块参数
    ├── global_map_server.yaml
    ├── local_mid360_simulator.yaml
    ├── uav_plant.yaml
    ├── imu_model.yaml
    ├── px4_mavros_sim.yaml
    ├── ugv_plant.yaml
    └── sim_visualizer.yaml
```

普通使用只需要选择一个场景文件：

| 场景文件 | 用途 | 启动内容 |
| --- | --- | --- |
| `uav_single.yaml` | 默认单无人机 | `/uav1`，全局地图，局部 MID360，四旋翼动力学，IMU，fake MAVROS，RViz marker。 |
| `uav_swarm6.yaml` | 6 机无人机集群 | `/uav1` 到 `/uav6`，每架 UAV 各自有 odom、IMU、局部点云、fake MAVROS 和 marker。 |
| `ugv_single.yaml` | 单无人车 | `/ugv1`，全局地图，局部 MID360，UGV 平面动力学，IMU，RViz marker。 |
| `ugv_swarm6.yaml` | 6 车无人车集群 | `/ugv1` 到 `/ugv6`，每辆 UGV 各自有 odom、IMU、局部点云和 marker。 |
| `uav_ugv_swarm6.yaml` | 6 机 + 6 车混合集群 | 同时创建 `/uav1` 到 `/uav6` 和 `/ugv1` 到 `/ugv6`。默认关闭感知链路，优先验证动力学、定位、控制和 marker 显示。 |

常用启动命令：

```bash
# 单 UAV，默认场景
roslaunch sunray_sim sunray_sim.launch

# 6 机 UAV 集群
roslaunch sunray_sim sunray_sim.launch scenario_config_file:=uav_swarm6.yaml

# 单 UGV
roslaunch sunray_sim sunray_sim.launch scenario_config_file:=ugv_single.yaml

# 6 车 UGV 集群
roslaunch sunray_sim sunray_sim.launch scenario_config_file:=ugv_swarm6.yaml

# 6 机 + 6 车混合集群
roslaunch sunray_sim sunray_sim.launch scenario_config_file:=uav_ugv_swarm6.yaml
```

场景文件里主要改这些内容：

| 参数 | 说明 |
| --- | --- |
| `uav/enable` | 是否启动 UAV 仿真。 |
| `uav/agent_name` | UAV 名称前缀，例如 `uav`。 |
| `uav/agent_ids` | UAV 编号列表，例如 `[1]` 或 `[1, 2, 3, 4, 5, 6]`。 |
| `ugv/enable` | 是否启动 UGV 仿真。 |
| `ugv/agent_name` | UGV 名称前缀，例如 `ugv`。 |
| `ugv/agent_ids` | UGV 编号列表，例如 `[1]` 或 `[1, 2, 3, 4, 5, 6]`。 |
| `global_frame_id` | 全包统一全局坐标系 frame，默认 `map`。全局地图、局部点云、TF、odom、navsat 都使用它。 |
| `enable_sensing` | 是否创建全局地图和局部 MID360 模块。 |
| `enable_status_print` | 是否由主节点统一周期打印所有关键模块状态。 |
| `status_print_hz` | 统一状态打印频率，单位 Hz。 |
| `vehicles/<agent>/init_x/y/z` | UAV 初始位置，单位 m，例如 `vehicles/uav1/init_x`。 |
| `vehicles/<agent>/init_x/y` | UGV 初始平面位置，单位 m，例如 `vehicles/ugv1/init_x`。 |
| `vehicles/<agent>/init_yaw` | 初始偏航角，单位 rad。 |

高级模块参数放在 `config/modules/` 中，通常只有需要调地图、雷达、动力学或显示效果时才修改：

| 模块配置 | 对应模块 | 说明 |
| --- | --- | --- |
| `modules/global_map_server.yaml` | `GlobalMapServer` | PCD 地图、全局点云话题、降采样和地图偏移。 |
| `modules/local_mid360_simulator.yaml` | `LocalMid360Simulator` | MID360 局部点云量程、视场、角分辨率、发布频率和雷达外参。 |
| `modules/uav_plant.yaml` | `UavPlant` | 四旋翼动力学频率、机体 frame、刚体参数和电机参数。 |
| `modules/imu_model.yaml` | `ImuModel` | IMU 噪声、bias、协方差、安装误差和时间异步。 |
| `modules/px4_mavros_sim.yaml` | `Px4MavrosSim` | MAVROS setpoint 转电机 RPM 的 PID、前馈、限幅、更新频率，以及 fake MAVROS 话题/服务参数。 |
| `modules/ugv_plant.yaml` | `UgvPlant` | UGV 底盘类型、速度/加速度限幅、cmd_vel 超时、odom 协方差和发布频率。 |
| `modules/sim_visualizer.yaml` | `SimVisualizer` | RViz MarkerArray 可视化开关、频率、车辆显示大小、轨迹和文字显示。 |

launch 仍然只有一个入口，主要参数是：

| launch 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `scenario_config_file` | `uav_single.yaml` | 从 `config/` 目录选择一个场景文件。 |
| `scenario_config` | `$(find sunray_sim)/config/$(arg scenario_config_file)` | 场景文件完整路径。一般不需要手动设置。 |
| `rviz` | `true` | 是否自动启动 RViz。无图形环境时可设置为 `false`。 |
| `rviz_config` | `$(find sunray_sim)/rviz/sunray_sim.rviz` | RViz 配置文件。 |

`subst_value="true"` 允许 YAML 中使用 `$(find sunray_sim)`。命令行中建议只传 `scenario_config_file:=xxx.yaml`，不要写 `$(find sunray_sim)`，避免被终端当作 shell 命令替换。

### 包结构

使用者主要关注 `launch/`、`config/`、`resource/`、`rviz/` 和 `test/`：

| 路径 | 用途 |
| --- | --- |
| `launch/sunray_sim.launch` | 唯一启动入口。 |
| `config/*.yaml` | 场景配置，选择单机、集群、UAV、UGV 或混合集群。 |
| `config/modules/*.yaml` | 高级模块参数，调地图、雷达、动力学、MAVROS 仿真接口和可视化。 |
| `resource/*.pcd` | 默认 PCD 地图资源。 |
| `rviz/sunray_sim.rviz` | 默认 RViz 显示配置。 |
| `test/minimal_closed_loop_check.py` | 最小闭环验证脚本。 |

源码按功能分组，CMake 也按同样思路收敛为 4 个内部库：

| 目录 | CMake 内部库 | 内容 |
| --- | --- | --- |
| `src/map_sensing/`、`src/imu/` | `sunray_sim_sensing` | 全局地图、局部 MID360、IMU 模型。 |
| `src/uav/` | `sunray_sim_uav` | UAV 动力学、四旋翼模型、PX4/MAVROS 仿真接口。 |
| `src/ugv/` | `sunray_sim_ugv` | UGV 平面动力学和 UGV 状态输出。 |
| `src/core/` | `sunray_sim_core` | 单 UAV/UGV 组合层和 RViz marker 可视化。 |

### 模块介绍

#### sunray_sim_node

模块功能：

创建唯一 ROS node，读取 `uav/agent_ids` 和 `ugv/agent_ids` 编号列表，创建一个环境级 `GlobalMapServer`，并为每个 UAV 创建 `SingleUavSimulator`，为每个 UGV 创建 `SingleUgvSimulator`，再按需创建 `SimVisualizer`。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `uav/enable` | `uav_single.yaml` 等场景配置 | 是否启动 UAV 仿真。 |
| `uav/agent_name` | `uav_single.yaml` 等场景配置 | UAV 名称前缀，例如 `uav`。 |
| `uav/agent_ids` | `uav_single.yaml` 等场景配置 | UAV 编号列表，例如 `[1]` 或 `[1, 2]`。 |
| `ugv/enable` | `uav_single.yaml` 等场景配置 | 是否启动 UGV 仿真。 |
| `ugv/agent_name` | `uav_single.yaml` 等场景配置 | UGV 名称前缀，例如 `ugv`。 |
| `ugv/agent_ids` | `uav_single.yaml` 等场景配置 | UGV 编号列表，例如 `[1]` 或 `[1, 2]`。 |
| `global_frame_id` | `uav_single.yaml` 等场景配置 | 全包统一全局坐标系 frame，默认 `map`。全局地图、局部点云、TF、odom、navsat 都使用它。 |
| `enable_sensing` | `uav_single.yaml` 等场景配置 | 是否创建全局地图和局部 MID360 模块。 |
| `enable_status_print` | `uav_single.yaml` 等场景配置 | 是否由主节点统一周期打印所有关键模块状态。 |
| `status_print_hz` | `uav_single.yaml` 等场景配置 | 统一状态打印频率，单位 Hz。 |

订阅话题：

无。

发布话题：

无。该模块本身只负责创建和调度其他模块。

#### SingleUavSimulator

模块功能：

单架无人机的组合层，按当前无人机名称读取初始位姿，创建并持有 `LocalMid360Simulator`、`UavPlant` 和 `Px4MavrosSim`。它不直接实现动力学、控制或传感器算法。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `vehicles/<agent>/init_x/y/z` | `uav_single.yaml` 等场景配置 | 指定无人机初始位置，单位 m，例如 `vehicles/uav1/init_x`。 |
| `vehicles/<agent>/init_yaw` | `uav_single.yaml` 等场景配置 | 指定无人机初始偏航角，单位 rad，例如 `vehicles/uav1/init_yaw`。 |

订阅话题：

无。

发布话题：

无。该模块本身只负责创建和持有单架无人机内部模块。

#### SingleUgvSimulator

模块功能：

单辆无人车的组合层，按当前无人车名称读取初始位姿，创建并持有 `LocalMid360Simulator` 和 `UgvPlant`。它不直接实现动力学或传感器算法。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `vehicles/<agent>/init_x/y` | `uav_single.yaml` 等场景配置 | 指定无人车初始平面位置，单位 m，例如 `vehicles/ugv1/init_x`。 |
| `vehicles/<agent>/init_yaw` | `uav_single.yaml` 等场景配置 | 指定无人车初始偏航角，单位 rad，例如 `vehicles/ugv1/init_yaw`。 |

订阅话题：

无。

发布话题：

无。该模块本身只负责创建和持有单辆无人车内部模块。

#### GlobalMapServer

模块功能：

读取 PCD 地图文件，执行地图偏移、边界补点和体素降采样，然后发布全局点云。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `map_name` | `modules/global_map_server.yaml` | PCD 地图路径。 |
| `global_map_topic` | `modules/global_map_server.yaml` | 全局点云发布话题。默认 `/sunray_sim/global_cloud`，这是环境级公共话题，不带无人机前缀。 |
| `global_frame_id` | `uav_single.yaml` 等场景配置 | 全局点云 frame_id。默认读取场景配置中的 `global_frame_id`。 |
| `add_boundary` | `modules/global_map_server.yaml` | 是否添加地图外包围盒边界点。 |
| `downsample_res` | `modules/global_map_server.yaml` | 体素降采样分辨率，单位 m。 |
| `map_offset_x/y/z` | `modules/global_map_server.yaml` | 地图整体平移，单位 m。 |
| `map_publish_rate` | `modules/global_map_server.yaml` | 全局地图发布频率，单位 Hz。 |

订阅话题：

无。

发布话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/sunray_sim/global_cloud` | `sensor_msgs/PointCloud2` | 全局 PCD 点云，话题名可由 `global_map_topic` 修改。 |

#### LocalMid360Simulator

模块功能：

根据全局地图、当前 odom 和传感器安装外参，模拟 MID360 风格局部点云，输出全局坐标系点云和 sensor 坐标系点云。odom 表示机体位姿，`sensor_offset_*` 和 `sensor_roll/pitch/yaw` 表示雷达相对机体的固定外参。本模块同时基于 odom 和全局点云做轻量碰撞检测；`enable_sensing=false` 时局部点云和碰撞检测都会关闭。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `global_frame_id` | `uav_single.yaml` 等场景配置 | `~/sunray_sim/cloud_world_frame` 的 frame_id，也是 TF 父坐标系。默认 `map`。 |
| `lidar_type` | `modules/local_mid360_simulator.yaml` | 雷达类型名称，用于日志显示。 |
| `sensor_frame_id` | `modules/local_mid360_simulator.yaml` | 传感器坐标系 frame_id。留空时自动使用 `<agent_name><agent_id>/sensor`，例如 `uav1/sensor`。 |
| `sensor_offset_x/y/z` | `modules/local_mid360_simulator.yaml` | 雷达坐标系原点相对机体 odom 坐标系的平移外参，单位 m，默认全 0。 |
| `sensor_roll/pitch/yaw` | `modules/local_mid360_simulator.yaml` | 雷达坐标系相对机体 odom 坐标系的姿态外参，单位 deg，默认全 0。 |
| `is_360lidar` | `modules/local_mid360_simulator.yaml` | 是否按 360 度水平视场输出。 |
| `sensing_horizon` | `modules/local_mid360_simulator.yaml` | 最大感知距离，单位 m。 |
| `sensing_rate` | `modules/local_mid360_simulator.yaml` | 局部点云发布频率，单位 Hz。 |
| `polar_resolution` | `modules/local_mid360_simulator.yaml` | 极坐标角分辨率，单位 deg。 |
| `yaw_fov` | `modules/local_mid360_simulator.yaml` | 水平视场角，单位 deg。 |
| `vertical_fov` | `modules/local_mid360_simulator.yaml` | 垂直视场角，单位 deg。 |
| `min_raylength` | `modules/local_mid360_simulator.yaml` | 最小有效量测距离，单位 m。 |
| `collision_check/enable` | `modules/local_mid360_simulator.yaml` | 是否启用碰撞检测。 |
| `collision_check/radius` | `modules/local_mid360_simulator.yaml` | 碰撞半径，单位 m。UAV 当前按 200 mm 轴距显示，默认 0.15 m。 |
| `collision_check/z_filter_enable` | `modules/local_mid360_simulator.yaml` | 是否只检查 odom.z 附近的点，避免飞行时被正下方地面点误判。 |
| `collision_check/z_margin` | `modules/local_mid360_simulator.yaml` | z 方向额外容差，单位 m；实际范围为 `odom.z ± (radius + z_margin)`。 |

外参作用方式：

```text
sensor_pos_world = body_pos_world + body_rot_world * sensor_offset_body
sensor_rot_world = body_rot_world * sensor_rot_body
```

局部点云视场裁剪、`cloud_sensor_frame`、`cloud_world_frame` 和 TF 都使用外参后的真实雷达位姿。因此如果配置雷达俯仰或横滚安装角，模拟点云会按倾斜后的雷达坐标系生成。

碰撞检测方式：

```text
以 odom 位置为中心，在全局点云中搜索 collision_check/radius 半径内的点。
如果启用 z_filter_enable，只保留 z 位于 odom.z ± (radius + z_margin) 内的点。
存在符合条件的点时认为发生碰撞，终端状态面板累计碰撞次数，RViz 显示红色“碰撞”。
```

订阅话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_sim/odom` | `nav_msgs/Odometry` | 当前 UAV/UGV 位姿。 |

发布话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_sim/cloud_world_frame` | `sensor_msgs/PointCloud2` | 全局坐标系局部点云，frame_id 由 `global_frame_id` 决定。 |
| `~/sunray_sim/cloud_sensor_frame` | `sensor_msgs/PointCloud2` | sensor 坐标系局部点云。 |
| `~/sunray_sim/collision` | `std_msgs/Bool` | 碰撞状态。`true` 表示当前 odom 位置半径内检测到地图点。 |
| `/tf` | `tf2_msgs/TFMessage` | `global_frame_id -> sensor_frame_id` 变换，默认 `map -> uav1/sensor`。 |

#### UavPlant

模块功能：

订阅电机期望转速，积分四旋翼动力学，输出 odom、IMU 和简化 GNSS 数据。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `global_frame_id` | `uav_single.yaml` 等场景配置 | odom 和 navsat 的 `header.frame_id`，默认 `map`。 |
| `base_frame_id` | `modules/uav_plant.yaml` | odom 的 `child_frame_id` 和 IMU 的 `header.frame_id`。留空时自动使用 `<agent_name><agent_id>/base_link`，例如 `uav1/base_link`。 |
| `dynamics_update_rate` | `modules/uav_plant.yaml` | 动力学积分频率，单位 Hz。 |
| `cmd_timeout` | `modules/uav_plant.yaml` | 电机指令超时时间，单位 s。 |
| `dynamics/mass` | `modules/uav_plant.yaml` | 机体质量，单位 kg。 |
| `dynamics/gravity` | `modules/uav_plant.yaml` | 重力加速度，单位 m/s^2。 |
| `dynamics/arm_length` | `modules/uav_plant.yaml` | 电机臂长，单位 m。 |
| `dynamics/Ixx/Iyy/Izz` | `modules/uav_plant.yaml` | 三轴转动惯量，单位 kg*m^2。 |
| `motor/k_F` | `modules/uav_plant.yaml` | 电机推力系数，推力 = `k_F * rpm^2`。 |
| `motor/k_T` | `modules/uav_plant.yaml` | 电机反扭矩系数，反扭矩 = `k_T * rpm^2`。 |
| `motor/tau_up` | `modules/uav_plant.yaml` | 电机加速响应时间常数，单位 s。 |
| `motor/tau_down` | `modules/uav_plant.yaml` | 电机减速响应时间常数，单位 s。 |
| `motor/rpm_min` | `modules/uav_plant.yaml` | 电机最小转速，单位 rpm。 |
| `motor/rpm_max` | `modules/uav_plant.yaml` | 电机最大转速，单位 rpm。 |

订阅话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_sim/cmd_RPM` | `std_msgs/Float32MultiArray` | 四个电机期望转速。 |

发布话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_sim/odom` | `nav_msgs/Odometry` | 仿真无人机里程计。 |
| `~/sunray_sim/imu` | `sensor_msgs/Imu` | 仿真 IMU。 |
| `~/sunray_sim/navsat` | `sensor_msgs/NavSatFix` | 简化 GNSS 数据。 |

#### ImuModel

模块功能：

作为 `UavPlant` 和 `UgvPlant` 的内部组件，对真实运动状态加入 IMU 噪声、bias、比例因子误差、安装误差、杆臂误差和时间异步。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `imu/enable` | `modules/imu_model.yaml` | 是否启用 IMU 误差模型。 |
| `imu/random_seed` | `modules/imu_model.yaml` | 随机种子。 |
| `imu/preset` | `modules/imu_model.yaml` | IMU 预设：`ideal`、`vn100`、`mti680g`、`adis16470`、`gq7`、`custom`。 |
| `imu/orientation_covariance` | `modules/imu_model.yaml` | 姿态协方差对角线。 |
| `imu/angular_velocity_covariance` | `modules/imu_model.yaml` | 角速度协方差对角线。 |
| `imu/linear_acceleration_covariance` | `modules/imu_model.yaml` | 线加速度协方差对角线。 |
| `imu/gyro_*` | `modules/imu_model.yaml` | 陀螺 bias、随机游走、比例因子、安装误差和时间相关误差。 |
| `imu/accel_*` | `modules/imu_model.yaml` | 加速度计 bias、随机游走、比例因子、安装误差、二次项和杆臂。 |
| `imu/gyro_accel_time_async_ms` | `modules/imu_model.yaml` | 陀螺和加速度计采样时间异步，单位 ms。 |

订阅话题：

无。该模块由 `UavPlant` 或 `UgvPlant` 直接调用。

发布话题：

无。IMU 消息由 `UavPlant` 或 `UgvPlant` 发布到 `~/sunray_sim/imu`。

#### UgvPlant

模块功能：

订阅 UGV 控制模块输出的 `cmd_vel`，按差速轮或麦克纳姆轮模型积分平面运动，输出 odom 和 IMU。该模块不模拟 MAVROS，UGV 控制链路直接使用 `cmd_vel`。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `global_frame_id` | `uav_single.yaml` 等场景配置 | odom 的 `header.frame_id`，默认 `map`。 |
| `ugv/publish_rate` | `modules/ugv_plant.yaml` | 动力学积分和 odom/imu 发布频率，单位 Hz。 |
| `ugv/odom_height` | `modules/ugv_plant.yaml` | UGV odom 发布的 z 高度，单位 m。默认 `1.0`，仅改变 odom 高度，不改变平面动力学。 |
| `ugv/drive_type` | `modules/ugv_plant.yaml` | 底盘类型：`differential` 或 `mecanum`。 |
| `ugv/cmd_timeout` | `modules/ugv_plant.yaml` | cmd_vel 超时时间，单位 s。 |
| `ugv/max_linear_x/y` | `modules/ugv_plant.yaml` | 车体系 x/y 最大速度，单位 m/s。差速轮会忽略 y 速度。 |
| `ugv/max_angular_z` | `modules/ugv_plant.yaml` | 最大 yaw 角速度，单位 rad/s。 |
| `ugv/linear_acc_limit` | `modules/ugv_plant.yaml` | 线速度变化率限制，单位 m/s^2。 |
| `ugv/angular_acc_limit` | `modules/ugv_plant.yaml` | yaw 角速度变化率限制，单位 rad/s^2。 |
| `ugv/odom_covariance_xy` | `modules/ugv_plant.yaml` | odom 中 x/y 协方差对角线。 |
| `ugv/odom_covariance_yaw` | `modules/ugv_plant.yaml` | odom 中 yaw 协方差对角线。 |

订阅话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray/ugv_control/cmd_vel` | `geometry_msgs/Twist` | UGV 控制模块输出的速度指令。 |

发布话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_sim/odom` | `nav_msgs/Odometry` | 仿真无人车里程计。 |
| `~/sunray_sim/imu` | `sensor_msgs/Imu` | 仿真 IMU。 |

#### Px4MavrosSim

模块功能：

订阅 MAVROS setpoint 和当前无人机状态，把位置、速度、姿态或角速度指令转换成四电机期望 RPM；同时把本包内部 odom、IMU、navsat 转成 MAVROS 常用话题，并提供 set_mode、arming、param/get、param/set 等常见 MAVROS 服务。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `px4_update_rate` | `modules/px4_mavros_sim.yaml` | `Px4MavrosSim` 内部控制定时器频率，单位 Hz。 |
| `setpoint_timeout` | `modules/px4_mavros_sim.yaml` | 支持的 MAVROS setpoint 超时时间，超时后输出一次零电机 RPM；设置为 0 表示关闭检查。 |
| `position_control/*` | `modules/px4_mavros_sim.yaml` | 位置环比例增益和 xy 速度前馈增益。 |
| `velocity_control/*` | `modules/px4_mavros_sim.yaml` | 速度环比例/积分增益、积分限幅、加速度滤波和前馈。 |
| `attitude_control/*` | `modules/px4_mavros_sim.yaml` | roll、pitch、yaw 姿态比例增益。 |
| `bodyrate_control/*` | `modules/px4_mavros_sim.yaml` | roll、pitch、yaw 角速度比例/积分增益和积分限幅。 |
| `limits/*` | `modules/px4_mavros_sim.yaml` | 速度、加速度、倾角和角速度限幅。 |
| `px4_mavros/*` | `modules/px4_mavros_sim.yaml` | fake MAVROS 发布频率、初始 connected/armed/mode 状态、电池状态和 landed_state 判断阈值。 |
| `dynamics/mass`、`motor/*` | `modules/uav_plant.yaml` | 混控和 RPM 计算需要的动力学/电机参数。 |

订阅话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | 位置、速度、加速度、yaw、yaw_rate setpoint。 |
| `~/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | 姿态、角速度、推力 setpoint。 |
| `~/sunray_sim/odom` | `nav_msgs/Odometry` | 当前无人机状态反馈。 |
| `~/sunray_sim/imu` | `sensor_msgs/Imu` | 内部仿真 IMU，转发到 `~/mavros/imu/data`。 |
| `~/sunray_sim/navsat` | `sensor_msgs/NavSatFix` | 内部仿真 GNSS，转发到 `~/mavros/global_position/global` 并生成简化 GPSRAW。 |

支持的 setpoint：

| 类型 | 支持范围 | 不支持情况 |
| --- | --- | --- |
| `PositionTarget` | 仅 `FRAME_LOCAL_NED`；支持 `pos_xy/pos_z`、`vel_xy/vel_z`、`acc_xy/acc_z`、`yaw`、`yaw_rate` 按字段叠加。 | `FORCE` 位、全部字段忽略、只给 x 不给 y、只给 vx 不给 vy、只给 ax 不给 ay、`LOCAL_OFFSET_NED/BODY_NED/BODY_OFFSET_NED`。 |
| `AttitudeTarget` | `body_rate + thrust`、`roll_pitch_yaw + thrust`、`roll_pitch_yawrate + thrust`。 | `IGNORE_THRUST`，以及其他 attitude/body_rate 混合组合。 |

不支持的 setpoint 会被拒绝，不会刷新当前有效 setpoint 时间，也不会改变当前控制模式。相同 `type_mask`/坐标系组合只打印一次告警，状态面板会显示最近一次拒绝原因。

发布话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_sim/cmd_RPM` | `std_msgs/Float32MultiArray` | 四电机期望转速。 |
| `~/mavros/state` | `mavros_msgs/State` | MAVROS 连接、解锁和模式状态。 |
| `~/mavros/extended_state` | `mavros_msgs/ExtendedState` | landed_state 等扩展状态。 |
| `~/mavros/sys_status` | `mavros_msgs/SysStatus` | 电池和系统状态。 |
| `~/mavros/estimator_status` | `mavros_msgs/EstimatorStatus` | EKF 状态。 |
| `~/mavros/local_position/odom` | `nav_msgs/Odometry` | MAVROS 风格本地 odom。 |
| `~/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | 从内部 odom 拆出的 MAVROS 风格本地 pose。 |
| `~/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | 从内部 odom 拆出的 MAVROS 风格本地速度。 |
| `~/mavros/global_position/global` | `sensor_msgs/NavSatFix` | 转发内部 navsat。 |
| `~/mavros/gpsstatus/gps1/raw` | `mavros_msgs/GPSRAW` | 由内部 navsat 生成的简化 GPSRAW，固定 3D fix。 |
| `~/mavros/imu/data` | `sensor_msgs/Imu` | MAVROS 风格 IMU。 |
| `~/mavros/setpoint_raw/target_local` | `mavros_msgs/PositionTarget` | local setpoint 回显。 |
| `~/mavros/setpoint_raw/target_attitude` | `mavros_msgs/AttitudeTarget` | attitude setpoint 回显。 |

服务：

| 服务 | 类型 | 说明 |
| --- | --- | --- |
| `~/mavros/set_mode` | `mavros_msgs/SetMode` | 修改 fake mode。 |
| `~/mavros/cmd/arming` | `mavros_msgs/CommandBool` | 修改 fake armed 状态。 |
| `~/mavros/cmd/command` | `mavros_msgs/CommandLong` | 接受 command_long。 |
| `~/mavros/param/get` | `mavros_msgs/ParamGet` | 读取 fake MAVROS 参数。 |
| `~/mavros/param/set` | `mavros_msgs/ParamSet` | 设置 fake MAVROS 参数。 |

#### SimVisualizer

模块功能：

订阅仿真状态，合成一个 RViz `MarkerArray`。UAV 显示为 X 型四旋翼机架、前绿后红电机盘、机头方向箭头、速度箭头、飞行轨迹和头顶位置文字；UGV 显示为矩形车体、驾驶舱、四个车轮、车头方向箭头、轨迹和头顶位置文字。该模块只负责显示，不参与动力学、控制或点云计算。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `visualizer/enable` | `modules/sim_visualizer.yaml` | 是否创建可视化模块。 |
| `visualizer/publish_rate` | `modules/sim_visualizer.yaml` | MarkerArray 发布频率，单位 Hz。 |
| `visualizer/marker_topic` | `modules/sim_visualizer.yaml` | 可视化话题。留空时使用 `~/sunray_sim/visualization`。 |
| `visualizer/marker_scale` | `modules/sim_visualizer.yaml` | UAV/UGV marker 显示缩放，不影响动力学。UAV 默认按 200 mm 轴距绘制；UGV 默认按 250 mm 车长绘制。 |
| `visualizer/path_max_points` | `modules/sim_visualizer.yaml` | 轨迹最大保留点数。 |
| `visualizer/path_min_interval` | `modules/sim_visualizer.yaml` | 新增轨迹点的最小位移间隔，单位 m。 |
| `visualizer/show_velocity_arrow` | `modules/sim_visualizer.yaml` | 是否显示速度箭头。 |
| `visualizer/show_status_text` | `modules/sim_visualizer.yaml` | 是否显示头顶状态文字。 |

订阅话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_sim/odom` | `nav_msgs/Odometry` | UAV/UGV 位姿、速度和 frame。 |
| `~/sunray_sim/cloud_world_frame` | `sensor_msgs/PointCloud2` | 用于状态面板显示最近局部点云点数。 |
| `~/sunray_sim/collision` | `std_msgs/Bool` | 碰撞状态。为 `true` 时在 RViz 中显示红色“碰撞”。 |
| `~/sunray_sim/cmd_RPM` | `std_msgs/Float32MultiArray` | 仅 UAV 订阅，用于显示平均电机转速。UGV 不订阅该话题。 |
| `~/mavros/state` | `mavros_msgs/State` | 仅 UAV 订阅，用于显示 fake MAVROS 状态。UGV 不订阅该话题。 |

发布话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_sim/visualization` | `visualization_msgs/MarkerArray` | 单一 RViz 可视化话题，包含 UAV/UGV 模型、轨迹和文字。 |

### 运行检查

启动后可以用下面命令检查主要输出。下面的 `~/...` 表示带智能体前缀的话题，实际运行命令时按当前 `uav/agent_ids` 或 `ugv/agent_ids` 展开成对应前缀。

```bash
rostopic list | grep -E "sunray_sim|mavros|global_cloud"
rostopic hz ~/sunray_sim/odom
rostopic hz ~/sunray_sim/imu
rostopic hz ~/sunray_sim/cloud_world_frame
rostopic hz ~/sunray_sim/visualization
rostopic echo ~/mavros/state -n 1
```

UGV 链路检查示例：

```bash
rostopic pub ~/sunray/ugv_control/cmd_vel geometry_msgs/Twist "{
  linear: {x: 0.5, y: 0.0, z: 0.0},
  angular: {x: 0.0, y: 0.0, z: 0.2}
}"
rostopic hz ~/sunray_sim/odom
rostopic echo ~/sunray_sim/odom -n 1
```

启动时默认会打开 RViz。如果不需要图形界面：

```bash
roslaunch sunray_sim sunray_sim.launch rviz:=false
```

运行最小闭环验证脚本：

```bash
rosrun sunray_sim minimal_closed_loop_check.py
```

该脚本会自动启动 `sunray_sim.launch`，并临时生成一份 `uav_single.yaml` 风格的单机临时场景配置，持续发布一个 `z=1.0 m` 的 MAVROS `PositionTarget`，然后检查：

| 检查项 | 默认要求 |
| --- | --- |
| `~/sunray_sim/odom` | 收到 odom，且 z 有明显上升。 |
| `~/sunray_sim/imu` | 收到 IMU。 |
| `~/mavros/state` | 收到 fake MAVROS 状态。 |
| `~/sunray_sim/cloud_world_frame` | 收到局部点云。 |
| `~/sunray_sim/visualization` | 收到 RViz MarkerArray。 |
| `~/sunray_sim/cmd_RPM` | 收到有效电机 RPM 输出。 |

如果已经手动启动了仿真节点，只想跑检查：

```bash
rosrun sunray_sim minimal_closed_loop_check.py --no-launch
```

检查集群中的第二架无人机时，先按双机配置手动启动仿真，再运行：

```bash
rosrun sunray_sim minimal_closed_loop_check.py --no-launch --agent-id 2
```

发布一个简单位置 setpoint：

```bash
rostopic pub ~/mavros/setpoint_raw/local mavros_msgs/PositionTarget "{
  header: {frame_id: 'map'},
  coordinate_frame: 1,
  type_mask: 3576,
  position: {x: 0.0, y: 0.0, z: 1.0},
  velocity: {x: 0.0, y: 0.0, z: 0.0},
  acceleration_or_force: {x: 0.0, y: 0.0, z: 0.0},
  yaw: 0.0,
  yaw_rate: 0.0
}" -r 20
```

</section>
