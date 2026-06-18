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
                            |
       +--------------------+--------------------+
       |                                         |
       | UAV 链路                                | UGV 链路
       |                                         |
 ~/mavros/setpoint_raw/local                     | ~/sunray/ugv_control/cmd_vel
 ~/mavros/setpoint_raw/attitude                  |          |
       |                                         |          v
       v                                         |  +----------------+
+----------------+                               |  |  UgvSimulator  |
| Px4ControlSim  | <----- ~/mavros/state         |  +----------------+
+----------------+                               |     |          |
       |                                         |     |          +--> ~/sunray_sim/imu
       v                                         |     +-------------> ~/sunray_sim/odom
~/sunray_sim/cmd_RPM                      |
       |                                         |
       v                                         |
+--------------------+                           |
| QuadrotorSimulator |---------------------------+
+--------------------+
   |       |       |
   |       |       +--> ~/sunray_sim/navsat
   |       +----------> ~/sunray_sim/imu ------------+
   +------------------> ~/sunray_sim/odom ----------+|
                                                            ||
                                                            vv
                                             +------------------+
                                             | FakeMavrosBridge |
                                             +------------------+
                                                |       |       |
                                                |       |       +--> ~/mavros/state
                                                |       +----------> ~/mavros/imu/data
                                                +------------------> ~/mavros/local_position/odom

~/sunray_sim/odom
~/sunray_sim/cloud_world_frame
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

配置文件按模块拆分，放在：

```text
/home/amov/Sunray_v2/sunray_sim/config
```

| 配置文件 | 对应模块 | 说明 |
| --- | --- | --- |
| `sunray_sim_node.yaml` | `sunray_sim_node` | 配置 `agent_name`、`agent_ids`、`global_frame_id` 和统一状态打印参数。 |
| `single_uav_simulator.yaml` | `SingleUavSimulator`、`SingleUgvSimulator` | 配置每架无人机/无人车的初始位置和初始 yaw。 |
| `global_map_server.yaml` | `GlobalMapServer` | 配置 PCD 地图、全局点云话题、降采样和地图偏移。 |
| `local_mid360_simulator.yaml` | `LocalMid360Simulator` | 配置 MID360 局部点云的量程、视场、角分辨率和发布频率。 |
| `quadrotor_simulator.yaml` | `QuadrotorSimulator` | 配置动力学更新频率、机体 frame、刚体参数和电机参数。 |
| `imu_model.yaml` | `ImuModel` | 配置 IMU 噪声、bias、协方差、安装误差和时间异步。 |
| `px4_control_sim.yaml` | `Px4ControlSim` | 配置 PX4 setpoint 转电机 RPM 的 PID、前馈、限幅和更新频率。 |
| `fake_mavros_bridge.yaml` | `FakeMavrosBridge` | 配置 fake MAVROS 发布频率、初始状态和电池状态。 |
| `ugv_simulator.yaml` | `UgvSimulator` | 配置无人车底盘类型、速度/加速度限幅、cmd_vel 超时、odom 协方差和发布频率。 |
| `sim_visualizer.yaml` | `SimVisualizer` | 配置 RViz MarkerArray 可视化开关、频率、UAV/UGV 显示缩放、轨迹和文字显示。 |

launch 中每份配置都用同一种方式加载：

```xml
<rosparam file="$(arg node_config)" command="load" subst_value="true" />
<rosparam file="$(arg single_uav_simulator_config)" command="load" subst_value="true" />
<rosparam file="$(arg global_map_server_config)" command="load" subst_value="true" />
<rosparam file="$(arg local_mid360_simulator_config)" command="load" subst_value="true" />
<rosparam file="$(arg quadrotor_simulator_config)" command="load" subst_value="true" />
<rosparam file="$(arg imu_model_config)" command="load" subst_value="true" />
<rosparam file="$(arg px4_control_sim_config)" command="load" subst_value="true" />
<rosparam file="$(arg fake_mavros_bridge_config)" command="load" subst_value="true" />
<rosparam file="$(arg ugv_simulator_config)" command="load" subst_value="true" />
<rosparam file="$(arg sim_visualizer_config)" command="load" subst_value="true" />
```

`subst_value="true"` 允许 YAML 中使用 `$(find sunray_sim)`。

### Launch 参数

| launch 参数 | 默认配置文件 |
| --- | --- |
| `node_config` | `$(find sunray_sim)/config/sunray_sim_node.yaml` |
| `single_uav_simulator_config` | `$(find sunray_sim)/config/single_uav_simulator.yaml` |
| `global_map_server_config` | `$(find sunray_sim)/config/global_map_server.yaml` |
| `local_mid360_simulator_config` | `$(find sunray_sim)/config/local_mid360_simulator.yaml` |
| `quadrotor_simulator_config` | `$(find sunray_sim)/config/quadrotor_simulator.yaml` |
| `imu_model_config` | `$(find sunray_sim)/config/imu_model.yaml` |
| `px4_control_sim_config` | `$(find sunray_sim)/config/px4_control_sim.yaml` |
| `fake_mavros_bridge_config` | `$(find sunray_sim)/config/fake_mavros_bridge.yaml` |
| `ugv_simulator_config` | `$(find sunray_sim)/config/ugv_simulator.yaml` |
| `sim_visualizer_config` | `$(find sunray_sim)/config/sim_visualizer.yaml` |
| `rviz` | `true`，默认启动 RViz；无图形环境或一键脚本中可设置为 `false`。 |
| `rviz_config` | `$(find sunray_sim)/rviz/sunray_sim.rviz` |

示例：使用另一张地图配置启动：

```bash
roslaunch sunray_sim sunray_sim.launch \
  global_map_server_config:=/home/amov/Sunray_v2/sunray_sim/config/global_map_server.yaml
```

示例：双机配置只需要改 YAML，不需要新增 launch 文件。

`sunray_sim_node.yaml`：

```yaml
uav:
  enable: true
  agent_name: uav
  agent_ids: [1, 2]

ugv:
  enable: false
  agent_name: ugv
  agent_ids: [1]
```

`single_uav_simulator.yaml`：

```yaml
vehicles:
  uav1:
    init_x: 0.0
    init_y: 0.0
    init_z: 0.0
    init_yaw: 0.0

  uav2:
    init_x: 2.0
    init_y: 0.0
    init_z: 0.0
    init_yaw: 0.0
```

### 模块介绍

#### sunray_sim_node

模块功能：

创建唯一 ROS node，读取 `uav/agent_ids` 和 `ugv/agent_ids` 编号列表，创建一个环境级 `GlobalMapServer`，并为每个 UAV 创建 `SingleUavSimulator`，为每个 UGV 创建 `SingleUgvSimulator`，再按需创建 `SimVisualizer`。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `uav/enable` | `sunray_sim_node.yaml` | 是否启动 UAV 仿真。 |
| `uav/agent_name` | `sunray_sim_node.yaml` | UAV 名称前缀，例如 `uav`。 |
| `uav/agent_ids` | `sunray_sim_node.yaml` | UAV 编号列表，例如 `[1]` 或 `[1, 2]`。 |
| `ugv/enable` | `sunray_sim_node.yaml` | 是否启动 UGV 仿真。 |
| `ugv/agent_name` | `sunray_sim_node.yaml` | UGV 名称前缀，例如 `ugv`。 |
| `ugv/agent_ids` | `sunray_sim_node.yaml` | UGV 编号列表，例如 `[1]` 或 `[1, 2]`。 |
| `agent_name`、`agent_ids` | `sunray_sim_node.yaml` | 兼容旧 UAV 配置的顶层参数，新配置优先使用 `uav/*`。 |
| `global_frame_id` | `sunray_sim_node.yaml` | 全包统一全局坐标系 frame，默认 `map`。全局地图、局部点云、TF、odom、navsat 都使用它。 |
| `enable_sensing` | `sunray_sim_node.yaml` | 是否创建全局地图和局部 MID360 模块。 |
| `enable_status_print` | `sunray_sim_node.yaml` | 是否由主节点统一周期打印所有关键模块状态。 |
| `status_print_hz` | `sunray_sim_node.yaml` | 统一状态打印频率，单位 Hz。 |

订阅话题：

无。

发布话题：

无。该模块本身只负责创建和调度其他模块。

#### SingleUavSimulator

模块功能：

单架无人机的组合层，按当前无人机名称读取初始位姿，创建并持有 `LocalMid360Simulator`、`QuadrotorSimulator`、`Px4ControlSim` 和 `FakeMavrosBridge`。它不直接实现动力学或传感器算法。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `vehicles/<agent>/init_x/y/z` | `single_uav_simulator.yaml` | 指定无人机初始位置，单位 m，例如 `vehicles/uav1/init_x`。 |
| `vehicles/<agent>/init_yaw` | `single_uav_simulator.yaml` | 指定无人机初始偏航角，单位 rad，例如 `vehicles/uav1/init_yaw`。 |

订阅话题：

无。

发布话题：

无。该模块本身只负责创建和持有单架无人机内部模块。

#### SingleUgvSimulator

模块功能：

单辆无人车的组合层，按当前无人车名称读取初始位姿，创建并持有 `LocalMid360Simulator` 和 `UgvSimulator`。它不直接实现动力学或传感器算法。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `vehicles/<agent>/init_x/y` | `single_uav_simulator.yaml` | 指定无人车初始平面位置，单位 m，例如 `vehicles/ugv1/init_x`。 |
| `vehicles/<agent>/init_yaw` | `single_uav_simulator.yaml` | 指定无人车初始偏航角，单位 rad，例如 `vehicles/ugv1/init_yaw`。 |

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
| `map_name` | `global_map_server.yaml` | PCD 地图路径。 |
| `global_map_topic` | `global_map_server.yaml` | 全局点云发布话题。默认 `/sunray_sim/global_cloud`，这是环境级公共话题，不带无人机前缀。 |
| `global_frame_id` | `sunray_sim_node.yaml` | 全局点云 frame_id。为兼容旧配置，代码仍可读取 `map_frame`，但默认配置统一使用 `global_frame_id`。 |
| `add_boundary` | `global_map_server.yaml` | 是否添加地图外包围盒边界点。 |
| `downsample_res` | `global_map_server.yaml` | 体素降采样分辨率，单位 m。 |
| `map_offset_x/y/z` | `global_map_server.yaml` | 地图整体平移，单位 m。 |
| `map_publish_rate` | `global_map_server.yaml` | 全局地图发布频率，单位 Hz。 |

订阅话题：

无。

发布话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/sunray_sim/global_cloud` | `sensor_msgs/PointCloud2` | 全局 PCD 点云，话题名可由 `global_map_topic` 修改。 |

#### LocalMid360Simulator

模块功能：

根据全局地图、当前 odom 和传感器安装外参，模拟 MID360 风格局部点云，输出全局坐标系点云和 sensor 坐标系点云。odom 表示机体位姿，`sensor_offset_*` 和 `sensor_roll/pitch/yaw` 表示雷达相对机体的固定外参。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `global_frame_id` | `sunray_sim_node.yaml` | `~/sunray_sim/cloud_world_frame` 的 frame_id，也是 TF 父坐标系。默认 `map`。 |
| `lidar_type` | `local_mid360_simulator.yaml` | 雷达类型名称，用于日志显示。 |
| `sensor_frame_id` | `local_mid360_simulator.yaml` | 传感器坐标系 frame_id。留空时自动使用 `<agent_name><agent_id>/sensor`，例如 `uav1/sensor`。 |
| `sensor_offset_x/y/z` | `local_mid360_simulator.yaml` | 雷达坐标系原点相对机体 odom 坐标系的平移外参，单位 m，默认全 0。 |
| `sensor_roll/pitch/yaw` | `local_mid360_simulator.yaml` | 雷达坐标系相对机体 odom 坐标系的姿态外参，单位 deg，默认全 0。 |
| `is_360lidar` | `local_mid360_simulator.yaml` | 是否按 360 度水平视场输出。 |
| `sensing_horizon` | `local_mid360_simulator.yaml` | 最大感知距离，单位 m。 |
| `sensing_rate` | `local_mid360_simulator.yaml` | 局部点云发布频率，单位 Hz。 |
| `polar_resolution` | `local_mid360_simulator.yaml` | 极坐标角分辨率，单位 deg。 |
| `yaw_fov` | `local_mid360_simulator.yaml` | 水平视场角，单位 deg。 |
| `vertical_fov` | `local_mid360_simulator.yaml` | 垂直视场角，单位 deg。 |
| `min_raylength` | `local_mid360_simulator.yaml` | 最小有效量测距离，单位 m。 |

外参作用方式：

```text
sensor_pos_world = body_pos_world + body_rot_world * sensor_offset_body
sensor_rot_world = body_rot_world * sensor_rot_body
```

局部点云视场裁剪、`cloud_sensor_frame`、`cloud_world_frame` 和 TF 都使用外参后的真实雷达位姿。因此如果配置雷达俯仰或横滚安装角，模拟点云会按倾斜后的雷达坐标系生成。

订阅话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_sim/odom` | `nav_msgs/Odometry` | 当前 UAV/UGV 位姿。 |

发布话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_sim/cloud_world_frame` | `sensor_msgs/PointCloud2` | 全局坐标系局部点云，frame_id 由 `global_frame_id` 决定。 |
| `~/sunray_sim/cloud_sensor_frame` | `sensor_msgs/PointCloud2` | sensor 坐标系局部点云。 |
| `/tf` | `tf2_msgs/TFMessage` | `global_frame_id -> sensor_frame_id` 变换，默认 `map -> uav1/sensor`。 |

#### QuadrotorSimulator

模块功能：

订阅电机期望转速，积分四旋翼动力学，输出 odom、IMU 和简化 GNSS 数据。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `global_frame_id` | `sunray_sim_node.yaml` | odom 和 navsat 的 `header.frame_id`，默认 `map`。 |
| `base_frame_id` | `quadrotor_simulator.yaml` | odom 的 `child_frame_id` 和 IMU 的 `header.frame_id`。留空时自动使用 `<agent_name><agent_id>/base_link`，例如 `uav1/base_link`。 |
| `dynamics_update_rate` | `quadrotor_simulator.yaml` | 动力学积分频率，单位 Hz。 |
| `cmd_timeout` | `quadrotor_simulator.yaml` | 电机指令超时时间，单位 s。 |
| `dynamics/mass` | `quadrotor_simulator.yaml` | 机体质量，单位 kg。 |
| `dynamics/gravity` | `quadrotor_simulator.yaml` | 重力加速度，单位 m/s^2。 |
| `dynamics/arm_length` | `quadrotor_simulator.yaml` | 电机臂长，单位 m。 |
| `dynamics/Ixx/Iyy/Izz` | `quadrotor_simulator.yaml` | 三轴转动惯量，单位 kg*m^2。 |
| `motor/k_F` | `quadrotor_simulator.yaml` | 电机推力系数，推力 = `k_F * rpm^2`。 |
| `motor/k_T` | `quadrotor_simulator.yaml` | 电机反扭矩系数，反扭矩 = `k_T * rpm^2`。 |
| `motor/tau_up` | `quadrotor_simulator.yaml` | 电机加速响应时间常数，单位 s。 |
| `motor/tau_down` | `quadrotor_simulator.yaml` | 电机减速响应时间常数，单位 s。 |
| `motor/rpm_min` | `quadrotor_simulator.yaml` | 电机最小转速，单位 rpm。 |
| `motor/rpm_max` | `quadrotor_simulator.yaml` | 电机最大转速，单位 rpm。 |

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

作为 `QuadrotorSimulator` 和 `UgvSimulator` 的内部组件，对真实运动状态加入 IMU 噪声、bias、比例因子误差、安装误差、杆臂误差和时间异步。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `imu/enable` | `imu_model.yaml` | 是否启用 IMU 误差模型。 |
| `imu/random_seed` | `imu_model.yaml` | 随机种子。 |
| `imu/preset` | `imu_model.yaml` | IMU 预设：`ideal`、`vn100`、`mti680g`、`adis16470`、`gq7`、`custom`。 |
| `imu/orientation_covariance` | `imu_model.yaml` | 姿态协方差对角线。 |
| `imu/angular_velocity_covariance` | `imu_model.yaml` | 角速度协方差对角线。 |
| `imu/linear_acceleration_covariance` | `imu_model.yaml` | 线加速度协方差对角线。 |
| `imu/gyro_*` | `imu_model.yaml` | 陀螺 bias、随机游走、比例因子、安装误差和时间相关误差。 |
| `imu/accel_*` | `imu_model.yaml` | 加速度计 bias、随机游走、比例因子、安装误差、二次项和杆臂。 |
| `imu/gyro_accel_time_async_ms` | `imu_model.yaml` | 陀螺和加速度计采样时间异步，单位 ms。 |

订阅话题：

无。该模块由 `QuadrotorSimulator` 或 `UgvSimulator` 直接调用。

发布话题：

无。IMU 消息由 `QuadrotorSimulator` 或 `UgvSimulator` 发布到 `~/sunray_sim/imu`。

#### UgvSimulator

模块功能：

订阅 UGV 控制模块输出的 `cmd_vel`，按差速轮或麦克纳姆轮模型积分平面运动，输出 odom 和 IMU。该模块不模拟 MAVROS，UGV 控制链路直接使用 `cmd_vel`。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `global_frame_id` | `sunray_sim_node.yaml` | odom 的 `header.frame_id`，默认 `map`。 |
| `ugv/publish_rate` | `ugv_simulator.yaml` | 动力学积分和 odom/imu 发布频率，单位 Hz。 |
| `ugv/drive_type` | `ugv_simulator.yaml` | 底盘类型：`differential` 或 `mecanum`。 |
| `ugv/cmd_timeout` | `ugv_simulator.yaml` | cmd_vel 超时时间，单位 s。 |
| `ugv/max_linear_x/y` | `ugv_simulator.yaml` | 车体系 x/y 最大速度，单位 m/s。差速轮会忽略 y 速度。 |
| `ugv/max_angular_z` | `ugv_simulator.yaml` | 最大 yaw 角速度，单位 rad/s。 |
| `ugv/linear_acc_limit` | `ugv_simulator.yaml` | 线速度变化率限制，单位 m/s^2。 |
| `ugv/angular_acc_limit` | `ugv_simulator.yaml` | yaw 角速度变化率限制，单位 rad/s^2。 |
| `ugv/odom_covariance_xy` | `ugv_simulator.yaml` | odom 中 x/y 协方差对角线。 |
| `ugv/odom_covariance_yaw` | `ugv_simulator.yaml` | odom 中 yaw 协方差对角线。 |

订阅话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray/ugv_control/cmd_vel` | `geometry_msgs/Twist` | UGV 控制模块输出的速度指令。 |

发布话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_sim/odom` | `nav_msgs/Odometry` | 仿真无人车里程计。 |
| `~/sunray_sim/imu` | `sensor_msgs/Imu` | 仿真 IMU。 |

#### Px4ControlSim

模块功能：

订阅 MAVROS setpoint、MAVROS state 和当前 odom，把位置、速度、姿态或角速度指令转换成四电机期望 RPM。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `px4_update_rate` | `px4_control_sim.yaml` | `Px4ControlSim` 内部 update 定时器频率，单位 Hz。 |
| `setpoint_timeout` | `px4_control_sim.yaml` | 支持的 MAVROS setpoint 超时时间，超时后输出一次零电机 RPM；设置为 0 表示关闭检查。 |
| `position_control/*` | `px4_control_sim.yaml` | 位置环比例增益和 xy 速度前馈增益。 |
| `velocity_control/*` | `px4_control_sim.yaml` | 速度环比例/积分增益、积分限幅、加速度滤波和前馈。 |
| `attitude_control/*` | `px4_control_sim.yaml` | roll、pitch、yaw 姿态比例增益。 |
| `bodyrate_control/*` | `px4_control_sim.yaml` | roll、pitch、yaw 角速度比例/积分增益和积分限幅。 |
| `limits/*` | `px4_control_sim.yaml` | 速度、加速度、倾角和角速度限幅。 |
| `dynamics/mass`、`motor/*` | `quadrotor_simulator.yaml` | 混控和 RPM 计算需要的动力学/电机参数。 |

订阅话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | 位置、速度、加速度、yaw、yaw_rate setpoint。 |
| `~/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | 姿态、角速度、推力 setpoint。 |
| `~/sunray_sim/odom` | `nav_msgs/Odometry` | 当前无人机状态反馈。 |
| `~/mavros/state` | `mavros_msgs/State` | fake MAVROS 发布的 armed/mode 状态。 |

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

#### FakeMavrosBridge

模块功能：

把本包内部 odom/imu/navsat 转成 MAVROS 常用输出，并提供常见 MAVROS 服务，使上层控制程序可以按 MAVROS 接口运行。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `fake_mavros_bridge/mavros_publish_rate` | `fake_mavros_bridge.yaml` | fake MAVROS 状态话题发布频率，单位 Hz。 |
| `fake_mavros_bridge/on_ground_height_threshold_m` | `fake_mavros_bridge.yaml` | 判断在地面的高度阈值，单位 m。 |
| `fake_mavros_bridge/on_ground_velocity_threshold_mps` | `fake_mavros_bridge.yaml` | 判断在地面的速度阈值，单位 m/s。 |
| `fake_mavros_bridge/battery_voltage_v` | `fake_mavros_bridge.yaml` | 假电池电压，单位 V。 |
| `fake_mavros_bridge/battery_current_a` | `fake_mavros_bridge.yaml` | 假电池电流，单位 A。 |
| `fake_mavros_bridge/battery_remaining` | `fake_mavros_bridge.yaml` | 假剩余电量，范围 0.0 到 1.0。 |
| `fake_mavros_bridge/system_load_raw` | `fake_mavros_bridge.yaml` | 假系统负载原始值。 |
| `fake_mavros_bridge/start_connected` | `fake_mavros_bridge.yaml` | 启动时 connected 状态。 |
| `fake_mavros_bridge/start_armed` | `fake_mavros_bridge.yaml` | 启动时 armed 状态。 |
| `fake_mavros_bridge/start_manual_input` | `fake_mavros_bridge.yaml` | 启动时 manual_input 状态。 |
| `fake_mavros_bridge/start_mode` | `fake_mavros_bridge.yaml` | 启动时飞行模式字符串。 |

订阅话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_sim/odom` | `nav_msgs/Odometry` | 内部仿真 odom。 |
| `~/sunray_sim/imu` | `sensor_msgs/Imu` | 内部仿真 IMU。 |
| `~/sunray_sim/navsat` | `sensor_msgs/NavSatFix` | 内部仿真 GNSS。 |
| `~/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | MAVROS local setpoint，回显为 target_local。 |
| `~/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | MAVROS attitude setpoint，回显为 target_attitude。 |

发布话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
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
| `visualizer/enable` | `sim_visualizer.yaml` | 是否创建可视化模块。 |
| `visualizer/publish_rate` | `sim_visualizer.yaml` | MarkerArray 发布频率，单位 Hz。 |
| `visualizer/marker_topic` | `sim_visualizer.yaml` | 可视化话题。留空时使用 `~/sunray_sim/visualization`。 |
| `visualizer/marker_scale` | `sim_visualizer.yaml` | UAV/UGV marker 显示缩放，不影响动力学。 |
| `visualizer/path_max_points` | `sim_visualizer.yaml` | 轨迹最大保留点数。 |
| `visualizer/path_min_interval` | `sim_visualizer.yaml` | 新增轨迹点的最小位移间隔，单位 m。 |
| `visualizer/show_velocity_arrow` | `sim_visualizer.yaml` | 是否显示速度箭头。 |
| `visualizer/show_status_text` | `sim_visualizer.yaml` | 是否显示头顶状态文字。 |

订阅话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_sim/odom` | `nav_msgs/Odometry` | UAV/UGV 位姿、速度和 frame。 |
| `~/sunray_sim/cmd_RPM` | `std_msgs/Float32MultiArray` | UAV 专用，用于状态面板显示平均电机转速。UGV 模式不依赖该话题。 |
| `~/mavros/state` | `mavros_msgs/State` | UAV 专用，用于状态面板显示 fake MAVROS 是否正常。UGV 模式不依赖该话题。 |
| `~/sunray_sim/cloud_world_frame` | `sensor_msgs/PointCloud2` | 用于状态面板显示最近局部点云点数。 |

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

该脚本会自动启动 `sunray_sim.launch`，并临时生成一份单机 `agent_ids: [1]` 和 `vehicles/uav1/init_*` 配置，持续发布一个 `z=1.0 m` 的 MAVROS `PositionTarget`，然后检查：

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
