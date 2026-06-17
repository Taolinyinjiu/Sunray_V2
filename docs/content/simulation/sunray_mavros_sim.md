<!-- title: sunray_mavros_sim 一体化仿真包 -->

<section id="simulation-sunray-mavros-sim">

## sunray_mavros_sim 一体化仿真包

`/home/amov/Sunray_v2/sunray_mavros_sim` 是一个单节点 ROS 仿真包。它在一个 node 中完成 PCD 全局地图发布、MID360 风格局部点云仿真、MAVROS setpoint 到电机转速的控制转换、四旋翼动力学、IMU 和 fake MAVROS 输出。

启动入口只有一个：

```bash
cd /home/amov/Sunray_v2
source devel/setup.bash
roslaunch sunray_mavros_sim sunray_mavros_sim.launch
```

默认配置 `agent_name=uav`、`agent_id=1`，程序运行时会组合出实际无人机命名空间。下面文档中的 `~` 表示“带无人机前缀的相对话题”，例如内部仿真话题统一写成：

```text
~/sunray_mavros_sim/...
```

fake MAVROS 话题统一使用：

```text
~/mavros/...
```

### 模块关系图

```text
                             PCD 地图文件
                                  |
                                  v
                         +-----------------+
                         | GlobalMapServer |
                         +-----------------+
                           |             |
                           |             +-----------------------------+
                           v                                           v
              /map_generator/global_cloud              +----------------------+
                                                       | LocalMid360Simulator |
                                                       +----------------------+
                                                          ^       |       |
                                                          |       |       |
                                                          |       |       +--> ~/sunray_mavros_sim/depth_img
                                                          |       +----------> ~/sunray_mavros_sim/cloud_sensor_frame
                                                          +------------------> ~/sunray_mavros_sim/cloud_world_frame
                                                          |
                                                          |
 ~/mavros/setpoint_raw/local                             |
 ~/mavros/setpoint_raw/attitude                          |
                  |                                      |
                  v                                      |
          +-----------------+                            |
          | PX4_CONTROL_SIM | <----- ~/mavros/state      |
          +-----------------+                            |
                  |                                      |
                  v                                      |
 ~/sunray_mavros_sim/cmd_RPM                             |
                  |                                      |
                  v                                      |
        +-------------------+                            |
        | QuadrotorSimulator |----------------------------+
        +-------------------+
             |       |       |
             |       |       +--> ~/sunray_mavros_sim/navsat
             |       +----------> ~/sunray_mavros_sim/imu ------------+
             +------------------> ~/sunray_mavros_sim/odom ----------+|
                                                                      ||
                                                                      vv
                                                       +------------------+
                                                       | FakeMavrosBridge |
                                                       +------------------+
                                                          |       |       |
                                                          |       |       +--> ~/mavros/state
                                                          |       +----------> ~/mavros/imu/data
                                                          +------------------> ~/mavros/local_position/odom
```

### 配置文件

配置文件按模块拆分，放在：

```text
/home/amov/Sunray_v2/sunray_mavros_sim/config
```

| 配置文件 | 对应模块 | 说明 |
| --- | --- | --- |
| `sunray_mavros_sim_node.yaml` | `sunray_mavros_sim_node` | 配置 `agent_name`、`agent_id`、`global_frame_id` 和统一状态打印参数。 |
| `global_map_server.yaml` | `GlobalMapServer` | 配置 PCD 地图、全局点云话题、降采样和地图偏移。 |
| `local_mid360_simulator.yaml` | `LocalMid360Simulator` | 配置 MID360 局部点云的量程、视场、角分辨率和发布频率。 |
| `quadrotor_simulator.yaml` | `QuadrotorSimulator` | 配置动力学更新频率、初始位姿、刚体参数和电机参数。 |
| `imu_model.yaml` | `ImuModel` | 配置 IMU 噪声、bias、协方差、安装误差和时间异步。 |
| `px4_control_sim.yaml` | `PX4_CONTROL_SIM` | 配置 PX4 setpoint 转电机 RPM 的 PID、前馈、限幅和更新频率。 |
| `fake_mavros_bridge.yaml` | `FakeMavrosBridge` | 配置 fake MAVROS 发布频率、初始状态和电池状态。 |

launch 中每份配置都用同一种方式加载：

```xml
<rosparam file="$(arg node_config)" command="load" subst_value="true" />
<rosparam file="$(arg global_map_server_config)" command="load" subst_value="true" />
<rosparam file="$(arg local_mid360_simulator_config)" command="load" subst_value="true" />
<rosparam file="$(arg quadrotor_simulator_config)" command="load" subst_value="true" />
<rosparam file="$(arg imu_model_config)" command="load" subst_value="true" />
<rosparam file="$(arg px4_control_sim_config)" command="load" subst_value="true" />
<rosparam file="$(arg fake_mavros_bridge_config)" command="load" subst_value="true" />
```

`subst_value="true"` 允许 YAML 中使用 `$(find sunray_mavros_sim)`。

### Launch 参数

| launch 参数 | 默认配置文件 |
| --- | --- |
| `node_config` | `$(find sunray_mavros_sim)/config/sunray_mavros_sim_node.yaml` |
| `global_map_server_config` | `$(find sunray_mavros_sim)/config/global_map_server.yaml` |
| `local_mid360_simulator_config` | `$(find sunray_mavros_sim)/config/local_mid360_simulator.yaml` |
| `quadrotor_simulator_config` | `$(find sunray_mavros_sim)/config/quadrotor_simulator.yaml` |
| `imu_model_config` | `$(find sunray_mavros_sim)/config/imu_model.yaml` |
| `px4_control_sim_config` | `$(find sunray_mavros_sim)/config/px4_control_sim.yaml` |
| `fake_mavros_bridge_config` | `$(find sunray_mavros_sim)/config/fake_mavros_bridge.yaml` |

示例：使用另一张地图配置启动：

```bash
roslaunch sunray_mavros_sim sunray_mavros_sim.launch \
  global_map_server_config:=/home/amov/Sunray_v2/sunray_mavros_sim/config/global_map_server.yaml
```

### 模块介绍

#### sunray_mavros_sim_node

模块功能：

创建唯一 ROS node，读取无人机编号，按顺序创建 `GlobalMapServer`、`LocalMid360Simulator`、`QuadrotorSimulator`、`PX4_CONTROL_SIM` 和 `FakeMavrosBridge`。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `agent_name` | `sunray_mavros_sim_node.yaml` | 无人机名称前缀，例如 `uav`。 |
| `agent_id` | `sunray_mavros_sim_node.yaml` | 无人机编号，例如 `1`。 |
| `global_frame_id` | `sunray_mavros_sim_node.yaml` | 全包统一全局坐标系 frame，默认 `map`。全局地图、局部点云、TF、odom、navsat 都使用它。 |
| `enable_status_print` | `sunray_mavros_sim_node.yaml` | 是否由主节点统一周期打印所有关键模块状态。 |
| `status_print_hz` | `sunray_mavros_sim_node.yaml` | 统一状态打印频率，单位 Hz。 |

订阅话题：

无。

发布话题：

无。该模块本身只负责创建和调度其他模块。

#### GlobalMapServer

模块功能：

读取 PCD 地图文件，执行地图偏移、边界补点和体素降采样，然后发布全局点云。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `map_name` | `global_map_server.yaml` | PCD 地图路径。 |
| `global_map_topic` | `global_map_server.yaml` | 全局点云发布话题。 |
| `global_frame_id` | `sunray_mavros_sim_node.yaml` | 全局点云 frame_id。为兼容旧配置，代码仍可读取 `map_frame`，但默认配置统一使用 `global_frame_id`。 |
| `add_boundary` | `global_map_server.yaml` | 是否添加地图外包围盒边界点。 |
| `downsample_res` | `global_map_server.yaml` | 体素降采样分辨率，单位 m。 |
| `map_offset_x/y/z` | `global_map_server.yaml` | 地图整体平移，单位 m。 |
| `map_publish_rate` | `global_map_server.yaml` | 全局地图发布频率，单位 Hz。 |

订阅话题：

无。

发布话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/map_generator/global_cloud` | `sensor_msgs/PointCloud2` | 全局 PCD 点云，话题名可由 `global_map_topic` 修改。 |

#### LocalMid360Simulator

模块功能：

根据全局地图和当前 odom，模拟 MID360 风格局部点云，输出全局坐标系点云、sensor 坐标系点云和深度图。`sensor_pose` 已删除，因为它与 odom 位姿重复。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `global_frame_id` | `sunray_mavros_sim_node.yaml` | `~/sunray_mavros_sim/cloud_world_frame` 的 frame_id，也是 TF 父坐标系。默认 `map`。 |
| `lidar_type` | `local_mid360_simulator.yaml` | 雷达类型名称，用于日志显示。 |
| `sensor_frame_id` | `local_mid360_simulator.yaml` | 传感器坐标系 frame_id。 |
| `is_360lidar` | `local_mid360_simulator.yaml` | 是否按 360 度水平视场输出。 |
| `sensing_horizon` | `local_mid360_simulator.yaml` | 最大感知距离，单位 m。 |
| `sensing_rate` | `local_mid360_simulator.yaml` | 局部点云发布频率，单位 Hz。 |
| `polar_resolution` | `local_mid360_simulator.yaml` | 极坐标角分辨率，单位 deg。 |
| `yaw_fov` | `local_mid360_simulator.yaml` | 水平视场角，单位 deg。 |
| `vertical_fov` | `local_mid360_simulator.yaml` | 垂直视场角，单位 deg。 |
| `min_raylength` | `local_mid360_simulator.yaml` | 最小有效量测距离，单位 m。 |
订阅话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_mavros_sim/odom` | `nav_msgs/Odometry` | 当前无人机位姿。 |

发布话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_mavros_sim/cloud_world_frame` | `sensor_msgs/PointCloud2` | 全局坐标系局部点云，frame_id 由 `global_frame_id` 决定。 |
| `~/sunray_mavros_sim/cloud_sensor_frame` | `sensor_msgs/PointCloud2` | sensor 坐标系局部点云。 |
| `~/sunray_mavros_sim/depth_img` | `sensor_msgs/Image` | 32FC1 深度图。 |
| `/tf` | `tf2_msgs/TFMessage` | `global_frame_id -> sensor_frame_id` 变换，默认 `map -> sensor`。 |

#### QuadrotorSimulator

模块功能：

订阅电机期望转速，积分四旋翼动力学，输出 odom、IMU 和简化 GNSS 数据。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `global_frame_id` | `sunray_mavros_sim_node.yaml` | odom 和 navsat 的 `header.frame_id`，默认 `map`。 |
| `dynamics_update_rate` | `quadrotor_simulator.yaml` | 动力学积分频率，单位 Hz。 |
| `cmd_timeout` | `quadrotor_simulator.yaml` | 电机指令超时时间，单位 s。 |
| `init_x/y/z` | `quadrotor_simulator.yaml` | 初始位置，单位 m。 |
| `init_yaw` | `quadrotor_simulator.yaml` | 初始偏航角，单位 rad。 |
| `dynamics/mass` | `quadrotor_simulator.yaml` | 机体质量，单位 kg。 |
| `dynamics/gravity` | `quadrotor_simulator.yaml` | 重力加速度，单位 m/s^2。 |
| `dynamics/arm_length` | `quadrotor_simulator.yaml` | 电机臂长，单位 m。 |
| `dynamics/Ixx/Iyy/Izz` | `quadrotor_simulator.yaml` | 三轴转动惯量，单位 kg*m^2。 |
| `motor/k_F` | `quadrotor_simulator.yaml` | 电机推力系数。 |
| `motor/k_T` | `quadrotor_simulator.yaml` | 电机反扭矩系数。 |
| `motor/tau_up` | `quadrotor_simulator.yaml` | 电机加速响应时间常数，单位 s。 |
| `motor/tau_down` | `quadrotor_simulator.yaml` | 电机减速响应时间常数，单位 s。 |
| `motor/omega_min` | `quadrotor_simulator.yaml` | 电机最小转速，单位 rad/s。 |
| `motor/omega_max` | `quadrotor_simulator.yaml` | 电机最大转速，单位 rad/s。 |

订阅话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_mavros_sim/cmd_RPM` | `std_msgs/Float32MultiArray` | 四个电机期望转速。 |

发布话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_mavros_sim/odom` | `nav_msgs/Odometry` | 仿真无人机里程计。 |
| `~/sunray_mavros_sim/imu` | `sensor_msgs/Imu` | 仿真 IMU。 |
| `~/sunray_mavros_sim/navsat` | `sensor_msgs/NavSatFix` | 简化 GNSS 数据。 |

#### ImuModel

模块功能：

作为 `QuadrotorSimulator` 的内部组件，对真实运动状态加入 IMU 噪声、bias、比例因子误差、安装误差、杆臂误差和时间异步。

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

无。该模块由 `QuadrotorSimulator` 直接调用。

发布话题：

无。IMU 消息由 `QuadrotorSimulator` 发布到 `~/sunray_mavros_sim/imu`。

#### PX4_CONTROL_SIM

模块功能：

订阅 MAVROS setpoint、MAVROS state 和当前 odom，把位置、速度、姿态或角速度指令转换成四电机期望 RPM。

参数：

| 参数 | 来源文件 | 说明 |
| --- | --- | --- |
| `px4_update_rate` | `px4_control_sim.yaml` | `PX4_CONTROL_SIM` 内部 update 定时器频率，单位 Hz。 |
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
| `~/sunray_mavros_sim/odom` | `nav_msgs/Odometry` | 当前无人机状态反馈。 |
| `~/mavros/state` | `mavros_msgs/State` | fake MAVROS 发布的 armed/mode 状态。 |

发布话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/sunray_mavros_sim/cmd_RPM` | `std_msgs/Float32MultiArray` | 四电机期望转速。 |

#### FakeMavrosBridge

模块功能：

把本包内部 odom/imu 转成 MAVROS 常用输出，并提供常见 MAVROS 服务，使上层控制程序可以按 MAVROS 接口运行。

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
| `~/sunray_mavros_sim/odom` | `nav_msgs/Odometry` | 内部仿真 odom。 |
| `~/sunray_mavros_sim/imu` | `sensor_msgs/Imu` | 内部仿真 IMU。 |
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

### 运行检查

启动后可以用下面命令检查主要输出。下面的 `~/...` 表示带无人机前缀的话题，实际运行命令时按当前 `agent_name/agent_id` 展开成对应前缀。

```bash
rostopic list | grep -E "sunray_mavros_sim|mavros|global_cloud"
rostopic hz ~/sunray_mavros_sim/odom
rostopic hz ~/sunray_mavros_sim/imu
rostopic hz ~/sunray_mavros_sim/cloud_world_frame
rostopic echo ~/mavros/state -n 1
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
