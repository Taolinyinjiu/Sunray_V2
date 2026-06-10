# MAVROS 话题与 PX4 uORB 对照说明

本文面向 `sunray_uav_control` 二次开发，说明 MAVROS 常见标准话题/服务、本包实际用到的 MAVROS 子集，以及和 PX4 内部 uORB 消息的大致对应关系。

注意：MAVROS 并不是直接暴露 uORB。实际链路通常是：

```text
ROS node <-> MAVROS topic/service <-> MAVLink message <-> PX4 mavlink module <-> uORB
```

因此“uORB 对照”应理解为 PX4 内部主要数据来源或主要落点，不是严格的一一等价关系。不同 PX4 版本、MAVROS 版本、参数配置、插件黑白名单和飞控模式可能会改变实际话题列表和内部路径。

## 1. MAVROS 标准话题总览

下面按 MAVROS 标准插件列出常见话题。这里的 `/<agent>/mavros` 对应实际命名空间，例如 Sunray 单机默认是：

```text
/uav1/mavros
```

具体运行时以命令输出为准：

```bash
rostopic list | grep /mavros
rosservice list | grep /mavros
```

本仓库常用 PX4 配置来自 MAVROS `px4.launch` / `px4_pluginlists.yaml`。默认 blacklist 中可能禁用 `safety_area`、`distance_sensor`、`rangefinder`、`wheel_odometry` 等插件，所以“文档列出”不等于“当前一定会发布”。

### 1.1 系统状态类

| 插件 | 话题 | 消息类型 | 方向 | 说明 |
| --- | --- | --- | --- | --- |
| `sys_status` | `/<agent>/mavros/state` | `mavros_msgs/State` | FCU -> ROS | MAVROS 连接、解锁、模式、手动输入状态 |
| `sys_status` | `/<agent>/mavros/extended_state` | `mavros_msgs/ExtendedState` | FCU -> ROS | 起降状态、VTOL 状态 |
| `sys_status` | `/<agent>/mavros/sys_status` | `mavros_msgs/SysStatus` | FCU -> ROS | 系统负载、电池、电源等 MAVLink SYS_STATUS |
| `sys_status` | `/<agent>/mavros/battery` | `sensor_msgs/BatteryState` | FCU -> ROS | 电池状态 |
| `sys_status` | `/<agent>/mavros/estimator_status` | `mavros_msgs/EstimatorStatus` | FCU -> ROS | 估计器健康状态 |
| `sys_status` | `/<agent>/mavros/statustext/recv` | `mavros_msgs/StatusText` | FCU -> ROS | 飞控状态文本 |
| `sys_status` | `/<agent>/mavros/statustext/send` | `mavros_msgs/StatusText` | ROS -> FCU | 向飞控发送状态文本，是否可用取决于版本/配置 |
| `sys_time` | `/<agent>/mavros/time_reference` | `sensor_msgs/TimeReference` | FCU -> ROS | FCU 时间参考 |
| `sys_time` | `/<agent>/mavros/timesync_status` | `mavros_msgs/TimesyncStatus` | FCU -> ROS | MAVLink timesync 状态 |
| `vfr_hud` | `/<agent>/mavros/vfr_hud` | `mavros_msgs/VFR_HUD` | FCU -> ROS | 空速、地速、高度、爬升率等 HUD 数据 |
| `altitude` | `/<agent>/mavros/altitude` | `mavros_msgs/Altitude` | FCU -> ROS | 不同参考系下的高度 |
| `home_position` | `/<agent>/mavros/home_position/home` | `mavros_msgs/HomePosition` | FCU -> ROS | Home 点 |
| `wind_estimation` | `/<agent>/mavros/wind_estimation` | `geometry_msgs/TwistWithCovarianceStamped` | FCU -> ROS | 风估计，取决于飞控是否输出 |

### 1.2 IMU 与传感器类

| 插件 | 话题 | 消息类型 | 方向 | 说明 |
| --- | --- | --- | --- | --- |
| `imu` | `/<agent>/mavros/imu/data` | `sensor_msgs/Imu` | FCU -> ROS | 融合后的 IMU/姿态数据 |
| `imu` | `/<agent>/mavros/imu/data_raw` | `sensor_msgs/Imu` | FCU -> ROS | 原始 IMU 数据 |
| `imu` | `/<agent>/mavros/imu/mag` | `sensor_msgs/MagneticField` | FCU -> ROS | 磁力计 |
| `imu` | `/<agent>/mavros/imu/static_pressure` | `sensor_msgs/FluidPressure` | FCU -> ROS | 静压 |
| `imu` | `/<agent>/mavros/imu/diff_pressure` | `sensor_msgs/FluidPressure` | FCU -> ROS | 差压 |
| `imu` | `/<agent>/mavros/imu/temperature_imu` | `sensor_msgs/Temperature` | FCU -> ROS | IMU 温度 |
| `imu` | `/<agent>/mavros/imu/temperature_baro` | `sensor_msgs/Temperature` | FCU -> ROS | 气压计温度 |
| `cam_imu_sync` | `/<agent>/mavros/cam_imu_sync/cam_imu_stamp` | `std_msgs/Header` | FCU -> ROS | 相机-IMU 同步时间戳，取决于飞控输出 |

### 1.3 位置、速度、GPS 类

| 插件 | 话题 | 消息类型 | 方向 | 说明 |
| --- | --- | --- | --- | --- |
| `local_position` | `/<agent>/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | FCU -> ROS | 局部位姿 |
| `local_position` | `/<agent>/mavros/local_position/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | FCU -> ROS | 带协方差局部位姿 |
| `local_position` | `/<agent>/mavros/local_position/odom` | `nav_msgs/Odometry` | FCU -> ROS | 局部里程计 |
| `local_position` | `/<agent>/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | FCU -> ROS | local frame 速度 |
| `local_position` | `/<agent>/mavros/local_position/velocity_body` | `geometry_msgs/TwistStamped` | FCU -> ROS | body frame 速度 |
| `local_position` | `/<agent>/mavros/local_position/accel` | `geometry_msgs/AccelWithCovarianceStamped` | FCU -> ROS | 局部加速度 |
| `global_position` | `/<agent>/mavros/global_position/global` | `sensor_msgs/NavSatFix` | FCU -> ROS | 融合后的全球定位 |
| `global_position` | `/<agent>/mavros/global_position/raw/fix` | `sensor_msgs/NavSatFix` | FCU -> ROS | 原始 GPS fix |
| `global_position` | `/<agent>/mavros/global_position/raw/gps_vel` | `geometry_msgs/TwistStamped` | FCU -> ROS | 原始 GPS 速度 |
| `global_position` | `/<agent>/mavros/global_position/raw/satellites` | `std_msgs/UInt32` | FCU -> ROS | 可见卫星数量 |
| `global_position` | `/<agent>/mavros/global_position/local` | `nav_msgs/Odometry` | FCU -> ROS | 全球定位转换到局部坐标 |
| `global_position` | `/<agent>/mavros/global_position/rel_alt` | `std_msgs/Float64` | FCU -> ROS | 相对高度 |
| `global_position` | `/<agent>/mavros/global_position/compass_hdg` | `std_msgs/Float64` | FCU -> ROS | 航向角 |
| `global_position` | `/<agent>/mavros/global_position/gp_origin` | `geographic_msgs/GeoPointStamped` | FCU -> ROS | global/local 原点 |
| `global_position` | `/<agent>/mavros/global_position/gp_lp_offset` | `geometry_msgs/Vector3Stamped` | FCU -> ROS | global/local 偏移 |
| `global_position` | `/<agent>/mavros/global_position/set_gp_origin` | `geographic_msgs/GeoPointStamped` | ROS -> FCU | 设置 global/local 原点，具体支持取决于飞控 |
| `gps_status` / `gps_rtk` | `/<agent>/mavros/gpsstatus/gps1/raw` | `mavros_msgs/GPSRAW` | FCU -> ROS | GPS1 原始状态 |
| `gps_status` / `gps_rtk` | `/<agent>/mavros/gpsstatus/gps2/raw` | `mavros_msgs/GPSRAW` | FCU -> ROS | GPS2 原始状态，若有 |

### 1.4 RC、手动控制与执行器类

| 插件 | 话题 | 消息类型 | 方向 | 说明 |
| --- | --- | --- | --- | --- |
| `rc_io` | `/<agent>/mavros/rc/in` | `mavros_msgs/RCIn` | FCU -> ROS | RC 输入通道和 RSSI |
| `rc_io` | `/<agent>/mavros/rc/out` | `mavros_msgs/RCOut` | FCU -> ROS | PWM/舵机输出 |
| `rc_io` | `/<agent>/mavros/rc/override` | `mavros_msgs/OverrideRCIn` | ROS -> FCU | RC override，谨慎使用 |
| `manual_control` | `/<agent>/mavros/manual_control/control` | `mavros_msgs/ManualControl` | ROS -> FCU | 手动控制输入 |
| `actuator_control` | `/<agent>/mavros/actuator_control` | `mavros_msgs/ActuatorControl` | ROS -> FCU | 直接执行器控制 |

### 1.5 Offboard/setpoint 类

| 插件 | 话题 | 消息类型 | 方向 | 说明 |
| --- | --- | --- | --- | --- |
| `setpoint_position` | `/<agent>/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | ROS -> FCU | 局部位置 setpoint |
| `setpoint_position` | `/<agent>/mavros/setpoint_position/global` | `geographic_msgs/GeoPoseStamped` | ROS -> FCU | 全球位置 setpoint |
| `setpoint_position` | `/<agent>/mavros/setpoint_position/global_to_local` | `geometry_msgs/PoseStamped` | FCU/ROS -> ROS | global setpoint 转换后的 local 表示，取决于插件 |
| `setpoint_velocity` | `/<agent>/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | ROS -> FCU | 速度 setpoint |
| `setpoint_velocity` | `/<agent>/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/Twist` | ROS -> FCU | 无时间戳速度 setpoint |
| `setpoint_accel` | `/<agent>/mavros/setpoint_accel/accel` | `geometry_msgs/Vector3Stamped` | ROS -> FCU | 加速度/力 setpoint |
| `setpoint_attitude` | `/<agent>/mavros/setpoint_attitude/attitude` | `geometry_msgs/PoseStamped` | ROS -> FCU | 姿态 setpoint |
| `setpoint_attitude` | `/<agent>/mavros/setpoint_attitude/cmd_vel` | `geometry_msgs/TwistStamped` | ROS -> FCU | 姿态角速度 setpoint |
| `setpoint_attitude` | `/<agent>/mavros/setpoint_attitude/thrust` | `mavros_msgs/Thrust` | ROS -> FCU | 推力 setpoint |
| `setpoint_raw` | `/<agent>/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | ROS -> FCU | 原始局部 setpoint |
| `setpoint_raw` | `/<agent>/mavros/setpoint_raw/global` | `mavros_msgs/GlobalPositionTarget` | ROS -> FCU | 原始全球 setpoint |
| `setpoint_raw` | `/<agent>/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | ROS -> FCU | 原始姿态/角速度/推力 setpoint |
| `setpoint_raw` | `/<agent>/mavros/setpoint_raw/target_local` | `mavros_msgs/PositionTarget` | FCU -> ROS | 飞控收到/回显的局部 target |
| `setpoint_raw` | `/<agent>/mavros/setpoint_raw/target_global` | `mavros_msgs/GlobalPositionTarget` | FCU -> ROS | 飞控收到/回显的全球 target |
| `setpoint_raw` | `/<agent>/mavros/setpoint_raw/target_attitude` | `mavros_msgs/AttitudeTarget` | FCU -> ROS | 飞控收到/回显的姿态 target |
| `setpoint_trajectory` | `/<agent>/mavros/setpoint_trajectory/local` | `trajectory_msgs/MultiDOFJointTrajectory` | ROS -> FCU | 轨迹 setpoint |
| `setpoint_trajectory` | `/<agent>/mavros/setpoint_trajectory/desired` | `trajectory_msgs/MultiDOFJointTrajectory` | FCU -> ROS | 期望轨迹回显，取决于飞控 |

### 1.6 外部定位输入类

| 插件 | 话题 | 消息类型 | 方向 | 说明 |
| --- | --- | --- | --- | --- |
| `vision_pose_estimate` | `/<agent>/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | ROS -> FCU | 外部视觉位姿 |
| `vision_pose_estimate` | `/<agent>/mavros/vision_pose/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | ROS -> FCU | 带协方差外部视觉位姿 |
| `odometry` | `/<agent>/mavros/odometry/in` | `nav_msgs/Odometry` | ROS -> FCU | 外部里程计输入 |
| `odometry` | `/<agent>/mavros/odometry/out` | `nav_msgs/Odometry` | FCU -> ROS | 飞控里程计输出，取决于 MAVLink ODOMETRY 输出 |
| `vision_speed_estimate` | `/<agent>/mavros/vision_speed/speed_twist` | `geometry_msgs/TwistStamped` | ROS -> FCU | 外部视觉速度 |
| `vision_speed_estimate` | `/<agent>/mavros/vision_speed/speed_twist_cov` | `geometry_msgs/TwistWithCovarianceStamped` | ROS -> FCU | 带协方差外部视觉速度 |
| `mocap_pose_estimate` | `/<agent>/mavros/mocap/pose` | `geometry_msgs/PoseStamped` | ROS -> FCU | 动捕位姿 |
| `fake_gps` | `/<agent>/mavros/fake_gps/mocap/pose` 等 | 多种 | ROS -> FCU | 用 mocap/vision 构造 fake GPS，取决于配置 |

### 1.7 任务、航点、地理围栏类

| 插件 | 话题/服务 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- | --- |
| `waypoint` | `/<agent>/mavros/mission/waypoints` | `mavros_msgs/WaypointList` | FCU -> ROS | 当前任务航点列表 |
| `waypoint` | `/<agent>/mavros/mission/reached` | `mavros_msgs/WaypointReached` | FCU -> ROS | 到达航点通知 |
| `waypoint` | `/<agent>/mavros/mission/pull` | `mavros_msgs/WaypointPull` | service | 从飞控拉取航点 |
| `waypoint` | `/<agent>/mavros/mission/push` | `mavros_msgs/WaypointPush` | service | 上传航点 |
| `waypoint` | `/<agent>/mavros/mission/clear` | `mavros_msgs/WaypointClear` | service | 清空航点 |
| `waypoint` | `/<agent>/mavros/mission/set_current` | `mavros_msgs/WaypointSetCurrent` | service | 设置当前航点 |
| `geofence` | `/<agent>/mavros/geofence/*` | 多种 service | service | 地理围栏读写，若插件启用 |
| `rallypoint` | `/<agent>/mavros/rallypoint/*` | 多种 service | service | rally point 读写，若插件启用 |

### 1.8 命令、模式、参数和文件类

| 插件 | 话题/服务 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- | --- |
| `command` | `/<agent>/mavros/cmd/arming` | `mavros_msgs/CommandBool` | service | 解锁/上锁 |
| `command` | `/<agent>/mavros/cmd/command` | `mavros_msgs/CommandLong` | service | MAVLink command long |
| `command` | `/<agent>/mavros/cmd/command_int` | `mavros_msgs/CommandInt` | service | MAVLink command int |
| `command` | `/<agent>/mavros/cmd/takeoff` | `mavros_msgs/CommandTOL` | service | 起飞命令 |
| `command` | `/<agent>/mavros/cmd/land` | `mavros_msgs/CommandTOL` | service | 降落命令 |
| `command` | `/<agent>/mavros/cmd/trigger_control` | `mavros_msgs/CommandTriggerControl` | service | 相机/触发器控制，取决于飞控 |
| `sys_status` / `command` | `/<agent>/mavros/set_mode` | `mavros_msgs/SetMode` | service | 切换飞行模式 |
| `param` | `/<agent>/mavros/param/get` | `mavros_msgs/ParamGet` | service | 读取参数 |
| `param` | `/<agent>/mavros/param/set` | `mavros_msgs/ParamSet` | service | 写入参数 |
| `param` | `/<agent>/mavros/param/pull` | `mavros_msgs/ParamPull` | service | 拉取参数表 |
| `param` | `/<agent>/mavros/param/push` | `mavros_msgs/ParamPush` | service | 推送参数表 |
| `ftp` | `/<agent>/mavros/ftp/*` | service/topic | both | MAVLink FTP，接口随版本变化 |

### 1.9 其他常见插件话题

| 插件 | 话题 | 消息类型 | 方向 | 说明 |
| --- | --- | --- | --- | --- |
| `3dr_radio` | `/<agent>/mavros/radio_status` | `mavros_msgs/RadioStatus` | FCU -> ROS | 数传电台状态 |
| `nav_controller_output` | `/<agent>/mavros/nav_controller_output` | `mavros_msgs/NavControllerOutput` | FCU -> ROS | 固定翼导航控制器输出 |
| `hil` | `/<agent>/mavros/hil/*` | 多种 | both | HIL 仿真接口 |
| `safety_area` | `/<agent>/mavros/safety_area/set` | `geometry_msgs/PolygonStamped` | ROS -> FCU | 设置安全区域，PX4 默认配置常禁用 |
| `landing_target` | `/<agent>/mavros/landing_target/*` | 多种 | both | 精准降落目标 |
| `distance_sensor` | `/<agent>/mavros/distance_sensor/*` | `sensor_msgs/Range` | both | 距离传感器，常在 extras 中 |
| `obstacle_distance` | `/<agent>/mavros/obstacle/send` | `sensor_msgs/LaserScan` 等 | ROS -> FCU | 避障距离输入，取决于插件版本 |

## 2. Sunray 当前使用的 MAVROS 链路

`sunray_uav_control` 中的 MAVROS 封装在：

```text
include/mavros_helper/mavros_helper.hpp
src/mavros_helper/mavros_helper.cpp
```

`MavrosHelper` 主要做三件事：

1. 订阅 MAVROS 状态、定位、IMU、RC、GPS、setpoint 回显话题。
2. 发布外部定位融合数据和底层控制 setpoint。
3. 调用 MAVROS 服务切换模式、解锁/上锁、发送 command long、读写 PX4 参数。

## 3. Sunray 使用的 MAVROS 输入话题

这些话题是 `MavrosHelper` 从 MAVROS/PX4 读取的数据。

| MAVROS 话题 | ROS 消息 | Sunray 用途 | PX4/uORB 主要来源 |
| --- | --- | --- | --- |
| `/<agent>/mavros/state` | `mavros_msgs/State` | 连接状态、解锁状态、飞行模式、是否有手动输入 | `vehicle_status`、`manual_control_setpoint` 等状态经 MAVLink `HEARTBEAT` 等消息输出 |
| `/<agent>/mavros/extended_state` | `mavros_msgs/ExtendedState` | 起降状态，判断在地面/空中/起飞/降落 | `vehicle_land_detected`、`vehicle_status` 经 MAVLink `EXTENDED_SYS_STATE` 输出 |
| `/<agent>/mavros/sys_status` | `mavros_msgs/SysStatus` | 电压、电流、电量、FCU 负载 | `battery_status`、系统状态经 MAVLink `SYS_STATUS` 输出 |
| `/<agent>/mavros/estimator_status` | `mavros_msgs/EstimatorStatus` | EKF/估计器健康状态检查 | estimator 状态经 MAVLink `ESTIMATOR_STATUS` 输出 |
| `/<agent>/mavros/local_position/odom` | `nav_msgs/Odometry` | MAVROS/PX4 局部位姿和速度缓存 | `vehicle_local_position` / `vehicle_odometry` |
| `/<agent>/mavros/imu/data` | `sensor_msgs/Imu` | IMU 姿态、角速度、加速度缓存，几何控制器可用 | `vehicle_attitude`、`vehicle_angular_velocity`、`sensor_combined` 等 |
| `/<agent>/mavros/setpoint_raw/target_local` | `mavros_msgs/PositionTarget` | 读取 PX4/MAVROS 最近接收的位置类 setpoint 回显 | offboard setpoint 相关 uORB，通常落到 `trajectory_setpoint` / `offboard_control_mode` |
| `/<agent>/mavros/setpoint_raw/target_attitude` | `mavros_msgs/AttitudeTarget` | 读取 PX4/MAVROS 最近接收的姿态类 setpoint 回显 | `vehicle_attitude_setpoint`、`vehicle_rates_setpoint` 等 |
| `/<agent>/mavros/gpsstatus/gps1/raw` | `mavros_msgs/GPSRAW` | GPS 卫星数、fix 状态、经纬高 | `sensor_gps` / `vehicle_gps_position` |
| `/<agent>/mavros/rc/in` | `mavros_msgs/RCIn` | RC 原始通道数组和 RSSI，转发到 `Px4State` | `input_rc`、`manual_control_setpoint` |

## 4. Sunray 使用的 MAVROS 输出话题

这些话题由 `sunray_uav_control` 发布给 MAVROS/PX4。

| MAVROS 话题 | ROS 消息 | Sunray 发布者 | MAVLink 消息 | PX4/uORB 主要落点 |
| --- | --- | --- | --- | --- |
| `/<agent>/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | `MavrosHelper::pub_vision_pose()` | `VISION_POSITION_ESTIMATE` / 相关 vision 输入 | `vehicle_visual_odometry` / estimator 外部视觉输入 |
| `/<agent>/mavros/odometry/in` | `nav_msgs/Odometry` | `MavrosHelper::pub_vision_pose()` 在 `fuse_odom_type=2` 时使用 | `ODOMETRY` | `vehicle_visual_odometry` / estimator 外部里程计输入 |
| `/<agent>/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | `PX4_OriginController` | `SET_POSITION_TARGET_LOCAL_NED` | `offboard_control_mode`、`trajectory_setpoint` |
| `/<agent>/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | `Geometric_Controller` | `SET_ATTITUDE_TARGET` | `vehicle_attitude_setpoint` 或 `vehicle_rates_setpoint` |

### 4.1 setpoint_raw/local

`mavros_msgs/PositionTarget` 是 MAVROS 对 MAVLink `SET_POSITION_TARGET_LOCAL_NED` 的 ROS 封装。它包含：

| 字段 | 含义 |
| --- | --- |
| `coordinate_frame` | 坐标系，如 local、body、offset |
| `type_mask` | 忽略哪些字段，例如忽略位置、速度、加速度、yaw 或 yaw_rate |
| `position` | 目标位置 |
| `velocity` | 目标速度 |
| `acceleration_or_force` | 目标加速度或力 |
| `yaw` | 目标偏航角 |
| `yaw_rate` | 目标偏航角速度 |

Sunray 中 `PX4_OriginController` 主要使用这个话题，让 PX4 原生位置/速度控制链路完成底层控制。

### 4.2 setpoint_raw/attitude

`mavros_msgs/AttitudeTarget` 是 MAVROS 对 MAVLink `SET_ATTITUDE_TARGET` 的 ROS 封装。它包含：

| 字段 | 含义 |
| --- | --- |
| `type_mask` | 忽略姿态、角速度、推力等字段 |
| `orientation` | 目标姿态四元数 |
| `body_rate` | 机体系角速度，单位 rad/s |
| `thrust` | 归一化推力，通常为 `[0, 1]` |

Sunray 中 `Geometric_Controller` 主要使用这个话题。配置 `geometric_controller_param.control_type` 决定输出姿态+推力，还是 bodyrate+推力。

## 5. Sunray 使用的 MAVROS 服务

| MAVROS 服务 | 服务类型 | Sunray 用途 | PX4/uORB 主要影响 |
| --- | --- | --- | --- |
| `/<agent>/mavros/cmd/arming` | `mavros_msgs/CommandBool` | 解锁/上锁 | 发送 MAVLink `COMMAND_LONG`，PX4 内部处理为 `vehicle_command` |
| `/<agent>/mavros/set_mode` | `mavros_msgs/SetMode` | 切换 `POSCTL`、`OFFBOARD`、`AUTO.LAND` 等模式 | 发送模式切换请求，影响 `vehicle_status.nav_state` 等 |
| `/<agent>/mavros/cmd/command` | `mavros_msgs/CommandLong` | emergency kill、reboot 等命令 | 发送 MAVLink `COMMAND_LONG`，PX4 内部处理为 `vehicle_command` |
| `/<agent>/mavros/param/set` | `mavros_msgs/ParamSet` | 写 PX4 参数 | PX4 参数系统 |
| `/<agent>/mavros/param/get` | `mavros_msgs/ParamGet` | 读 PX4 参数 | PX4 参数系统 |

## 6. Sunray 汇总话题：`/sunray/px4_state`

Sunray 额外发布了一个汇总话题：

```text
/<agent>/sunray/px4_state
```

消息类型：

```text
sunray_msgs/Px4State
```

它把多个 MAVROS 话题整理到一个消息里，方便上层程序、监控工具和调试脚本使用。

| `Px4State` 字段 | 数据来源 | 说明 |
| --- | --- | --- |
| `connected` | `/mavros/state.connected` | MAVROS 是否连接 PX4 |
| `rc_available` | `/mavros/state.manual_input` | PX4/MAVROS 是否认为 RC 输入可用 |
| `armed` | `/mavros/state.armed` | 飞控是否解锁 |
| `flight_mode` | `/mavros/state.mode` | PX4 模式，Sunray 转为内部枚举 |
| `system_status` | `/mavros/state.system_status` | MAVLink `MAV_STATE` |
| `rc_channels` | `/mavros/rc/in.channels` | RC 原始通道数组 |
| `rc_rssi` | `/mavros/rc/in.rssi` | RC 信号强度 |
| `landed_state` | `/mavros/extended_state.landed_state` | 起降状态 |
| `battery_voltage_v` | `/mavros/sys_status.voltage_battery` | 电池电压，代码中转为 V |
| `battery_current_a` | `/mavros/sys_status.current_battery` | 电池电流，代码中转为 A |
| `battery_percentage` | `/mavros/sys_status.battery_remaining` | 电量百分比，代码中转为 `[0, 1]` |
| `fcu_load` | `/mavros/sys_status.load` | 飞控负载 |
| `external_pose` / `external_velocity` | Sunray 传给 MAVROS 的外部定位 | 外部定位融合输入快照 |
| `local_pose` / `local_velocity` | `/mavros/local_position/odom` | PX4/MAVROS 局部位姿与速度 |
| `pos_setpoint` / `vel_setpoint` / `acc_setpoint` | `/mavros/setpoint_raw/target_local` | 最近位置类 setpoint |
| `orientation_setpoint` / `body_rate_setpoint` / `thrust_setpoint` | `/mavros/setpoint_raw/target_attitude` | 最近姿态类 setpoint |
| `satellites` / `gps_status` / `latitude` 等 | `/mavros/gpsstatus/gps1/raw` | GPS 信息 |

RC 通道仅作为原始数组发布，不在 `Px4State` 中解释成 roll、pitch、throttle、yaw。不同遥控器、接收机、PX4 `RC_MAP_*` 参数和 MAVROS 配置都可能改变通道含义。

## 7. uORB 对照表

下面表格适合开发时快速定位“这个 MAVROS 话题大概对应 PX4 内部哪个 uORB”。实际调试 PX4 固件时，应以当前 PX4 版本源码和 `listener <uorb_topic>` 输出为准。

| 功能 | MAVROS 话题/服务 | MAVLink 主要消息 | PX4 uORB 主要对应 |
| --- | --- | --- | --- |
| 飞控连接/模式/解锁 | `/mavros/state` | `HEARTBEAT` | `vehicle_status`、`actuator_armed`、`manual_control_setpoint` |
| 起降状态 | `/mavros/extended_state` | `EXTENDED_SYS_STATE` | `vehicle_land_detected`、`vehicle_status` |
| 系统/电池状态 | `/mavros/sys_status` | `SYS_STATUS` | `battery_status`、系统状态 |
| 估计器状态 | `/mavros/estimator_status` | `ESTIMATOR_STATUS` | estimator/EKF 状态相关 uORB |
| 局部位姿/速度输出 | `/mavros/local_position/odom` | `LOCAL_POSITION_NED` / `ODOMETRY` | `vehicle_local_position`、`vehicle_odometry` |
| IMU 输出 | `/mavros/imu/data` | `HIGHRES_IMU` / `ATTITUDE` / `ATTITUDE_QUATERNION` | `sensor_combined`、`vehicle_attitude`、`vehicle_angular_velocity` |
| GPS 输出 | `/mavros/gpsstatus/gps1/raw` | `GPS_RAW_INT` | `sensor_gps` / `vehicle_gps_position` |
| RC 输入 | `/mavros/rc/in` | `RC_CHANNELS` | `input_rc`、`manual_control_setpoint` |
| 外部视觉位姿输入 | `/mavros/vision_pose/pose` | `VISION_POSITION_ESTIMATE` | `vehicle_visual_odometry`，供 EKF 融合 |
| 外部里程计输入 | `/mavros/odometry/in` | `ODOMETRY` | `vehicle_visual_odometry`，供 EKF 融合 |
| 位置/速度/加速度 setpoint | `/mavros/setpoint_raw/local` | `SET_POSITION_TARGET_LOCAL_NED` | `offboard_control_mode`、`trajectory_setpoint` |
| 姿态/角速度/推力 setpoint | `/mavros/setpoint_raw/attitude` | `SET_ATTITUDE_TARGET` | `vehicle_attitude_setpoint`、`vehicle_rates_setpoint` |
| 解锁/上锁 | `/mavros/cmd/arming` | `COMMAND_LONG` | `vehicle_command`，影响 `actuator_armed` |
| 模式切换 | `/mavros/set_mode` | `SET_MODE` / command | `vehicle_command`、`vehicle_status` |
| 参数读写 | `/mavros/param/get`、`/mavros/param/set` | `PARAM_REQUEST_READ`、`PARAM_SET` | PX4 参数系统 |

## 8. 开发注意事项

### 8.1 ENU / NED 坐标差异

ROS 常用 ENU，PX4/MAVLink 很多接口是 NED。MAVROS 插件会做坐标转换，但使用 `setpoint_raw` 时仍需要确认字段语义。不要把 MAVLink 名字里的 `LOCAL_NED` 直接理解为 ROS 侧消息一定是 NED。

Sunray 内部控制命令通常按 ROS local 右手系语义使用：

```text
x: 前方
y: 左方
z: 上方
```

实际坐标定义还取决于定位模块输出的 frame。

### 8.2 type_mask 很关键

`PositionTarget.type_mask` 和 `AttitudeTarget.type_mask` 决定 PX4 采用哪些字段、忽略哪些字段。二次开发时不要只填数值，还要确保 mask 和控制意图一致。

例如：

| 控制意图 | 应关注 |
| --- | --- |
| 位置控制 | 不应忽略 `position`，通常忽略不需要的速度/加速度字段 |
| 速度控制 | 不应忽略 `velocity`，需要明确是否锁高度 |
| bodyrate 控制 | `AttitudeTarget` 通常忽略 `orientation`，使用 `body_rate + thrust` |
| attitude 控制 | `AttitudeTarget` 通常使用 `orientation + thrust`，忽略不需要的 rate |

### 8.3 Offboard 需要持续 setpoint

PX4 Offboard 控制需要持续收到 setpoint。Sunray 控制器高频线程会持续向 MAVROS 发布底层 setpoint；上层 `UAVControlCMD` 中，速度和轨迹类命令也需要持续发布，位置点和起降类命令发送一次即可。

### 8.4 RC 通道不要硬编码

`/mavros/rc/in` 和 `Px4State.rc_channels` 是原始通道值。建议二次开发时通过配置文件定义通道含义，例如：

```yaml
rc_channel_map:
  roll: 0
  pitch: 1
  throttle: 2
  yaw: 3
  mode: 4
```

不要在通用控制库里直接假设第 1 通道一定是 roll、第 3 通道一定是 throttle。

## 9. 参考资料

- MAVROS ROS Wiki: https://wiki.ros.org/mavros
- MAVROS plugin list: https://wiki.ros.org/mavros/Plugins
- PX4 MAVROS installation guide: https://docs.px4.io/main/en/ros/mavros_installation
- MAVLink common message set: https://mavlink.io/en/messages/common.html
- MAVROS `State.msg`: https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/State.html
- MAVROS `ExtendedState.msg`: https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/ExtendedState.html
- MAVROS `SysStatus.msg`: https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/SysStatus.html
- MAVROS `EstimatorStatus.msg`: https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/EstimatorStatus.html
- MAVROS `RCIn.msg`: https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/RCIn.html
- MAVROS `PositionTarget.msg`: https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/PositionTarget.html
- MAVROS `AttitudeTarget.msg`: https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/AttitudeTarget.html
- PX4 uORB messaging: https://docs.px4.io/main/en/middleware/uorb
- PX4 uORB `VehicleStatus`: https://docs.px4.io/main/en/msg_docs/VehicleStatus.html
- PX4 uORB `InputRc`: https://docs.px4.io/main/en/msg_docs/InputRc.html
- PX4 uORB `ManualControlSetpoint`: https://docs.px4.io/main/en/msg_docs/ManualControlSetpoint.html
- PX4 uORB `VehicleOdometry`: https://docs.px4.io/main/en/msg_docs/VehicleOdometry.html
