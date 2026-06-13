<!-- title: MAVROS -->

<section id="appendix-mavros">

## MAVROS

MAVROS 是 ROS 与 MAVLink 飞控之间的桥。它把 PX4 发出的 MAVLink 消息转换成 ROS topic，也把 ROS topic/service 转换成 MAVLink 发给 PX4。

在 Sunray_v2 中，`drivers/sunray_mavros` 对 MAVROS 做了项目级封装：统一命名空间、配置 PX4 插件、设置串口/UDP 连接，并让 UAV 控制模块能够通过固定的 `/uav1/mavros/*` 接口访问 PX4。

### 在 Sunray 中的启动位置

启动入口：

```bash
roslaunch sunray_mavros mavros.launch agent_name:=uav agent_id:=1 fcu_url:=/dev/ttyACM0:921600
```

对应文件：

```text
drivers/sunray_mavros/launch/mavros.launch
drivers/sunray_mavros/config/exp_px4_config.yaml
drivers/sunray_mavros/config/exp_px4_pluginlists.yaml
```

启动后，MAVROS 被放到智能体命名空间下：

```text
/uav1/mavros/state
/uav1/mavros/local_position/odom
/uav1/mavros/setpoint_raw/local
```

多机时应修改 `agent_id`、`fcu_url`、UDP 端口和 PX4 system id，避免多个 MAVROS 实例连接到同一个飞控或互相抢同一端口。

### MAVROS 插件机制

MAVROS 由多个 plugin 组成。每个 plugin 负责一组 MAVLink 消息和 ROS 接口，例如：

| Plugin | 常见话题/服务 | 作用 |
| --- | --- | --- |
| `state` / core | `/mavros/state` | MAVLink 连接、模式、解锁状态。 |
| `command` | `/mavros/cmd/arming` 等服务 | 解锁、起飞、降落、命令发送。 |
| `setpoint_raw` | `/mavros/setpoint_raw/local`、`target_local` | 原始位置/速度/加速度 setpoint。 |
| `setpoint_attitude` | `/mavros/setpoint_raw/attitude`、`target_attitude` | 姿态、角速度、推力 setpoint。 |
| `local_position` | `/mavros/local_position/odom` | PX4 本地位置和速度。 |
| `vision_pose_estimate` | `/mavros/vision_pose/pose` | 外部视觉位姿输入 PX4。 |
| `vision_speed_estimate` | `/mavros/vision_speed/speed_twist` | 外部视觉速度输入 PX4。 |
| `odom` | `/mavros/odometry/in`、`/mavros/odometry/out` | MAVLink ODOMETRY 输入/输出。 |
| `rc_io` | `/mavros/rc/in` | RC 原始通道输入。 |
| `sys_status` | `/mavros/sys_status`、电池相关话题 | 电池、电流、系统状态。 |
| `param` | `/mavros/param/get`、`/mavros/param/set` | PX4 参数读取和写入。 |
| `imu` | `/mavros/imu/data_raw`、`/mavros/imu/data` | IMU 数据。 |
| `gps_status` | `/mavros/gpsstatus/gps1/raw` | GPS 原始状态。 |

Sunray 的 `exp_px4_pluginlists.yaml` 当前白名单启用了 `command`、`distance_sensor`、`global_position`、`gps_status`、`imu`、`local_position`、`manual_control`、`param`、`rc_io`、`setpoint_raw`、`setpoint_attitude`、`sys_status`、`sys_time`、`vision_pose_estimate`、`vision_speed_estimate`、`odom` 等插件。

如果你发现某个 `/uav1/mavros/*` 话题不存在，优先检查：

1. MAVROS 是否连接成功。
2. 插件是否在 whitelist 中。
3. 对应 MAVLink 数据流是否由 PX4 发送。
4. 话题名是否在 `/uav1` 命名空间下。

### Sunray 常用 MAVROS 话题

以 `agent_name=uav`、`agent_id=1` 为例。

| 话题 | 类型 | Sunray 用途 |
| --- | --- | --- |
| `/uav1/mavros/state` | `mavros_msgs/State` | 连接、解锁、模式、系统状态。 |
| `/uav1/mavros/extended_state` | `mavros_msgs/ExtendedState` | 落地状态、VTOL 状态。 |
| `/uav1/mavros/local_position/odom` | `nav_msgs/Odometry` | PX4/MAVROS 本地 odom，写入 `Px4State.local_pose/local_velocity`。 |
| `/uav1/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | 控制节点向 PX4 发布位置/速度/加速度/yaw setpoint。 |
| `/uav1/mavros/setpoint_raw/target_local` | `mavros_msgs/PositionTarget` | PX4/MAVROS 回显当前 local setpoint。 |
| `/uav1/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | 几何控制器或姿态控制器发布姿态/角速度/推力 setpoint。 |
| `/uav1/mavros/setpoint_raw/target_attitude` | `mavros_msgs/AttitudeTarget` | PX4/MAVROS 回显当前 attitude setpoint。 |
| `/uav1/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | 外部视觉/定位位姿输入 PX4。 |
| `/uav1/mavros/vision_speed/speed_twist` | `geometry_msgs/TwistStamped` | 外部视觉/定位速度输入 PX4。 |
| `/uav1/mavros/odometry/in` | `nav_msgs/Odometry` | 通过 MAVLink ODOMETRY 输入外部定位。 |
| `/uav1/mavros/rc/in` | `mavros_msgs/RCIn` | RC 通道和 RSSI，汇总到 `Px4State`。 |
| `/uav1/mavros/sys_status` | `mavros_msgs/SysStatus` | 电池、电流、电量比例、飞控负载。 |
| `/uav1/mavros/gpsstatus/gps1/raw` | `mavros_msgs/GPSRAW` | GPS fix、卫星数、经纬高。 |
| `/uav1/mavros/imu/data_raw` | `sensor_msgs/Imu` | 原始 IMU，VINS 等定位源可能使用。 |

### Sunray 常用 MAVROS 服务

| 服务 | 类型 | 用途 |
| --- | --- | --- |
| `/uav1/mavros/cmd/arming` | `mavros_msgs/CommandBool` | 解锁或上锁。 |
| `/uav1/mavros/set_mode` | `mavros_msgs/SetMode` | 切换 PX4 模式，例如 OFFBOARD。 |
| `/uav1/mavros/cmd/takeoff` | MAVROS command service | PX4 原生命令起飞，Sunray 控制节点不一定直接依赖它。 |
| `/uav1/mavros/cmd/land` | MAVROS command service | PX4 原生命令降落。 |
| `/uav1/mavros/cmd/command` | `mavros_msgs/CommandLong` | 发送通用 MAVLink command。 |
| `/uav1/mavros/param/get` | `mavros_msgs/ParamGet` | 读取 PX4 参数。 |
| `/uav1/mavros/param/set` | `mavros_msgs/ParamSet` | 写入 PX4 参数。 |
| `/uav1/mavros/param/pull` | MAVROS param service | 从飞控拉取参数缓存。 |
| `/uav1/mavros/param/push` | MAVROS param service | 推送参数缓存到飞控。 |

`tools/px4_param_check` 就是基于 MAVROS 参数服务实现 PX4 参数检查和写入的。

### MAVROS 标准话题分类

下面按 MAVROS 插件分类列出常见话题。注意：实际是否存在取决于 MAVROS 版本、plugin whitelist/blacklist、PX4 固件版本和飞控是否输出对应 MAVLink 数据。运行时以命令结果为准：

```bash
rostopic list | grep /uav1/mavros
rosservice list | grep /uav1/mavros
```

系统状态类：

| 插件 | 话题 | 方向 | 说明 |
| --- | --- | --- | --- |
| `sys_status` | `/uav1/mavros/state` | FCU -> ROS | MAVROS 连接、解锁、模式、手动输入状态。 |
| `sys_status` | `/uav1/mavros/extended_state` | FCU -> ROS | 起降状态、VTOL 状态。 |
| `sys_status` | `/uav1/mavros/sys_status` | FCU -> ROS | 电池、电源、系统负载等状态。 |
| `sys_status` | `/uav1/mavros/battery` | FCU -> ROS | 电池状态，是否存在取决于插件和版本。 |
| `sys_status` | `/uav1/mavros/estimator_status` | FCU -> ROS | EKF/估计器健康状态。 |
| `sys_status` | `/uav1/mavros/statustext/recv` | FCU -> ROS | 飞控状态文本。 |
| `sys_time` | `/uav1/mavros/time_reference` | FCU -> ROS | 飞控时间参考。 |
| `sys_time` | `/uav1/mavros/timesync_status` | FCU -> ROS | MAVLink timesync 状态。 |
| `vfr_hud` | `/uav1/mavros/vfr_hud` | FCU -> ROS | 空速、地速、高度、爬升率等 HUD 数据。 |
| `altitude` | `/uav1/mavros/altitude` | FCU -> ROS | 不同参考系高度。 |
| `home_position` | `/uav1/mavros/home_position/home` | FCU -> ROS | Home 点。 |

IMU、位置和 GPS 类：

| 插件 | 话题 | 方向 | 说明 |
| --- | --- | --- | --- |
| `imu` | `/uav1/mavros/imu/data` | FCU -> ROS | 融合后的 IMU/姿态数据。 |
| `imu` | `/uav1/mavros/imu/data_raw` | FCU -> ROS | 原始 IMU 数据。 |
| `imu` | `/uav1/mavros/imu/mag` | FCU -> ROS | 磁力计。 |
| `imu` | `/uav1/mavros/imu/static_pressure` | FCU -> ROS | 静压。 |
| `imu` | `/uav1/mavros/imu/diff_pressure` | FCU -> ROS | 差压。 |
| `local_position` | `/uav1/mavros/local_position/pose` | FCU -> ROS | 局部位姿。 |
| `local_position` | `/uav1/mavros/local_position/odom` | FCU -> ROS | 局部里程计。 |
| `local_position` | `/uav1/mavros/local_position/velocity_local` | FCU -> ROS | local frame 速度。 |
| `local_position` | `/uav1/mavros/local_position/velocity_body` | FCU -> ROS | body frame 速度。 |
| `global_position` | `/uav1/mavros/global_position/global` | FCU -> ROS | 融合后的全球定位。 |
| `global_position` | `/uav1/mavros/global_position/raw/fix` | FCU -> ROS | 原始 GPS fix。 |
| `global_position` | `/uav1/mavros/global_position/raw/gps_vel` | FCU -> ROS | 原始 GPS 速度。 |
| `global_position` | `/uav1/mavros/global_position/raw/satellites` | FCU -> ROS | 可见卫星数量。 |
| `global_position` | `/uav1/mavros/global_position/local` | FCU -> ROS | 全球定位转换到局部坐标。 |
| `gps_status` | `/uav1/mavros/gpsstatus/gps1/raw` | FCU -> ROS | GPS1 原始状态。 |
| `gps_status` | `/uav1/mavros/gpsstatus/gps2/raw` | FCU -> ROS | GPS2 原始状态，若有。 |

RC、手动控制和执行器类：

| 插件 | 话题 | 方向 | 说明 |
| --- | --- | --- | --- |
| `rc_io` | `/uav1/mavros/rc/in` | FCU -> ROS | RC 输入通道和 RSSI。 |
| `rc_io` | `/uav1/mavros/rc/out` | FCU -> ROS | PWM/舵机输出。 |
| `rc_io` | `/uav1/mavros/rc/override` | ROS -> FCU | RC override，谨慎使用。 |
| `manual_control` | `/uav1/mavros/manual_control/control` | ROS -> FCU | 手动控制输入。 |
| `actuator_control` | `/uav1/mavros/actuator_control` | ROS -> FCU | 直接执行器控制，Sunray 默认不建议任务层使用。 |

Offboard/setpoint 类：

| 插件 | 话题 | 方向 | 说明 |
| --- | --- | --- | --- |
| `setpoint_position` | `/uav1/mavros/setpoint_position/local` | ROS -> FCU | 局部位置 setpoint。 |
| `setpoint_position` | `/uav1/mavros/setpoint_position/global` | ROS -> FCU | 全球位置 setpoint。 |
| `setpoint_velocity` | `/uav1/mavros/setpoint_velocity/cmd_vel` | ROS -> FCU | 速度 setpoint。 |
| `setpoint_accel` | `/uav1/mavros/setpoint_accel/accel` | ROS -> FCU | 加速度或力 setpoint。 |
| `setpoint_attitude` | `/uav1/mavros/setpoint_attitude/attitude` | ROS -> FCU | 姿态 setpoint。 |
| `setpoint_attitude` | `/uav1/mavros/setpoint_attitude/cmd_vel` | ROS -> FCU | 姿态角速度 setpoint。 |
| `setpoint_attitude` | `/uav1/mavros/setpoint_attitude/thrust` | ROS -> FCU | 推力 setpoint。 |
| `setpoint_raw` | `/uav1/mavros/setpoint_raw/local` | ROS -> FCU | 原始局部 setpoint，Sunray PX4 原生控制器常用。 |
| `setpoint_raw` | `/uav1/mavros/setpoint_raw/global` | ROS -> FCU | 原始全球 setpoint。 |
| `setpoint_raw` | `/uav1/mavros/setpoint_raw/attitude` | ROS -> FCU | 原始姿态/角速度/推力 setpoint，Sunray 几何控制器常用。 |
| `setpoint_raw` | `/uav1/mavros/setpoint_raw/target_local` | FCU -> ROS | 飞控收到或回显的局部 target。 |
| `setpoint_raw` | `/uav1/mavros/setpoint_raw/target_attitude` | FCU -> ROS | 飞控收到或回显的姿态 target。 |
| `setpoint_trajectory` | `/uav1/mavros/setpoint_trajectory/local` | ROS -> FCU | 轨迹 setpoint。 |

外部定位输入类：

| 插件 | 话题 | 方向 | 说明 |
| --- | --- | --- | --- |
| `vision_pose_estimate` | `/uav1/mavros/vision_pose/pose` | ROS -> FCU | 外部视觉位姿。 |
| `vision_pose_estimate` | `/uav1/mavros/vision_pose/pose_cov` | ROS -> FCU | 带协方差外部视觉位姿。 |
| `vision_speed_estimate` | `/uav1/mavros/vision_speed/speed_twist` | ROS -> FCU | 外部视觉速度。 |
| `vision_speed_estimate` | `/uav1/mavros/vision_speed/speed_twist_cov` | ROS -> FCU | 带协方差外部视觉速度。 |
| `odom` | `/uav1/mavros/odometry/in` | ROS -> FCU | 外部里程计输入。 |
| `odom` | `/uav1/mavros/odometry/out` | FCU -> ROS | 飞控里程计输出。 |
| `mocap_pose_estimate` | `/uav1/mavros/mocap/pose` | ROS -> FCU | 动捕位姿。 |
| `fake_gps` | `/uav1/mavros/fake_gps/*` | ROS -> FCU | 用外部定位构造 fake GPS，取决于配置。 |

任务、命令、参数和其他插件：

| 插件 | 话题/服务 | 方向 | 说明 |
| --- | --- | --- | --- |
| `waypoint` | `/uav1/mavros/mission/*` | both/service | 航点任务拉取、上传、清空和到达通知。 |
| `geofence` | `/uav1/mavros/geofence/*` | service | 地理围栏读写，若插件启用。 |
| `rallypoint` | `/uav1/mavros/rallypoint/*` | service | rally point 读写，若插件启用。 |
| `command` | `/uav1/mavros/cmd/arming` | service | 解锁/上锁。 |
| `command` | `/uav1/mavros/cmd/command` | service | MAVLink command long。 |
| `command` | `/uav1/mavros/cmd/takeoff` | service | PX4 原生命令起飞。 |
| `command` | `/uav1/mavros/cmd/land` | service | PX4 原生命令降落。 |
| `sys_status` / `command` | `/uav1/mavros/set_mode` | service | 切换飞行模式。 |
| `param` | `/uav1/mavros/param/get` | service | 读取参数。 |
| `param` | `/uav1/mavros/param/set` | service | 写入参数。 |
| `param` | `/uav1/mavros/param/pull` | service | 拉取参数表。 |
| `param` | `/uav1/mavros/param/push` | service | 推送参数表。 |
| `distance_sensor` | `/uav1/mavros/distance_sensor/*` | both | 距离传感器。 |
| `landing_target` | `/uav1/mavros/landing_target/*` | both | 精准降落目标。 |
| `obstacle_distance` | `/uav1/mavros/obstacle/send` | ROS -> FCU | 避障距离输入。 |

### MAVROS、MAVLink、uORB 对照

MAVROS 并不是直接暴露 PX4 uORB。实际链路通常是：

```text
ROS topic/service
  -> MAVROS plugin
  -> MAVLink message
  -> PX4 mavlink 模块
  -> PX4 uORB / commander / estimator / controller
```

所以“uORB 对照”只能作为调试索引，不能理解成严格字段一一对应。不同 PX4 版本、MAVROS 版本和参数配置会影响实际路径。

FCU 输出到 ROS 的常见对照：

| 功能 | MAVROS 话题 | MAVLink 主要消息 | PX4/uORB 主要来源 |
| --- | --- | --- | --- |
| 连接/模式/解锁 | `/mavros/state` | `HEARTBEAT` | `vehicle_status`、`actuator_armed`、`manual_control_setpoint` |
| 起降状态 | `/mavros/extended_state` | `EXTENDED_SYS_STATE` | `vehicle_land_detected`、`vehicle_status` |
| 系统/电池状态 | `/mavros/sys_status` | `SYS_STATUS` | `battery_status`、系统状态 |
| 估计器状态 | `/mavros/estimator_status` | `ESTIMATOR_STATUS` | estimator/EKF 状态相关 uORB |
| 局部位姿/速度 | `/mavros/local_position/odom` | `LOCAL_POSITION_NED` / `ODOMETRY` | `vehicle_local_position`、`vehicle_odometry` |
| IMU/姿态 | `/mavros/imu/data` | `HIGHRES_IMU` / `ATTITUDE` / `ATTITUDE_QUATERNION` | `sensor_combined`、`vehicle_attitude`、`vehicle_angular_velocity` |
| GPS | `/mavros/gpsstatus/gps1/raw` | `GPS_RAW_INT` | `sensor_gps` / `vehicle_gps_position` |
| RC 输入 | `/mavros/rc/in` | `RC_CHANNELS` | `input_rc`、`manual_control_setpoint` |
| local setpoint 回显 | `/mavros/setpoint_raw/target_local` | target/setpoint 相关 MAVLink 数据 | `trajectory_setpoint` / `offboard_control_mode` 相关状态 |
| attitude setpoint 回显 | `/mavros/setpoint_raw/target_attitude` | target/setpoint 相关 MAVLink 数据 | `vehicle_attitude_setpoint`、`vehicle_rates_setpoint` 相关状态 |

ROS 输入到 FCU 的常见对照：

| 功能 | MAVROS 话题/服务 | MAVLink 主要消息 | PX4/uORB 主要落点 |
| --- | --- | --- | --- |
| 外部视觉位姿 | `/mavros/vision_pose/pose` | `VISION_POSITION_ESTIMATE` | `vehicle_visual_odometry`，供 EKF 融合 |
| 外部里程计 | `/mavros/odometry/in` | `ODOMETRY` | `vehicle_visual_odometry`，供 EKF 融合 |
| 位置/速度/加速度 setpoint | `/mavros/setpoint_raw/local` | `SET_POSITION_TARGET_LOCAL_NED` | `offboard_control_mode`、`trajectory_setpoint` |
| 姿态/角速度/推力 setpoint | `/mavros/setpoint_raw/attitude` | `SET_ATTITUDE_TARGET` | `vehicle_attitude_setpoint`、`vehicle_rates_setpoint` |
| 解锁/上锁 | `/mavros/cmd/arming` | `COMMAND_LONG` | `vehicle_command`，影响 `actuator_armed` |
| 模式切换 | `/mavros/set_mode` | `SET_MODE` / command | `vehicle_command`、`vehicle_status` |
| 参数读写 | `/mavros/param/get`、`/mavros/param/set` | `PARAM_REQUEST_READ`、`PARAM_SET` | PX4 参数系统 |

`mavros_msgs/PositionTarget` 是 MAVROS 对 `SET_POSITION_TARGET_LOCAL_NED` 的 ROS 封装，核心字段包括 `coordinate_frame`、`type_mask`、`position`、`velocity`、`acceleration_or_force`、`yaw` 和 `yaw_rate`。Sunray 的 PX4 原生控制器主要通过它发布位置/速度类目标。

`mavros_msgs/AttitudeTarget` 是 MAVROS 对 `SET_ATTITUDE_TARGET` 的 ROS 封装，核心字段包括 `type_mask`、`orientation`、`body_rate` 和 `thrust`。Sunray 的几何控制器主要通过它发布姿态/角速度/推力目标。

### MAVROS 与 Px4State

`common/sunray_msgs/msg/Px4State.msg` 是 Sunray 对 MAVROS 常用状态的二次封装。它的目的不是替代 MAVROS，而是降低上层模块订阅大量 `/mavros/*` 话题的成本。

典型关系：

```text
/uav1/mavros/state
/uav1/mavros/extended_state
/uav1/mavros/rc/in
/uav1/mavros/sys_status
/uav1/mavros/local_position/odom
/uav1/mavros/setpoint_raw/target_local
/uav1/mavros/setpoint_raw/target_attitude
/uav1/mavros/gpsstatus/gps1/raw
  -> sunray_uav_control::MavrosHelper
  -> /uav1/sunray/px4_state
```

开发建议：

- 任务层需要 PX4 状态时，优先订阅 `/uav1/sunray/px4_state`。
- 需要调试 MAVROS 插件或原始字段时，再直接 echo `/uav1/mavros/*`。
- 如果新增 `Px4State` 字段，需要同步修改消息定义、`MavrosHelper`、文档和可能的 UI/监控工具。

### 与控制命令的关系

Sunray 用户发布：

```text
/uav1/sunray/uav_control/control_cmd
```

控制节点内部根据控制器类型输出 MAVROS setpoint：

| Sunray 控制器 | 常见 MAVROS 输出 |
| --- | --- |
| PX4 原生控制器 | `/uav1/mavros/setpoint_raw/local` |
| Geometric Controller | `/uav1/mavros/setpoint_raw/attitude` |
| 外部定位输入 | `/uav1/mavros/vision_pose/pose`、`/uav1/mavros/vision_speed/speed_twist` 或 `/uav1/mavros/odometry/in` |

因此，对新手开发者来说，不建议一开始绕过 `sunray_uav_control` 直接控制 `/mavros/setpoint_raw/*`。直接发 MAVROS setpoint 时，你需要自己处理频率、模式切换、解锁、failsafe、坐标系和任务状态。

### 常见排查

| 现象 | 优先检查 |
| --- | --- |
| `/uav1/mavros/state.connected=false` | `fcu_url`、串口权限、波特率、飞控是否上电、USB/UDP 是否被占用。 |
| 没有 `/uav1/mavros/rc/in` | `rc_io` plugin 是否启用、PX4 是否收到 RC、遥控器接收机是否正常。 |
| 没有 local odom | `local_position` plugin、PX4 估计器是否有位置、EKF 是否健康。 |
| setpoint 发了但不动 | PX4 模式、解锁状态、Offboard setpoint 频率、坐标系、type_mask。 |
| 参数读写失败 | MAVROS 是否连接、参数名是否存在、PX4 是否允许写入、是否需要重启生效。 |
| 多机话题混乱 | namespace、sysid、UDP 端口、`agent_id` 是否一一对应。 |

### 官方资料

- MAVROS GitHub：https://github.com/mavlink/mavros
- ROS Wiki MAVROS：http://wiki.ros.org/mavros
- MAVROS Plugins：http://wiki.ros.org/mavros/Plugins
- MAVROS Noetic 消息 API：https://docs.ros.org/en/noetic/api/mavros_msgs/html/
- PX4 ROS/MAVROS 相关文档：https://docs.px4.io/main/en/ros/mavros_installation.html

</section>
