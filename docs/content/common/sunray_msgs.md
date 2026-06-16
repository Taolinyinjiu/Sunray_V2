<!-- title: sunray_msgs -->

<section id="common-sunray-msgs">

## sunray_msgs

`common/sunray_msgs` 是 Sunray_v2 最重要的接口包。定位、控制、规划、集群、系统工具之间尽量通过这里的 `.msg` 和 `.srv` 通信，而不是互相包含对方内部 C++ 类。

对二次开发者来说，最重要的原则是：如果你的节点要和 Sunray 其他模块交互，优先查这里是否已有消息能表达你的需求。

### 包结构

```text
common/sunray_msgs/
├── msg/
│   ├── Formation.msg
│   ├── OdomState.msg
│   ├── PlanningWaypoint.msg
│   ├── Px4State.msg
│   ├── SystemInfo.msg
│   ├── UAVControlCMD.msg
│   ├── UAVControlCommandStatus.msg
│   ├── UAVControlState.msg
│   ├── UAVPlanningCMD.msg
│   ├── UAVPlanningState.msg
│   ├── UAVSwarmCMD.msg
│   ├── UAVSwarmState.msg
│   ├── UGVControlCMD.msg
│   ├── UGVControlState.msg
│   ├── UGVSwarmCMD.msg
│   ├── UGVSwarmState.msg
│   └── Vector2.msg
└── srv/
    ├── GetFeatures.srv
    ├── ListFeatures.srv
    ├── StartFeature.srv
    └── StopFeature.srv
```

依赖关系包括 `std_msgs`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`、`geographic_msgs`、`mavros_msgs` 等。修改消息或服务后，需要重新编译 `sunray_msgs` 和所有依赖它的包。

### 分类总览

| 类别 | 消息/服务 | 用途 |
| --- | --- | --- |
| 基础类型 | `Vector2` | 二维向量，当前主要给 UAV 机体系 XY 位置/速度使用。 |
| 定位状态 | `OdomState` | `localization_fusion` 对外发布定位源、里程计、TF 和健康状态。 |
| PX4/MAVROS 状态 | `Px4State` | `sunray_uav_control` 汇总 MAVROS/PX4 状态、RC、GPS 和 setpoint 回显。 |
| UAV 控制 | `UAVControlCMD`、`UAVControlState`、`UAVControlCommandStatus` | 无人机控制命令、控制器状态和本地命令执行状态。 |
| UGV 控制 | `UGVControlCMD`、`UGVControlState` | 无人车点位/速度控制命令和状态反馈。 |
| 规划 | `PlanningWaypoint`、`UAVPlanningCMD`、`UAVPlanningState` | 规划模块目标点、规划命令和规划状态。 |
| 集群 | `Formation`、`UAVSwarmCMD`、`UAVSwarmState`、`UGVSwarmCMD`、`UGVSwarmState` | 多智能体集群命令、队形参数和集群状态。 |
| 系统管理 | `SystemInfo`、`StartFeature`、`StopFeature`、`GetFeatures`、`ListFeatures` | 系统功能启动/停止/查询和运行信息。 |

### Vector2.msg

```text
float32 x
float32 y
```

最小二维向量类型。当前主要用于：

- `UAVControlCMD.desired_body_xy_pos`
- `UAVControlCMD.desired_body_xy_vel`

它只表达二维数值，不携带 frame、单位、时间戳。具体语义由使用它的上层消息决定。

### OdomState.msg

典型话题：

```text
/uav1/sunray/localization/odom_state
/ugv1/sunray/localization/odom_state
```

发布者：`localization_fusion`。

用途：描述定位融合当前使用哪个定位源、订阅哪个外部 odom、当前定位是否有效、输出了哪些 local/global odom，以及 TF 缓存状态。

定位源枚举：

| 枚举 | 值 | 说明 |
| --- | ---: | --- |
| `VIOBOT` | `0` | VIOBOT 系列定位源。 |
| `MOCAP` | `1` | 动捕定位源。 |
| `VINS` | `2` | VINS 算法输出。 |
| `GAZEBO` | `3` | Gazebo 仿真。 |
| `GAZEBO_ARUCO` | `4` | Gazebo 中的 ArUco 重定位。 |
| `PENGYU_SIM` | `5` | Pengyu 仿真。 |
| `FASTLIO_EKF` | `6` | FAST-LIO + EKF 高频定位链路。 |

字段说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 状态消息时间戳。 |
| `external_source` | `uint8` | 当前定位源枚举。 |
| `subtopic_name_external_odom` | `string` | `localization_fusion` 当前订阅的外部 odom 话题。 |
| `odometry_valid` | `bool` | 外部 odom 是否在超时范围内，仅按时间戳/接收时间判断。 |
| `odometry_update_hz` | `float32` | 外部 odom 更新频率。 |
| `subtopic_name_external_relocalization` | `string` | 重定位话题名；为空表示没有配置重定位。 |
| `pubtopic_name_local_odom` | `string` | local odom 输出话题。 |
| `pubtopic_name_global_odom` | `string` | global odom 输出话题。 |
| `local_odom` | `nav_msgs/Odometry` | 当前局部里程计快照。 |
| `global_odom` | `nav_msgs/Odometry` | 当前全局里程计快照。 |
| `world_frame_name` | `string` | world frame 名。 |
| `global_frame_name` | `string` | `{agent}/sunray_global` frame 名。 |
| `local_frame_name` | `string` | `{agent}/sunray_local` frame 名。 |
| `base_frame_name` | `string` | `{agent}/base_link` frame 名。 |
| `world_to_global_tf` | `geometry_msgs/TransformStamped` | `world -> sunray_global`。 |
| `global_to_local_tf` | `geometry_msgs/TransformStamped` | `sunray_global -> sunray_local`。 |
| `local_to_base_tf` | `geometry_msgs/TransformStamped` | `sunray_local -> base_link`。 |

二次开发建议：

- 控制、规划、监控优先看 `odometry_valid` 和 `odometry_update_hz` 判断定位健康。
- 想知道当前到底接的是哪个定位源，看 `external_source` 和 `subtopic_name_external_odom`。
- 想调全局/局部坐标关系，看 `global_to_local_tf`。

### Px4State.msg

典型话题：

```text
/uav1/sunray/px4_state
```

发布者：`sunray_uav_control` 中的 `MavrosHelper`。

用途：把常用 MAVROS 状态、定位、RC、GPS、setpoint 回显整合到一个话题，方便地面站、监控工具、调试脚本和二次开发模块直接订阅。

注意：

- `flight_mode`、`landed_state`、`coordinate_frame`、`type_mask` 等枚举值保持 MAVROS/MAVLink 语义。
- ROS/MAVROS 侧常用 ENU 或 ROS frame；PX4/MAVLink 内部常用 NED，MAVROS 会做部分坐标转换。
- `rc_channels` 是原始通道数组，不在本消息中解释为 roll/pitch/throttle/yaw。通道含义由遥控器、接收机、PX4 `RC_MAP_*` 参数和 MAVROS 配置共同决定。

字段按来源分组：

| 字段 | 类型 | 来源 | 说明 |
| --- | --- | --- | --- |
| `connected` | `bool` | `/mavros/state` | MAVROS 是否与 PX4/FCU 建立连接。 |
| `rc_available` | `bool` | `/mavros/state.manual_input` | PX4/MAVROS 是否认为存在手动输入，不等价于 `rc_channels` 一定有效。 |
| `armed` | `bool` | `/mavros/state` | 飞控是否解锁。 |
| `flight_mode` | `uint8` | `/mavros/state.mode` | PX4 当前飞行模式，代码会将字符串转成内部枚举。 |
| `system_status` | `uint8` | `/mavros/state.system_status` | MAVLink `MAV_STATE`。 |
| `rc_channels` | `uint16[]` | `/mavros/rc/in.channels` | RC 原始通道值数组。 |
| `rc_rssi` | `uint8` | `/mavros/rc/in.rssi` | RC 信号强度。 |
| `landed_state` | `uint8` | `/mavros/extended_state` | 起降状态。 |
| `battery_voltage_v` | `float32` | `/mavros/sys_status` | 电池电压，单位 V。 |
| `battery_current_a` | `float32` | `/mavros/sys_status` | 电池电流，单位 A。 |
| `battery_percentage` | `float32` | `/mavros/sys_status` | 电量比例，通常 `[0, 1]`。 |
| `fcu_load` | `uint16` | `/mavros/sys_status` | 飞控负载，字段类型为历史兼容保留。 |
| `external_pose` | `geometry_msgs/Pose` | Sunray -> MAVROS | 控制器传给 MAVROS 的外部定位位姿快照。 |
| `external_velocity` | `geometry_msgs/Twist` | Sunray -> MAVROS | 控制器传给 MAVROS 的外部定位速度快照。 |
| `local_pose` | `geometry_msgs/Pose` | `/mavros/local_position/odom` | PX4/MAVROS 局部位姿。 |
| `local_velocity` | `geometry_msgs/Twist` | `/mavros/local_position/odom` | PX4/MAVROS 局部速度。 |
| `setpoint_coordinate_frame` | `uint8` | `/mavros/setpoint_raw/target_local` | `PositionTarget.coordinate_frame`。 |
| `setpoint_local_type_mask` | `uint16` | `/mavros/setpoint_raw/target_local` | `PositionTarget.type_mask`。 |
| `pos_setpoint` | `geometry_msgs/Vector3` | `/mavros/setpoint_raw/target_local` | 位置 setpoint，单位 m。 |
| `vel_setpoint` | `geometry_msgs/Vector3` | `/mavros/setpoint_raw/target_local` | 速度 setpoint，单位 m/s。 |
| `acc_setpoint` | `geometry_msgs/Vector3` | `/mavros/setpoint_raw/target_local` | 加速度或力 setpoint。 |
| `yaw_setpoint` | `float32` | `/mavros/setpoint_raw/target_local` | 偏航角，单位 rad。 |
| `yaw_rate_setpoint` | `float32` | `/mavros/setpoint_raw/target_local` | 偏航角速度，单位 rad/s。 |
| `setpoint_att_type_mask` | `uint16` | `/mavros/setpoint_raw/target_attitude` | `AttitudeTarget.type_mask`。 |
| `orientation_setpoint` | `geometry_msgs/Quaternion` | `/mavros/setpoint_raw/target_attitude` | 姿态 setpoint。 |
| `body_rate_setpoint` | `geometry_msgs/Vector3` | `/mavros/setpoint_raw/target_attitude` | 机体系角速度，单位 rad/s。 |
| `thrust_setpoint` | `float32` | `/mavros/setpoint_raw/target_attitude` | 归一化推力，典型范围 `[0, 1]`。 |
| `satellites` | `uint8` | `/mavros/gpsstatus/gps1/raw` | 可见卫星数量。 |
| `gps_status` | `int8` | `/mavros/gpsstatus/gps1/raw` | GPS fix type。 |
| `gps_service` | `uint8` | `/mavros/gpsstatus/gps1/raw` | 历史兼容字段，当前填入 fix type 的无符号副本。 |
| `latitude` | `float64` | GPSRAW | 纬度，单位 deg。 |
| `longitude` | `float64` | GPSRAW | 经度，单位 deg。 |
| `altitude` | `float64` | GPSRAW | 高度，单位 m。 |
| `latitude_raw` | `float64` | GPSRAW | 原始纬度整数值，未乘 `1e-7`。 |
| `longitude_raw` | `float64` | GPSRAW | 原始经度整数值，未乘 `1e-7`。 |
| `altitude_amsl` | `float64` | GPSRAW | AMSL 高度，单位 m。 |

### UAVControlCMD.msg

典型话题：

```text
/uav1/sunray/uav_control/control_cmd
```

发布者：任务节点、地面站、终端工具、规划模块、集群模块、示例程序。

订阅者：`sunray_uav_control`。

用途：无人机控制主输入接口。

命令来源枚举：

| 枚举 | 值 | 说明 |
| --- | ---: | --- |
| `UNDEFINE` | `0` | 未定义。 |
| `SUNRAY_STATION` | `1` | Sunray 地面站。 |
| `RC_CONTROLLER` | `2` | RC 遥控器控制。 |
| `TERMINAL` | `3` | 终端/TUI/命令行控制。 |
| `SWARM_CONTROL` | `4` | 集群控制模块。 |
| `PLANNING` | `5` | 规划模块。 |
| `EXAMPLE_DEMO` | `6` | 控制示例模块。 |

控制命令枚举和发布频率：

| 命令 | 值 | 发布规则 | 主要字段 |
| --- | ---: | --- | --- |
| `TAKEOFF` | `1` | 发一次即可 | `takeoff_relative_height`、`takeoff_max_velocity` 可选覆盖配置。 |
| `LAND` | `2` | 发一次即可 | `land_max_velocity` 可选覆盖配置。 |
| `RETURN` | `3` | 发一次即可 | 返回起始点并降落。 |
| `KILL` | `4` | 发一次即可 | 紧急锁桨，危险操作。 |
| `HOVER` | `5` | 发一次即可 | 读取当前位置并悬停。 |
| `MOVE_POINT` | `6` | 发一次即可 | `desired_pos`、`yaw_mode`、`desired_yaw`。 |
| `MOVE_VELOCITY` | `7` | 需要持续发布，建议 `>=10 Hz` | `desired_vel`、`fixed_height`、yaw 字段。 |
| `MOVE_TRAJECTORY` | `8` | 需要持续发布，建议 `>=100 Hz` 或与规划器一致 | `desired_pos`、`desired_vel`、`desired_acc`、`desired_jerk`、yaw 字段。 |
| `MOVE_POINT_BODY` | `9` | 发一次即可 | `desired_body_xy_pos`、`fixed_height`、`desired_yaw`。 |
| `MOVE_VELOCITY_BODY` | `10` | 需要持续发布，建议 `>=10 Hz` | `desired_body_xy_vel`、`fixed_height`、yaw 字段。 |
| `MOVE_POINT_WGS84` | `11` | 发一次即可 | `desired_wgs84_pos`，当前暂未完整支持。 |

字段说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 命令时间戳和 frame。 |
| `cmd_source` | `uint8` | 命令来源枚举。 |
| `control_cmd` | `uint8` | 控制命令枚举。 |
| `desired_pos` | `geometry_msgs/Vector3` | 惯性系目标位置，单位 m。 |
| `desired_vel` | `geometry_msgs/Vector3` | 惯性系目标速度，单位 m/s。 |
| `desired_acc` | `geometry_msgs/Vector3` | 惯性系目标加速度，单位 m/s^2。 |
| `desired_jerk` | `geometry_msgs/Vector3` | jerk，主要为轨迹接口预留。 |
| `desired_body_xy_pos` | `Vector2` | 机体系水平位置偏移。 |
| `desired_body_xy_vel` | `Vector2` | 机体系水平速度。 |
| `fixed_height` | `float32` | body 控制必须填写；速度控制中大于 0 时用于锁高。 |
| `takeoff_relative_height` | `float32` | 起飞高度覆盖值，`>0` 生效。 |
| `takeoff_max_velocity` | `float32` | 起飞速度覆盖值，`>0` 生效。 |
| `land_max_velocity` | `float32` | 降落速度覆盖值，`>0` 生效。 |
| `desired_wgs84_pos` | `geographic_msgs/GeoPoint` | 经纬高目标。 |
| `yaw_mode` | `uint8` | yaw 控制模式。 |
| `desired_yaw` | `float32` | 目标 yaw，单位 rad。 |
| `desired_yaw_rate` | `float32` | 目标 yaw rate，单位 rad/s。 |
| `tracking_token` | `uint64` | 协议无关的本地命令跟踪标识；由 bridge 或其他上游发布者生成，供本地控制链路和本地状态回传关联同一条命令。 |

Yaw 模式：

| 枚举 | 值 | 说明 |
| --- | ---: | --- |
| `KEEP_YAW` | `0` | 保持当前 yaw。 |
| `SET_YAW` | `1` | 控制到指定 yaw。 |
| `SET_YAWRATE` | `2` | 控制 yaw rate。 |

补充说明：

- `UAVControlCMD` 现在只保留本地控制语义，不再承载 YunLink 协议字段。
- 如果上层需要携带 `session_id / message_id / correlation_id` 这类协议追踪信息，应改走 `common/yunlink_msgs` 中的 `CommandMeta.msg`。

### UAVControlState.msg

典型话题：

```text
/uav1/sunray/uav_control/control_state
```

发布者：`sunray_uav_control`。

用途：反馈 UAV 控制模块完整运行状态。

控制状态枚举：

| 枚举 | 值 | 说明 |
| --- | ---: | --- |
| `OFF` | `0` | 当前不满足正常工作条件。 |
| `INIT` | `1` | 初始化完成，可等待起飞。 |
| `TAKEOFF` | `2` | 起飞中。 |
| `HOVER` | `3` | 悬停中。 |
| `RETURN` | `4` | 返航中。 |
| `LAND` | `5` | 降落中。 |
| `MOVE` | `6` | 执行移动/速度/轨迹命令。 |
| `EMERGENCY_KILL` | `7` | 紧急锁桨状态。 |

字段说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `agent_name` | `string` | 机器人名称前缀，例如 `uav`。 |
| `agent_id` | `uint8` | 机器人编号。 |
| `controller_types` | `uint8` | 控制器类型，当前常见为 PX4 原生或几何控制器。 |
| `takeoff_relative_height` | `float64` | 当前生效起飞相对高度。 |
| `takeoff_max_velocity` | `float64` | 当前生效起飞最大速度。 |
| `land_type` | `uint8` | 当前降落类型。 |
| `land_max_velocity` | `float64` | 当前生效降落最大速度。 |
| `home_point` | `geometry_msgs/Vector3` | 当前返航目标点。 |
| `control_state` | `uint8` | 当前 FSM 状态。 |
| `last_cmd` | `UAVControlCMD` | 当前执行或最近收到的控制指令。 |
| `self_odom` | `nav_msgs/Odometry` | 原始本机里程计。 |
| `odometry_lost` | `bool` | 里程计是否丢失。 |
| `odometry_valid` | `bool` | 里程计是否通过基本合法性检查。 |
| `controller_output_type` | `uint8` | 底层控制输出类型。 |
| `position_target` | `mavros_msgs/PositionTarget` | 最近一次发布的 MAVROS 位置 setpoint。 |
| `attitude_target` | `mavros_msgs/AttitudeTarget` | 最近一次发布的 MAVROS 姿态 setpoint。 |

输出类型：

| 枚举 | 值 | 说明 |
| --- | ---: | --- |
| `OUTPUT_NONE` | `0` | 无输出。 |
| `OUTPUT_POSITION_TARGET` | `1` | 发布 `mavros_msgs/PositionTarget`。 |
| `OUTPUT_ATTITUDE_TARGET` | `2` | 发布 `mavros_msgs/AttitudeTarget`。 |

### UAVControlCommandStatus.msg

典型话题：

```text
/uav1/sunray/uav_control/command_status_local
```

发布者：`sunray_uav_control`。

用途：描述本地控制链路里一条 UAV 控制命令的执行生命周期。它只表达控制侧事实，不携带 YunLink 会话、消息号、关联号等协议字段。

边界说明：

- `tracking_token` 是这条本地命令的身份标识，用来把 `UAVControlCMD` 和本地执行状态关联起来。
- 如果需要对外发布协议侧兼容状态，应由 `yunlink_ros_bridge` 把本消息映射为 `yunlink_msgs/CommandExecutionStatus`，再发布到 `/uavX/yunlink/command_execution_status`。
- `sunray_uav_control` 不应直接依赖 `yunlink_msgs` 或 YunLink runtime。

命令类型：

| 枚举 | 值 | 说明 |
| --- | ---: | --- |
| `COMMAND_UNKNOWN` | `0` | 未知命令。 |
| `COMMAND_TAKEOFF` | `1` | 起飞。 |
| `COMMAND_LAND` | `2` | 降落。 |
| `COMMAND_RETURN` | `3` | 返航。 |
| `COMMAND_GOTO` | `4` | 去目标点。 |
| `COMMAND_VELOCITY_SETPOINT` | `5` | 速度设定。 |

执行状态：

| 枚举 | 值 | 说明 |
| --- | ---: | --- |
| `IDLE` | `0` | 空闲。 |
| `ACCEPTED` | `1` | 命令已接受。 |
| `RUNNING` | `2` | 正在执行。 |
| `WAITING_PHYSICAL_STATE` | `3` | 等待实际物理状态满足条件。 |
| `SUCCEEDED` | `4` | 成功。 |
| `FAILED` | `5` | 失败。 |
| `CANCELLED` | `6` | 已取消。 |
| `TIMEOUT` | `7` | 超时。 |

字段说明：

| 字段 | 类型 | 说明 |
| --- | --- |
| `header` | `std_msgs/Header` | 状态时间戳。 |
| `agent_name` / `agent_id` | `string` / `uint8` | 命令所属智能体。 |
| `tracking_token` | `uint64` | 本地命令跟踪标识，和 `UAVControlCMD.tracking_token` 对应。 |
| `command_kind` | `uint8` | 本地归一化命令类型。 |
| `execution_state` | `uint8` | 当前执行状态。 |
| `progress_percent` | `uint8` | 进度百分比。 |
| `active` | `bool` | 是否仍在执行。 |
| `terminal` | `bool` | 是否已进入终态。 |
| `success` | `bool` | 是否成功。 |
| `result_code` | `uint16` | 结果码，便于程序做细分判断。 |
| `detail` | `string` | 详细文本说明。 |
| `control_state` | `uint8` | 当前 UAV 控制状态。 |
| `px4_landed_state` | `uint8` | PX4 起降状态。 |
| `ready_for_takeoff` | `bool` | 当前是否满足起飞条件。 |
| `ready_for_land` | `bool` | 当前是否满足降落条件。 |
| `busy_reason` | `string` | 忙碌、拒绝或等待原因。 |

### UGVControlCMD.msg

典型话题：

```text
/ugv1/sunray/ugv_control/control_cmd
```

发布者：上层程序、控制面板、规划器、示例程序。

订阅者：`sunray_ugv_control`。

命令来源：

| 枚举 | 值 | 说明 |
| --- | ---: | --- |
| `UNDEFINE` | `0` | 未定义。 |
| `SUNRAY_STATION` | `1` | 地面站/Qt 面板。 |
| `RC_CONTROLLER` | `2` | RC 控制。 |
| `TERMINAL` | `3` | 终端工具。 |
| `CONTROL_CMD` | `4` | Sunray 内部控制模块。 |
| `SWARM_CONTROL` | `5` | 集群控制模块。 |
| `PLANNING` | `6` | 规划模块。 |
| `EXAMPLE_DEMO` | `7` | 示例程序。 |

控制命令：

| 命令 | 值 | 发布规则 | 说明 |
| --- | ---: | --- | --- |
| `HOLD` | `1` | 发一次即可 | 停车，控制器持续发布零速度。 |
| `MOVE_POINT` | `3` | 发一次即可 | 点位控制，读取 `desired_pos.x/y` 和 `desired_yaw`。 |
| `MOVE_VELOCITY` | `4` | 需要持续发布 | 世界系速度，适合全向底盘，差速底盘默认不支持。 |
| `MOVE_VELOCITY_BODY` | `5` | 需要持续发布 | 车体系速度，差速底盘只用 `linear.x` 和 `angular.z`。 |
| `MOVE_WGS84` | `6` | 预留 | 经纬高控制，当前 UGV 控制器暂不执行。 |

字段说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 建议填写当前 ROS 时间；速度类指令用 stamp 做超时保护。 |
| `cmd_source` | `uint8` | 命令来源。 |
| `control_cmd` | `uint8` | 控制命令。 |
| `desired_pos` | `geometry_msgs/Point` | `MOVE_POINT` 目标点，UGV 主要使用 x/y。 |
| `desired_vel` | `geometry_msgs/Vector3` | `MOVE_VELOCITY` 世界系速度。 |
| `cmd_vel` | `geometry_msgs/Twist` | `MOVE_VELOCITY_BODY` 车体系速度，语义与底盘 `/cmd_vel` 一致。 |
| `desired_yaw` | `float64` | 世界系目标 yaw，单位 rad。 |
| `desired_wgs84_pos` | `geographic_msgs/GeoPoint` | 经纬高目标，当前预留。 |

### UGVControlState.msg

典型话题：

```text
/ugv1/sunray/ugv_control/control_state
```

发布者：`sunray_ugv_control`。

用途：反馈无人车控制状态、诊断、目标点、实际 `cmd_vel` 和地理围栏状态。

底盘类型：

| 枚举 | 值 | 说明 |
| --- | ---: | --- |
| `DRIVE_UNKNOWN` | `0` | 未知底盘。 |
| `DRIVE_MECANUM` | `1` | 麦克纳姆轮。 |
| `DRIVE_DIFFERENTIAL` | `2` | 差速轮。 |

诊断等级：

| 枚举 | 值 | 说明 |
| --- | ---: | --- |
| `DIAGNOSTIC_OK` | `0` | 正常。 |
| `DIAGNOSTIC_WARN` | `1` | 告警，可能降级执行。 |
| `DIAGNOSTIC_ERROR` | `2` | 错误，命令被拒绝或已切 HOLD。 |

FSM 状态：

| 枚举 | 值 | 说明 |
| --- | ---: | --- |
| `FSM_INIT` | `0` | 初始化或未收到有效输入。 |
| `FSM_HOLD` | `1` | 停车。 |
| `FSM_MOVE` | `3` | 执行点位/速度/WGS84 命令。 |

字段说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `agent_name` / `agent_id` | `string` / `uint8` | 车辆身份。 |
| `drive_type` | `uint8` | 底盘类型。 |
| `control_cmd_valid` | `bool` | 是否已收到有效控制命令。 |
| `inside_geo_fence` | `bool` | 当前是否在地理围栏内；无 odom 时默认 true。 |
| `diagnostic_level` | `uint8` | 诊断等级。 |
| `diagnostic_msg` | `string` | 命令拒绝、降级、自动 HOLD 等原因。 |
| `fsm_state` | `uint8` | UGV 控制状态。 |
| `active_ugv_control_cmd` | `UGVControlCMD` | 最近一次有效命令。 |
| `self_odom` | `nav_msgs/Odometry` | 本车里程计。 |
| `odom_valid` | `bool` | 是否收到定位/里程计。 |
| `target_valid` | `bool` | 当前是否有明确点位目标。 |
| `target_pos` | `geometry_msgs/Point` | 当前目标点。 |
| `target_yaw` | `float64` | 当前目标 yaw。 |
| `controller_cmd_vel` | `geometry_msgs/Twist` | 实际发布到底盘的速度。 |
| `geo_fence_min/max` | `geometry_msgs/Point` | 地理围栏边界。 |

### PlanningWaypoint.msg

用途：规划模块的单个航点数据类型。

字段说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `position` | `geometry_msgs/Point` | 航点位置。 |
| `yaw` | `float32` | 航点目标 yaw。 |
| `hold_time` | `float32` | 多航点任务中，到达该点后悬停多久再去下一个点。 |

当前备注：FSM 下发指令到 planner 时复用这个消息，但会忽略 `hold_time`。

### UAVPlanningCMD.msg

典型话题：

```text
/uav1/sunray/uav_planning/planning_cmd
```

发布者：RViz goal bridge、上层任务、规划示例。

订阅者：`sunray_planning`。

用途：规划模块上层入口。

命令枚举：

| 命令 | 值 | 说明 |
| --- | ---: | --- |
| `TAKEOFF` | `1` | 起飞。 |
| `LAND` | `2` | 降落。 |
| `HOVER` | `3` | 打断当前运动并悬停。 |
| `PLAN_RETURN` | `4` | 通过规划方式回到 home 点。 |
| `PLAN_LOCAL_GOAL` | `5` | 局部规划到某一点。 |
| `PLAN_GLOBAL_GOAL` | `6` | 全局规划到某一点。 |

字段说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 命令时间戳和 frame。 |
| `plan_cmd_source` | `string` | 命令来源展示字段，不参与优先级判断。 |
| `plan_cmd` | `uint8` | 规划命令。 |
| `waypoints` | `PlanningWaypoint[]` | 单航点任务 size=1，多航点任务 size>1。 |

当前行为：如果正在执行多航点任务时收到新 `UAVPlanningCMD`，会抛弃原任务转而执行新任务。

### UAVPlanningState.msg

典型话题：

```text
/uav1/sunray/uav_planning/planning_state
```

发布者：`sunray_planning`。

用途：反馈规划状态机和当前规划命令。

状态枚举：

| 枚举 | 值 | 说明 |
| --- | ---: | --- |
| `OFF` | `0` | 当前不满足 planning 基本运行条件。 |
| `INIT` | `1` | planning 正常启动，可接收命令。 |
| `TAKEOFF` | `2` | 同步 UAV 控制起飞状态。 |
| `LAND` | `3` | 同步 UAV 控制降落状态。 |
| `PLANNING` | `4` | 正在移动/规划。 |
| `ARRIVED` | `5` | 到达目标点。 |
| `PLAN_FAILED` | `6` | 目标点无法到达或被外部指令打断。 |

字段说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `planner_type_string` | `string` | 当前交互的规划器名称，例如 `ego`、`diff`。 |
| `sunray_planning_state` | `uint8` | 当前规划状态。 |
| `home_point` | `geometry_msgs/Vector3` | 规划模块 home 点。 |
| `planning_cmd` | `UAVPlanningCMD` | 当前或最近规划命令。 |

### Formation.msg

用途：描述 UAV/UGV 集群编队目标。只在 swarm 命令为 `SWARM_FORMATION` 时使用。

阵型类型：

| 枚举 | 值 | 说明 |
| --- | ---: | --- |
| `STATIC_KEEP_FORMATION` | `0` | 抓拍当前相对位置关系，并整体平移到虚拟 leader 附近。 |
| `STATIC_FORMATION_LINE` | `1` | 惯性系直线阵型，方向由 `static_line_angle` 指定。 |
| `STATIC_FORMATION_POLYGON` | `2` | 正 N 边形顶点阵型，N=集群数量。 |
| `STATIC_FORMATION_RANDOM` | `3` | 场地安全范围内随机生成目标点。 |
| `STATIC_FORMATION_CUSTOM` | `9` | 外部自定义每个智能体相对 leader 的位置/yaw 偏移。 |
| `DYNAMIC_FORMATION_RING` | `11` | 圆环动态编队。 |
| `DYNAMIC_FORMATION_POLYGON` | `12` | 正 N 边形边线动态运动。 |
| `DYNAMIC_FORMATION_LEMNISCATE` | `13` | 8 字形动态轨迹。 |

通用字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `formation_type` | `uint8` | 阵型类型。 |
| `leader_pos` | `geometry_msgs/Point` | 虚拟 leader 目标位置，世界坐标系。 |
| `leader_yaw` | `float32` | leader 目标 yaw；动态圆环中也作为初始相位偏置。 |
| `dynamic_time` | `float32` | 动态阵型持续时间，单位 s。 |

静态阵型字段：

| 字段 | 说明 |
| --- | --- |
| `static_line_spacing` | 直线阵型相邻间距，单位 m。 |
| `static_line_angle` | 直线阵型与世界 X 轴夹角，单位 deg。 |
| `static_polygon_spacing` | 正 N 边形相邻边长，单位 m。 |
| `custom_offsets_pos` | 自定义阵型位置偏移，`i` 对应 `agent_id=i+1`。 |
| `custom_offsets_yaw` | 自定义阵型 yaw 偏移。 |

动态阵型字段：

| 字段 | 说明 |
| --- | --- |
| `dynamic_ring_radius` | 圆环半径，单位 m。 |
| `dynamic_ring_move_speed` | 圆环切向速度，正值逆时针。 |
| `dynamic_polygon_spacing` | 动态正 N 边形边长，单位 m。 |
| `dynamic_polygon_move_speed` | 沿正 N 边形周长循环运动速度。 |
| `dynamic_lemniscate_x_radius` | 8 字轨迹 X 方向尺度。 |
| `dynamic_lemniscate_y_radius` | 8 字轨迹 Y 方向尺度。 |
| `dynamic_lemniscate_move_speed` | 8 字轨迹相位推进等效速度。 |

### UAVSwarmCMD.msg

用途：无人机集群控制命令。

命令来源：

| 枚举 | 值 | 说明 |
| --- | ---: | --- |
| `UNDEFINE` | `0` | 未定义。 |
| `GROUND_STATION` | `1` | 地面站。 |
| `TERMINAL` | `2` | 终端工具。 |

集群命令：

| 命令 | 值 | 说明 |
| --- | ---: | --- |
| `SWARM_TAKEOFF` | `1` | 集群起飞。 |
| `SWARM_LAND` | `2` | 集群降落。 |
| `SWARM_HOVER` | `3` | 集群悬停。 |
| `SWARM_RETURN` | `4` | 集群返航。 |
| `SWARM_FORMATION` | `5` | 集群编队，配合 `formation_cmd`。 |

字段说明：

| 字段 | 说明 |
| --- | --- |
| `agent_id` | 指定响应智能体；`99` 表示所有智能体响应。 |
| `swarm_cmd` | 集群命令。 |
| `formation_cmd` | 编队参数。 |

### UAVSwarmState.msg

典型话题：

```text
/sunray/swarm/uav_swarm_state
```

用途：UAV 集群状态反馈。

状态枚举：

| 枚举 | 值 | 说明 |
| --- | ---: | --- |
| `INIT` | `0` | 初始化。 |
| `TAKEOFF` | `1` | 起飞。 |
| `LAND` | `2` | 降落。 |
| `RETURN_HOME` | `3` | 返航。 |
| `ARRIVED` | `4` | 到达。 |
| `SWARM_STATIC_FORMATION` | `5` | 静态编队控制。 |
| `SWARM_DYNAMIC_FORMATION` | `6` | 动态编队控制。 |
| `SWARM_DYNAMIC_FORMATION_PREPARE` | `7` | 动态编队初始就位阶段。 |

字段说明：

| 字段 | 说明 |
| --- | --- |
| `agent_id` / `swarm_num` | 本机 ID 和集群数量。 |
| `self_odom` | 本机里程计。 |
| `self_odom_ready` / `peers_odom_ready` | 本机/其他成员 odom 是否就绪。 |
| `ready_peer_num` | 已就绪成员数量。 |
| `swarm_cmd` | 当前执行的集群命令。 |
| `fsm_state` | 当前集群状态。 |
| `target_valid` | 当前是否有明确目标点。 |
| `target_pos` / `target_yaw` | 当前集群控制目标点。 |
| `uav_cmd` | 当前发布给无人机控制器的 `UAVControlCMD`。 |

### UGVSwarmCMD.msg

用途：无人车集群控制命令。

命令来源：

| 枚举 | 值 | 说明 |
| --- | ---: | --- |
| `UNDEFINE` | `0` | 未定义。 |
| `GROUND_STATION` | `1` | 地面站。 |
| `TERMINAL` | `2` | 终端工具。 |

集群命令：

| 命令 | 值 | 说明 |
| --- | ---: | --- |
| `SWARM_HOLD` | `1` | 集群停车/保持。 |
| `SWARM_RETURN` | `2` | 集群返航。 |
| `SWARM_FORMATION` | `3` | 集群编队。 |

字段说明：

| 字段 | 说明 |
| --- | --- |
| `agent_id` | 指定响应车辆；`99` 表示所有车辆响应。 |
| `swarm_cmd` | UGV 集群命令。 |
| `formation_cmd` | 编队参数。 |

### UGVSwarmState.msg

典型话题：

```text
/sunray/swarm/ugv_swarm_state
```

用途：UGV 集群状态反馈。

状态枚举：

| 枚举 | 值 | 说明 |
| --- | ---: | --- |
| `INIT` | `0` | 初始化。 |
| `RETURN_HOME` | `1` | 返航。 |
| `ARRIVED` | `2` | 到达。 |
| `SWARM_STATIC_FORMATION` | `3` | 静态编队。 |
| `SWARM_DYNAMIC_FORMATION` | `4` | 动态编队。 |
| `SWARM_DYNAMIC_FORMATION_PREPARE` | `5` | 动态编队初始就位。 |

字段说明：

| 字段 | 说明 |
| --- | --- |
| `agent_id` / `swarm_num` | 本车 ID 和集群数量。 |
| `self_odom` | 本车里程计。 |
| `self_odom_ready` / `peers_odom_ready` | 本车/其他成员 odom 是否就绪。 |
| `ready_peer_num` | 已就绪成员数量。 |
| `swarm_cmd` | 当前执行的 UGV 集群命令。 |
| `fsm_state` | 当前 UGV 集群状态。 |
| `target_valid` | 当前是否有目标点。 |
| `target_pos` / `target_yaw` | 当前目标点和 yaw。 |
| `ugv_cmd` | 当前发布给无人车控制器的 `UGVControlCMD`。 |

### SystemInfo.msg

用途：系统状态信息，适合系统监控、地面站和功能管理工具使用。

字段说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 时间戳。 |
| `airframe_type` | `string` | 当前机型/平台类型。 |
| `cpu_percent` | `float32` | CPU 使用率百分比。 |
| `memory_percent` | `float32` | 内存使用率百分比。 |
| `active_ros_nodes` | `string[]` | 当前活跃 ROS 节点列表。 |

### StartFeature.srv

用途：启动某个系统功能 feature。

请求：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `feature_name` | `string` | 要启动的功能名称。 |
| `override_args` | `string[]` | 启动时覆盖参数。 |
| `restart_if_running` | `bool` | 如果已运行，是否重启。 |
| `start_with_terminal` | `bool` | 是否在终端中启动。 |

响应：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `success` | `bool` | 是否启动成功。 |
| `message` | `string` | 结果说明。 |

### StopFeature.srv

用途：停止某个系统功能 feature。

请求：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `feature_name` | `string` | 要停止的功能名称。 |
| `force` | `bool` | 是否强制停止。 |

响应：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `success` | `bool` | 是否停止成功。 |
| `message` | `string` | 结果说明。 |

### ListFeatures.srv

用途：列出系统可用功能。

请求为空。

响应：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `feature_names` | `string[]` | 可用功能名称列表。 |

### GetFeatures.srv

用途：查询某个 feature 的详细信息。

请求：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `feature_name` | `string` | 要查询的功能名称。 |

响应：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `success` | `bool` | 查询是否成功。 |
| `message` | `string` | 查询结果说明。 |
| `name` | `string` | feature 名称。 |
| `group` | `string` | feature 所属分组。 |
| `running` | `bool` | 当前是否运行。 |
| `description` | `string` | 功能描述。 |
| `auto_start` | `bool` | 是否自动启动。 |
| `depends_on` | `string[]` | 依赖的其他 feature。 |
| `stop_timeout_sec` | `float32` | 停止超时时间。 |
| `start_preview_units` | `string[]` | 启动预览 unit 列表。 |
| `start_preview_commands` | `string[]` | 启动预览命令列表。 |

</section>
