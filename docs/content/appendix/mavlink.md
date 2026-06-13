<!-- title: MAVLink -->

<section id="appendix-mavlink">

## MAVLink

MAVLink 是飞控、地面站、伴随计算机和外设之间常用的轻量级通信协议。PX4 与 MAVROS 之间的通信本质上就是 MAVLink 消息流。

在 Sunray_v2 中，普通二次开发者通常不需要直接拼 MAVLink 包。你发布的是 `sunray_msgs/UAVControlCMD`，控制节点再调用 MAVROS topic/service，MAVROS 最后把 ROS 消息转换成 MAVLink 发送给 PX4。

推荐理解为：

```text
Sunray ROS 消息
  -> MAVROS ROS topic/service
  -> MAVLink message
  -> PX4 uORB / commander / controller
```

### MAVLink 解决什么问题

MAVLink 负责在不同设备之间传递结构化消息。它不规定你的任务怎么写，也不直接提供 ROS 节点，而是定义了消息格式、消息 ID、字段、坐标系枚举、命令枚举和序列化方式。

常见使用对象：

| 对象 | 通过 MAVLink 做什么 |
| --- | --- |
| QGroundControl | 参数配置、模式切换、任务上传、日志下载、状态显示。 |
| MAVROS | 把 ROS topic/service 转成 MAVLink，与 PX4/ArduPilot 通信。 |
| 伴随计算机程序 | 直接用 MAVSDK、pymavlink 或自定义程序控制飞控。 |
| PX4 | 输出状态、接收 setpoint、处理命令、广播 telemetry。 |

Sunray 选择 MAVROS 作为 ROS 桥，因此大部分 MAVLink 细节被 MAVROS 封装起来。

### 基本概念

| 概念 | 说明 |
| --- | --- |
| Message | MAVLink 的基础通信单元，例如 HEARTBEAT、RC_CHANNELS、ODOMETRY。 |
| Command | 一类特殊消息，常通过 `COMMAND_LONG` 或 `COMMAND_INT` 携带，例如解锁、设置模式、返航。 |
| Dialect | 消息定义集合，例如 `common.xml`、`development.xml`，不同飞控可扩展自己的 dialect。 |
| System ID | 系统编号，通常一架飞机一个 sysid。 |
| Component ID | 同一系统内组件编号，例如 autopilot、camera、gimbal。 |
| Frame | 坐标系枚举，例如 LOCAL_NED、BODY_NED、GLOBAL。 |
| MAVLink 2 | MAVLink 新版本，支持扩展字段、签名、更大的消息 ID 空间。 |

MAVLink 包通常包含消息 ID、系统 ID、组件 ID、序列号、payload、CRC 等内容。MAVLink 2 还可以启用签名，用于防止链路中的未授权注入。

### Sunray 常见 MAVLink 消息

下面列的是 Sunray/PX4/MAVROS 链路中最容易遇到的 MAVLink 消息类别。开发时多数情况下看到的是 MAVROS topic，而不是原始 MAVLink。

| MAVLink 消息 | MAVROS/Sunray 侧表现 | 用途 |
| --- | --- | --- |
| `HEARTBEAT` | `/uav1/mavros/state` | 连接状态、模式、解锁状态、系统类型。 |
| `EXTENDED_SYS_STATE` | `/uav1/mavros/extended_state` | 落地/飞行状态。 |
| `SYS_STATUS` | `/uav1/mavros/sys_status`，`Px4State` 电池字段 | 电池、电流、系统负载、传感器健康概览。 |
| `RC_CHANNELS` | `/uav1/mavros/rc/in` | 遥控器通道原始值和 RSSI。 |
| `LOCAL_POSITION_NED` / `ODOMETRY` | `/uav1/mavros/local_position/odom` | 本地位置、速度、姿态。 |
| `SET_POSITION_TARGET_LOCAL_NED` | `/uav1/mavros/setpoint_raw/local` | 位置、速度、加速度、yaw/yaw rate setpoint。 |
| `SET_ATTITUDE_TARGET` | `/uav1/mavros/setpoint_raw/attitude` | 姿态、角速度、推力 setpoint。 |
| `COMMAND_LONG` / `COMMAND_INT` | `/uav1/mavros/cmd/*` 服务 | 解锁、起飞、降落、设置 home、模式相关命令。 |
| `GPS_RAW_INT` | `/uav1/mavros/gpsstatus/gps1/raw` | GPS fix、卫星数、经纬高。 |
| `PARAM_VALUE` / 参数协议 | `/uav1/mavros/param/*` 服务 | 读取和写入 PX4 参数。 |

### 坐标系：NED 与 ENU

这是 MAVLink 接入 ROS 时最容易出错的地方。

| 系统 | 常用坐标约定 |
| --- | --- |
| PX4/MAVLink | NED：x 北、y 东、z 下。 |
| ROS | ENU：x 东、y 北、z 上。 |
| Sunray 控制接口 | 面向 ROS 侧开发者，通常按 ROS/ENU 语义理解。 |

MAVROS 会对许多常用话题做坐标转换，例如 local position、vision pose、setpoint_raw 等。但是不是所有字段都能凭直觉判断方向，尤其是 raw setpoint、body frame、yaw、外部定位输入和自定义 MAVLink 消息。

调试建议：

- 不确定坐标系时，先让飞机不解锁，在仿真或架台上观察 `setpoint_raw/target_*` 和 local odom。
- 不要在同一链路里重复做 ENU/NED 转换。
- 对外部定位输入 PX4 时，重点确认 frame、姿态、速度方向和时间戳。
- 在 Sunray 任务层优先使用 `UAVControlCMD`，不要直接发布 MAVLink raw topic。

### MAVLink 与 uORB、ROS 的关系

MAVLink、uORB、ROS topic 是三套不同层级的通信机制。

| 名称 | 运行位置 | 作用 | 是否能 `rostopic echo` |
| --- | --- | --- | --- |
| uORB | PX4 内部 | PX4 模块之间通信。 | 不能，需要 PX4 shell 的 `listener`。 |
| MAVLink | PX4 与外部设备之间 | 跨串口/UDP/网络传输飞控消息。 | 不能直接 echo，需要 MAVROS/MAVSDK/pymavlink 等工具解析。 |
| ROS topic | 伴随计算机 ROS 系统 | Sunray、MAVROS、算法节点之间通信。 | 可以。 |

典型转换：

```text
PX4 uORB vehicle_local_position
  -> PX4 MAVLink module
  -> MAVLink LOCAL_POSITION_NED / ODOMETRY
  -> MAVROS local_position plugin
  -> /uav1/mavros/local_position/odom
```

反向 setpoint：

```text
/uav1/mavros/setpoint_raw/local
  -> MAVROS setpoint_raw plugin
  -> MAVLink SET_POSITION_TARGET_LOCAL_NED
  -> PX4 MAVLink receiver
  -> PX4 内部 setpoint / controller
```

### 什么时候需要直接研究 MAVLink

大多数 Sunray 二次开发不需要直接写 MAVLink。但以下场景建议阅读 MAVLink 文档：

| 场景 | 原因 |
| --- | --- |
| 新增 MAVROS 尚未封装的飞控能力 | 需要知道对应 MAVLink 消息、字段和坐标系。 |
| 接入非 ROS 地面站或自研通信链路 | 可能需要直接解析或转发 MAVLink。 |
| 调试 QGC、MAVROS、PX4 三者状态不一致 | 需要理解 HEARTBEAT、mode、command ack 等协议语义。 |
| 新增云台、相机、载荷设备协议 | 可能涉及 MAVLink camera/gimbal/mount 消息。 |
| 多机系统 ID 冲突 | 需要理解 sysid、compid、路由和 UDP 端口。 |

### 对 Sunray 开发者的建议

- 任务节点优先发布 `sunray_msgs/UAVControlCMD`。
- 状态读取优先订阅 `/uav1/sunray/px4_state` 和 `/uav1/sunray/uav_control/control_state`。
- 只有当 Sunray 没有暴露某个 PX4 能力时，才考虑直接使用 `/uav1/mavros/*`。
- 只有当 MAVROS 也没有封装时，才考虑直接写 MAVLink 或改 MAVROS 插件。
- 多机系统中要确认每架飞机的 MAVLink system id、ROS namespace、UDP 端口不要冲突。

### 官方资料

- MAVLink 开发指南：https://mavlink.io/en/
- MAVLink 序列化与包格式：https://mavlink.io/en/guide/serialization.html
- MAVLink Common 消息集：https://mavlink.io/en/messages/common.html
- MAVLink 坐标系枚举：https://mavlink.io/en/messages/common.html#MAV_FRAME
- MAVLink 消息签名：https://mavlink.io/en/guide/message_signing.html

</section>
