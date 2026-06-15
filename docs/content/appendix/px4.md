<!-- title: PX4 -->

<section id="appendix-px4">

## PX4

PX4 是 Sunray 真机飞行链路中的飞控软件。它运行在飞控板上，负责传感器采集、状态估计、姿态控制、位置控制、模式管理、失控保护和电机输出。Sunray_v2 通常不要求开发者直接修改 PX4 源码，而是通过 MAVROS 和 MAVLink 与 PX4 交互。

对二次开发者来说，推荐先建立这条边界：

```text
你的任务节点
  -> sunray_msgs/UAVControlCMD
  -> sunray_uav_control
  -> MAVROS topic/service
  -> MAVLink
  -> PX4
  -> 电机/舵机/飞控输出
```

也就是说，大多数任务开发不直接发布 PX4 setpoint，更不直接改 PX4 控制器。你应该优先使用 Sunray 的控制接口，只有在需要新增底层控制模式、改变飞控参数策略或调试 PX4 内部状态时，才进入 PX4 资料和源码。

### PX4 在 Sunray 中的角色

Sunray 与 PX4 的关系可以理解为“上层智能任务”和“底层飞行安全执行器”的关系。

| 层级 | 主要负责 | Sunray 中的位置 |
| --- | --- | --- |
| 任务层 | 航点、跟踪、搜索、编队、实验逻辑 | `examples`、`planning`、`swarm`、用户自定义节点 |
| Sunray 控制层 | 把任务命令转换成 PX4 能执行的 setpoint 或服务调用 | `uav_control/sunray_uav_control` |
| ROS/MAVLink 桥 | 把 ROS topic/service 转成 MAVLink 消息 | `drivers/sunray_mavros` + MAVROS |
| PX4 飞控 | 模式、解锁、安全保护、估计器、姿态/位置控制、电机输出 | 飞控板内部固件 |

常见操作对应关系：

| Sunray 行为 | MAVROS/PX4 侧动作 |
| --- | --- |
| 启动 `sunray_mavros` | 建立 MAVLink 连接，生成 `/uav1/mavros/*` 话题和服务。 |
| 启动 `sunray_uav_control` | 订阅定位和控制命令，发布 setpoint，调用解锁/模式服务。 |
| `TAKEOFF` | 控制节点切换合适模式、解锁并生成起飞过程 setpoint。 |
| `MOVE_POINT` | 控制节点生成位置 setpoint，让 PX4 或几何控制器执行。 |
| `MOVE_VELOCITY` | 控制节点持续转发速度目标，发布频率不足会触发超时保护。 |
| `/uav1/sunray/px4_state` | 把 MAVROS/PX4 状态聚合成 Sunray 统一状态。 |

### PX4 软件架构

PX4 官方文档把系统拆成模块化架构。对 Sunray 开发者来说，最需要理解下面几个概念：

| 概念 | 说明 | 开发时关注点 |
| --- | --- | --- |
| Module | PX4 内部可运行模块，例如 commander、navigator、mc_pos_control、ekf2。 | 调试飞控内部状态时会看到模块名。 |
| uORB | PX4 内部发布/订阅中间件，模块之间通过 uORB topic 通信。 | 它不是 ROS topic，不能直接用 `rostopic echo` 查看。 |
| MAVLink | PX4 和伴随计算机/地面站通信的协议。 | Sunray 通过 MAVROS 间接使用 MAVLink。 |
| Estimator | 估计位置、速度、姿态，例如 EKF2。 | 定位源接入 PX4 时要关注 EKF2 参数和输入质量。 |
| Controller | 位置、速度、姿态控制器。 | 使用 PX4 原生控制器时，位置/速度 setpoint 最终进入这里。 |
| Commander | 飞行模式、解锁、failsafe 管理。 | Offboard 进不去、自动降落、拒绝解锁通常与这里有关。 |

PX4 内部典型链路：

```text
传感器驱动
  -> uORB: sensor_combined / vehicle_gps_position / vehicle_visual_odometry ...
  -> EKF2 等估计器
  -> uORB: vehicle_local_position / vehicle_attitude
  -> 位置/姿态控制器
  -> mixer / control allocator
  -> 电机输出
```

Sunray 在 ROS 侧看到的是 MAVROS 转换后的结果，例如 `/uav1/mavros/local_position/odom`、`/uav1/mavros/state`、`/uav1/mavros/setpoint_raw/target_local`。这些话题不是 PX4 内部 uORB 原始话题，而是经过 MAVLink 和 MAVROS 适配后的 ROS 表达。

### Offboard 模式

Sunray 真机控制经常依赖 PX4 Offboard 能力。Offboard 的核心含义是：飞控接受外部计算机持续发送的位置、速度、加速度、姿态或角速度目标。

需要特别注意：

- Offboard 不是“发一次目标点就飞过去”的底层协议。
- PX4 需要持续收到外部 setpoint 或 liveliness 信号，频率过低会触发 Offboard loss failsafe。
- Sunray 在上层把命令分成“发一次即可”和“需要持续发布”，这是对用户接口的约定；控制节点内部仍会按 PX4 需要持续维护 setpoint。
- 速度和轨迹命令天然代表实时目标，应由任务节点持续发布。
- 起飞、降落、位置点命令可以在 Sunray 接口层发一次，由控制状态机接管后续执行。

Sunray 中和 Offboard 强相关的地方：

| 文件/模块 | 作用 |
| --- | --- |
| `uav_control/sunray_uav_control` | 控制状态机、PX4 原生控制器、几何控制器、MAVROS helper。 |
| `drivers/sunray_mavros` | 启动 MAVROS，并加载 setpoint、state、command、param 等插件。 |
| `common/sunray_msgs/msg/UAVControlCMD.msg` | 用户发布的 Sunray 控制命令。 |
| `common/sunray_msgs/msg/Px4State.msg` | MAVROS/PX4 状态汇总，方便开发者调试。 |

### 常见 PX4 参数方向

不同 PX4 版本参数会变化，具体以 QGroundControl 或 PX4 官方参数文档为准。下面只列 Sunray 调试时常见的参数类别。

| 参数类别 | 典型参数 | 作用 |
| --- | --- | --- |
| Offboard failsafe | `COM_OF_LOSS_T`、`COM_OBL_RC_ACT` | Offboard setpoint 丢失多久触发保护，以及触发后的动作。 |
| RC 输入 | `COM_RC_IN_MODE`、`RC_MAP_*` | 是否需要 RC、摇杆通道映射。 |
| MAVLink | `MAV_*` | MAVLink 实例、串口、模式和数据流配置。 |
| EKF2 | `EKF2_*` | 外部视觉、GPS、气压计、IMU 融合策略。 |
| 多旋翼位置控制 | `MPC_*` | PX4 原生位置/速度控制参数。 |
| 解锁检查 | `COM_ARM_*`、`CBRK_*` | 解锁前检查与断路器参数，真机不建议随意关闭安全检查。 |

仓库中的 `tools/px4_param_check` 用于检查或写入 PX4 参数。新增机型时，建议把参数模板维护到该工具的配置文件里，而不是靠人工在 QGroundControl 中逐项记忆。

### uORB 与 ROS/MAVROS 对照

下面是常见概念级对照，不代表字段完全一一对应。PX4 内部字段、MAVLink 消息和 MAVROS topic 之间通常会经过坐标系、单位和字段裁剪。

| PX4/uORB 概念 | MAVLink 消息方向 | MAVROS/Sunray 侧常见话题 | 说明 |
| --- | --- | --- | --- |
| `vehicle_status` | HEARTBEAT / EXTENDED_SYS_STATE 等 | `/uav1/mavros/state`、`/uav1/mavros/extended_state` | 模式、解锁、系统状态、落地状态。 |
| `vehicle_local_position` | LOCAL_POSITION_NED / ODOMETRY | `/uav1/mavros/local_position/odom` | PX4 本地位置和速度。 |
| `vehicle_attitude` | ATTITUDE / ODOMETRY | `/uav1/mavros/imu/data`、local odom 姿态 | 姿态估计输出。 |
| `vehicle_visual_odometry` | VISION_POSITION_ESTIMATE / ODOMETRY 输入 | `/uav1/mavros/vision_pose/pose`、`/uav1/mavros/odometry/in` | 外部定位输入 PX4。 |
| `manual_control_setpoint` / RC 输入 | RC_CHANNELS / MANUAL_CONTROL | `/uav1/mavros/rc/in`、`/uav1/mavros/manual_control/*` | 遥控器和手动输入。 |
| 位置/速度 setpoint | SET_POSITION_TARGET_LOCAL_NED | `/uav1/mavros/setpoint_raw/local` | 外部位置/速度/加速度/yaw 目标输入。 |
| 姿态 setpoint | SET_ATTITUDE_TARGET | `/uav1/mavros/setpoint_raw/attitude` | 外部姿态、角速度、推力输入。 |

### 调试建议

PX4 相关问题通常不要只看一个话题，应同时检查连接、模式、定位、setpoint 和 failsafe。

常用 ROS 侧检查：

```bash
rostopic echo /uav1/mavros/state
rostopic echo /uav1/mavros/extended_state
rostopic echo /uav1/mavros/local_position/odom
rostopic echo /uav1/mavros/setpoint_raw/target_local
rostopic echo /uav1/sunray/px4_state
```

常用 PX4 shell 检查：

```bash
mavlink status
listener vehicle_status
listener vehicle_local_position
listener vehicle_attitude
listener vehicle_visual_odometry
```

常见现象：

| 现象 | 优先检查 |
| --- | --- |
| MAVROS 未连接 | 飞控串口、波特率、USB 权限、`fcu_url`、PX4 MAVLink 实例。 |
| 无法进入 Offboard | setpoint 是否持续发布、定位是否有效、PX4 模式切换返回值。 |
| 解锁失败 | QGC preflight check、传感器校准、RC/安全开关、`/mavros/state`。 |
| 位置飞偏 | ENU/NED 坐标系、定位外参、PX4 EKF 输入源、local/global frame。 |
| 飞一会自动退出 Offboard | setpoint 频率、网络/串口延迟、`COM_OF_LOSS_T` 和 failsafe 动作。 |

### 官方资料

- PX4 用户文档：https://docs.px4.io/main/en/
- PX4 架构说明：https://docs.px4.io/main/en/concept/architecture.html
- PX4 uORB：https://docs.px4.io/main/en/middleware/uorb.html
- PX4 Offboard 模式：https://docs.px4.io/main/en/flight_modes/offboard.html
- PX4 参数参考：https://docs.px4.io/main/en/advanced_config/parameter_reference.html

</section>
