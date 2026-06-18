<!-- title: yunlink_ros_bridge -->

<section id="communication-yunlink-ros-bridge">

## yunlink_ros_bridge

`communication/yunlink_ros_bridge` 是 Sunray_v2 和 YunLink 之间唯一的 ROS 适配边界。它启动一个
YunLink runtime，把 Sunray 的 ROS topic/service 映射成 YunLink 语义消息，也把地面站或外部系统发来的
YunLink 命令映射回 Sunray ROS 消息。

这个包保持 thin bridge 设计：它只做连接维护、字段转换、转发、诊断和服务适配，不在桥接层判断
TAKEOFF/LAND/MOVE 是否执行完成。真实控制执行、状态机和完成判断仍属于 `sunray_uav_control`；
功能启动/停止的实际执行仍属于 `sunray_system`。

### 当前边界

| 层 | 职责 |
| --- | --- |
| `thirdparty/yunlink` | ROS 无关的 runtime、TCP/UDP 会话、认证、语义消息、QoS 和端点发现数据结构。 |
| `communication/yunlink_ros_bridge` | ROS topic/service 与 YunLink 语义消息之间的适配、连接维护、诊断和 TUI。 |
| `control/sunray_uav_control` | 飞行命令执行、状态机、执行状态发布和完成判断。 |
| `sunray_system` | feature 列表、查询、启动、停止等系统功能管理。 |

### 启动

默认 launch 会同时启动 bridge 节点和只读 TUI 监听面板：

```bash
roslaunch yunlink_ros_bridge yunlink_ros_bridge.launch
```

仿真配置是默认值：

```bash
roslaunch yunlink_ros_bridge yunlink_ros_bridge.launch enable_sim:=true
```

真机配置会额外启用外部里程计诊断 topic：

```bash
roslaunch yunlink_ros_bridge yunlink_ros_bridge.launch enable_sim:=false
```

不启动 TUI：

```bash
roslaunch yunlink_ros_bridge yunlink_ros_bridge.launch enable_tui_monitor:=false
```

构建时使用仓库的模块系统：

```bash
./build.sh --dry-run yunlink_ros_bridge
./build.sh yunlink_ros_bridge
```

`CMakeLists.txt` 会从 `thirdparty/yunlink` 引入本地 YunLink 库；如果子模块不存在，需要先初始化：

```bash
git submodule update --init --recursive thirdparty/yunlink
```

### 关键参数

参数主要来自：

- `communication/yunlink_ros_bridge/config/yunlink_ros_bridge_base.yaml`
- `communication/yunlink_ros_bridge/config/yunlink_ros_bridge_sim.yaml`
- `communication/yunlink_ros_bridge/config/yunlink_ros_bridge_real.yaml`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `udp_bind_port` | `9696` | YunLink runtime 本地 UDP 绑定端口，用于 UDP 消息和发现相关回复。 |
| `udp_target_port` | `9898` | 主动发送 UDP 时的默认目标端口；实际业务仍受 QoS、会话和已知对端端点约束。 |
| `tcp_listen_port` | `9696` | 本地 TCP 监听端口。默认 monitor 手动连接 bridge 时连这个端口。 |
| `remote_ip` | 空 | 主动连接目标 IP。为空时不主动拨号。 |
| `remote_tcp_port` | `0` | 主动连接目标 TCP 端口。为 `0` 时不主动拨号。 |
| `shared_secret` | `yunlink-default-secret` | YunLink 会话认证口令，只决定会话认证，不等同于控制权授权。 |
| `agent_id` | `1` | 当前 bridge 代表的 UAV 编号，会影响 YunLink identity 和默认 `/uavN` 语义。 |
| `enable_endpoint_discovery` | `true` | 是否周期广播端点发现信息，供 monitor 发现并连接。 |
| `discovery_port` | `9966` | 端点发现广播端口。 |
| `discovery_target_ip` | `255.255.255.255` | 端点发现广播目标地址。 |
| `discovery_period_ms` | `1000` | 端点发现广播周期。 |
| `endpoint_name_prefix` | `yundrone_uav` | 端点显示名前缀。 |
| `endpoint_id_file` | `$HOME/.config/yunlink_ros_bridge/endpoint_id` | 本机稳定端点 ID 存储位置。 |
| `qos_profile` | `balanced` | YunLink QoS profile，可选 `reliable`、`balanced`、`low_latency`。这是 YunLink 层 QoS，不是 ROS2 DDS QoS。 |
| `qos_udp_fallback_to_tcp` | `true` | UDP 发送失败或没有可用 UDP 端点时，允许可回退通道改走 TCP。 |
| `enable_system_services` | `true` | 是否桥接 `sunray_system` 的 feature service。 |
| `sunray_system_ns` | `/sunray_system` | `sunray_system` service 命名空间。 |
| `system_service_timeout_sec` | `3.0` | feature service 默认等待超时。 |
| `default_start_with_terminal` | `false` | feature_start 请求未显式指定时的默认终端启动策略。 |
| `enable_runtime_diagnostics` | `true` | 是否发布 YunLink runtime 诊断 snapshot 和本地 `DiagnosticArray`。 |
| `diagnostic_publish_rate_hz` | `2.0` | 诊断发布频率。 |
| `diagnostic_stale_timeout_ms` | `1000` | 诊断判定 topic 过期的默认超时。 |
| `monitor_diagnostic_topic` | `/yunlink_ros_bridge/monitor_diagnostics` | 本地 TUI/监控使用的 `diagnostic_msgs/DiagnosticArray`。 |
| `print_hz` | `2.0` | TUI 终端刷新频率。 |
| `stale_timeout_ms` | `1000` | TUI 显示层 stale 超时。 |

默认连接模式是被动入站：`remote_ip=""` 且 `remote_tcp_port=0` 时，bridge 不主动连接地面站，
而是在 `tcp_listen_port=9696` 等待 monitor 或外部系统连入。端点发现会把 bridge 的存在广播出去。
如果要让 bridge 主动拨号，需要同时设置 `remote_ip` 和 `remote_tcp_port`。

### QoS 通道

`qos_channels` 按 bridge 语义通道配置，不直接按原始 ROS topic 名配置。ROS 侧字段包括：

| 字段 | 说明 |
| --- | --- |
| `queue_size` | ROS 发布/订阅队列长度。高频状态默认多为 `1`，避免积压旧状态。 |
| `tcp_no_delay` | ROS1 TCPROS 是否禁用 Nagle。 |
| `publish_hz` | ROS -> YunLink 最高转发频率；`0.0` 表示收到每条 ROS 消息都转发。 |
| `timeout_sec` | system service 请求/响应通道超时。 |

YunLink 侧字段包括 `qos_class`、`transport` 和 fallback 策略。关键命令和 service 通道必须保持可靠有序，
例如 `takeoff_command`、`land_command`、`return_command`、`goto_command`、`velocity_setpoint_command`、
`feature_start`、`feature_stop`、`session`、`authority`、`command_result`。如果关键通道被配置成
`best_effort`、`udp` 或 `bulk`，bridge 启动时会报 `invalid-qos-config`，不会静默改写配置。

### Sunray 侧话题

默认 topic：

| 参数 | 默认 topic | 方向 | 说明 |
| --- | --- | --- | --- |
| `local_odom_topic` | `/uav1/sunray/localization/local_odom` | ROS -> YunLink | 本地里程计 snapshot。 |
| `odom_state_topic` | `/uav1/sunray/localization/odom_state` | ROS -> YunLink | 定位状态 snapshot，同时更新诊断中的里程计 topic 信息。 |
| `global_odom_topic` | `/uav1/sunray/localization/global_odom` | 诊断输入 | 全局里程计诊断输入。 |
| `external_odom_topic` | 仿真为空，真机为 `/uav1/sunray/localization/external_odom` | 诊断输入 | 外部里程计诊断输入；为空会显示 `UNCONFIGURED`，不代表业务错误。 |
| `control_state_topic` | `/uav1/sunray/uav_control/control_state` | ROS -> YunLink | 控制状态 snapshot。 |
| `command_execution_status_topic` | `/uav1/sunray/uav_control/command_execution_status` | ROS -> YunLink | 控制侧命令执行状态 snapshot，是执行结果的权威来源。 |
| `control_cmd_topic` | `/uav1/sunray/uav_control/control_cmd` | 双向 | 本地控制命令会被镜像到 YunLink；YunLink 飞行命令也会发布到这里。 |
| `px4_state_topic` | `/uav1/sunray/px4_state` | ROS -> YunLink | PX4 状态 snapshot，并缓存高度供机体系速度命令填充 `fixed_height`。 |

外部系统下发控制时，桥接节点最终会发布 `sunray_msgs/UAVControlCMD`，而不是直接控制 MAVROS。

### ROS -> YunLink

bridge 订阅 Sunray 标准 topic，并发布对应 YunLink snapshot：

| Sunray 输入 | YunLink 输出语义 | 代码入口 |
| --- | --- | --- |
| `nav_msgs/Odometry` 本地里程计 | `LocalOdomSnapshot` | `onLocalOdom()` |
| `sunray_msgs/OdomState` | `OdomStateSnapshot` | `onOdomState()` |
| `sunray_msgs/UAVControlCMD` | `UavControlCmdSnapshot` | `onControlCmd()` |
| `sunray_msgs/UAVControlState` | `UavControlStateSnapshot` | `onControlState()` |
| `sunray_msgs/UAVCommandExecutionStatus` | `CommandExecutionStatusSnapshot` | `onCommandExecutionStatus()` |
| `sunray_msgs/Px4State` | `Px4StateSnapshot` | `onPx4State()` |

`UAVCommandExecutionStatus` 中的 `yunlink_session_id`、`yunlink_message_id`、`yunlink_correlation_id`
会映射回 YunLink 执行状态，用来把控制侧状态关联回原始 YunLink 命令。

### YunLink -> ROS

bridge 订阅 YunLink 命令，并转换成 `sunray_msgs/UAVControlCMD` 发布到 `control_cmd_topic`：

| YunLink 命令 | Sunray 控制命令 | 说明 |
| --- | --- | --- |
| `TakeoffCommand` | `UAVControlCMD::TAKEOFF` | 转发相对起飞高度和最大速度。 |
| `LandCommand` | `UAVControlCMD::LAND` | 转发降落最大速度。 |
| `ReturnCommand` | `UAVControlCMD::RETURN` | 转发返航命令。 |
| `GotoCommand` | `UAVControlCMD::MOVE_POINT` | 转发目标位置，yaw 使用 `SET_YAW`。 |
| `VelocitySetpointCommand` | `MOVE_VELOCITY` 或 `MOVE_VELOCITY_BODY` | 惯性系速度写入 `desired_vel`；机体系速度写入 `desired_body_xy_vel`，高度用最近 `px4_state` 的 `local_pose.position.z` 填充 `fixed_height`。 |

发布控制命令时，bridge 会把 YunLink envelope 身份信息透传进 Sunray 控制链路：

- `yunlink_session_id`
- `yunlink_message_id`
- `yunlink_correlation_id`

这些字段用于后续执行状态回传关联，不代表 bridge 自己完成了命令执行判断。

### System Service 桥接

当 `enable_system_services=true` 时，bridge 会把 YunLink feature 请求转成 `sunray_system` ROS service 调用：

| YunLink 请求 | ROS service |
| --- | --- |
| `FeatureListRequest` | `/sunray_system/list_features` |
| `FeatureGetRequest` | `/sunray_system/get_features` |
| `FeatureStartRequest` | `/sunray_system/start_feature` |
| `FeatureStopRequest` | `/sunray_system/stop_feature` |

service 请求先进入 bridge 后台 worker 队列，再串行调用 ROS service，避免阻塞 YunLink runtime 回调线程。

### 诊断和 TUI

bridge 周期发布两类诊断：

| 输出 | 说明 |
| --- | --- |
| YunLink `SunrayRuntimeDiagnosticSnapshot` | 发给 YunLink 对端，用于 monitor 通过 YunLink 读取运行状态。 |
| ROS `/yunlink_ros_bridge/monitor_diagnostics` | 本地 `diagnostic_msgs/DiagnosticArray`，供 TUI 节点显示。 |

诊断覆盖连接状态、runtime 是否启动、peer/session 状态、ROS -> YunLink 计数、YunLink -> ROS 计数、
最近转发失败、topic 是否有 publisher、消息频率、消息年龄和最近事件。

常见状态含义：

| 状态 | 含义 |
| --- | --- |
| `OK` | topic 已配置且有新鲜消息。 |
| `WAIT` | YunLink session 尚未建立，默认被动模式下通常表示等待 monitor 连入。 |
| `WAIT_MESSAGE` | topic 已配置但还没有收到消息。 |
| `NO_PUBLISHER` | ROS topic 当前没有 publisher。 |
| `STALE` | 曾经收到消息，但超过 `diagnostic_stale_timeout_ms` 没有更新。 |
| `UNCONFIGURED` | 该诊断输入没有配置，例如仿真默认 `external_odom_topic` 为空。 |
| `ERROR` | bridge 检测到明确错误，例如转发失败。 |

`yunlink_ros_bridge_tui_node` 只订阅 `monitor_diagnostic_topic` 并刷新终端面板，不直接读业务 topic，
也不向控制链路发命令。

### 主要源码

| 文件 | 作用 |
| --- | --- |
| `src/main.cpp` | 节点入口。 |
| `src/bridge_node.hpp` | 主节点接口、ROS 句柄、runtime 状态、诊断状态和内部 helper 声明。 |
| `src/bridge_runtime_types.hpp` | 参数结构和 QoS 通道配置结构。 |
| `src/bridge_mapping.cpp` / `src/bridge_mapping.hpp` | ROS 消息与 YunLink snapshot 的字段映射。 |
| `src/bridge_topics.cpp` | Sunray ROS 状态 topic 到 YunLink snapshot 的转发。 |
| `src/bridge_commands.cpp` | YunLink 飞行命令到 Sunray `UAVControlCMD` 的适配。 |
| `src/bridge_system_services.cpp` | YunLink feature 请求到 `sunray_system` ROS service 的适配。 |
| `src/runtime/bridge_runtime_core.cpp` | 参数加载后的初始化顺序、runtime 启动、端点发现和定时器。 |
| `src/runtime/bridge_runtime_params.cpp` | ROS/YAML 参数和 `qos_channels` 读取。 |
| `src/runtime/bridge_runtime_qos*.cpp` | YunLink QoS profile、通道策略和配置校验。 |
| `src/runtime/bridge_runtime_session.cpp` | peer session 复用、被动等待、主动拨号和重连维护。 |
| `src/runtime/bridge_runtime_subscribers.cpp` | ROS topic、YunLink command 和 system-service 订阅初始化。 |
| `src/runtime/bridge_control_publish.cpp` | 向 Sunray `control_cmd_topic` 发布控制命令并透传 envelope ID。 |
| `src/runtime/bridge_runtime_services.cpp` | system-service client、worker 生命周期和端点发现定时器。 |
| `src/discovery/bridge_endpoint_discovery.*` | 端点发现广播。 |
| `src/diagnostics/*` | 运行诊断 snapshot、`DiagnosticArray`、转发失败记录和 monitor 状态。 |
| `src/tui/*` | 只读终端监听面板。 |

### 二次开发

- 新增外部命令时，先确认 `sunray_msgs` 是否已有对应控制/规划消息。bridge 只映射到 ROS 消息，不新增业务执行逻辑。
- 新增状态上报时，优先从 Sunray 标准状态 topic 读取，不要直接读控制器内部变量。
- 新增命令执行结果时，权威来源应在 `sunray_uav_control` 发布的执行状态里，bridge 只转成 YunLink snapshot。
- 新增 feature 类能力时，优先走 `sunray_system` service，bridge 只做 request/response 适配。
- 修改 YunLink 语义字段时，需要同步 `thirdparty/yunlink` 的消息定义、bridge 映射、monitor/地面站解析和本文档。
- 修改 topic 名、命名空间、多机编号或真机/仿真差异时，优先通过 YAML 参数覆盖。
- 修改 QoS 时按语义通道调整 `qos_channels`，不要把 `qos_profile` 理解成 ROS2 DDS QoS。
- 诊断或 TUI 只能观察 bridge 状态，不应直接接管控制命令、完成判断或系统功能执行。

</section>
