<!-- title: yunlink_ros_bridge -->

<section id="communication-yunlink-ros-bridge">

## yunlink_ros_bridge

`communication/yunlink_ros_bridge` 是 Sunray 与 YunLink/地面站/外部系统之间的 ROS 桥接包。它把 Sunray 的状态话题、控制命令、系统服务映射到外部通信协议。

### 启动

```bash
roslaunch yunlink_ros_bridge yunlink_ros_bridge.launch
```

### 关键参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `udp_bind_port` | `9696` | 本地 UDP 绑定端口。 |
| `udp_target_port` | `9898` | 目标 UDP 端口。 |
| `tcp_listen_port` | `9696` | 本地 TCP 监听端口。 |
| `remote_ip` | 空 | 远端 IP。 |
| `shared_secret` | `yunlink-default-secret` | 通信共享密钥。 |
| `agent_id` | `1` | 默认智能体 ID。 |
| `enable_endpoint_discovery` | `true` | 是否启用端点发现。 |
| `enable_system_services` | `true` | 是否桥接 `sunray_system` 服务。 |
| `enable_runtime_diagnostics` | `true` | 是否发布运行诊断。 |
| `enable_tui_monitor` | `true` | 是否启动桥接 TUI 监控节点。 |

### Sunray 侧话题

默认订阅/发布：

```text
/uav1/sunray/localization/local_odom
/uav1/sunray/localization/global_odom
/uav1/sunray/localization/odom_state
/uav1/sunray/uav_control/control_state
/uav1/sunray/uav_control/command_execution_status
/uav1/sunray/uav_control/control_cmd
/uav1/sunray/px4_state
```

外部系统下发控制时，桥接节点最终会发布 `sunray_msgs/UAVControlCMD`，而不是直接控制 MAVROS。

### 主要源码

| 文件 | 作用 |
| --- | --- |
| `src/main.cpp` | 节点入口。 |
| `src/bridge_topics.cpp` | 状态话题桥接。 |
| `src/bridge_commands.cpp` | 外部命令解析。 |
| `src/runtime/bridge_control_publish.cpp` | 发布 Sunray 控制命令。 |
| `src/bridge_system_services.cpp` | 桥接 `sunray_system` 服务。 |
| `src/discovery/*` | 端点发现。 |
| `src/diagnostics/*` | 运行诊断。 |
| `src/tui/*` | 终端监控界面。 |

### 二次开发

- 新增外部命令时，先确认 `sunray_msgs` 是否已有对应控制/规划消息。
- 新增状态上报时，优先从 Sunray 标准状态话题读取，不要直接读控制器内部变量。
- 修改协议字段时，要同步桥接映射、地面站解析和文档。

</section>
