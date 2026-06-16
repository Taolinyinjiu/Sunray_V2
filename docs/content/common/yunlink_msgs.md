<!-- title: yunlink_msgs -->

<section id="common-yunlink-msgs">

## yunlink_msgs

`common/yunlink_msgs` 是 YunLink 协议侧的 ROS 消息包。它服务于 `yunlink_ros_bridge` 和外部兼容话题，负责表达 YunLink 命令元数据、协议命令和对外统一执行状态。

边界上，这个包和 `sunray_msgs` 的职责不同：

- `sunray_msgs` 负责 Sunray 本地控制/规划/系统语义。
- `yunlink_msgs` 负责 YunLink 协议和 bridge 对外 canonical 语义。
- `sunray_uav_control` 不应直接依赖 `yunlink_msgs`；协议字段和协议状态转换应由 `yunlink_ros_bridge` 处理。

### 包结构

```text
common/yunlink_msgs/
└── msg/
    ├── CommandExecutionStatus.msg
    ├── CommandMeta.msg
    ├── GotoCommand.msg
    ├── LandCommand.msg
    ├── ReturnCommand.msg
    ├── TakeoffCommand.msg
    └── VelocitySetpointCommand.msg
```

### 分类总览

| 类别 | 消息 | 用途 |
| --- | --- | --- |
| 协议元数据 | `CommandMeta` | 承载 YunLink `session_id / message_id / correlation_id`。 |
| 协议执行状态 | `CommandExecutionStatus` | bridge 对外发布的 canonical 执行状态。 |
| 协议命令 | `TakeoffCommand`、`LandCommand`、`ReturnCommand`、`GotoCommand`、`VelocitySetpointCommand` | YunLink 命令在 ROS 侧的承载格式。 |

### CommandMeta.msg

```text
uint64 session_id
uint64 message_id
uint64 correlation_id
```

用途：为一条 YunLink 命令或状态携带链路追踪信息。

字段说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `session_id` | `uint64` | 当前 YunLink 会话 ID。 |
| `message_id` | `uint64` | 当前消息 ID。 |
| `correlation_id` | `uint64` | 用于把请求与回传状态关联起来的 ID。 |

### CommandExecutionStatus.msg

典型话题：

```text
/uav1/yunlink/command_execution_status
```

发布者：`yunlink_ros_bridge`。

用途：这是 YunLink 侧对外统一的 canonical 执行状态消息。当前 bridge 从本地 `sunray_msgs/UAVControlCommandStatus` 读取控制事实，再补回 YunLink `CommandMeta`，最终发布本消息。

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
| --- | --- | --- |
| `header` | `std_msgs/Header` | 状态时间戳。 |
| `agent_name` / `agent_id` | `string` / `uint8` | 命令所属智能体。 |
| `meta` | `yunlink_msgs/CommandMeta` | 协议链路追踪信息。 |
| `command_kind` | `uint8` | 归一化命令类型。 |
| `execution_state` | `uint8` | 当前执行状态。 |
| `progress_percent` | `uint8` | 进度百分比。 |
| `active` | `bool` | 是否仍在执行。 |
| `terminal` | `bool` | 是否已进入终态。 |
| `success` | `bool` | 是否成功。 |
| `result_code` | `uint16` | 结果码。 |
| `detail` | `string` | 详细说明。 |
| `control_state` | `uint8` | 当前 Sunray UAV 控制状态。 |
| `px4_landed_state` | `uint8` | PX4 起降状态。 |
| `ready_for_takeoff` | `bool` | 当前是否满足起飞条件。 |
| `ready_for_land` | `bool` | 当前是否满足降落条件。 |
| `busy_reason` | `string` | 忙碌、拒绝或等待原因。 |

### TakeoffCommand.msg

```text
std_msgs/Header header
yunlink_msgs/CommandMeta meta
float32 relative_height_m
float32 max_velocity_mps
```

用途：YunLink 起飞命令。bridge 收到后会映射为 `sunray_msgs/UAVControlCMD` 的 `TAKEOFF`。

### LandCommand.msg

```text
std_msgs/Header header
yunlink_msgs/CommandMeta meta
float32 max_velocity_mps
```

用途：YunLink 降落命令。bridge 收到后会映射为本地 `LAND` 控制命令。

### ReturnCommand.msg

```text
std_msgs/Header header
yunlink_msgs/CommandMeta meta
float32 loiter_before_return_s
```

用途：YunLink 返航命令。当前语义仍由 bridge 转成本地 `RETURN` 控制命令。

### GotoCommand.msg

```text
std_msgs/Header header
yunlink_msgs/CommandMeta meta
float32 x_m
float32 y_m
float32 z_m
float32 yaw_rad
```

用途：YunLink 点位控制命令。bridge 会把它映射到本地 `MOVE_POINT` 或对应归一化移动语义。

### VelocitySetpointCommand.msg

```text
std_msgs/Header header
yunlink_msgs/CommandMeta meta
float32 vx_mps
float32 vy_mps
float32 vz_mps
float32 yaw_rate_radps
bool body_frame
```

用途：YunLink 速度控制命令。bridge 根据 `body_frame` 选择映射为本地机体系或惯性系速度命令。
