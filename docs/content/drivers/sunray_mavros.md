<!-- title: sunray_mavros -->

<section id="driver-sunray-mavros">

## sunray_mavros

`drivers/sunray_mavros` 是 Sunray 对 MAVROS 的启动封装。它不重新实现 MAVLink 协议，而是通过 MAVROS 连接 PX4 飞控，并把 MAVROS 节点放到 `/uav1`、`/uav2` 这样的命名空间下。

### 启动方式

```bash
roslaunch sunray_mavros mavros.launch agent_name:=uav agent_id:=1 fcu_url:=/dev/ttyACM0:921600
```

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `agent_name` | `uav` | 智能体名前缀。 |
| `agent_id` | `1` | 智能体编号。 |
| `fcu_url` | `/dev/ttyACM0:921600` | 飞控连接地址，串口和波特率按实际设备修改。 |
| `gcs_ip` | `0.0.0.0` | 地面站转发目标 IP。 |

启动后 MAVROS 位于：

```text
/uav1/mavros
```

### 配置文件

```text
drivers/sunray_mavros/config/exp_px4_config.yaml
drivers/sunray_mavros/config/exp_px4_pluginlists.yaml
```

`exp_px4_config.yaml` 主要配置 MAVROS 插件参数，例如心跳、timesync、local/global position frame、setpoint_raw 推力缩放、setpoint 参考 frame 等。

`exp_px4_pluginlists.yaml` 控制加载哪些 MAVROS plugin。若缺少某个 `/mavros/*` 话题，先检查 plugin list 是否启用。

### 常用检查话题

```bash
rostopic echo /uav1/mavros/state
rostopic echo /uav1/mavros/extended_state
rostopic echo /uav1/mavros/rc/in
rostopic echo /uav1/mavros/local_position/odom
```

UAV 控制模块会通过 MAVROS 完成：

- 解锁/上锁。
- 切换 PX4 模式。
- 发布 setpoint。
- 读取 PX4 状态、电池、GPS、RC 和当前位置。

### 二次开发边界

- 任务开发不要直接发布 `/mavros/setpoint_raw/*`，优先发布 `sunray_msgs/UAVControlCMD`。
- 如果只是换串口、波特率或多机编号，改 launch 参数即可。
- 如果新增 MAVROS 状态字段，应同步检查 `Px4State.msg` 和 `sunray_uav_control` 中的 MAVROS helper。

</section>
