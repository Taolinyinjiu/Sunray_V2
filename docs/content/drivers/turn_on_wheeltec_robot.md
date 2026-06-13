<!-- title: turn_on_wheeltec_robot -->

<section id="driver-turn-on-wheeltec-robot">

## turn_on_wheeltec_robot

`drivers/turn_on_wheeltec_robot` 是轮趣底盘驱动和其配套导航 launch 的集合。Sunray UGV 控制最终会输出 `geometry_msgs/Twist`，底盘驱动负责把速度命令发给底层单片机。

### 常用启动

底盘基础驱动：

```bash
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch car_mode:=mini_mec
```

Sunray 早期 UGV 驱动入口：

```bash
roslaunch turn_on_wheeltec_robot ugv_driver_robot.launch ugv_id:=1
```

### 常用参数

| 参数 | 说明 |
| --- | --- |
| `car_mode` | 车型，例如 `mini_mec`、`mini_diff`、`senior_mec_bs` 等。 |
| `navigation` | 是否启动 2D 导航相关 launch。 |
| `pure3d_nav` | 是否启动纯 3D 导航相关 launch。 |
| `odom_frame_id` | odom frame 名。 |
| `usart_port_name` | 串口名，常见为 `/dev/ttyACM0`。 |
| `serial_baud_rate` | 串口波特率。 |

### 和 Sunray UGV 控制的关系

推荐链路：

```text
任务/面板
  -> /ugv1/sunray/ugv_control/control_cmd
  -> sunray_ugv_control
  -> /ugv1/sunray/ugv_control/cmd_vel
  -> 底盘驱动 remap
  -> 单片机
```

如果接轮趣底盘，应重点确认驱动实际订阅的 `/cmd_vel` 是否 remap 到 Sunray UGV 控制输出。

### 二次开发边界

- 换车型、串口、波特率优先改 launch 参数。
- Sunray 的点位控制、速度限制、地理围栏应放在 `sunray_ugv_control`，底盘驱动只负责执行速度。
- 如果驱动发布 odom，可把 odom 接入 `localization_fusion` 或 UGV 控制状态反馈。

</section>
