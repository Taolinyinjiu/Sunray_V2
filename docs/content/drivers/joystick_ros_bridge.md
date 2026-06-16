<!-- title: joystick_ros_bridge -->

<section id="driver-joystick-ros-bridge">

## joystick_ros_bridge

`joystick_ros_bridge` 位于 `drivers/joystick_ros_bridge`，用于把 Linux joystick 设备接入 ROS：一方面发布过滤后的 `sensor_msgs/Joy`，另一方面直接发布 MAVROS 的 `mavros_msgs/OverrideRCIn`，让手柄输入可以覆盖 PX4 的 RC 通道。

该驱动的配置入口是 YAML 文件，而不是 launch 参数：

```text
drivers/joystick_ros_bridge/config/joystick_px4_mapping.yaml
```

启动命令：

```bash
roslaunch joystick_ros_bridge joystick_ros_bridge.launch
```

### 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/joy` | `sensor_msgs/Joy` | 发布过滤后的手柄数据，当前只保留 axis0~axis3 和 X/Y/B/A 四个按钮 |
| `/mavros/rc/override` | `mavros_msgs/OverrideRCIn` | 发布 PX4 RC override，默认开启 |

话题名可以在 YAML 中通过 `pub_joy_topic` 和 `rc_override_topic` 修改。

### PX4 RC 映射

PX4 的 RC 控制遵循 NED 坐标系。当前默认映射如下：

| 输入 | 控制含义 | 输出通道 | 映射关系 |
| --- | --- | --- | --- |
| `axis0` | 偏航 yaw | `rc_ch[3]` / RC4 | `-1` 表示左转，对应 `1000`；`+1` 对应 `2000` |
| `axis1` | 油门 throttle | `rc_ch[2]` / RC3 | 反向：`-1` 表示上升，对应 `2000`；`+1` 对应 `1000` |
| `axis2` | 俯仰 pitch | `rc_ch[1]` / RC2 | `-1` 表示前进，对应 `1000`；`+1` 对应 `2000` |
| `axis3` | 横滚 roll | `rc_ch[0]` / RC1 | `-1` 表示向右飞，对应 `2000`；`+1` 对应 `1000` |
| `axis4` / `axis5` | 不使用 | 无 | Joy 和 RC 都不发布 |
| `button3` | X 键 | `rc_ch[4]` / RC5 | 默认 `1000`，每按下一次在 `1000`/`2000` 间切换 |
| `button0` | Y 键 | `rc_ch[5]` / RC6 | 默认 `1000`，每按下一次在 `1000`/`2000` 间切换 |
| `button1` | B 键 | `rc_ch[7]` / RC8 | 默认 `1000`，每按下一次在 `1000`/`2000` 间切换 |
| `button2` | A 键 | `rc_ch[6]` / RC7 | 默认 `1000`，每按下一次在 `1000`/`2000` 间切换 |

未映射的 RC 通道默认发布 `1000`。用户原始描述中第 9 条将 A 键也写成 `button1`，但 `button1` 已映射 B 键；本配置按常见手柄布局假设 A 键是 `button2`。如果实测不是这样，修改 YAML 中 RC7 的 `source` 即可。

### YAML 配置结构

关键参数：

```yaml
pub_axis_indices: [0, 1, 2, 3]
pub_button_indices: [3, 0, 1, 2]
rc_default: 1000
rc_channels:
  - {channel: 0, type: axis, source: 3, in_min: -1.0, in_max: 1.0, out_min: 2000, out_max: 1000}
  - {channel: 1, type: axis, source: 2, in_min: -1.0, in_max: 1.0, out_min: 1000, out_max: 2000}
  - {channel: 2, type: axis, source: 1, in_min: -1.0, in_max: 1.0, out_min: 2000, out_max: 1000}
  - {channel: 3, type: axis, source: 0, in_min: -1.0, in_max: 1.0, out_min: 1000, out_max: 2000}
  - {channel: 4, type: button, source: 3, mode: toggle, low: 1000, high: 2000}
  - {channel: 5, type: button, source: 0, mode: toggle, low: 1000, high: 2000}
  - {channel: 6, type: button, source: 2, mode: toggle, low: 1000, high: 2000}
  - {channel: 7, type: button, source: 1, mode: toggle, low: 1000, high: 2000}
```

`channel` 是 0 基下标，`channel: 0` 对应 RC1。`type: axis` 使用手柄轴线性映射到 PWM。`type: button` 支持按键映射；当前使用 `mode: toggle`，表示每次按下沿切换一次输出值，默认 `1000`，第一次按下变为 `2000`，再按一次回到 `1000`。

### 终端可视化

节点会在终端刷屏显示遥控器状态，包括：

- 当前手柄设备名
- Joy 话题和 RC override 话题
- 原始 axis/button 值
- 过滤后 `/joy` 数据
- 前 8 路 RC PWM 输出，分两行显示 RC1~RC4 和 RC5~RC8

显示开关和频率由以下 YAML 参数控制：

```yaml
print_status: true
print_rate: 10.0
```

调试映射时建议先观察终端显示和 `rostopic echo /mavros/rc/override`，确认方向和 PWM 值符合预期后再连接真机飞控执行动作。

</section>
