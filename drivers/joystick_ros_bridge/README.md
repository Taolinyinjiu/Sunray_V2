# joystick_ros_bridge

读取 Linux joystick 设备，例如 `/dev/input/js0`，发布过滤后的 `sensor_msgs/Joy`，并默认发布 MAVROS 的 `mavros_msgs/OverrideRCIn` 到 PX4 遥控覆盖话题。

所有运行参数都在 YAML 中配置：

```text
drivers/joystick_ros_bridge/config/joystick_px4_mapping.yaml
```

## 运行

```bash
roslaunch joystick_ros_bridge joystick_ros_bridge.launch
```

也可以指定配置文件：

```bash
roslaunch joystick_ros_bridge joystick_ros_bridge.launch \
  config_file:=/home/yundrone/Sunray_v2/drivers/joystick_ros_bridge/config/joystick_px4_mapping.yaml
```

## 当前映射

PX4 RC 遵循 NED 坐标系，当前配置将手柄映射为：

| 手柄输入 | 含义 | RC 输出 |
| --- | --- | --- |
| axis0 | 偏航，-1 左转 | RC4=1000，+1 时 RC4=2000 |
| axis1 | 油门，反向，-1 上升 | RC3=2000，+1 时 RC3=1000 |
| axis2 | 俯仰，反向，-1 前进 | RC2=1000，+1 时 RC2=2000 |
| axis3 | 横滚，-1 右飞 | RC1=2000，+1 时 RC1=1000 |
| axis4/axis5 | 不使用 | Joy 和 RC 都不发布 |
| button3 X | 模式/功能通道 | RC5：每按下一次在 1000/2000 间切换 |
| button0 Y | 模式/功能通道 | RC6：每按下一次在 1000/2000 间切换 |
| button1 B | 模式/功能通道 | RC8：每按下一次在 1000/2000 间切换 |
| button2 A | 模式/功能通道 | RC7：每按下一次在 1000/2000 间切换 |

未映射的 RC 通道默认发布 `1000`。

> 注意：用户描述里第 9 条写的是 `button1` 对应 A 键，但第 8 条已经使用 `button1` 对应 B 键。这里按常见手柄布局假设 A 键是 `button2`。如果实测不同，只需要修改 YAML 中 RC7 的 `source`。

## 终端状态显示

节点会在终端刷新显示：

- 设备名、Joy 话题、RC override 话题
- 原始 axis/button 值
- 过滤后发布到 `/joy` 的值
- 18 路 MAVROS RC 输出 PWM

显示开关和刷新频率由 YAML 中的 `print_status`、`print_rate` 控制。
