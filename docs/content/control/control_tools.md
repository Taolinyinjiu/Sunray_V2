<!-- title: control_tools -->

<section id="control-tools">

## control_tools

`control/control_tools` 是 Sunray 控制模块的通用调试工具包。它不实现无人机或无人车的底层控制律，而是提供终端、面板和测试节点，用来向 `sunray_uav_control`、`sunray_ugv_control` 发布控制命令并观察状态。

对小白开发者来说，这个包适合用来“验证接口是否通”，但正式二次开发更推荐参考 `examples/` 中的示例包：

```text
examples/sunray_uav_control_example/
examples/sunray_ugv_control_example/
```

工具节点通常有较多交互逻辑和 UI 代码，不如示例程序适合直接复制成业务节点。

### 目录结构

```text
control/control_tools/
├── include/
│   └── terminal_control/
│       └── terminal_control.hpp
├── launch/
│   ├── uav_control_panel.launch
│   └── ugv_terminal_control.launch
├── resources/
│   ├── control_tools.qrc
│   └── icons/
└── src/
    ├── terminal_control/
    │   ├── uav_terminal_control.cpp
    │   └── ugv_terminal_control.cpp
    ├── uav_control_panel_node.cpp
    └── velocity_test/
        └── triangle_velocity_test.cpp
```

### 工具列表

| 工具 | 代码 | 用途 |
| --- | --- | --- |
| UAV 终端控制 | `src/terminal_control/uav_terminal_control.cpp` | 终端交互式发布 `UAVControlCMD`。 |
| UGV 终端控制 | `src/terminal_control/ugv_terminal_control.cpp` | 终端交互式发布 `UGVControlCMD`。 |
| UAV 控制面板 | `src/uav_control_panel_node.cpp` | Qt 面板，发布 UAV 命令、订阅控制状态和里程计，并发布 RViz marker。 |
| 三角速度测试 | `src/velocity_test/triangle_velocity_test.cpp` | 自动执行起飞、速度闭环三角轨迹、悬停和降落，用于压力测试速度控制链路。 |

### UAV 终端控制

源码：

```text
control/control_tools/src/terminal_control/uav_terminal_control.cpp
```

功能：在终端里选择命令类型，发布到：

```text
/uav1/sunray/uav_control/control_cmd
```

消息类型：

```text
sunray_msgs/UAVControlCMD
```

参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `uav_id` | `1` | 无人机编号，工具会拼出 `/uav<id>/sunray/uav_control/control_cmd`。 |

支持的典型命令：

| 命令 | 发布规则 | 说明 |
| --- | --- | --- |
| `TAKEOFF` | 发一次即可 | 起飞。 |
| `LAND` | 发一次即可 | 降落。 |
| `RETURN` | 发一次即可 | UAV 控制层当前仍有返航命令。 |
| `MOVE_POINT` | 发一次即可 | 输入目标位置和 yaw。 |
| `MOVE_POINT_BODY` | 发一次即可 | 输入机体系相对位置和固定高度。 |
| `MOVE_VELOCITY` | 持续发布 | 工具会按持续时间循环发布速度命令。 |
| `MOVE_VELOCITY_BODY` | 持续发布 | 工具会按持续时间循环发布机体系速度命令。 |

速度类命令必须持续发布；位置、起降、悬停类命令通常发一次即可。这个规则应与 `UAVControlCMD.msg`、控制面板和终端工具保持一致。

### UGV 终端控制

启动：

```bash
roslaunch control_tools ugv_terminal_control.launch agent_name:=ugv agent_id:=1
```

源码：

```text
control/control_tools/src/terminal_control/ugv_terminal_control.cpp
```

发布话题：

```text
/ugv1/sunray/ugv_control/control_cmd
```

消息类型：

```text
sunray_msgs/UGVControlCMD
```

launch 参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `agent_name` | `ugv` | 车辆名前缀。 |
| `agent_id` | `1` | 车辆编号。 |
| `node_name` | `ugv_terminal_control_node` | ROS 节点名。 |
| `launch_prefix` | 空 | 可用于 `xterm -e`、调试前缀等。 |

支持命令：

| 命令 | 发布规则 | 说明 |
| --- | --- | --- |
| `HOLD` | 发一次即可 | 停车。 |
| `MOVE_POINT` | 发一次即可 | 本地坐标目标点。 |
| `MOVE_VELOCITY` | 持续发布 | 世界系速度，适合麦克纳姆轮；差速轮默认不支持。 |
| `MOVE_VELOCITY_BODY` | 持续发布 | 车体系速度，差速轮建议使用该命令。 |

### UAV 控制面板

启动：

```bash
roslaunch control_tools uav_control_panel.launch uav_id:=1
```

源码：

```text
control/control_tools/src/uav_control_panel_node.cpp
```

默认参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `uav_id` | `1` | 控制 `/uav1`。 |
| `cmd_topic` | `/uav1/sunray/uav_control/control_cmd` | UAV 控制命令输出。 |
| `fsm_state_topic` | `/uav1/sunray/uav_control/control_state` | 控制状态输入。 |
| `odom_topic` | `/uav1/sunray/localization/local_odom` | 里程计输入，用于显示当前位置。 |
| `rviz_frame_id` | `world` | marker frame。 |
| `marker_topic` | `/uav1/sunray/uav_control_panel/markers` | 面板发布的 RViz marker。 |

面板会：

- 发布 `sunray_msgs/UAVControlCMD`。
- 订阅 `sunray_msgs/UAVControlState`。
- 订阅 `nav_msgs/Odometry`。
- 发布 `visualization_msgs/MarkerArray` 用于 RViz 辅助显示。

面板中的速度命令和轨迹类命令按持续发布处理；起飞、降落、悬停、位置点命令按单次发布处理。

### 三角速度测试

源码：

```text
control/control_tools/src/velocity_test/triangle_velocity_test.cpp
```

用途：自动测试 UAV 速度控制链路。它会等待状态、发起飞命令、按三角形路径持续发布 `MOVE_VELOCITY`，最后悬停并降落。

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `uav_id` | `1` | 无人机编号。 |
| `uav_name` | `uav` | 无人机名前缀。 |
| `rate_hz` | `20.0` | 速度命令发布频率。 |
| `triangle_radius` | `1.0` | 三角轨迹半径。 |
| `num_points_per_edge` | `20` | 每条边采样点数量。 |
| `k_p_xy` | `1.0` | XY 速度控制比例系数。 |
| `k_p_z` | `0.5` | Z 速度控制比例系数。 |
| `max_vel_xy` | `1.0` | XY 最大速度。 |
| `max_vel_z` | `0.6` | Z 最大速度。 |
| `pos_tol_xy` | `0.15` | XY 到点误差。 |
| `pos_tol_z` | `0.15` | Z 到点误差。 |
| `takeoff_wait_timeout_s` | `30.0` | 起飞等待超时。 |
| `land_wait_timeout_s` | `30.0` | 降落等待超时。 |

订阅：

```text
/uav1/sunray/uav_control/control_state
/uav1/sunray/px4_state
```

发布：

```text
/uav1/sunray/uav_control/control_cmd
```

### 二次开发建议

- 想写自己的任务节点时，优先复制 `examples/sunray_uav_control_example` 或 `examples/sunray_ugv_control_example`。
- 想做手动调试时，用本包的终端工具和面板。
- 想验证速度控制稳定性时，用 `triangle_velocity_test`。
- 不建议把 UI 工具直接改成正式业务逻辑；工具节点通常混合了输入解析、显示、日志和命令发布，不利于长期维护。

</section>
