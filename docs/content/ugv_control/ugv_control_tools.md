<!-- title: ugv_control_tools -->

<section id="ugv-control-tools">

## ugv_control_tools

`ugv_control/ugv_control_tools` 是 UGV 控制调试工具包，目前主要提供无人车终端控制节点。它用于手动发布 `sunray_msgs/UGVControlCMD`，验证 `sunray_ugv_control` 的命令入口是否工作。

正式二次开发时，更推荐优先参考：

```text
ugv_control/sunray_ugv_control_example/
```

示例程序更接近真实业务节点；终端工具适合临时调试和排查接口。

### 目录结构

```text
ugv_control/ugv_control_tools/
├── launch/
│   └── ugv_terminal_control.launch
├── package.xml
├── CMakeLists.txt
└── src/
    └── terminal_control/
        └── ugv_terminal_control.cpp
```

### UGV 终端控制

启动：

```bash
roslaunch ugv_control_tools ugv_terminal_control.launch agent_name:=ugv agent_id:=1
```

源码：

```text
ugv_control/ugv_control_tools/src/terminal_control/ugv_terminal_control.cpp
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
| `HOLD` | 发一次即可 | 停车并保持。 |
| `MOVE_POINT` | 发一次即可 | 本地坐标目标点。 |
| `MOVE_VELOCITY` | 持续发布 | 世界系速度，适合麦克纳姆轮；差速轮默认不支持。 |
| `MOVE_VELOCITY_BODY` | 持续发布 | 车体系速度，差速轮建议使用该命令。 |

速度类命令需要持续发布；停车和点位命令通常发一次即可。调试时可以同时查看：

```bash
rostopic echo /ugv1/sunray/ugv_control/control_state
rostopic echo /ugv1/sunray/ugv_control/cmd_vel
```

</section>
