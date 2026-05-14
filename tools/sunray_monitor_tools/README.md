# sunray_monitor_tools

`sunray_monitor_tools` 是 Sunray 的运行态监控工具包。当前初版提供一个 Qt 面板：

- 左侧表格监听多个模块状态话题，显示在线状态、频率、延迟、消息类型和话题名。
- 右侧嵌入 RViz 三维视图，默认显示 Grid、TF 和 `/sunray/monitor/markers`。
- 配置文件驱动，可同时覆盖 UAV、UGV、Planning、Swarm 等模块。

当前版本先做“状态话题聚合 + 三维视图框架”。后续可以在此基础上继续解析 `UAVControlState.msg`、`UAVPlanningState.msg`、`UGVControlState.msg` 等具体字段。

## 启动方式

```bash
cd /home/amov/Sunray_v2
source devel/setup.bash
roslaunch sunray_monitor_tools sunray_monitor_panel.launch
```

指定配置文件：

```bash
roslaunch sunray_monitor_tools sunray_monitor_panel.launch \
  monitor_config:=/home/amov/Sunray_v2/tools/sunray_monitor_tools/config/sunray_monitor.yaml
```

也可以从 `simulation_tools` 的 Sunray 启动器中启动：

```bash
roslaunch simulation_tools sunray_launcher_panel.launch
```

然后在 `Gazebo仿真器模块` 标签页中选择 `Sunray状态监控面板`。

## 配置文件

默认配置文件：

```text
tools/sunray_monitor_tools/config/sunray_monitor.yaml
```

示例：

```yaml
fixed_frame: "world"
status_timeout: 1.0
refresh_hz: 5.0

state_topics:
  - name: "UAV控制-uav1"
    module: "UAV_CONTROL"
    agent_type: "UAV"
    agent_name: "uav"
    agent_id: 1
    topic: "/uav1/sunray/uav_control/control_state"
```

字段说明：

| 字段 | 说明 |
| --- | --- |
| `fixed_frame` | RViz 固定坐标系，默认 `world` |
| `status_timeout` | 状态话题超时阈值，单位秒；超过该时间未收到消息则显示离线 |
| `refresh_hz` | Qt 表格刷新频率 |
| `state_topics` | 要监听的状态话题列表 |
| `name` | 表格中显示的名称 |
| `module` | 模块类型，例如 `UAV_CONTROL`、`UGV_CONTROL`、`UAV_PLANNING`、`UAV_SWARM` |
| `agent_type` | 智能体类型，例如 `UAV`、`UGV`、`SWARM` |
| `agent_name` | 智能体名前缀，例如 `uav`、`ugv` |
| `agent_id` | 智能体编号；集群聚合话题可填 `0` |
| `topic` | 实际订阅的话题名 |

## 界面说明

| 区域 | 说明 |
| --- | --- |
| `状态话题监控` | 显示每个配置话题的在线状态、频率、延迟、消息类型和完整话题名 |
| `RViz 三维视图` | 嵌入式 RViz RenderPanel，当前默认加载 Grid、TF 和 MarkerArray |

在线状态颜色：

- 绿色：最近 `status_timeout` 秒内收到消息。
- 橙色：尚未收到消息或已超时。

## 后续扩展建议

第一阶段：

- 针对 `UAVControlState`、`UGVControlState`、`UAVPlanningState` 增加强类型订阅和字段解析。
- 表格增加 FSM 状态、目标点、当前命令、odom 健康状态等列。

第二阶段：

- 发布统一 `/sunray/monitor/markers`，在三维视图中显示 UAV/UGV 模型、轨迹、目标点、任务文本。
- 从配置文件生成多机状态监听列表，例如 `agent_name=uav, agent_num=6` 自动展开 `/uav1` 到 `/uav6`。

第三阶段：

- 和 `simulation_tools` 启动器联动，支持一键启动推荐联调流程。
- 增加异常告警，例如状态超时、规划失败、定位丢失、地理围栏越界。

## 相关文件

```text
tools/sunray_monitor_tools/
├── config/sunray_monitor.yaml
├── launch/sunray_monitor_panel.launch
├── rviz/sunray_monitor.rviz
├── src/sunray_monitor_panel_node.cpp
├── CMakeLists.txt
└── package.xml
```

