# sunray_monitor_tools

`sunray_monitor_tools` 提供一个 Qt + RViz 综合地面站，用于集中查看 Sunray UAV/UGV 的控制、定位、规划、集群状态，并下发常用控制指令。

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

## 界面能力

| 区域 | 说明 |
| --- | --- |
| `概览` | 按 UAV/UGV 列出在线状态、控制状态、定位状态、当前位置、速度、目标点和最近指令 |
| `控制` | 支持 UAV 起飞/悬停/降落/返航/KILL，UGV HOLD/返航，位置点、世界系速度、机体系速度，以及 UAV/UGV 集群快捷指令和阵型指令 |
| `详情` | 显示控制、定位、规划、集群状态的完整摘要，并保留通用话题健康表和操作日志 |
| `三维态势` | 内嵌 RViz，默认显示 Grid、TF 和 `/sunray/monitor/markers` 中的智能体、航向、目标点和连线 |

速度类单机指令会以 5 Hz 持续发布，点击 `停止速度` 后停止。

## 配置文件

默认配置文件：

```text
tools/sunray_monitor_tools/config/sunray_monitor.yaml
```

核心字段：

| 字段 | 说明 |
| --- | --- |
| `fixed_frame` | RViz 固定坐标系，默认 `world` |
| `marker_topic` | monitor 发布的三维态势 MarkerArray 话题 |
| `status_timeout` | 状态超时阈值，单位秒 |
| `refresh_hz` | UI 刷新频率 |
| `uav_agents` | UAV 列表，每个 agent 配置状态、控制、定位、规划话题 |
| `ugv_agents` | UGV 列表，每个 agent 配置状态、控制、定位话题 |
| `state_topics` | 详情页“话题健康”中额外监听的通用话题列表 |

默认示例覆盖 `uav1` 和 `ugv1`。多机时复制 agent 项并修改 `id`、`namespace` 和相关话题即可。

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
