# simulation_tools

`simulation_tools` 是 Sunray 的图形化启动器工具包，用于集中启动定位、控制、规划、集群控制和仿真相关 ROS launch 文件。

启动器不会替代各模块自身 launch 文件，只负责从配置文件读取常用启动项，并在独立 terminal 中执行对应 `roslaunch` 命令。

## 1. 启动方式

先编译并 source 当前工作空间：

```bash
cd ~/Sunray_v2
catkin_make
source devel/setup.bash
```

启动默认面板：

```bash
roslaunch simulation_tools sunray_launcher_panel.launch
```

## 2. 配置文件

默认配置文件：

```text
tools/simulation_tools/config/sunray_launcher.yaml
```

配置结构如下：

```yaml
launch_groups:
  - name: "定位融合模块"
    items:
      - title: "无人机定位融合"
        package: "localization_fusion"
        launch: "localization_fusion.launch"
        args:
          - "source_id:=5"
          - "agent_name:=uav"
          - "agent_id:=1"
        description: "启动单架无人机 localization_fusion。"
```

字段说明：

| 字段 | 必填 | 说明 |
| --- | --- | --- |
| `launch_groups` | 是 | 顶层数组，每一项对应界面上的一个模块标签页 |
| `name` | 是 | 模块标签页名称 |
| `items` | 是 | 当前模块下可启动的 launch 列表 |
| `title` | 是 | Launch 列表中显示的名称 |
| `package` | 是 | ROS package 名称，等价于 `roslaunch <package> ...` |
| `launch` | 是 | launch 文件名；支持 package 的 `launch/` 子目录路径，例如 `launch_uav_demo/sunray_sim_1uav.launch` |
| `args` | 否 | 默认 roslaunch 参数，格式为 `name:=value` |
| `description` | 否 | 启动控制区显示的说明文字 |

新增启动项时，只需要在对应模块的 `items` 下面追加一项。启动器会自动生成默认命令：

```bash
roslaunch <package> <launch> <args...>
```

## 3. 界面说明

启动器界面主要分为四块：

| 区域 | 作用 |
| --- | --- |
| `模块列表` | 按模块标签页分类显示可启动的 launch。状态列只显示 `未运行` 或 `运行 Ns`，正在运行的 launch 行会显示绿色背景。 |
| `启动控制` | 显示当前选中项的标题和描述；下方命令行可直接编辑，最终执行的就是这条命令。 |
| `INFO` | 显示启动器内部日志，例如完整启动命令、停止请求、退出码等。 |
| `系统状态监控` | 显示 CPU、内存、当前 ROS node 列表，并提供 `停止全部 Launch` 和 `清空日志` 按钮。 |

启动器默认允许同时启动多个不同 launch。每个启动项只限制自己不能重复启动。

启动命令编辑框支持直接修改参数，例如：

```bash
roslaunch localization_fusion localization_fusion.launch source_id:=5 agent_name:=uav agent_id:=2
```

启动时，程序会解析命令中的 `package` 和 `launch`，并在内部转换为绝对 launch 文件路径执行。这样可以避免带子目录的 launch 文件在 terminal 中解析失败。

## 4. 启动和停止机制

点击 `启动` 后，启动器会打开一个独立 terminal，并在其中执行对应 `roslaunch`。

点击 `停止` 后，启动器会向对应 roslaunch wrapper 发送 `SIGINT`，wrapper 会继续转发给 roslaunch。

手动关闭 terminal 时，wrapper 会收到 `SIGHUP/TERM/INT` 并转发给 roslaunch，避免出现 terminal 已关闭但 ROS 节点仍残留的情况。

roslaunch 结束后 terminal 会自动关闭，不需要手动按回车。

## 5. 外部包注意事项

`pengyu_sim仿真器模块` 中的 `px4_control_simulator` 和 `ugv_simulator` 是外部 ROS 包。使用这些启动项前，需要先 source 外部工作空间，例如：

```bash
source /home/amov/pengyu_sim/devel/setup.bash
source /home/amov/Sunray_v2/devel/setup.bash
```

如果 `rospack find px4_control_simulator` 或 `rospack find ugv_simulator` 找不到包，启动器会提示找不到 launch 文件。

## 6. 常见问题

### 启动项显示未运行，但节点还在

优先用以下命令确认节点是否来自当前启动器启动的 terminal：

```bash
rosnode list
ps -ef | grep roslaunch
```

如果节点不是通过启动器当前 terminal 启动，启动器无法管理它，需要手动停止对应 roslaunch 或节点。

### 修改 YAML 后界面没有变化

需要重新启动启动器。配置文件是在启动器节点启动时由 roslaunch 加载到私有参数空间的。

### 找不到 launch 文件

检查三点：

```bash
rospack find <package>
ls $(rospack find <package>)/launch
```

如果 launch 不在 package 的 `launch/` 目录下，`launch` 字段需要写相对 package 根目录的路径。

### SSH 环境下打不开界面

Qt 界面需要图形环境。通过 SSH 使用时，需要开启 X11 转发或在本机显示：

```bash
ssh -X user@host
```

如果只需要启动模块而不需要图形界面，建议直接使用对应模块的 `roslaunch` 命令。

## 7. 相关文件

```text
tools/simulation_tools/
├── config/sunray_launcher.yaml          # 启动项配置
├── launch/sunray_launcher_panel.launch  # 启动器 launch
├── logo/                                # 界面 logo 和窗口图标
├── src/tools/sunray_launcher_node.cpp
├── CMakeLists.txt
└── package.xml
```
