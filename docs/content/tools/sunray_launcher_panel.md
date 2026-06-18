<!-- title: sunray_launcher_panel -->

<section id="tools-sunray-launcher-panel">

## sunray_launcher_panel

`sunray_launcher_panel` 是 Sunray 的图形化启动器工具包，用于集中启动定位、控制、规划、集群控制和仿真相关 ROS launch 文件。

启动器不会替代各模块自身 launch 文件，只负责从配置文件读取常用启动项，并在独立 terminal 中执行对应 `roslaunch` 命令。

### 启动方式

先编译并 source 当前工作空间：

```bash
cd ~/Sunray_v2
catkin_make
source devel/setup.bash
```

启动默认面板：

```bash
roslaunch sunray_launcher_panel sunray_launcher_panel.launch
```

### 配置文件

默认配置文件拆成两类：

```text
tools/sunray_launcher_panel/config/sunray_launch_groups.yaml
tools/sunray_launcher_panel/config/sunray_quick_launch_groups.yaml
```

`sunray_launch_groups.yaml` 配置模块列表和默认启动命令：

```yaml
external_workspaces: []

launch_groups:
  - name: "sunray_sim"
    items:
      - title: "sunray_sim"
        package: "sunray_sim"
        launch: "sunray_sim.launch"
        description: "启动一体化轻量仿真。"

  - name: "定位融合"
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

`sunray_quick_launch_groups.yaml` 配置一键启动脚本，只引用 `launch_groups` 中已有启动项：

```yaml
quick_launch_groups:
  - category: "sunray_sim"
    scripts:
      - title: "6机无人机集群控制验证"
        description: "sunray_sim 6机仿真 + 定位 + 控制 + 集群控制"
        items:
          - ref: "sunray_sim/sunray_sim_6机集群"
            delay_sec: 2.0
          - ref: "定位融合/集群无人机定位融合"
            delay_sec: 1.0
```

字段说明：

| 字段 | 必填 | 说明 |
| --- | --- | --- |
| `external_workspaces` | 否 | 外部 catkin workspace 列表，支持 `~/xxx` 路径；启动器会扫描这些 workspace 的 ROS package 并在启动 terminal 时补充 ROS 环境变量。 |
| `launch_groups` | 是 | 顶层数组，每一项对应界面上的一个模块标签页。 |
| `launch_groups[].name` | 是 | 模块标签页名称。 |
| `launch_groups[].items` | 是 | 当前模块下可启动的 launch 列表。 |
| `items[].title` | 是 | Launch 列表中显示的名称。 |
| `items[].package` | 是 | ROS package 名称，等价于 `roslaunch <package> ...`。 |
| `items[].launch` | 是 | launch 文件名；支持 package 的 `launch/` 子目录路径。 |
| `items[].args` | 否 | 默认 roslaunch 参数，格式为 `name:=value`。 |
| `items[].description` | 否 | 启动文件列表“功能描述”列显示的说明文字。 |
| `quick_launch_groups` | 否 | 一键启动脚本分类列表。 |
| `quick_launch_groups[].category` | 是 | 一键启动脚本分类标签，例如 `sunray_sim`、`Gazebo`、`真机`。 |
| `quick_launch_groups[].scripts` | 是 | 当前分类下的一键启动脚本列表。 |

新增启动项时，只需要在对应模块的 `items` 下面追加一项。启动器会自动生成默认命令：

```bash
roslaunch <package> <launch> <args...>
```

### 界面说明

启动器界面主要分为四块：

| 区域 | 作用 |
| --- | --- |
| `一键启动脚本` | 按 `sunray_sim / Gazebo / 真机` 等标签显示常用联调脚本，一键在同一个 Ubuntu Terminal 窗口中打开多个 tab，每个 tab 启动一个 launch。 |
| `启动文件列表` | 按模块标签页分类显示可启动的 launch，表格包含 `标题 / 状态 / ROS Package / launch文件名 / 功能描述`。选中一键启动脚本时，即将被启动的 launch 行、对应模块 tab 和选中项本身会显示预览高亮；实际运行中的 launch 会显示绿色高亮和更明显的前缀标记。 |
| `操作日志` | 显示启动器内部日志，强调启动、停止、退出码、预览提示等关键事件；不实时复制 roslaunch 的 stdout/stderr。 |
| `系统状态监控` | 显示 CPU、内存、当前 ROS node 列表，并提供 `停止全部 Launch` 按钮。 |

启动器默认允许同时启动多个不同 launch。每个启动项只限制自己不能重复启动。

启动器打开后默认不选中任何一键启动脚本或模块 launch。点击某一行会选中，再次点击同一行或点击列表空白区域会取消选中。

启动命令编辑框支持直接修改参数，例如：

```bash
roslaunch localization_fusion localization_fusion.launch source_id:=5 agent_name:=uav agent_id:=2
```

启动时，程序会解析命令中的 `package` 和 `launch`，并在内部转换为绝对 launch 文件路径执行。这样可以避免带子目录的 launch 文件在 terminal 中解析失败。

### 一键启动

`一键启动脚本` 位于启动文件列表上方，用来把一组常用 launch 编排成一个脚本，例如单无人机控制联调、单无人车控制联调、6机/6车集群仿真等。

一键启动脚本支持按标签分类显示。当前默认配置包含 `sunray_sim`、`Gazebo`、`真机` 三类，其中常用 UAV/UGV 仿真联调脚本都归入 `sunray_sim`，`Gazebo` 和 `真机` 先保留占位脚本，后续可以直接在配置文件中扩展。

一键启动使用 Ubuntu 自带的 `gnome-terminal --tab` 打开一个 terminal 窗口，并让每个 launch 独占一个 terminal tab。这样不会把多个 launch 的输出混在一起，直接点击 terminal 顶部标签即可切换。

启动器内部使用 `gnome-terminal --command` 为每个 tab 绑定独立脚本，并持续把 terminal tab 标题设置为对应启动项的中文标题，避免标题被 shell 或 roslaunch 刷新成脚本名/launch 文件名。实际执行时内部会把 launch 解析为绝对路径以保证可靠性，但 terminal 顶部和操作日志中的 `Command` 仍显示为 `roslaunch package file.launch args` 的可读形式。

一键启动配置示例：

```yaml
quick_launch_groups:
  - category: "sunray_sim"
    scripts:
      - title: "单无人机控制联调"
        description: "sunray_sim 单机 UAV 仿真 + 定位融合 + UAV 控制"
        items:
          - ref: "sunray_sim/sunray_sim"
            delay_sec: 1.0
          - ref: "定位融合/无人机定位融合"
            delay_sec: 1.0
          - ref: "无人机控制/单机无人机控制"
```

一键启动脚本字段：

| 字段 | 必填 | 说明 |
| --- | --- | --- |
| `title` | 是 | 脚本名称，显示在一键启动脚本列表中。 |
| `description` | 否 | 脚本功能描述。 |
| `items` | 是 | 当前脚本按顺序启动的 launch 步骤。 |
| `items[].ref` | 是 | 引用已有模块启动项，格式为 `模块名/Launch标题`。 |
| `items[].delay_sec` | 否 | 当前步骤启动后，到下一个步骤启动前等待的秒数；第一步立即启动，后续步骤按前面步骤的 `delay_sec` 累积延迟启动。 |

一键启动默认使用被引用启动项在 `sunray_launch_groups.yaml` 中配置的参数。如果需要临时修改某一步参数，先在启动文件列表中选中对应 launch，编辑底部启动命令，再启动一键启动脚本；启动器会优先使用你刚编辑过的命令。

### 启动和停止机制

点击 `启动` 后，启动器会打开一个独立 terminal，并在其中执行对应 `roslaunch`。

点击 `停止` 后，启动器会向对应 roslaunch wrapper 发送 `SIGINT`，wrapper 会继续转发给 roslaunch。

点击一键启动的 `一键停止` 后，启动器会向该一键启动脚本对应的 terminal tab 脚本发送 `SIGINT`，各 tab 中的 roslaunch 会收到中断请求。

手动关闭 terminal 时，wrapper 会收到 `SIGHUP/TERM/INT` 并转发给 roslaunch，避免出现 terminal 已关闭但 ROS 节点仍残留的情况。

roslaunch 结束后 terminal 会自动关闭，不需要手动按回车。

### 外部 Workspace 适配

默认配置不再依赖外部 `pengyu_sim` workspace，`sunray_sim` 已经放在当前 Sunray_v2 工作空间内。`external_workspaces` 默认保持空列表：

```yaml
external_workspaces: []
```

如果后续确实需要接入外部 catkin workspace，可以在 `external_workspaces` 中补充路径。启动器会做两件事：

| 动作 | 说明 |
| --- | --- |
| 启动器内部解析 | 扫描外部 workspace 下的 `package.xml`，即使当前终端只 source 了 `~/Sunray_v2/devel/setup.bash`，也能找到外部包的 launch 文件。 |
| terminal 环境补充 | 打开独立 terminal 后，自动给 `ROS_PACKAGE_PATH`、`CMAKE_PREFIX_PATH`、`PATH`、`LD_LIBRARY_PATH`、`PYTHONPATH` 补充外部 workspace 的路径。 |

这个机制只用于兼容外部独立 workspace。当前推荐直接使用当前仓库内的 `sunray_sim`。

如果外部包仍然启动失败，先检查路径是否存在：

```bash
ls <external_workspace>/devel
ls <external_workspace>/src
```

### 常见问题

#### 启动项显示未运行，但节点还在

优先用以下命令确认节点是否来自当前启动器启动的 terminal：

```bash
rosnode list
ps -ef | grep roslaunch
```

如果节点不是通过启动器当前 terminal 启动，启动器无法管理它，需要手动停止对应 roslaunch 或节点。

#### 修改 YAML 后界面没有变化

需要重新启动启动器。配置文件是在启动器节点启动时由 roslaunch 加载到私有参数空间的。

#### 找不到 launch 文件

检查三点：

```bash
rospack find <package>
ls $(rospack find <package>)/launch
```

如果 launch 不在 package 的 `launch/` 目录下，`launch` 字段需要写相对 package 根目录的路径。

#### SSH 环境下打不开界面

Qt 界面需要图形环境。通过 SSH 使用时，需要开启 X11 转发或在本机显示：

```bash
ssh -X user@host
```

如果只需要启动模块而不需要图形界面，建议直接使用对应模块的 `roslaunch` 命令。

### 相关文件

```text
tools/sunray_launcher_panel/
├── config/sunray_launch_groups.yaml       # 模块列表和默认 launch 命令
├── config/sunray_quick_launch_groups.yaml # 一键启动脚本
├── launch/sunray_launcher_panel.launch    # 启动器 launch
├── logo/                                  # 界面 logo 和窗口图标
├── src/tools/sunray_launcher_node.cpp
├── CMakeLists.txt
└── package.xml
```

</section>
