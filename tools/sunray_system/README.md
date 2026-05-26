# sunray_system

`sunray_system` 是面向真机部署的 Sunray 常驻 supervisor，用来按“功能(feature)”维度启动、停止和查询一组 ROS launch。

## 功能

- 从 YAML 配置读取 feature 列表
- 通过 ROS 服务启动/停止 feature
- 支持 feature 依赖
- 维护每个 feature 对应的 launch 子进程
- 发布 `/sunray/system_info` 话题，提供系统级运行信息
- 可配合 `scripts_manage` 提供独立终端管理界面

## 启动

```bash
cd ~/Sunray_v2
./build.sh -y sunray_msgs sunray_system
source devel/setup.bash
roslaunch sunray_system sunray_system.launch
```

## 服务

```bash
rosservice call /sunray_system/list_features
rosservice call /sunray_system/start_feature "feature_name: 'single_uav_basic'
override_args: []
restart_if_running: false
start_with_terminal: false"
rosservice call /sunray_system/get_features "feature_name: 'single_uav_basic'"
rosservice call /sunray_system/stop_feature "feature_name: 'single_uav_basic'
force: false"
```

### `StartFeature.srv`

```srv
string feature_name
string[] override_args
bool restart_if_running
bool start_with_terminal
---
bool success
string message
```

作用：

- 启动一个 feature

字段说明：

- `feature_name`：要启动的 feature 名称
- `override_args`：追加到该 feature 各个 launch 单元后的动态参数，格式为 `name:=value`
- `restart_if_running`：如果 feature 已经在运行，是否先停掉再重新启动
- `start_with_terminal`：是否通过独立终端可视化启动。`false` 表示后台启动；`true` 表示为当前 feature 打开一个终端窗口，并按 launch 单元拆分为多个 tab 执行；如果存在 `depends_on`，则会先按依赖顺序把依赖 feature 的 launch 单元也放进同一个窗口中
- `success`：启动是否成功
- `message`：返回说明

### `StopFeature.srv`

```srv
string feature_name
bool force
---
bool success
string message
```

作用：

- 停止一个 feature

字段说明：

- `feature_name`：要停止的 feature 名称
- `force`：是否直接强制结束；当前实现中 `false` 会先优雅退出，超时后仍会升级为强制结束
- `success`：停止是否成功
- `message`：返回说明

### `ListFeatures.srv`

```srv
---
string[] feature_names
```

作用：

- 返回当前 `sunray_system` 已加载的所有 feature 名称
- 适合做功能列表、功能选择器

说明：

- 当前只返回 `feature_names`
- 不再返回 `running`、`description`
- 这些更适合通过 `GetFeatures` 查询单个 feature 时获取

### `GetFeatures.srv`

```srv
string feature_name
---
bool success
string message
string name
string group
bool running
string description
bool auto_start
string[] depends_on
float32 stop_timeout_sec
string[] start_preview_units
```

作用：

- 查询单个 feature 的当前状态和关键配置

字段说明：

- `feature_name`：请求时传入的目标 feature 名称
- `success`：查询是否成功
- `message`：返回说明，例如 `ok` 或 `unknown feature: xxx`
- `name`：该 feature 的名称
- `group`：该 feature 所属分组，供管理界面分组展示
- `running`：该 feature 当前是否在运行
- `description`：该 feature 的描述
- `auto_start`：该 feature 是否配置为 `sunray_system` 启动时自动启动
- `depends_on`：该 feature 依赖的其他 feature 名称列表
- `stop_timeout_sec`：停止该 feature 时的优雅退出等待时间
- `start_preview_units`：启动预览列表，表示启动该 feature 时会按顺序拉起的 launch 单元；若包含依赖，依赖项也会出现在这个列表中

## 话题

```bash
rostopic echo /sunray/system_info
```

`/sunray/system_info` 当前包含：

- CPU 占用率
- 内存占用率
- 当前活跃的 ROS 节点名列表

## 管理界面

旧版 `sunray_system` 内置 TUI 已移除。新的终端管理界面独立放在：

```text
tools/scripts_manage
```

启动方式：

```bash
cd ~/Sunray_v2
source devel/setup.bash
rosrun scripts_manage scripts_manage_tui
```

或：

```bash
roslaunch scripts_manage scripts_manage_tui.launch
```

## 配置

默认配置文件：

```text
tools/sunray_system/config/features.yaml
```

每个 feature 由若干 launch 单元组成：

```yaml
features:
  - name: "single_uav_basic"
    description: "单机无人机基础链路：定位融合 + 飞控控制"
    group: "单机无人机"
    auto_start: false
    depends_on: []
    stop_timeout_sec: 8.0
    launches:
      - name: "localization"
        package: "localization_fusion"
        file: "localization_fusion.launch"
        args:
          - "agent_id:=1"
```

`override_args` 为启动时动态附加参数，会追加到 feature 里每个 launch 单元的参数末尾。

`sunray_system` 在实际拉起 launch 时，会先统一注入 ROS 运行环境，再执行 `roslaunch`。默认会依次加载：

- `ros_setup_file`
- `workspace_setup_file`
- `external_workspaces` 对应的环境补充（如有）

这样即使在 systemd、自启动或机载原生终端环境中，也不会依赖当前 shell 是否手动 `source` 过工作空间。

### 字段说明

- `name`：feature 的唯一标识符。`sunray_system` 内部用它做索引，服务调用里的 `feature_name`、`depends_on` 引用的也是它。
- `description`：给管理界面和状态展示用的中文说明。
- `group`：功能分组名称，供管理界面左侧按组展示。建议写中文短名称，例如 `单机无人机`、`地面站`、`基础模块`。
- `auto_start`：当 `sunray_system` 节点启动时，是否自动启动这个 feature。
- `depends_on`：当前 feature 依赖的其他 feature 名称列表。启动当前 feature 时会先启动依赖项。
- 当使用 `start_with_terminal=true` 时，依赖项不会再各自弹出一个新终端窗口，而是和主 feature 一起合并到同一个 `gnome-terminal` 窗口中，按顺序拆成多个 tab。
- `stop_timeout_sec`：停止 feature 时的优雅退出等待时间；超时后会升级为强制结束。
- `launches`：当前 feature 包含的 launch 单元列表。
- `launches[].name`：当前 launch 单元的显示名。
- `launches[].package`：ROS package 名称。
- `launches[].file`：launch 文件名，默认从对应 package 的 `launch/` 目录查找。
- `launches[].args`：传给 `roslaunch` 的参数列表，格式为 `name:=value`。
- `launches[].delay_sec`：当前 launch 单元启动后，到下一个 launch 单元启动前的延迟秒数。

### 启动环境参数

`sunray_system.launch` 现在支持以下参数：

- `ros_setup_file`：ROS 发行版环境脚本，默认 `/opt/ros/noetic/setup.bash`
- `workspace_setup_file`：当前工作空间环境脚本，默认 `/home/yundrone/Sunray_v2/devel/setup.bash`
- `external_workspaces`：可选的外部 workspace 列表，用于补充 `ROS_PACKAGE_PATH`、`CMAKE_PREFIX_PATH`、`PATH`、`LD_LIBRARY_PATH`、`PYTHONPATH`

示例：

```bash
roslaunch sunray_system sunray_system.launch \
  workspace_setup_file:=/home/yundrone/Sunray_v2/devel/setup.bash \
  external_workspaces:="['/home/yundrone/pengyu_sim']"
```

### `name` 能不能写中文

技术上可以。当前实现只要求：

- `name` 非空
- `name` 在整个 `features` 列表里唯一

代码里没有限制 `name` 必须是英文或 ASCII。

但目前不建议把 `name` 写成中文，原因是：

- `name` 是内部标识符，不只是显示文本。
- `depends_on` 要精确引用它。
- `start_feature / stop_feature` 服务也要传这个名字。
- 后面如果接脚本、外部面板、日志检索，英文标识更稳。

建议做法：

- `name`：用稳定的英文标识，例如 `single_uav_basic`
- `description`：用中文说明，例如 `单机无人机基础链路`

如果你后面明确希望管理界面左侧直接显示中文名称，更合适的方式是后续增加一个单独的 `display_name` 字段，而不是直接把 `name` 改成中文。

### 如何新增一个 feature

1. 打开配置文件：

```text
tools/sunray_system/config/features.yaml
```

2. 在 `features:` 下新增一项，至少包含：

- `name`
- `description`
- `launches`

最小示例：

```yaml
features:
  - name: "single_ugv_basic"
    description: "单车无人车基础链路"
    auto_start: false
    stop_timeout_sec: 8.0
    launches:
      - name: "ugv_control"
        package: "sunray_ugv_control"
        file: "ugv_control.launch"
        args:
          - "agent_name:=ugv"
          - "agent_id:=1"
```

3. 如果这个 feature 依赖别的 feature，补上：

```yaml
depends_on:
  - "single_uav_basic"
```

4. 保存后，重启 `sunray_system`，让它重新加载配置。

```bash
roslaunch sunray_system sunray_system.launch
```

或者如果当前已经在运行，先结束再重新启动。
