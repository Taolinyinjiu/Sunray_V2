<!-- title: scripts_manage -->

<section id="tools-scripts-manage">

## scripts_manage

`scripts_manage` 是独立于 `sunray_system` 的新版终端 GUI。它复用 `sunray_system` 已有的 ROS 服务接口，不直接解析 feature YAML。

界面基于 FTXUI 实现，不再使用 Python curses。FTXUI 会在终端窗口尺寸变化时重新计算布局，避免旧 TUI 在鼠标缩放终端后出现残留、错位和越界绘制。

### 构建

本包直接引用：

```text
tools/build_scripts/tui/third_party/ftxui
```

如果依赖目录不存在，先执行：

```bash
tools/build_scripts/tui/check_dependencies.sh
```

然后按工程原有方式编译 catkin workspace。

### 运行

先启动 `sunray_system`：

```bash
roslaunch sunray_system sunray_system.launch
```

如果按机型加载 feature 配置，TUI 顶部会显示当前机型，功能详情中也会显示该功能对应的机型：

```bash
roslaunch sunray_system sunray_system.launch \
  airframe_type:=sunray_150
```

再启动新版 GUI：

```bash
rosrun scripts_manage scripts_manage_tui
```

也可以使用 launch：

```bash
roslaunch scripts_manage scripts_manage_tui.launch
```

### 依赖接口

```text
/sunray_system/list_features
/sunray_system/get_features
/sunray_system/start_feature
/sunray_system/stop_feature
/sunray/system_info
```

这些服务和话题由 `sunray_system` 提供。因此新增可管理功能时，应修改 `sunray_system` 的 feature YAML，而不是在 TUI 中写死启动命令。

### 快捷键

| 快捷键 | 作用 |
| --- | --- |
| `↑/↓` | 移动当前列表选择。 |
| `←/→` 或 `Tab` | 切换分组/功能列表焦点。 |
| `Enter` / `F1` | 启动当前功能。 |
| `F2` | 停止当前功能。 |
| `F3` | 切换后台启动/终端启动。 |
| `F5` | 刷新。 |
| `Esc` / `q` | 退出。 |

### 二次开发

- 需要新增可管理功能时，改 `sunray_system` 的 feature YAML，不需要改 TUI。
- 需要新增 UI 字段时，优先扩展 `GetFeatures.srv` 或 `SystemInfo.msg`，再更新 TUI 展示。
- 如果需要新增启动模式，应先扩展 `StartFeature.srv` 或 `sunray_system` 的启动能力，再让 TUI 调用。

</section>
