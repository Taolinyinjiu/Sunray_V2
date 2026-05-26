# scripts_manage

`scripts_manage` 是独立于 `sunray_system` 的新版终端 GUI。它复用
`sunray_system` 已有的 ROS 服务接口：

- `/sunray_system/list_features`
- `/sunray_system/get_features`
- `/sunray_system/start_feature`
- `/sunray_system/stop_feature`
- `/sunray/system_info`

界面基于 FTXUI 实现，不再使用 Python curses。FTXUI 会在终端窗口尺寸变化时
重新计算布局，避免旧 TUI 在鼠标缩放终端后出现残留、错位和越界绘制。

## 构建

本包直接引用 `tools/build_scripts/tui/third_party/ftxui`。如果依赖目录不存在，
先执行：

```bash
tools/build_scripts/tui/check_dependencies.sh
```

然后按工程原有方式编译 catkin workspace。

## 运行

先启动 `sunray_system`：

```bash
roslaunch sunray_system sunray_system.launch
```

再启动新版 GUI：

```bash
rosrun scripts_manage scripts_manage_tui
```

也可以使用 launch：

```bash
roslaunch scripts_manage scripts_manage_tui.launch
```

## 快捷键

- `↑/↓`：移动当前列表选择
- `←/→` 或 `Tab`：切换分组/功能列表焦点
- `Enter` / `F2`：启动当前功能
- `F3`：停止当前功能
- `F1`：切换后台启动/终端启动
- `F5`：刷新
- `Esc` / `q`：退出
