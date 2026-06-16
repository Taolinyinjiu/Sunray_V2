<!-- title: sunray_flight_logger -->

<section id="tools-sunray-flight-logger">

## sunray_flight_logger

`tools/sunray_flight_logger` 提供一个基于 FTXUI 的终端界面，用于通过 MAVROS FTP 浏览、下载和删除 PX4 飞控 SD 卡中的飞行日志。

该工具面向真机调试后的日志回收场景：上层只需要启动 TUI，不需要直接调用 MAVROS FTP 服务。

### 功能能力

| 功能 | 说明 |
| --- | --- |
| 日志列表 | 递归扫描飞控 `/fs/microsd/log` 目录，展示日志 ID、文件名和大小。 |
| 多选 | 支持单个日志勾选，也支持一键全选/取消全选。 |
| 下载 | 将选中的日志下载到本地默认目录。 |
| 删除 | 删除选中的远端日志，也可以清除当前列表中的全部日志。 |
| 自动连接 | 默认自动发现已有 MAVROS FTP 服务；没有 MAVROS 时自动启动 `sunray_mavros`。 |

默认只显示 `.ulg` 日志文件。需要扫描其他后缀时，可以通过 `log_extension_filter` 修改。

### 构建

```bash
cd ~/Sunray_v2
./build.sh sunray_flight_logger
source devel/setup.bash
```

本包复用 `tools/build_scripts/tui/third_party/ftxui`。如果 FTXUI 依赖目录不存在，先执行：

```bash
tools/build_scripts/tui/check_dependencies.sh
```

### 启动方式

```bash
roslaunch sunray_flight_logger sunray_flight_logger_tui.launch
```

默认启动时不需要传入 `mavros_ns`。工具会先从 ROS master 中查找已有的 MAVROS FTP 服务，例如：

```text
/uav1/mavros/ftp/list
/uav1/mavros/ftp/open
/uav1/mavros/ftp/read
/uav1/mavros/ftp/close
/uav1/mavros/ftp/remove
```

如果已经有 MAVROS 节点并且 FTP 服务完整，工具会直接使用对应命名空间。如果没有发现 MAVROS 节点，工具会自动启动：

```bash
roslaunch sunray_mavros mavros.launch \
  agent_name:=uav agent_id:=1 \
  fcu_url:=/dev/ttyACM0:921600 \
  gcs_ip:=0.0.0.0
```

如果已有 MAVROS 节点但没有 FTP 服务，工具不会再启动第二个 MAVROS。此时应检查 `drivers/sunray_mavros/config/exp_px4_pluginlists.yaml` 是否启用了 `ftp` 插件。

### 常用启动参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `mavros_ns` | 空 | 手动指定 MAVROS 命名空间；为空时自动发现。 |
| `log_root` | `/fs/microsd/log` | 飞控端日志根目录。 |
| `download_root` | 空 | 本地下载目录；为空时使用仓库根目录下的 `sunray_logs/flight_log`。 |
| `log_extension_filter` | `.ulg` | 日志后缀过滤；设为空字符串可显示全部文件。 |
| `refresh_period_sec` | `10.0` | 自动刷新周期，单位秒。 |
| `max_scan_depth` | `6` | 递归扫描目录的最大深度。 |
| `auto_start_mavros` | `true` | 未发现 MAVROS 时是否自动启动 `sunray_mavros`。 |
| `mavros_start_wait_sec` | `18.0` | 自动启动 MAVROS 后等待 FTP 服务出现的时间。 |
| `agent_name` | `uav` | 自动启动 `sunray_mavros` 时传入的智能体名前缀。 |
| `agent_id` | `1` | 自动启动 `sunray_mavros` 时传入的智能体编号。 |
| `fcu_url` | `/dev/ttyACM0:921600` | 自动启动 `sunray_mavros` 时使用的飞控连接地址。 |
| `gcs_ip` | `0.0.0.0` | 自动启动 `sunray_mavros` 时使用的地面站转发 IP。 |

示例：

```bash
# 手动指定已有 MAVROS 命名空间
roslaunch sunray_flight_logger sunray_flight_logger_tui.launch \
  mavros_ns:=/uav1/mavros

# 只使用已有 MAVROS，不自动启动
roslaunch sunray_flight_logger sunray_flight_logger_tui.launch \
  auto_start_mavros:=false

# 指定下载目录
roslaunch sunray_flight_logger sunray_flight_logger_tui.launch \
  download_root:=/home/amov/flight_logs

# 自动启动 MAVROS 时指定串口
roslaunch sunray_flight_logger sunray_flight_logger_tui.launch \
  fcu_url:=/dev/ttyUSB0:921600
```

### 下载路径

默认下载目录为：

```text
<repo>/sunray_logs/flight_log
```

其中 `<repo>` 会由工具根据 `sunray_flight_logger` 包路径自动向上查找。下载时会保留飞控日志目录下的相对路径，例如：

```text
/fs/microsd/log/2026-06-16/13_20_01.ulg
```

会保存为：

```text
<repo>/sunray_logs/flight_log/2026-06-16/13_20_01.ulg
```

### 界面操作

| 按键 | 作用 |
| --- | --- |
| `↑/↓` 或 `k/j` | 移动当前日志选择。 |
| `PageUp/PageDown` | 快速翻页。 |
| `Home/End` | 跳到第一条或最后一条日志。 |
| `Space` | 勾选或取消勾选当前日志。 |
| `a` | 全选或取消全选。 |
| `Enter` / `F1` | 下载已选日志。 |
| `d` | 删除已选日志。 |
| `D` | 清除当前列表中的全部日志。 |
| `F5` / `r` | 手动刷新日志列表。 |
| `Esc` / `q` | 退出界面。 |

删除操作会直接调用飞控端 MAVROS FTP remove 服务。真机使用时建议先下载需要保留的日志，再执行删除或清除全部。

### 依赖接口

工具依赖 MAVROS FTP 服务：

```text
${mavros_ns}/ftp/list
${mavros_ns}/ftp/open
${mavros_ns}/ftp/read
${mavros_ns}/ftp/close
${mavros_ns}/ftp/remove
```

这些服务由 MAVROS `ftp` plugin 提供。Sunray 默认通过 `sunray_mavros` 启动 MAVROS，飞控连接地址、智能体编号和地面站 IP 仍由 `sunray_mavros` launch 参数控制。

### 相关文件

```text
tools/sunray_flight_logger/
├── launch/sunray_flight_logger_tui.launch
├── src/sunray_flight_logger_tui.cpp
├── CMakeLists.txt
└── package.xml
```

</section>
