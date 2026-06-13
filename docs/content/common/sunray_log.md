<!-- title: sunray_log -->

<section id="common-sunray-log">

## sunray_log

`common/sunray_log` 是 Sunray 对 `spdlog` 的轻量封装。它提供统一日志宏、控制台/文件输出配置和一个示例包。

### 目录结构

```text
common/sunray_log/
├── spdlog/
├── sunray_log.hpp
└── sunray_log_example/
    ├── include/sunray_log_example/localization_example.hpp
    ├── src/localization_example.cpp
    └── src/main.cpp
```

### sunray_log.hpp

核心头文件：

```text
common/sunray_log/sunray_log.hpp
```

它提供：

- `SunrayLogLevel`：日志等级枚举。
- `SunrayLogConfig`：日志器配置。
- `SunrayLogger`：全局单例日志器。
- `SUNRAY_TRACE / DEBUG / INFO / WARN / ERROR / CRITICAL` 宏。

日志等级：

| 等级 | 说明 |
| --- | --- |
| `trace` | 最详细调试信息。 |
| `debug` | 调试信息。 |
| `info` | 普通运行信息。 |
| `warn` | 告警，不一定导致失败。 |
| `error` | 错误，功能可能失败。 |
| `critical` | 严重错误。 |

配置结构：

| 字段 | 默认值 | 说明 |
| --- | --- | --- |
| `name` | `sunray` | logger 名称。 |
| `console_level` | `info` | 控制台输出等级。 |
| `file_path` | 空 | 日志文件路径；为空则不写文件。 |
| `file_level` | `trace` | 文件输出等级。 |
| `async` | `false` | 是否启用异步日志。 |
| `async_queue_size` | `8192` | 异步队列大小。 |
| `async_thread_count` | `1` | 异步后台线程数。 |

### 使用方式

在 CMake 中加入 include 路径：

```cmake
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../common/sunray_log
  ${CMAKE_CURRENT_SOURCE_DIR}/../../common/sunray_log/spdlog/include
)
```

在代码中 include：

```cpp
#include "sunray_log.hpp"
```

初始化：

```cpp
SunrayLogConfig cfg;
cfg.name = "localization_example";
cfg.console_level = SunrayLogLevel::info;
cfg.file_path = "logs/localization_example.log";
cfg.file_level = SunrayLogLevel::debug;
cfg.async = false;
SunrayLogger::instance().Init(cfg);
```

打印：

```cpp
SUNRAY_INFO("节点启动");
SUNRAY_WARN("高度告警: {:.2f} m", height);
SUNRAY_ERROR("收到无效里程计数据");
```

如果没有调用 `Init()`，默认 logger 也会初始化到控制台输出，避免直接使用宏时崩溃。

### sunray_log_example

示例包：

```text
common/sunray_log/sunray_log_example
```

节点：

```text
localization_example_node
```

示例逻辑：

1. 初始化 `SunrayLogger`。
2. 订阅 `/uav<id>/sunray/odometry`。
3. 打印位姿和速度。
4. 当高度超过阈值时输出 warn。
5. 当位置为 NaN 时输出 error。

示例参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `uav_id` | `1` | 订阅 `/uav1/sunray/odometry`。 |
| `height_warn_threshold` | `3.0` | 高度告警阈值，单位 m。 |

### 二次开发建议

- 新包如果只需要 ROS 常规日志，可以继续用 `ROS_INFO` 等宏。
- 如果希望日志格式统一、同时输出文件，使用 `sunray_log`。
- 高频循环里不要无节制打印 `INFO/WARN`，建议节流或只在状态变化时打印。
- 文件日志路径建议放到 launch 参数或配置文件中，不要写死到用户目录。

</section>
