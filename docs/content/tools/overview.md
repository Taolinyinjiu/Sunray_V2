<!-- title: 工具总览 -->

<section id="tools-overview">

## 工具总览

`tools` 目录放置 Sunray 的开发、部署、启动、监控和参数检查工具。它们不是底层控制算法，但会显著影响真机调试效率。

| 功能包/目录 | 作用 |
| --- | --- |
| `sunray_launcher_panel` | 图形化启动器，用配置文件组织常用 launch 和一键启动脚本。 |
| `sunray_monitor_tools` | Qt + RViz 综合监控面板，查看状态并下发常用控制指令。 |
| `sunray_system` | 真机 feature supervisor，通过服务启动/停止功能组合。 |
| `scripts_manage` | 终端 TUI，调用 `sunray_system` 服务管理 feature。 |
| `px4_param_check` | PX4 参数检查和写入脚本。 |
| `sunray_flight_logger` | 终端 TUI，通过 MAVROS FTP 浏览、下载和删除 PX4 飞行日志。 |
| `build_scripts` | `build.sh` 背后的模块化构建脚本。 |
| `code_intel` | 代码智能/编译数据库辅助脚本。 |

二次开发时，工具模块常见改动是新增启动项、监控字段、feature 配置或 PX4 参数模板。

</section>
