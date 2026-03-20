uav_control是Sunray项目的控制核心部分，主要包含以Base_Controller为基类，延伸出来的几类控制器，分别是

- 基于PX4位置环设计的控制器，位置控制的精度由PX4位置环精度决定，PX4内部的位置环为单P环节，也就是说纯比例环节，因此性能没有其他几个控制器好
- 基于PX4速度环设计的PID控制器，位置控制的精度由PID参数决定，相较于基于PX4位置环设计的控制器，其为完整的PID控制，因此性能会相对较好
- 基于PX4姿态环设计的Sunray自定义控制器，位置控制的精度由控制器的参数决定
- 基于强化学习设计的RAPTOR控制器，位置控制的精度由模型决定

这四种控制器，对应了四个文件夹
- px4_position_controller 基于px4位置环的控制器
- px4_velocity_controller 基于px4速度环的控制器
- sunray_attitude_controller 基于px4姿态环的控制器
- sunray_raptor_controller 基于强化学习RAPTOR模型的控制器

除此之外，还存在一个核心的状态机模块，也就是Sunray_statemachine，通常我们简称为Sunray_fsm（fsm指的是有限状态机）

总的来说sunray_fsm是一个相对封闭的模块，向外暴露出特定的接口，允许用户通过这些接口组合实现自己的算法
sunray_fsm所提供的接口可以分为这几类

1. 与无人机运动控制相关
 - 起飞(先service服务 后topic话题)
    takeoff_request
    takeoff_command
 - 降落
    land_request
    land_command
 - 紧急降落
    emergency_land_request
    emergency_land_command
 - 运动控制
    control_request
    control_command

2. 与无人机状态相关的
 - 状态机状态变量（10hz）
    sunray_fsm_state
 - 无人机运动相关变量
    uav_odom_state
 - 传感器相关状态
    uav_sensor_status

## 状态机图自动生成

`uav_control` 提供了基于源码自动生成状态机图的脚本：

```bash
python3 control/uav_control/scripts/gen_fsm_diagram.py --write
```

生成文件：

- `control/uav_control/state_machine_diagram.md`

一致性检查（适合 CI）：

```bash
python3 control/uav_control/scripts/gen_fsm_diagram.py --check --strict
```

## 光流评估脚本（ULog + Mocap）

新增脚本：`control/sunray_uav_control/scripts/flow_eval_ulog_mocap.py`

用途：在室内测试中，基于 `PX4 ulog` 和 `mocap CSV` 自动计算光流相关 KPI，并输出 `PASS/FAIL`。

依赖：

```bash
pip3 install pyulog numpy
```

基础用法：

```bash
python3 control/sunray_uav_control/scripts/flow_eval_ulog_mocap.py \
  --ulog /path/to/flight.ulg \
  --mocap-csv /path/to/mocap.csv \
  --output-json /tmp/flow_eval_report.json
```

脚本默认会自动识别常见 mocap 列名（包括 `rostopic echo -p` 导出的 `pose/odom` CSV），通常只需 `--ulog` 和 `--mocap-csv` 两个参数即可运行。

常用参数：

- `--mocap-time-col/--mocap-x-col/--mocap-y-col/--mocap-z-col`：指定 CSV 列名，默认 `timestamp,x,y,z`
- `--mocap-time-unit {auto,s,ms,us,ns}`：指定 mocap 时间戳单位
- `--mocap-frame {enu,ned}`：mocap 坐标系，默认 `enu`（脚本会转换为 NED）
- `--hover-start-sec --hover-end-sec`：指定悬停评分时间窗（相对对齐后起点）
- `--flow-quality-min` 以及 `--max-xy-rmse` 等：调整 KPI 阈值

输出内容（含 `PASS/FAIL`）：

- 光流融合占比、光流故障占比、光流质量占比
- 光流/速度/位置/高度等创新检验比（P95）
- 悬停 `XY RMSE`、`XY P95`、漂移速度（m/s）

## 动捕 CSV 导出脚本（带时间戳命名）

新增脚本：`control/sunray_uav_control/scripts/export_mocap_pose_csv.sh`

用途：从 VRPN `pose` 话题导出 CSV，并自动按时间戳命名，避免覆盖旧文件。

示例（从 bag 导出）：

```bash
./control/sunray_uav_control/scripts/export_mocap_pose_csv.sh \
  --bag /path/to/your.bag \
  --topic /vrpn_client_node/uav1/pose \
  --out-dir /home/taolin/Desktop/Sunray_OpticalFlow_TestDir/MTF01/Mocap_Logs
```

示例（在线导出）：

```bash
./control/sunray_uav_control/scripts/export_mocap_pose_csv.sh \
  --live \
  --topic /vrpn_client_node/uav1/pose \
  --out-dir /home/taolin/Desktop/Sunray_OpticalFlow_TestDir/MTF01/Mocap_Logs
```

## tmux 双窗格录制脚本（rosbag + csv）

新增脚本：`control/sunray_uav_control/scripts/record_mocap_tmux.sh`

用途：通过一个 `tmux` 会话同时运行两路录制：

- 左窗格：`rosbag record -o ... <pose> <twist>`
- 右窗格：实时导出 `pose` 为 CSV（带时间戳文件名）

示例（你的话题命名风格）：

```bash
./control/sunray_uav_control/scripts/record_mocap_tmux.sh start \
  --out-dir /home/taolin/Desktop/Sunray_OpticalFlow_TestDir/MTF01/Mocap_Logs \
  --bag-prefix Sunray_OpticalFlow_MTF01_Mocap_logs \
  --pose-topic /vrpn_client_node_1/uav1/pose \
  --twist-topic /vrpn_client_node_1/uav1/twist
```

以上参数已在脚本中设置为默认值，因此也可以直接运行：

```bash
./control/sunray_uav_control/scripts/record_mocap_tmux.sh start
```

默认启用 `tmux` 同步输入（`synchronize-panes on`），因此在任一窗格按一次 `Ctrl+C` 会同时结束 `rosbag` 和 `csv` 导出。
如果不希望同步输入，可在启动时追加：

```bash
./control/sunray_uav_control/scripts/record_mocap_tmux.sh start --no-sync-panes
```

默认也启用 `--auto-kill-on-exit`：当任一窗格退出（例如你在窗格里按 `Ctrl+C`）时，脚本会停止另一个窗格并自动结束整个 tmux 会话。  
如果你希望会话保留不自动结束，可在启动时追加：

```bash
./control/sunray_uav_control/scripts/record_mocap_tmux.sh start --no-auto-kill-on-exit
```

停止（向两个窗格发送 Ctrl+C 并关闭会话）：

```bash
./control/sunray_uav_control/scripts/record_mocap_tmux.sh stop --kill
```
