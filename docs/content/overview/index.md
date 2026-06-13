<!-- title: 项目总览 -->

<section id="intro">

## 手册定位

Sunray_v2 是云纵科技 “Sunray 系列科研无人机” 的配套开源代码，面向无人机智能化和自主化飞行实验，覆盖定位、控制、规划、集群、仿真、感知、驱动、工具和通信等模块。

这份手册面向刚接触 Sunray_v2 的开发者。它不假设你已经熟悉整个 ROS 工程，而是先帮你建立三个判断：

| 问题 | 应该先看哪里 |
| --- | --- |
| 我想控制无人机或无人车，该发什么消息？ | `common/sunray_msgs`、控制模块、示例程序。 |
| 我想换定位源、仿真器或传感器，应该改哪里？ | 定位模块、驱动模块、仿真模块。 |
| 我想把自己的算法接入 Sunray，应该放在哪一层？ | 先看本页架构，再根据算法类型进入规划、感知、集群或示例章节。 |

Sunray_v2 的核心设计是：上层任务尽量通过 `sunray_msgs` 和标准 ROS 话题与系统交互，不直接依赖 PX4、MAVROS、FAST-LIO、EGO、底盘驱动或某个第三方算法的内部类。这样同一个任务节点更容易在仿真、真机、不同机型和不同定位源之间迁移。

典型控制链路：

```text
定位源 / 仿真器 / 传感器
  -> localization_fusion
  -> /uav1/sunray/localization/local_odom
  -> sunray_uav_control
  -> MAVROS / PX4
  -> 无人机
```

典型二次开发链路：

```text
你的任务节点
  -> 发布 sunray_msgs/UAVControlCMD 或 UGVControlCMD
  -> 订阅 ControlState / OdomState / SwarmState
  -> 根据状态继续发下一步命令
```

对新手最重要的一点：不要一上来改底层控制器或第三方算法。先用示例程序确认“发布命令、读取状态、判断执行结果”这条链路能跑通，再逐步进入更深层模块。

</section>

<section id="repo-map">

## 仓库地图

Sunray_v2 是一个多功能包 ROS/catkin 工程。顶层目录可以按“接口层、能力层、应用层、支撑层”理解。

### 接口层

| 目录 | 作用 | 为什么重要 |
| --- | --- | --- |
| `common/sunray_msgs` | Sunray 自定义消息和服务。 | 控制、定位、规划、集群、工具、通信之间主要靠它传递结构化数据。 |
| `common/sunray_common` | 公共工具库。 | 多个包共享的工具函数和基础能力。 |
| `common/sunray_log` | 日志示例和日志相关能力。 | 用于统一调试输出风格。 |

`sunray_msgs` 是整个项目的“桥梁”。如果你只想写上层任务，通常只需要理解消息字段、话题名和发布频率规则，不需要进入控制器内部。

### 核心能力层

| 目录 | 作用 | 主要输出 |
| --- | --- | --- |
| `localization` | 把动捕、VIO、FAST-LIO、仿真真值、全局重定位等转换成统一里程计和 TF。 | `local_odom`、`global_odom`、`OdomState`。 |
| `control` | UAV/UGV 单机控制状态机和控制工具。 | UAV MAVROS setpoint、UGV `cmd_vel`、`ControlState`。 |
| `planning` | 把上层目标点和外部 planner 输出转换成 UAV 轨迹控制命令。 | `UAVControlCMD::MOVE_TRAJECTORY`、`UAVPlanningState`。 |
| `swarm` | 多 UAV/UGV 编队、ORCA 避障、集群状态机和集群工具。 | 单机控制命令、`UAVSwarmState`、`UGVSwarmState`。 |
| `perception` | ArUco/NPU 检测和跟踪。 | `DetectionArray`、`Tracking`。 |

这些目录是二次开发最常接触的部分。你写任务时通常调用它们的接口；你做平台能力扩展时才修改它们内部实现。

### 硬件与仿真层

| 目录 | 作用 | 常见修改 |
| --- | --- | --- |
| `drivers` | MAVROS、Livox、RealSense、云台、动捕、USB 相机、轮趣底盘等驱动。 | 串口、IP、设备号、话题 remap、launch 参数。 |
| `simulation` | Gazebo/RViz 仿真、仿真插件、比赛 demo 和仿真工具。 | world、模型、传感器插件、仿真启动链路。 |

驱动和仿真层通常不直接承载任务逻辑。它们负责把真实或仿真的设备数据接进 ROS，再交给定位、控制、规划等模块。

### 应用与工具层

| 目录 | 作用 | 使用方式 |
| --- | --- | --- |
| `examples` | UAV/UGV/规划/集群示例程序。 | 新手优先复制这里的代码做二次开发。 |
| `tools` | 启动面板、监控工具、真机系统功能管理、脚本管理、PX4 参数检查。 | 用于调试、部署、启动和现场排错。 |
| `communication` | YunLink ROS 桥和通信库。 | 用于和地面站或外部系统交换状态和命令。 |

示例程序是学习入口，工具模块是调试入口，通信模块是外部系统接入口。

</section>

<section id="build-run">

## 编译与启动

Sunray_v2 提供了模块化构建脚本 `build.sh`。新手不建议一开始全量编译所有第三方包，因为驱动、仿真、规划算法依赖较多，某个不相关模块失败会干扰主线学习。

常用命令：

```bash
cd /home/amov/Sunray_v2

# 查看可构建模块和模块组
./build.sh --list
./build.sh --groups

# 交互式构建
./build.sh

# 构建某个模块
./build.sh -y uav_control
./build.sh -y ugv_control
./build.sh -y localization_fusion

# 构建后加载环境
source devel/setup.bash
```

如果修改了 `common/sunray_msgs/msg` 或 `srv`，应先重新编译消息包，再编译依赖它的控制、规划、集群、工具和示例包：

```bash
./build.sh -y sunray_msgs
source devel/setup.bash
./build.sh -y uav_control
```

### 常用启动入口

仿真或模块启动面板：

```bash
roslaunch sunray_launcher_panel sunray_launcher_panel.launch
```

UAV 单机控制：

```bash
roslaunch sunray_uav_control uav_control.launch \
  agent_name:=uav \
  agent_id:=1 \
  airframe_type:=sunray_150
```

UGV 单车控制：

```bash
roslaunch sunray_ugv_control ugv_control.launch \
  agent_name:=ugv \
  agent_id:=1 \
  airframe:=mecanum_default
```

定位融合：

```bash
roslaunch localization_fusion localization_fusion.launch \
  source_id:=3 \
  agent_name:=uav \
  agent_id:=1
```

真机后台功能管理：

```bash
roslaunch sunray_system sunray_system.launch airframe_type:=sunray_150
roslaunch sunray_system sunray_system.launch airframe_type:=sunray_300
```

脚本管理 TUI：

```bash
roslaunch scripts_manage scripts_manage_tui.launch
```

### 启动前检查

任何控制、规划或集群任务开始前，建议先确认三类话题：

```bash
# 定位是否正常
rostopic echo /uav1/sunray/localization/local_odom
rostopic echo /uav1/sunray/localization/odom_state

# 控制状态是否正常
rostopic echo /uav1/sunray/uav_control/control_state
rostopic echo /ugv1/sunray/ugv_control/control_state

# 命令是否真的发出
rostopic echo /uav1/sunray/uav_control/control_cmd
rostopic echo /ugv1/sunray/ugv_control/control_cmd
```

如果状态话题没有数据，先不要继续发任务命令。先检查 launch 是否启动、命名空间是否正确、定位源是否有效、MAVROS 或底盘驱动是否在线。

</section>

<section id="dev-path">

## 学习路线

### 第一阶段：只做接口调用

目标：会写一个自己的 ROS 节点，订阅状态并发布 Sunray 控制命令。

推荐顺序：

1. 读 `common/sunray_msgs`，理解 `UAVControlCMD`、`UGVControlCMD`、`ControlState`、`OdomState`。
2. 读控制模块中对应平台的页面，确认话题名、命令类型和发布频率。
3. 运行 `examples/sunray_uav_control_example` 或 `examples/sunray_ugv_control_example`。
4. 复制最接近的示例，改目标点、速度、高度和结束动作。

这一阶段不要改控制器内部逻辑。

### 第二阶段：接入定位、仿真和真实设备

目标：让同一个任务节点能在不同环境下运行。

推荐顺序：

1. 读定位模块总览和 `localization_fusion`。
2. 根据使用环境选择定位源：Gazebo/Pengyu 仿真、动捕、VIO、FAST-LIO 或 VINS。
3. 读驱动模块中对应硬件页面，例如 MAVROS、Livox、RealSense、VRPN、底盘驱动。
4. 读仿真模块，确认仿真器输出的话题是否能进入 `localization_fusion`。

这一阶段重点是话题、frame、外参、命名空间和配置文件，不是算法本身。

### 第三阶段：接入规划、感知或集群

目标：让自己的算法变成 Sunray 可执行任务。

常见接入方式：

| 算法输出 | 推荐接入方式 |
| --- | --- |
| 单个目标点 | 发布 `UAVControlCMD::MOVE_POINT` 或 `UGVControlCMD::MOVE_POINT`。 |
| 连续速度 | 持续发布 `MOVE_VELOCITY` 或 `MOVE_VELOCITY_BODY`，并更新时间戳。 |
| 轨迹点 | 通过 `sunray_planning` 转成 `UAVControlCMD::MOVE_TRAJECTORY`。 |
| 检测/跟踪结果 | 先进入任务层决策，再发布控制或规划命令。 |
| 多机队形 | 发布 `UAVSwarmCMD` / `UGVSwarmCMD`。 |

不要让算法直接发布 MAVROS setpoint 或直接控制底盘驱动，除非你明确要绕过 Sunray 的状态机、监控和安全边界。

### 第四阶段：修改底层能力

只有当现有接口无法满足需求时，才进入底层模块修改：

| 需求 | 修改位置 |
| --- | --- |
| 新增跨包状态或命令字段 | `common/sunray_msgs`，并同步所有使用者。 |
| 新增 UAV 控制模式 | `control/sunray_uav_control`、控制面板、终端工具、示例。 |
| 新增 UGV 底盘类型 | `control/sunray_ugv_control/src/controller` 和 airframe YAML。 |
| 新增定位源 | `localization/localization_fusion/config/localization_sources.yaml` 和适配节点。 |
| 新增 planner | `planning/sunray_planning` 的 planner adapter。 |
| 新增集群阵型 | `swarm/formation`、`Formation.msg`、终端/Qt/RViz 工具。 |

修改底层接口时要同步文档、示例和工具 UI。否则新功能虽然能跑，但小白用户很难知道怎么正确调用。

</section>
