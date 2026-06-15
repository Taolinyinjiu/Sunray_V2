<!-- title: 无人车控制 -->

<section id="sunray-ugv-control">

## sunray_ugv_control

`sunray_ugv_control` 是 Sunray 中的无人车底层控制包。它负责接收上层程序、控制面板、规划器或测试程序发布的 `sunray_msgs/UGVControlCMD`，结合定位里程计计算底盘速度，并把 `geometry_msgs/Twist` 发布到底盘驱动或仿真环境。

本文面向刚接触 ROS 和 Sunray 的开发者，重点说明这个包的输入输出、启动方式、配置文件、代码架构和二次开发入口。

## 1. 这个包做什么

简单理解，这个包就是一个“无人车控制中间层”：

```text
上层程序 / 控制面板 / 规划器 / 示例程序
        |
        | /ugv1/sunray/ugv_control/control_cmd
        v
UGVControlFSM 状态机
        |
        | 调用不同底盘控制器
        v
MecanumController / DifferentialController
        |
        | /ugv1/sunray/ugv_control/cmd_vel
        v
底盘驱动 / 仿真底盘
```

它不直接控制电机，也不直接做定位。它需要外部定位模块持续发布本车里程计，然后它输出标准 `/cmd_vel` 语义的速度指令。

## 2. 目录结构

```text
sunray_ugv_control/
├── config/
│   ├── ugv_control_base.yaml            # 所有车型共享的基础参数
│   └── airframes/                       # 不同车型/仿真环境的覆盖参数
│       ├── mecanum_default.yaml
│       ├── differential_default.yaml
│       ├── gazebo_sim.yaml
│       └── pengyu_sim.yaml
├── include/
│   └── ugv_control_fsm.h                # 控制状态机头文件
├── launch/
│   ├── ugv_control.launch               # 单车控制节点启动文件
│   ├── ugv_control_panel.launch         # Qt 控制面板启动文件
│   └── ugv_control_swarm.launch         # 多车控制节点启动文件
├── meshes/
│   └── ugv_vehicle.dae                  # RViz 显示模型
├── rviz/
│   └── ugv_control.rviz                 # RViz 配置
├── src/
│   ├── controller/
│   │   ├── ugv_controller.*             # 控制器基类和通用参数
│   │   ├── mecanum_controller.*         # 麦克纳姆轮控制器
│   │   └── differential_controller.*    # 差速轮控制器
│   ├── tools/
│   │   ├── ugv_control_monitor_node.cpp # 终端状态监控工具
│   │   ├── ugv_control_panel_node.cpp   # Qt 控制面板
│   │   └── rviz_visualization_ugv_control_node.cpp
│   ├── ugv_control_fsm.cpp              # 控制状态机实现
│   └── ugv_control_node.cpp             # ROS 节点入口
├── CMakeLists.txt
└── package.xml
```

## 3. 启动方式

### 3.1 启动单台无人车控制节点

```bash
roslaunch sunray_ugv_control ugv_control.launch agent_name:=ugv agent_id:=1 airframe:=mecanum_default
```

默认会启动：

- `sunray_ugv_control` 主控制节点
- 单车终端状态监控节点
- RViz Marker 发布节点

如果只想启动控制节点，不启动监控和可视化：

```bash
roslaunch sunray_ugv_control ugv_control.launch \
  agent_name:=ugv \
  agent_id:=1 \
  airframe:=differential_default \
  enable_single_monitor:=false \
  enable_rviz_visualization:=false \
  enable_rviz:=false
```

如果希望同时打开 RViz GUI：

```bash
roslaunch sunray_ugv_control ugv_control.launch enable_rviz:=true
```

SSH 或无图形界面环境下，建议保持 `enable_rviz:=false`。

### 3.2 启动多台无人车控制节点

```bash
roslaunch sunray_ugv_control ugv_control_swarm.launch swarm_num:=3 airframe:=mecanum_default
```

该启动文件会按编号启动 `/ugv1`、`/ugv2`、`/ugv3` 的控制节点。每台车的话题前缀由 `agent_name + agent_id` 拼出来，例如：

```text
/ugv1
/ugv2
/ugv3
```

### 3.3 启动 Qt 控制面板

```bash
roslaunch sunray_ugv_control ugv_control_panel.launch agent_name:=ugv agent_id:=1
```

控制面板会发布 `UGVControlCMD`，并订阅 `UGVControlState` 显示当前控制状态。它适合手动测试停车、点位移动、车体系速度等底层控制命令。

## 4. 核心话题

下面以 `agent_name:=ugv`、`agent_id:=1` 为例。

### 4.1 输入话题

| 话题 | 类型 | 作用 |
| --- | --- | --- |
| `/ugv1/sunray/localization/local_odom` | `nav_msgs/Odometry` | 本车定位里程计，控制器需要用它读取当前位置、速度和 yaw |
| `/ugv1/sunray/localization/odom_state` | `sunray_msgs/OdomState` | 定位状态，当前代码已订阅，但回调里暂未做安全逻辑 |
| `/ugv1/sunray/ugv_control/control_cmd` | `sunray_msgs/UGVControlCMD` | 上层发给 UGV 控制器的控制命令 |

### 4.2 输出话题

| 话题 | 类型 | 作用 |
| --- | --- | --- |
| `/ugv1/sunray/ugv_control/cmd_vel` | `geometry_msgs/Twist` | 控制器输出到底盘或仿真的速度指令 |
| `/ugv1/sunray/ugv_control/control_state` | `sunray_msgs/UGVControlState` | 控制状态、当前里程计、目标点、底盘类型、围栏状态等 |
| `/ugv1/sunray/ugv_control_debug` | `sunray_msgs/UGVControlCMD` | 最近一次控制命令的调试输出 |
| `/ugv1/sunray/ugv_control/rviz_markers` | `visualization_msgs/MarkerArray` | RViz 可视化工具发布的模型、围栏、目标点和速度箭头 |

## 5. 控制命令接口

控制命令定义在：

```text
common/sunray_msgs/msg/UGVControlCMD.msg
```

典型发布话题：

```text
/ugv1/sunray/ugv_control/control_cmd
```

### 5.1 命令类型

| 命令 | 数值 | 需要填写的字段 | 发布频率 | 说明 |
| --- | --- | --- | --- | --- |
| `HOLD` | 1 | 无 | 发一次即可 | 停车，控制节点会持续发布零速度 |
| `MOVE_POINT` | 3 | `desired_pos.x/y`、`desired_yaw` | 发一次即可 | 移动到本地坐标系目标点 |
| `MOVE_VELOCITY` | 4 | `desired_vel.x/y`、`desired_yaw` | 需要持续发布 | 世界系速度控制，适合麦克纳姆轮；差速轮默认不支持 |
| `MOVE_VELOCITY_BODY` | 5 | `cmd_vel` | 需要持续发布 | 车体系速度控制，语义与底盘 `/cmd_vel` 一致 |
| `MOVE_WGS84` | 6 | `desired_wgs84_pos` | 暂不建议使用 | 经纬高接口预留，当前控制器不执行 |

速度类命令必须持续发布，并且建议每次设置：

```cpp
cmd.header.stamp = ros::Time::now();
```

原因是状态机会用 `header.stamp` 判断速度命令是否超时。默认超时时间由 `wait_velcmd_time` 配置，默认值为 `5.0` 秒。超过该时间后，控制器会切换到 `HOLD` 并停车。

点位和停车属于事件类命令，发布一次即可。

`sunray_ugv_control` 不提供 `RETURN` 返航接口。返航属于任务层/导航层语义，建议由上层模块保存 home 点，并在需要返航时向本包发布普通 `MOVE_POINT`。

### 5.2 坐标系约定

- `desired_pos`：本地世界坐标系位置，单位 m；UGV 主要使用 `x/y`，`z` 通常填 0。
- `desired_vel`：本地世界坐标系速度，单位 m/s；UGV 主要使用 `x/y`。
- `desired_yaw`：本地世界坐标系 yaw，单位 rad。
- `cmd_vel`：车体坐标系速度，语义与 ROS 常见底盘 `/cmd_vel` 一致。

### 5.3 最小 C++ 发布示例

发布一次点位命令：

```cpp
#include <ros/ros.h>
#include <sunray_msgs/UGVControlCMD.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "ugv_move_point_demo");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sunray_msgs::UGVControlCMD>(
      "/ugv1/sunray/ugv_control/control_cmd", 10);

  ros::Duration(0.5).sleep();

  sunray_msgs::UGVControlCMD cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.cmd_source = sunray_msgs::UGVControlCMD::EXAMPLE_DEMO;
  cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_POINT;
  cmd.desired_pos.x = 1.0;
  cmd.desired_pos.y = 0.0;
  cmd.desired_pos.z = 0.0;
  cmd.desired_yaw = 0.0;
  pub.publish(cmd);

  ros::spinOnce();
  return 0;
}
```

持续发布车体系速度命令：

```cpp
#include <ros/ros.h>
#include <sunray_msgs/UGVControlCMD.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "ugv_velocity_body_demo");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sunray_msgs::UGVControlCMD>(
      "/ugv1/sunray/ugv_control/control_cmd", 10);

  ros::Rate rate(20.0);
  while (ros::ok()) {
    sunray_msgs::UGVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd_source = sunray_msgs::UGVControlCMD::EXAMPLE_DEMO;
    cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY;
    cmd.cmd_vel.linear.x = 0.2;
    cmd.cmd_vel.linear.y = 0.0;
    cmd.cmd_vel.angular.z = 0.0;
    pub.publish(cmd);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
```

## 6. 控制状态接口

控制状态定义在：

```text
common/sunray_msgs/msg/UGVControlState.msg
```

典型订阅话题：

```text
/ugv1/sunray/ugv_control/control_state
```

常用字段：

| 字段 | 说明 |
| --- | --- |
| `agent_name` / `agent_id` | 当前车辆身份，例如 `ugv` + `1` |
| `drive_type` | 底盘类型：`DRIVE_MECANUM=1`、`DRIVE_DIFFERENTIAL=2` |
| `fsm_state` | 状态机状态：`INIT`、`HOLD`、`MOVE` |
| `control_cmd_valid` | 是否已经收到有效控制命令 |
| `odom_valid` | 是否已经收到本机里程计 |
| `inside_geo_fence` | 当前位姿是否在配置的地理围栏内 |
| `diagnostic_level` | 控制器诊断等级：`DIAGNOSTIC_OK=0`、`DIAGNOSTIC_WARN=1`、`DIAGNOSTIC_ERROR=2` |
| `diagnostic_msg` | 面向开发者的简短诊断文本，用于解释命令拒绝、降级执行或自动 HOLD 原因 |
| `active_ugv_control_cmd` | 最近一次有效控制命令 |
| `self_odom` | 当前本机里程计 |
| `target_valid` | 当前是否有明确目标点 |
| `target_pos` / `target_yaw` | 当前目标点和目标 yaw |
| `controller_cmd_vel` | 控制器最近一次发布到底盘的速度 |
| `geo_fence_min` / `geo_fence_max` | 当前配置的围栏边界 |

调试时建议先看：

```bash
rostopic echo /ugv1/sunray/ugv_control/control_state
rostopic echo /ugv1/sunray/ugv_control/cmd_vel
```

如果 `odom_valid=false`，说明控制节点还没有收到定位信息，点位控制无法正常工作。

如果车辆没有按预期执行，建议优先查看 `diagnostic_level` 和 `diagnostic_msg`。当前会反馈的典型情况包括：

- 速度命令 `header.stamp` 为 0。
- 速度命令超过 `wait_velcmd_time` 后超时。
- 差速底盘收到不支持的 `MOVE_VELOCITY`。
- 差速底盘收到 `MOVE_VELOCITY_BODY.linear.y` 并自动忽略横向速度。
- 开启地理围栏保护后车辆越界并自动 HOLD。
- 收到未知控制命令。

## 7. 状态机逻辑

状态机实现在：

```text
src/ugv_control_fsm.cpp
```

内部状态：

| 状态 | 对外枚举 | 说明 |
| --- | --- | --- |
| `INIT` | `FSM_INIT=0` | 初始状态，尚未进入控制 |
| `HOLD` | `FSM_HOLD=1` | 停车状态，持续发布零速度 |
| `MOVE` | `FSM_MOVE=3` | 执行点位、速度、预留 WGS84 命令 |

主要流程：

1. `odom_callback()` 接收 `/local_odom`，更新当前位置、速度和 yaw。
2. `control_cmd_callback()` 接收 `UGVControlCMD`，根据 `control_cmd` 切换状态。
3. `control_timer_callback()` 以 100 Hz 调用状态处理函数。
4. 控制器计算 `cmd_vel` 并发布。
5. 每个控制周期发布 `UGVControlState` 和调试命令。

点位控制到达条件由下面两个参数决定：

```yaml
point_pos_tolerance: 0.05
point_yaw_tolerance: 0.10
```

到达目标点后，状态机会切换到 `HOLD`。

## 8. 底盘类型和控制器

底盘类型由配置文件中的 `drive_type` 决定：

```yaml
drive_type: 1  # 1=麦克纳姆轮，2=差速轮
```

### 8.1 麦克纳姆轮

对应代码：

```text
src/controller/mecanum_controller.cpp
```

特点：

- 支持 `MOVE_POINT`
- 支持 `MOVE_VELOCITY`
- 支持 `MOVE_VELOCITY_BODY`
- 支持车体侧向速度 `cmd_vel.linear.y`

麦克纳姆轮可以横移，因此更适合世界系 `x/y` 速度控制。

### 8.2 差速轮

对应代码：

```text
src/controller/differential_controller.cpp
```

特点：

- 支持 `MOVE_POINT`
- 支持 `MOVE_VELOCITY_BODY` 中的 `linear.x` 和 `angular.z`
- 不支持车体侧向速度 `linear.y`
- 当前状态机默认拒绝 `MOVE_VELOCITY` 世界系速度命令，并切换到 `HOLD`

差速轮无法横移。如果给 `MOVE_VELOCITY_BODY` 填了 `linear.y`，状态机会打印 warning，并强制忽略该值。

## 9. 配置文件说明

配置文件采用“基础参数 + 车型参数”的两层结构：

```text
config/
├── ugv_control_base.yaml
└── airframes/
    ├── mecanum_default.yaml
    ├── differential_default.yaml
    ├── gazebo_sim.yaml
    └── pengyu_sim.yaml
```

加载优先级：

```text
ugv_control_base.yaml < airframes/<airframe>.yaml < launch 参数 agent_name / agent_id
```

也就是说，车型文件可以覆盖基础文件中的同名参数；`agent_name` 和 `agent_id` 最终由 launch 参数覆盖，避免多车启动时被 YAML 写死。

启动时用 `airframe` 选择车型：

```bash
roslaunch sunray_ugv_control ugv_control.launch airframe:=differential_default
roslaunch sunray_ugv_control ugv_control.launch airframe:=gazebo_sim
roslaunch sunray_ugv_control ugv_control_swarm.launch swarm_num:=3 airframe:=pengyu_sim
```

当前预设：

| airframe | 说明 |
| --- | --- |
| `mecanum_default` | 默认麦克纳姆轮实车参数，支持横向速度 |
| `differential_default` | 默认差速轮实车参数，只支持 `vx + wz` |
| `gazebo_sim` | Gazebo 仿真参数，默认按麦克纳姆轮处理 |
| `pengyu_sim` | pengyu_sim 仿真参数，默认按差速轮处理 |

常用参数：

| 参数 | 说明 |
| --- | --- |
| `agent_name` | 机器人名称前缀，例如 `ugv` |
| `agent_id` | 机器人编号，从 1 开始 |
| `drive_type` | 底盘类型：1=麦克纳姆轮，2=差速轮 |
| `kp_linear` | 位置误差到线速度的比例增益 |
| `kp_angular` | yaw 误差到角速度的比例增益 |
| `max_linear_vel` | 最大线速度限制 |
| `max_angular_vel` | 最大角速度限制 |
| `wait_velcmd_time` | 速度命令超时时间，单位 s |
| `point_pos_tolerance` | 点位到达的位置误差阈值，单位 m |
| `point_yaw_tolerance` | 点位到达的 yaw 误差阈值，单位 rad |
| `enable_geo_fence_protection` | 是否开启地理围栏越界自动 HOLD，默认 `false` |
| `fence_min_x/y/z` | 地理围栏最小边界 |
| `fence_max_x/y/z` | 地理围栏最大边界 |

差速轮倒车策略参数：

| 参数 | 说明 |
| --- | --- |
| `differential_controller.enable_reverse` | 是否允许差速轮在近距离目标位于车身后方时倒车 |
| `differential_controller.reverse_angle_threshold` | 触发倒车的后方夹角阈值，单位 rad |
| `differential_controller.reverse_distance_threshold` | 允许倒车的最大目标距离，单位 m |
| `differential_controller.reverse_speed_ratio` | 倒车速度相对最大前进速度的比例 |
| `differential_controller.final_yaw_distance_threshold` | 距离目标多近时切换为原地调整最终 yaw |

注意：默认情况下，地理围栏只会反映在 `UGVControlState.inside_geo_fence` 字段中。将 `enable_geo_fence_protection` 设为 `true` 后，控制节点收到有效 odom 且检测到车辆越界时，会自动切换到 `HOLD` 并发布零速度。

## 10. 工具节点

### 10.1 终端状态监控

源码：

```text
src/tools/ugv_control_monitor_node.cpp
```

作用：订阅一台或多台车的 `UGVControlState`，在终端打印状态、目标点、速度输出、围栏状态等信息。

### 10.2 RViz 可视化

源码：

```text
src/tools/rviz_visualization_ugv_control_node.cpp
```

作用：把 `UGVControlState` 转成 RViz Marker，用于显示车体模型、地理围栏、目标点、速度箭头和状态文本。

### 10.3 Qt 控制面板

源码：

```text
src/tools/ugv_control_panel_node.cpp
```

作用：提供图形界面，方便手动发布停车、点位移动、速度控制等命令，并查看控制状态。

## 11. 二次开发怎么入手

### 11.1 只想写上层控制程序

只需要发布：

```text
/ugv1/sunray/ugv_control/control_cmd
```

并根据需要订阅：

```text
/ugv1/sunray/ugv_control/control_state
```

建议先从 `UGVControlCMD.msg` 和 `UGVControlState.msg` 看起，再参考本文第 5 节的最小发布示例。

仓库中也提供了可直接运行的 UGV 控制示例包：

```text
ugv_control/sunray_ugv_control_example
```

该示例包包含 `HOLD`、`MOVE_POINT`、`MOVE_VELOCITY_BODY` 三个最基础例程，并配套 launch 和 README。小白用户建议先运行这些示例，再复制其中最接近需求的源码做二次开发。

### 11.2 想增加一种新命令

一般需要改这些位置：

1. 在 `common/sunray_msgs/msg/UGVControlCMD.msg` 中增加命令枚举和字段注释。
2. 在 `src/ugv_control_fsm.cpp` 的 `control_cmd_callback()` 中决定该命令如何切换状态。
3. 在 `process_move()` 或新增状态处理函数中实现执行逻辑。
4. 在 `UGVControlState.msg` 中补充必要的状态反馈字段。
5. 更新 Qt 面板、终端监控和 README。

### 11.3 想增加一种新底盘

建议参考现有两个控制器：

```text
src/controller/mecanum_controller.cpp
src/controller/differential_controller.cpp
```

新增底盘时通常需要：

1. 继承 `UGVControllerBase`。
2. 实现 `move_point()` 和 `move_velocity()`。
3. 明确 `supports_world_velocity()` 和 `supports_lateral_velocity()`。
4. 在 `UGVControlFSM` 构造函数中根据新的 `drive_type` 创建新控制器。
5. 在 `UGVControlState.msg` 中增加新的底盘类型枚举。
6. 更新配置文件和文档。

### 11.4 想接入真实底盘

需要保证真实底盘驱动订阅：

```text
/ugv1/sunray/ugv_control/cmd_vel
```

并且定位模块发布：

```text
/ugv1/sunray/localization/local_odom
```

如果真实底盘实际订阅的话题不是这个名字，可以用 remap 或桥接节点转换。

## 12. 常见问题

### 12.1 发布了 MOVE_POINT，但是车不动

优先检查：

```bash
rostopic echo /ugv1/sunray/localization/local_odom
rostopic echo /ugv1/sunray/ugv_control/control_state
rostopic echo /ugv1/sunray/ugv_control/cmd_vel
```

如果 `odom_valid=false`，说明没有定位输入。
如果 `cmd_vel` 有输出但车不动，说明问题可能在底盘驱动、仿真订阅话题或 remap。

### 12.2 速度命令一会儿就停车

`MOVE_VELOCITY` 和 `MOVE_VELOCITY_BODY` 必须持续发布，并且 `header.stamp` 应该设置为当前时间。如果只发布一次，超过 `wait_velcmd_time` 后会自动切换到 `HOLD`。

如果忘记填写 `header.stamp`，控制器会拒绝该速度命令并切换 `HOLD`，同时在 `UGVControlState.diagnostic_msg` 中提示 `velocity command header.stamp is zero`。如果命令发布频率过低或中断，`diagnostic_msg` 会提示速度命令超时。

### 12.3 差速车发送 MOVE_VELOCITY 后停车

这是当前设计。差速底盘默认不支持世界系 `MOVE_VELOCITY`，建议使用：

- `MOVE_POINT`：给目标点。
- `MOVE_VELOCITY_BODY`：给车体系 `linear.x` 和 `angular.z`。

### 12.4 地理围栏会自动停车吗

默认不会。地理围栏状态会发布到 `UGVControlState.inside_geo_fence`，用于监控和二次开发。

如果希望越界后自动停车，可以在 `config/ugv_control_base.yaml` 或具体 `config/airframes/*.yaml` 中设置：

```yaml
enable_geo_fence_protection: true
```

开启后，只要控制节点已经收到有效 odom，并检测到当前位置超出 `fence_min_*` / `fence_max_*` 范围，就会切换到 `HOLD` 并发布零速度。


</section>
