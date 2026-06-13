<!-- title: 无人机控制 -->

<section id="sunray-uav-control">

## sunray_uav_control

`sunray_uav_control` 是 Sunray 无人机控制链路的核心包。它接收上层模块发来的 `sunray_msgs/UAVControlCMD` 控制指令，读取定位里程计和 MAVROS/PX4 状态，通过状态机决定当前任务阶段，再调用具体控制器向 MAVROS 发布底层 setpoint。

本文按“小白开发者能照着启动和改接口”的目标来写，重点说明输入输出、架构关系、启动方式、配置文件和二次开发入口。

## 1. 这个包解决什么问题

简单理解：

```text
上层程序 / 地面站 / 规划器
        |
        | 发布 /uav1/sunray/uav_control/control_cmd
        v
Sunray_FSM 状态机
        |
        | 根据 TAKEOFF / HOVER / MOVE / LAND 等状态调用控制器接口
        v
Controller_Interface 具体实现
        |
        | 发布 MAVROS setpoint / 调 MAVROS 服务
        v
PX4 飞控
```

它不直接做路径规划，也不直接读遥控器摇杆。它的职责是把“我要起飞、悬停、移动到点、按速度飞、降落”等上层命令转换为 PX4 能执行的控制输出。

## 2. 主要目录

```text
sunray_uav_control/
├── config/
│   ├── sunray_control_base.yaml         # 所有机型共用的基础控制配置
│   └── airframes/                       # 不同实机/仿真机型的参数覆盖文件
├── launch/
│   ├── uav_control.launch               # 单机启动
│   └── uav_control_swarm.launch         # 多机递归启动
├── include/
│   ├── statemachine/                    # 状态机状态、事件、参数、主类声明
│   ├── controller/                      # 控制器接口和具体控制器声明
│   ├── control_data_types/              # 控制命令、目标点、MAVROS 数据结构
│   ├── mavros_helper/                   # MAVROS 封装
│   └── utils/                           # 到达判断、限幅、姿态工具、曲线工具
├── src/
│   ├── uav_control_node.cpp             # 主节点入口
│   ├── statemachine/                    # FSM 参数读取、状态转移、控制调度、检查
│   ├── controller/                      # PX4 原生控制器和几何控制器实现
│   ├── mavros_helper/                   # MAVROS 话题/服务封装实现
│   └── tools/                           # 终端监控、RViz 可视化辅助节点
└── test/                                # 编译测试、工具测试、控制器运行测试
```

## 3. 启动方式

### 3.1 单机启动

默认启动 `uav1`：

```bash
roslaunch sunray_uav_control uav_control.launch
```

指定编号和机型：

```bash
roslaunch sunray_uav_control uav_control.launch agent_name:=uav agent_id:=1 airframe_type:=sunray_150
```

常用 launch 参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `airframe_type` | `sunray_150` | 机型覆盖配置名，对应 `config/airframes/<airframe_type>.yaml` |
| `config_base_path` | `config/sunray_control_base.yaml` | 公共基础配置路径 |
| `airframe_config_path` | `config/airframes/<airframe_type>.yaml` | 机型覆盖配置路径 |
| `agent_name` | `uav` | 智能体名前缀 |
| `agent_id` | `1` | 智能体编号 |
| `node_name` | `uav_control_node_<id>` | ROS 节点名 |
| `set_global_params` | `true` | 是否写入全局 `/agent_name`、`/agent_id` |
| `use_private_agent_key` | `false` | 是否从节点私有参数组合 agent key |
| `enable_uav_control_monitor` | `true` | 是否启动控制状态终端监控 |
| `enable_rviz_visualization` | `true` | 是否启动 RViz marker 可视化辅助节点 |
| `rviz_frame_id` | `world` | Marker 使用的 frame |

单机启动后，核心话题通常是：

```text
/uav1/sunray/uav_control/control_cmd      # 输入：上层控制命令
/uav1/sunray/uav_control/control_state    # 输出：控制状态、最近命令、里程计、底层输出
/uav1/sunray/localization/local_odom      # 输入：本机局部里程计，具体可由 YAML 改
/uav1/sunray/localization/odom_state      # 输入：定位模块状态
/uav1/sunray/px4_state                    # 输出：MAVROS/PX4 汇总状态，由控制器定时发布
```

### 3.2 多机启动

启动 `uav1` 到 `uav6`：

```bash
roslaunch sunray_uav_control uav_control_swarm.launch agent_num:=6
```

`uav_control_swarm.launch` 会递归 include `uav_control.launch`，每个 agent 使用自己的私有 `agent_name/agent_id`，并只启动一个集群监控节点。

## 4. 输入接口

### 4.1 控制命令话题

主输入话题：

```text
/<agent_name><agent_id>/sunray/uav_control/control_cmd
```

例如：

```text
/uav1/sunray/uav_control/control_cmd
```

消息类型：

```text
sunray_msgs/UAVControlCMD
```

关键字段：

| 字段 | 作用 |
| --- | --- |
| `cmd_source` | 命令来源，如地面站、终端、规划器、集群控制 |
| `control_cmd` | 控制命令，如起飞、降落、悬停、移动 |
| `desired_pos` | 惯性系目标位置，用于 `MOVE_POINT`、`MOVE_TRAJECTORY` |
| `desired_vel` | 惯性系目标速度，用于 `MOVE_VELOCITY`、`MOVE_TRAJECTORY` |
| `desired_acc` | 惯性系目标加速度，主要用于轨迹跟踪 |
| `desired_jerk` | jerk，结构中保留，部分控制器不使用 |
| `desired_body_xy_pos` | 机体系水平位移，用于 `MOVE_POINT_BODY` |
| `desired_body_xy_vel` | 机体系水平速度，用于 `MOVE_VELOCITY_BODY` |
| `fixed_height` | 固定高度，body 控制必须填；速度控制中大于 0 时用于锁高 |
| `desired_wgs84_pos` | 经纬高目标，目前接口保留，控制实现尚未完整支持 |
| `yaw_mode` | 偏航控制模式：保持、指定 yaw、指定 yaw rate |
| `desired_yaw` | 目标 yaw，单位 rad |
| `desired_yaw_rate` | 目标 yaw rate，单位 rad/s |

### 4.2 控制命令枚举

| 命令 | 值 | 发送频率建议 | 说明 |
| --- | ---: | --- | --- |
| `TAKEOFF` | 1 | 发一次即可 | 原地起飞到配置高度 |
| `LAND` | 2 | 发一次即可 | 降落 |
| `RETURN` | 3 | 发一次即可 | 返回起飞记录的 home 点，可配置是否自动降落 |
| `KILL` | 4 | 发一次即可 | 强制锁桨，危险操作 |
| `HOVER` | 5 | 发一次即可 | 在当前有效里程计位置悬停 |
| `MOVE_POINT` | 6 | 发一次即可 | 惯性系位置点控制 |
| `MOVE_VELOCITY` | 7 | 建议持续发布，至少 10Hz | 惯性系速度控制 |
| `MOVE_TRAJECTORY` | 8 | 建议高频，约 100Hz | 惯性系轨迹跟踪 |
| `MOVE_POINT_BODY` | 9 | 发一次即可 | 机体系水平位置 + 惯性系固定高度 |
| `MOVE_VELOCITY_BODY` | 10 | 建议持续发布，至少 10Hz | 机体系水平速度 + 惯性系固定高度 |
| `MOVE_POINT_WGS84` | 11 | 暂不建议使用 | 接口保留，控制层未完整实现 |

注意：FSM 内部用收到消息的当前 ROS 时间覆盖 `header.stamp`，所以新手开发时不用过分依赖消息自带时间戳。但仍建议正常填写 `cmd.header.stamp = ros::Time::now()`。

### 4.3 发送一个位置点命令示例

C++ 示例：

```cpp
#include <ros/ros.h>
#include <sunray_msgs/UAVControlCMD.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_uav_cmd_demo");
    ros::NodeHandle nh;

    ros::Publisher pub =
        nh.advertise<sunray_msgs::UAVControlCMD>("/uav1/sunray/uav_control/control_cmd", 10);
    ros::Duration(1.0).sleep();

    sunray_msgs::UAVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd_source = sunray_msgs::UAVControlCMD::EXAMPLE_DEMO;
    cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_POINT;
    cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
    cmd.desired_pos.x = 1.0;
    cmd.desired_pos.y = 0.0;
    cmd.desired_pos.z = 0.8;
    cmd.desired_yaw = 0.0;

    pub.publish(cmd);
    ros::spinOnce();
    return 0;
}
```

常见任务顺序：

```text
等待 control_state 进入 INIT
发 TAKEOFF
等待 control_state 进入 HOVER
发 MOVE_POINT / MOVE_VELOCITY / MOVE_TRAJECTORY
任务结束发 HOVER 或 LAND
```

更完整的控制接口示例请优先参考：

```text
examples/sunray_uav_control_example/
```

这个目录按 `UAVControlCMD.control_cmd` 类型拆分示例，适合小白开发者照着改：

| 需求 | 建议先看 |
| --- | --- |
| 起飞、悬停、降落的最小流程 | `src/basic/takeoff_land.cpp` |
| 惯性系位置点控制 | `src/MOVE_POINT/move_point.cpp`、`move_multipoint.cpp`、`move_point_and_return.cpp` |
| 机体系相对位置控制 | `src/MOVE_POINT_BODY/move_point_body.cpp` |
| 经纬高位置接口 | `src/MOVE_POINT_WGS84/move_point_wgs84.cpp` |
| 惯性系速度控制 | `src/MOVE_VELOCITY/move_velocity.cpp`、`circle_velocity.cpp`、`lemniscate_velocity.cpp`、`move_velocity_fixed_height.cpp` |
| 带 yaw 控制的速度示例 | `src/MOVE_VELOCITY/circle_velocity_yaw.cpp` |
| 机体系速度控制 | `src/MOVE_VELOCITY_BODY/move_velocity_body.cpp` |
| 轨迹控制 | `src/MOVE_TRAJECTORY/move_trajectory.cpp`、`circle_trajectory.cpp`、`lemniscate_trajectory.cpp` |

`control/control_tools` 下面的代码也可以参考，但它们更偏“工具”和“测试”：

| 文件 | 用途 |
| --- | --- |
| `control/control_tools/src/terminal_control/uav_terminal_control.cpp` | 终端交互式发命令，适合快速手动测试话题是否通 |
| `control/control_tools/src/uav_control_panel_node.cpp` | Qt/RViz 控制面板，适合地面站 UI 操作和状态观察 |
| `control/control_tools/src/velocity_test/triangle_velocity_test.cpp` | 速度闭环压力测试，不建议作为新手二次开发模板 |

写自己的控制节点时，优先复制 `examples/sunray_uav_control_example` 中最接近需求的示例，再按任务修改目标点、速度、轨迹生成逻辑。注意发布频率规则保持一致：`TAKEOFF`、`LAND`、`RETURN`、`HOVER`、`MOVE_POINT`、`MOVE_POINT_BODY`、`MOVE_POINT_WGS84` 通常发一次即可；`MOVE_VELOCITY`、`MOVE_VELOCITY_BODY` 和 `MOVE_TRAJECTORY` 需要在任务执行期间持续发布。

## 5. 输出接口

### 5.1 控制状态话题

主输出话题：

```text
/<agent_name><agent_id>/sunray/uav_control/control_state
```

消息类型：

```text
sunray_msgs/UAVControlState
```

主要内容：

| 字段 | 说明 |
| --- | --- |
| `agent_name` / `agent_id` | 当前无人机标识 |
| `controller_types` | 当前控制器类型 |
| `takeoff_relative_height` / `land_type` 等 | 当前起降配置 |
| `home_point` | 返航点 |
| `control_state` | FSM 当前状态 |
| `last_cmd` | 最近收到并缓存的上层控制命令 |
| `self_odom` | 最近收到的本机里程计 |
| `odometry_lost` | 里程计是否超时 |
| `odometry_valid` | 最近里程计是否通过基本合法性检查 |
| `controller_output_type` | 当前底层输出类型 |
| `position_target` | 最近一次 MAVROS `PositionTarget` |
| `attitude_target` | 最近一次 MAVROS `AttitudeTarget` |

状态枚举：

| 状态 | 值 | 含义 |
| --- | ---: | --- |
| `OFF` | 0 | 控制器或里程计尚未就绪 |
| `INIT` | 1 | 初始化完成，等待起飞 |
| `TAKEOFF` | 2 | 起飞中 |
| `HOVER` | 3 | 悬停中 |
| `RETURN` | 4 | 返航中 |
| `LAND` | 5 | 降落中 |
| `MOVE` | 6 | 执行移动/速度/轨迹命令 |
| `EMERGENCY_KILL` | 7 | 紧急锁桨状态 |

### 5.2 MAVROS/PX4 相关输出

底层输出由 `MavrosHelper` 封装：

| 输出 | 话题/服务 | 说明 |
| --- | --- | --- |
| 外部位姿融合 | `/uav1/mavros/vision_pose/pose` | `fuse_odom_type=1` 时发布 |
| 外部里程计融合 | `/uav1/mavros/odometry/in` | `fuse_odom_type=2` 时发布 |
| 位置 setpoint | `/uav1/mavros/setpoint_raw/local` | `PX4_OriginController` 常用 |
| 姿态 setpoint | `/uav1/mavros/setpoint_raw/attitude` | `Geometric_Controller` 常用 |
| PX4 汇总状态 | `/uav1/sunray/px4_state` | 给监控工具和其他模块使用 |
| 解锁服务 | `/uav1/mavros/cmd/arming` | 控制器起飞流程中调用 |
| 模式切换服务 | `/uav1/mavros/set_mode` | 切换 POSCTL/OFFBOARD/AUTO.LAND 等 |
| 命令服务 | `/uav1/mavros/cmd/command` | KILL、重启等命令 |

### 5.3 PX4 汇总状态话题

话题：

```text
/uav1/sunray/px4_state
```

消息类型：

```text
sunray_msgs/Px4State
```

这个话题由当前控制器内部的 `MavrosHelper` 定时发布，默认频率在控制器里是 `100Hz`。它不是新的 MAVROS 数据源，而是把常用 MAVROS 输入、外部定位融合数据、控制 setpoint 和 PX4 状态整理到一个 Sunray 自定义消息里，方便监控工具和二次开发模块读取。

主要字段：

| 字段 | 来源 | 说明 |
| --- | --- | --- |
| `connected` | `/mavros/state` | MAVROS 是否连接 PX4 |
| `rc_available` | `/mavros/state.manual_input` | PX4/MAVROS 是否认为 RC 输入可用 |
| `armed` | `/mavros/state` | 飞控是否解锁 |
| `flight_mode` | `/mavros/state.mode` | PX4 飞行模式，已转为 Sunray 内部枚举值 |
| `system_status` | `/mavros/state` | PX4 系统状态 |
| `rc_channels` | `/mavros/rc/in` | RC 原始通道数组，通道含义取决于遥控器和 PX4 映射 |
| `rc_rssi` | `/mavros/rc/in` | RC 信号强度 |
| `landed_state` | `/mavros/extended_state` | PX4 起降状态 |
| `battery_voltage_v/current_a/percentage` | `/mavros/sys_status` | 电池电压、电流、电量百分比 |
| `external_pose/external_velocity` | 控制器传给 MAVROS 的外部定位 | 当前用于融合的外部里程计快照 |
| `local_pose/local_velocity` | `/mavros/local_position/odom` | PX4/MAVROS 局部位姿和速度 |
| `pos_setpoint/vel_setpoint/acc_setpoint/yaw_setpoint` | `/mavros/setpoint_raw/target_local` | 最近一次位置类 setpoint |
| `orientation_setpoint/body_rate_setpoint/thrust_setpoint` | `/mavros/setpoint_raw/target_attitude` | 最近一次姿态类 setpoint |
| `satellites/gps_status/latitude/longitude/altitude` | `/mavros/gpsstatus/gps1/raw` | GPS 状态和经纬高信息 |

RC 通道只是原始数组，不在这里强行解释成 roll/pitch/throttle/yaw。不同遥控器、PX4 参数和 MAVROS 配置可能有不同通道映射，二次开发时应根据自己的遥控器映射表解释 `rc_channels[i]`。

## 6. 内部架构

### 6.1 主节点

入口文件：

```text
src/uav_control_node.cpp
```

启动流程：

```text
ros::init
创建 Sunray_FSM
Sunray_FSM::init()
启动 AsyncSpinner 处理订阅回调
按 supervisor_update_frequency 循环调用 Sunray_FSM::process()
```

`Sunray_FSM::init()` 内部做四件事：

```text
load_param()          # 从 YAML 读参数
init_publisher()      # 发布 control_state 等
init_subscriber()     # 订阅 odom、control_cmd、odom_state
register_controller() # 根据 controller_types 创建具体控制器
```

### 6.2 两条循环

代码里有两条关键循环：

| 循环 | 频率来源 | 作用 |
| --- | --- | --- |
| FSM 监督循环 | `basic_param.supervisor_update_frequency` | 检查超时、判断任务完成、处理状态事件、发布 `UAVControlState` |
| 控制器高频循环 | `basic_param.controller_update_frequency` | 把最新有效里程计注入控制器，根据 FSM 状态调用控制器输出 |

这种设计让“状态切换”和“底层控制输出”解耦。状态机不用 100Hz 处理复杂逻辑，控制器也不用等低频状态发布。

### 6.3 状态机事件关系

核心状态转移在：

```text
src/statemachine/sunray_fsm_transition.cpp
```

主要流程：

```text
OFF --控制器和里程计就绪--> INIT
INIT --TAKEOFF_REQUEST--> TAKEOFF
TAKEOFF --TAKEOFF_COMPLETED--> HOVER
HOVER --POINT/VELOCITY/TRAJECTORY_REQUEST--> MOVE
MOVE --任务完成或 HOVER_REQUEST--> HOVER
HOVER/MOVE --RETURN_REQUEST--> RETURN
HOVER/MOVE/RETURN --LAND_REQUEST--> LAND
LAND --LAND_COMPLETED--> INIT
任意状态 --KILL_REQUEST--> EMERGENCY_KILL
```

上层命令到事件的映射在：

```text
src/statemachine/sunray_fsm.cpp
Sunray_FSM::uav_control_cmd_callback()
```

当前状态到控制器接口的映射在：

```text
src/statemachine/sunray_fsm_control.cpp
Sunray_FSM::update_controller_output()
```

例如：

| FSM 状态 | 调用控制器接口 |
| --- | --- |
| `INIT` | `set_position_mode()` |
| `TAKEOFF` | `takeoff()` |
| `HOVER` | `hover()` |
| `LAND` | `land()` |
| `RETURN` | `move_point(home_point)` |
| `MOVE + MOVE_POINT` | `move_point()` |
| `MOVE + MOVE_VELOCITY` | `move_velocity()` |
| `MOVE + MOVE_TRAJECTORY` | `move_trajectory()` |
| `EMERGENCY_KILL` | `emergency_kill()` |

### 6.4 控制器接口

统一接口在：

```text
include/controller/controller_interface.hpp
```

已有实现：

| `controller_types` | 控制器 | 主要输出 |
| ---: | --- | --- |
| `0` | `PX4_OriginController` | `mavros_msgs/PositionTarget` |
| `1` | `Geometric_Controller` | `mavros_msgs/AttitudeTarget` |

`PX4_OriginController` 更依赖 PX4 原生位置控制环；`Geometric_Controller` 在包内计算姿态/角速度和推力，再发给 PX4。

## 7. 配置文件

默认配置采用“公共配置 + 机型覆盖配置”：

```text
config/sunray_control_base.yaml
config/airframes/<airframe_type>.yaml
```

加载顺序固定为：先读取 `sunray_control_base.yaml`，再读取 `airframes/<airframe_type>.yaml`。如果两个文件中有相同参数路径，机型文件优先覆盖；数组参数整体覆盖，不做逐元素合并。

默认 launch 支持四个机型：

| `airframe_type` | 用途 |
| --- | --- |
| `sunray_150` | Sunray 150 系列实机 |
| `sunray_300` | Sunray 300 系列实机 |
| `gazebo_sim` | Gazebo 仿真 |
| `pengyu_sim` | Pengyu 仿真 |

启动示例：

```bash
roslaunch sunray_uav_control uav_control.launch airframe_type:=sunray_150
roslaunch sunray_uav_control uav_control.launch airframe_type:=gazebo_sim
```

兼容说明：代码仍支持外部自定义 launch 只传 `config_yamlfile_path` 的单文件配置方式；但仓库默认 launch 不再提供单文件配置，统一使用 `config_base_path + airframe_config_path`。

### 7.1 `basic_param`

| 参数 | 说明 |
| --- | --- |
| `controller_types` | 控制器类型，`0` PX4 原生，`1` 几何控制 |
| `controller_update_frequency` | 控制器高频循环频率 |
| `supervisor_update_frequency` | FSM 监督循环频率 |
| `odom_topic_name` | 输入里程计话题，可写 `${agent_key}` 占位符 |
| `fuse_odom_type` | 外部里程计融合方式，`0` 关闭，`1` vision_pose，`2` odometry |
| `fuse_odom_frequency` | 向 MAVROS 发布外部定位的频率 |

`odom_topic_name` 默认：

```yaml
odom_topic_name: "${agent_key}/sunray/localization/local_odom"
```

如果 `agent_key` 是 `/uav1`，实际订阅就是：

```text
/uav1/sunray/localization/local_odom
```

### 7.2 `takeoff_land_param`

| 参数 | 说明 |
| --- | --- |
| `takeoff_relative_height` | 起飞相对高度 |
| `takeoff_max_velocity` | 起飞最大速度 |
| `land_type` | `0` Sunray 控制降落，`1` PX4 AUTO.LAND |
| `land_max_velocity` | 降落最大速度 |
| `return_with_land` | 返航到 home 点后是否自动降落 |

### 7.3 `arrival_judge_param`

控制“是否到达目标点”的判定阈值。`MOVE_POINT`、起飞完成、返航完成等逻辑都会依赖这些参数。

| 参数 | 说明 |
| --- | --- |
| `judge_stabile_time_s` | 误差进入范围后需要保持的时间 |
| `pos_stabile_err_m` | 位置稳定误差 |
| `vel_stabile_err_mps` | 速度稳定误差 |
| `max_pos_err_m` | 放宽判断时允许的最大位置误差 |
| `yaw_stabile_err_deg` | yaw 稳定误差 |
| `yaw_rate_stabile_err_deg_s` | yaw rate 稳定误差 |

### 7.4 `velocity_param`

| 参数 | 说明 |
| --- | --- |
| `max_velocity` | 代码控制下的 xyz 最大速度 |
| `max_velocity_with_rc` | 遥控器控制下的 xyz 最大速度，当前 FSM 中未完整使用 |
| `yaw_rate` | yaw 角速度上限，配置里是 deg/s，代码读取后转 rad/s |

### 7.5 `geometric_controller_param`

只给 `Geometric_Controller` 使用；`controller_types=0` 时这些参数不会被 `PX4_OriginController` 使用。

| 参数 | 说明 |
| --- | --- |
| `mass_kg` | 无人机质量，几何控制器物理参数 |
| `gravity` | 重力加速度 |
| `hover_thrust_percent` | 归一化悬停推力初值 |
| `control_type` | `0` 输出姿态+推力，`1` 输出角速度+推力 |
| `hover_thrust_estimator_type` | `0` RLS，`1` EKF 加速度观测 |
| `takeoff_land_type` | `0` 直接推力起降，`1` 加速度前馈起降 |
| `pos_kp/ki/kd` | 位置环 PID |
| `vel_kp/ki/kd` | 速度环 PID |
| `max_acc` | 反馈加速度限幅 |
| `attitude_tau` | 姿态控制时间常数 |

## 8. 二次开发怎么做

### 8.1 写一个上层控制节点

这是最常见的二次开发方式。你的节点只需要：

1. 订阅 `/uav1/sunray/uav_control/control_state`，知道当前状态。
2. 发布 `/uav1/sunray/uav_control/control_cmd`，发送任务命令。
3. 对速度和轨迹类命令持续发布，对起飞/降落/位置点命令发一次即可。

建议从 `examples/sunray_uav_control_example` 开始阅读和复制代码，而不是直接从控制面板或终端工具改起。示例目录已经按命令类型分好类，代码更短，依赖更少，也更接近二次开发节点的真实写法：

| 二次开发目标 | 推荐示例 |
| --- | --- |
| 最小起降流程 | `examples/sunray_uav_control_example/src/basic/takeoff_land.cpp` |
| 发一个或多个位置点 | `examples/sunray_uav_control_example/src/MOVE_POINT/` |
| 发机体系相对位置 | `examples/sunray_uav_control_example/src/MOVE_POINT_BODY/move_point_body.cpp` |
| 发速度命令并自己做外环逻辑 | `examples/sunray_uav_control_example/src/MOVE_VELOCITY/` |
| 发机体系速度 | `examples/sunray_uav_control_example/src/MOVE_VELOCITY_BODY/move_velocity_body.cpp` |
| 接轨迹生成器或规划器 | `examples/sunray_uav_control_example/src/MOVE_TRAJECTORY/` |

`control/control_tools/src/terminal_control/uav_terminal_control.cpp`、`control/control_tools/src/uav_control_panel_node.cpp` 和 `control/control_tools/src/velocity_test/triangle_velocity_test.cpp` 可以作为发布逻辑、UI 交互、测试流程的补充参考，但不建议作为新手第一份模板。

推荐状态判断方式：

```text
启动后等待 control_state == INIT
发 TAKEOFF
等待 control_state == HOVER
发任务命令
任务结束发 HOVER 或 LAND
```

### 8.2 接入规划器

规划器一般输出位置、速度、加速度、yaw。转换为：

```text
control_cmd = MOVE_TRAJECTORY
desired_pos = 规划器位置
desired_vel = 规划器速度
desired_acc = 规划器加速度
yaw_mode = SET_YAW 或 SET_YAWRATE
```

发布频率建议接近控制器频率，至少不要低于轨迹本身的更新频率。仓库里的 `examples/sunray_uav_planning_example` 已经有 `position_cmd -> UAVControlCMD` 的转换思路。

### 8.3 新增一种控制命令

如果你要新增 `UAVControlCMD` 命令类型，需要改这些地方：

1. `common/sunray_msgs/msg/UAVControlCMD.msg`：增加枚举和字段。
2. `include/control_data_types/uav_control_cmd_types.hpp`：把 ROS msg 解析到内部结构体。
3. `include/statemachine/sunray_state_types.hpp`：如果需要新事件，增加 `SunrayEvent`。
4. `src/statemachine/sunray_fsm.cpp`：在 `uav_control_cmd_callback()` 中把命令映射到事件。
5. `src/statemachine/sunray_fsm_transition.cpp`：增加状态转移规则。
6. `src/statemachine/sunray_fsm_control.cpp`：在 `update_controller_output()` 中调用控制器接口。
7. `include/controller/controller_interface.hpp`：如需新控制器能力，增加虚函数。
8. 两个控制器实现中补齐对应函数。
9. `common/sunray_msgs/msg/UAVControlState.msg`：如需对外反馈更多状态，增加字段。
10. `src/tools/uav_control_monitor_node.cpp`、`src/tools/rviz_visualization_uav_control_node.cpp`：补充显示逻辑。

### 8.4 新增一个控制器

如果不是新增命令，而是新增底层控制算法：

1. 新建一个类继承 `Controller_Interface`。
2. 实现 `init()`、`is_ready()`、`set_current_odom()`、`takeoff()`、`hover()`、`move_point()`、`land()` 等接口。
3. 在 `CMakeLists.txt` 把新 `.cpp` 加入 `sunray_uav_control_lib`。
4. 在 `config/sunray_control_base.yaml` 约定一个新的 `controller_types` 值。
5. 在 `Sunray_FSM::register_controller()` 中按新值创建控制器。
6. 在参数读取函数里放开 `controller_types` 的校验范围。

新增控制器时，优先保持 `Controller_Interface` 的语义不变。FSM 不应该知道控制器内部是 PID、几何控制、MPC 还是其他算法。

## 9. 调试建议

查看状态：

```bash
rostopic echo /uav1/sunray/uav_control/control_state
```

查看上层命令：

```bash
rostopic echo /uav1/sunray/uav_control/control_cmd
```

查看 MAVROS 输出：

```bash
rostopic echo /uav1/mavros/setpoint_raw/local
rostopic echo /uav1/mavros/setpoint_raw/attitude
```

查看 PX4 汇总状态：

```bash
rostopic echo /uav1/sunray/px4_state
```

常见问题：

| 现象 | 优先检查 |
| --- | --- |
| 一直停在 `OFF` | 是否有有效 local odom，MAVROS 状态是否新鲜，控制器 `is_ready()` 是否通过 |
| 发 `TAKEOFF` 没反应 | 当前是否已经是 `INIT`，是否有有效里程计，是否允许切 OFFBOARD/ARM |
| 速度命令很快停住 | `MOVE_VELOCITY` / `MOVE_VELOCITY_BODY` 需要持续发布，FSM 内部 0.2s 未收到新命令会认为指令流停止 |
| 位置点到达后抖动 | 调 `arrival_judge_param`、速度限幅和控制器 PID |
| RViz 没显示 | 检查 `enable_rviz_visualization`、`state_topic`、`marker_topic`、`rviz_frame_id` |

### MAVROS、MAVLink 与 uORB 的关系

`sunray_uav_control` 不是 MAVROS 教程，也不直接暴露 PX4 uORB。这个控制包只关心两类接口：

- 从 MAVROS/PX4 读取必要状态，例如连接、解锁、模式、起降状态、RC、电池、GPS、local odom 和 setpoint 回显。
- 向 MAVROS/PX4 发布外部定位、位置/速度 setpoint、姿态 setpoint，并调用解锁、模式切换、参数读写等服务。

整体链路可以简化理解为：

```text
Sunray 控制命令
  -> sunray_uav_control
  -> MAVROS topic/service
  -> MAVLink message
  -> PX4 内部 uORB / commander / controller
```

本章只保留控制包实际相关的接口说明。MAVROS 标准话题列表、MAVLink 消息含义、PX4 uORB 对照、ENU/NED 坐标差异和插件配置，请阅读文档最后的“附录 -> MAVROS”。

控制调试时最常用的检查命令是：

```bash
rostopic echo /uav1/mavros/state
rostopic echo /uav1/mavros/extended_state
rostopic echo /uav1/mavros/local_position/odom
rostopic echo /uav1/mavros/setpoint_raw/target_local
rostopic echo /uav1/mavros/setpoint_raw/target_attitude
rostopic echo /uav1/sunray/px4_state
```

如果你只是写任务层代码，优先订阅 `/uav1/sunray/px4_state` 和 `/uav1/sunray/uav_control/control_state`，发布 `/uav1/sunray/uav_control/control_cmd`。不要一开始绕过 Sunray 控制节点直接发布 `/uav1/mavros/setpoint_raw/*`，否则需要自己处理 Offboard 频率、模式切换、解锁、failsafe、坐标系和 `type_mask`。


</section>
