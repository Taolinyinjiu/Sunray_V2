<!-- title: 无人机控制示例 -->

<section id="examples-uav-control">

## 无人机控制示例

路径：

```text
examples/sunray_uav_control_example
```

这个包集中演示如何通过 `sunray_msgs/UAVControlCMD` 调用 `sunray_uav_control` 的上层控制接口。它比控制包正文更具体：重点讲每个示例怎么启动、会做什么、代码里该改哪里，以及新手最容易踩到的点。

这些示例不是底层控制器。它们的角色是“任务节点”：

```text
示例节点
  发布 /uav1/sunray/uav_control/control_cmd
  订阅 /uav1/sunray/uav_control/control_state
  部分示例订阅 /uav1/sunray/localization/local_odom

sunray_uav_control
  接收 control_cmd
  运行状态机和控制器
  向 MAVROS/PX4 发布底层 setpoint
```

### 运行前准备

运行任意例程前，至少需要先启动：

1. PX4/MAVROS 或仿真环境。
2. 定位模块，保证 `/<agent_key>/sunray/localization/local_odom` 有数据。
3. `sunray_uav_control`，保证 `/<agent_key>/sunray/uav_control/control_state` 能进入 `INIT`。

常用启动方式：

```bash
cd /home/amov/Sunray_v2
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch sunray_uav_control uav_control.launch \
  agent_name:=uav \
  agent_id:=1 \
  airframe_type:=gazebo_sim
```

启动后建议先确认话题存在：

```bash
rostopic echo /uav1/sunray/uav_control/control_state
rostopic echo /uav1/sunray/localization/local_odom
```

多数示例 launch 都包含：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `agent_name` | `uav` | 无人机名字前缀。 |
| `agent_id` | `1` | 无人机编号。 |
| `node_name` | 各示例自己的节点名 | ROS 节点名。 |
| `use_private_agent_key` | `false` | 是否从本示例节点私有参数读取 `agent_name/agent_id`。 |

代码里会把 `agent_name + agent_id` 组合成 `agent_key`。例如 `agent_name:=uav`、`agent_id:=1` 会得到 `/uav1`，于是发布和订阅的话题是：

```text
/uav1/sunray/uav_control/control_cmd
/uav1/sunray/uav_control/control_state
/uav1/sunray/localization/local_odom
```

注意：示例 launch 虽然写了私有参数 `agent_name/agent_id`，但默认 `use_private_agent_key=false`，此时代码会读取全局参数 `/agent_name` 和 `/agent_id`。如果你已经用 `uav_control.launch` 启动，并保持 `set_global_params:=true`，默认值就能正常工作。

如果你想让示例节点完全使用本 launch 传入的参数，启动时加：

```bash
use_private_agent_key:=true
```

当前源码空间下，推荐直接使用 launch 文件路径启动示例：

```bash
roslaunch examples/sunray_uav_control_example/launch/basic/takeoff_land.launch \
  use_private_agent_key:=true \
  agent_name:=uav \
  agent_id:=1
```

这种路径写法已经在当前仓库环境中验证过。`roslaunch sunray_uav_control_example basic/takeoff_land.launch` 这类“包名 + 子目录 launch”的写法在当前环境下不能直接解析到文件。

### 命令发布频率规则

不同 `UAVControlCMD.control_cmd` 的发布方式不同，这是二次开发最重要的规则之一：

| 命令 | 发布方式 | 原因 |
| --- | --- | --- |
| `TAKEOFF` | 发一次 | 状态机收到后进入起飞流程。 |
| `LAND` | 发一次 | 状态机收到后进入降落流程。 |
| `RETURN` | 发一次 | 状态机收到后返航，是否自动降落由配置决定。 |
| `HOVER` | 发一次 | 状态机收到后在当前点悬停。 |
| `MOVE_POINT` | 发一次 | 控制器内部会跟踪这个目标点直到到达。 |
| `MOVE_POINT_BODY` | 发一次 | 控制器内部会跟踪这个机体系相对目标。 |
| `MOVE_VELOCITY` | 持续发布 | 速度命令代表“当前时刻想怎么飞”，停止发布就不应继续依赖旧速度。 |
| `MOVE_VELOCITY_BODY` | 持续发布 | 机体系速度同理，需要任务执行期间连续刷新。 |
| `MOVE_TRAJECTORY` | 持续高频发布 | 轨迹命令包含位置、速度、加速度、jerk，是随时间变化的参考量。 |

本包速度和轨迹示例大多使用 `ros::Rate(50.0)` 或 `ros::Rate(1.0 / kControlDt)` 控制约 `50Hz` 发布频率。实际项目中，轨迹命令建议按规划器输出频率或控制器期望频率发布。

### 状态流程速查

Sunray 示例节点通常不直接控制电机，而是通过 `control_cmd` 触发 `sunray_uav_control` 的状态机。

起飞降落示例：

```text
等待 INIT
  -> 发一次 TAKEOFF
  -> 等待 HOVER
  -> 悬停一段时间
  -> 发一次 LAND
```

位置点示例：

```text
等待 INIT
  -> 发一次 TAKEOFF
  -> 等待 HOVER
  -> 发一次 MOVE_POINT / MOVE_POINT_BODY
  -> 等待 MOVE
  -> 等待重新回到 HOVER
  -> 发一次 LAND 或 RETURN
```

速度和轨迹示例：

```text
等待 INIT
  -> 发一次 TAKEOFF
  -> 等待 HOVER
  -> 等待有效 local_odom
  -> 持续发布 MOVE_VELOCITY / MOVE_VELOCITY_BODY / MOVE_TRAJECTORY
  -> 任务结束后发一次 RETURN
```

最容易出错的是最后一类：`TAKEOFF`、`LAND`、`RETURN`、`MOVE_POINT` 是事件型命令，通常发一次；`MOVE_VELOCITY` 和 `MOVE_TRAJECTORY` 是连续参考量，任务运行期间必须按固定频率持续发。

### 例程总览

| 类别 | launch | 源码 | 状态 |
| --- | --- | --- | --- |
| 起降 | `launch/basic/takeoff_land.launch` | `src/basic/takeoff_land.cpp` | 可运行 |
| 起降 Python | `launch/basic/takeoff_land_py.launch` | `scripts/takeoff_land.py` | 可运行 |
| 位置点 | `launch/MOVE_POINT/move_point.launch` | `src/MOVE_POINT/move_point.cpp` | 可运行 |
| 多位置点 | `launch/MOVE_POINT/move_multipoint.launch` | `src/MOVE_POINT/move_multipoint.cpp` | 可运行 |
| 位置点后返航 | `launch/MOVE_POINT/move_point_and_return.launch` | `src/MOVE_POINT/move_point_and_return.cpp` | 可运行 |
| 机体系位置 | `launch/MOVE_POINT_BODY/move_point_body.launch` | `src/MOVE_POINT_BODY/move_point_body.cpp` | 可运行 |
| 经纬高位置 | `launch/MOVE_POINT_WGS84/move_point_wgs84.launch` | `src/MOVE_POINT_WGS84/move_point_wgs84.cpp` | 当前为空，不建议使用 |
| 速度矩形航点 | `launch/MOVE_VELOCITY/move_velocity.launch` | `src/MOVE_VELOCITY/move_velocity.cpp` | 可运行 |
| 速度固定高度 | `launch/MOVE_VELOCITY/move_velocity_fixed_height.launch` | `src/MOVE_VELOCITY/move_velocity_fixed_height.cpp` | 可运行 |
| 速度圆形 | `launch/MOVE_VELOCITY/circle_velocity.launch` | `src/MOVE_VELOCITY/circle_velocity.cpp` | 可运行 |
| 速度圆形 + yaw | `launch/MOVE_VELOCITY/circle_velocity_yaw.launch` | `src/MOVE_VELOCITY/circle_velocity_yaw.cpp` | 可运行 |
| 速度 8 字 | `launch/MOVE_VELOCITY/lemniscate_velocity.launch` | `src/MOVE_VELOCITY/lemniscate_velocity.cpp` | 可运行 |
| 机体系速度 | `launch/MOVE_VELOCITY_BODY/move_velocity_body.launch` | `src/MOVE_VELOCITY_BODY/move_velocity_body.cpp` | 可运行 |
| 轨迹矩形航点 | `launch/MOVE_TRAJECTORY/move_trajectory.launch` | `src/MOVE_TRAJECTORY/move_trajectory.cpp` | 可运行 |
| 轨迹圆形 | `launch/MOVE_TRAJECTORY/circle_trajectory.launch` | `src/MOVE_TRAJECTORY/circle_trajectory.cpp` | 可运行 |
| 轨迹 8 字 | `launch/MOVE_TRAJECTORY/lemniscate_trajectory.launch` | `src/MOVE_TRAJECTORY/lemniscate_trajectory.cpp` | 可运行 |

### Basic 示例

#### takeoff_land

启动：

```bash
roslaunch examples/sunray_uav_control_example/launch/basic/takeoff_land.launch \
  use_private_agent_key:=true agent_name:=uav agent_id:=1
```

用途：最小起降流程，适合第一次确认控制链路是否通。

执行流程：

```text
等待 control_state == INIT
发一次 TAKEOFF
等待 control_state == HOVER
悬停 5 秒
发一次 LAND
退出节点
```

新手重点看：

| 位置 | 说明 |
| --- | --- |
| `uav_state_callback()` | 接收控制状态。 |
| `control_cmd_pub.publish(uav_cmd)` | 发布控制命令。 |
| `uav_cmd.control_cmd = TAKEOFF/LAND` | 起飞和降落命令只发一次。 |

修改起飞高度、降落速度等参数时，不改这个示例节点，应该改 `sunray_uav_control/config/sunray_control_base.yaml` 或对应 `config/airframes/<airframe_type>.yaml`。

安全边界：

| 项目 | 当前行为 |
| --- | --- |
| 飞行范围 | 不主动水平移动，只执行起飞、悬停、降落。 |
| 高度 | 由 `sunray_uav_control` 起飞参数决定，不在本示例中硬编码。 |
| 结束动作 | `LAND`，节点发出降落命令后退出。 |
| 实机建议 | 第一次只用这个例程确认解锁、起飞、悬停、降落链路。 |

#### takeoff_land_py

启动：

```bash
roslaunch examples/sunray_uav_control_example/launch/basic/takeoff_land_py.launch \
  use_private_agent_key:=true agent_name:=uav agent_id:=1
```

用途：`rospy` 版本的最小起降流程，适合习惯用 Python 写上层任务逻辑的开发者参考。

它和 C++ 版 `takeoff_land` 的控制行为一致：

```text
等待 control_state == INIT
发一次 TAKEOFF
等待 control_state == HOVER
悬停 5 秒
发一次 LAND
退出节点
```

新手重点看：

| 位置 | 说明 |
| --- | --- |
| `TakeoffLandDemo._control_state_cb()` | 接收控制状态。 |
| `TakeoffLandDemo._publish_cmd_once()` | 填充并发布 `UAVControlCMD`。 |
| `TakeoffLandDemo._wait_for_state()` | 用 `rospy.Rate` 等待目标状态。 |
| `use_private_agent_key` 参数 | 和 C++ 示例保持一致，决定读取私有参数还是全局参数。 |

### MOVE_POINT 示例

#### move_point

启动：

```bash
roslaunch examples/sunray_uav_control_example/launch/MOVE_POINT/move_point.launch \
  use_private_agent_key:=true agent_name:=uav agent_id:=1
```

用途：飞到一个惯性系目标点，然后降落。

关键目标点在源码顶部：

```cpp
#define Point_X 1.0
#define Point_Y 0.0
#define Point_Z 0.6
```

执行流程：

```text
INIT -> TAKEOFF -> HOVER
发一次 MOVE_POINT，desired_pos=(1.0, 0.0, 0.6)
等待控制状态从 MOVE 回到 HOVER
发一次 LAND
```

适合学习：最简单的位置控制命令怎么填 `desired_pos`。

安全边界：

| 项目 | 当前行为 |
| --- | --- |
| 目标点 | 惯性系 `(1.0, 0.0, 0.6)`。 |
| 飞行范围 | 起飞点前方约 `1m`，高度约 `0.6m`。 |
| 命令频率 | `MOVE_POINT` 只发布一次。 |
| 结束动作 | 到点后 `LAND`，在目标点下方降落。 |
| 实机建议 | 首次测试可把 `Point_X` 改小，例如 `0.3`。 |

#### move_point_and_return

启动：

```bash
roslaunch examples/sunray_uav_control_example/launch/MOVE_POINT/move_point_and_return.launch \
  use_private_agent_key:=true agent_name:=uav agent_id:=1
```

用途：飞到一个惯性系目标点，然后执行 `RETURN`。

和 `move_point` 的区别：

| 示例 | 任务结束命令 | 结果 |
| --- | --- | --- |
| `move_point` | `LAND` | 在目标点下方降落。 |
| `move_point_and_return` | `RETURN` | 返回 home 点，是否自动降落由配置决定。 |

`RETURN` 行为由 `sunray_uav_control/config/sunray_control_base.yaml` 中 `basic_param.return_with_land` 控制，airframe 配置也可以覆盖。

安全边界：

| 项目 | 当前行为 |
| --- | --- |
| 目标点 | 惯性系 `(1.0, 0.0, 0.6)`。 |
| 飞行范围 | 起飞点前方约 `1m`，高度约 `0.6m`。 |
| 命令频率 | `MOVE_POINT` 和 `RETURN` 都只发布一次。 |
| 结束动作 | 到点后 `RETURN`，是否自动降落由 `return_with_land` 决定。 |
| 实机建议 | 确认 home 点正确后再测试返航。 |

#### move_multipoint

启动：

```bash
roslaunch examples/sunray_uav_control_example/launch/MOVE_POINT/move_multipoint.launch \
  use_private_agent_key:=true agent_name:=uav agent_id:=1
```

用途：依次发送多个惯性系位置点，演示“多个位置目标点如何串起来”。

默认四个点：

```cpp
Eigen::Vector3d Point_1(1, 1, 0.6);
Eigen::Vector3d Point_2(1, -1, 0.6);
Eigen::Vector3d Point_3(-1, -1, 0.6);
Eigen::Vector3d Point_4(-1, 1, 0.6);
```

代码逻辑是每到一个点后，再发下一个 `MOVE_POINT`。位置点命令仍然是“一点发一次”，不需要像速度命令那样一直发布。

安全边界：

| 项目 | 当前行为 |
| --- | --- |
| 目标点 | 惯性系 `(+/-1m, +/-1m, 0.6m)` 矩形四角。 |
| 飞行范围 | 以起飞点附近约 `2m x 2m` 区域为主。 |
| 命令频率 | 每个目标点只发布一次 `MOVE_POINT`。 |
| 结束动作 | 完成四个点后 `RETURN`，是否自动降落由 `return_with_land` 决定。 |
| 实机建议 | 首次测试可把四个点缩小到 `+/-0.3m`。 |

### MOVE_POINT_BODY 示例

#### move_point_body

启动：

```bash
roslaunch examples/sunray_uav_control_example/launch/MOVE_POINT_BODY/move_point_body.launch \
  use_private_agent_key:=true agent_name:=uav agent_id:=1
```

用途：演示机体系位置接口 `MOVE_POINT_BODY`。

默认目标：

```cpp
#define Point_X 1.0
#define Point_Y 0.0
#define Point_Z 0.6
```

理解方式：

| 字段 | 含义 |
| --- | --- |
| `control_cmd = MOVE_POINT_BODY` | 使用机体系位置类接口。 |
| `desired_pos.x` | 机体系 x 方向相对目标，通常可理解为机头前方。 |
| `desired_pos.y` | 机体系 y 方向相对目标。 |
| `desired_pos.z` | 高度或垂向目标，具体以控制器实现为准。 |

这个例程适合学习“相对飞机当前朝向”的位置控制。实机测试前，要确认机体系坐标方向和控制器实现一致。

安全边界：

| 项目 | 当前行为 |
| --- | --- |
| 目标点 | 机体系相对目标 `(1.0, 0.0, 0.6)`。 |
| 飞行范围 | 通常表示朝机头前方约 `1m`，高度约 `0.6m`。 |
| 命令频率 | `MOVE_POINT_BODY` 只发布一次。 |
| 结束动作 | 到点后 `LAND`。 |
| 实机建议 | 先确认机头方向和机体系 x/y 正方向，再缩小目标测试。 |

### MOVE_POINT_WGS84 示例

当前目录下有：

```text
src/MOVE_POINT_WGS84/move_point_wgs84.cpp
launch/MOVE_POINT_WGS84/move_point_wgs84.launch
```

但这两个文件目前都是空文件，`CMakeLists.txt` 里也没有编译 `move_point_wgs84_node`。因此这个例程当前不能运行，不建议新手参考。

如果后续补齐，它应该演示通过经纬高目标发送 `MOVE_POINT_WGS84`，并说明坐标源、原点、高度基准和 GPS 可用性检查。

安全边界：当前示例为空，不会启动节点，也不应作为实机测试入口。

### MOVE_VELOCITY 示例

速度类示例和位置点示例最大的区别是：速度命令需要持续发布。

#### move_velocity

启动：

```bash
roslaunch examples/sunray_uav_control_example/launch/MOVE_VELOCITY/move_velocity.launch \
  use_private_agent_key:=true agent_name:=uav agent_id:=1
```

用途：用一个简单 P 控制器，把当前位置误差转换成惯性系速度命令，依次飞过四个矩形角点。

核心逻辑：

```text
订阅 local_odom
计算 err_pos = desired_pos - current_pos
velocity_cmd.xy = kp * err_pos.xy
限制速度幅值
发布 MOVE_VELOCITY
循环频率约 50Hz
```

当前代码中 `MOVE_VELOCITY` 使用 `desired_vel.x/y + fixed_height` 的混合语义，z 轴目标通过 `fixed_height` 表达。

适合学习：

| 内容 | 位置 |
| --- | --- |
| 如何订阅里程计 | `uav_odom_callback()`。 |
| 如何写一个最简单外环 | `velocity_controller_update()`。 |
| 如何判断到达目标点 | `check_arrived()`。 |
| 如何持续发布速度命令 | 主循环中的 `ros::Rate control_rate(50.0)` 和 `control_rate.sleep()`。 |

安全边界：

| 项目 | 当前行为 |
| --- | --- |
| 目标点 | 惯性系 `(+/-1m, +/-1m, 0.6m)` 矩形四角。 |
| 飞行范围 | 以起飞点附近约 `2m x 2m` 区域为主。 |
| 速度限制 | `max_vel/min_vel = 0.5/-0.5 m/s`。 |
| 依赖输入 | 必须收到有效 `local_odom` 后才开始任务。 |
| 结束动作 | 完成后 `RETURN`，是否自动降落由 `return_with_land` 决定。 |
| 实机建议 | 首次测试先降低 `max_vel` 并缩小四个目标点。 |

#### move_velocity_fixed_height

启动：

```bash
roslaunch examples/sunray_uav_control_example/launch/MOVE_VELOCITY/move_velocity_fixed_height.launch \
  use_private_agent_key:=true agent_name:=uav agent_id:=1
```

用途：更明确地演示 `MOVE_VELOCITY` 的固定高度用法。

关键字段：

```cpp
uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_VELOCITY;
uav_cmd.desired_vel.x = velocity_cmd.x();
uav_cmd.desired_vel.y = velocity_cmd.y();
uav_cmd.desired_vel.z = 0.0;
uav_cmd.fixed_height = fixed_height;
```

如果你想开发“水平速度控制 + 高度保持”的任务节点，优先参考这个例程。

安全边界：

| 项目 | 当前行为 |
| --- | --- |
| 目标点 | 惯性系 `(+/-1m, +/-1m, fixed_height)` 矩形四角。 |
| 固定高度 | `fixed_height = 0.6m`。 |
| 速度限制 | `max_vel/min_vel = 0.5/-0.5 m/s`。 |
| 依赖输入 | 必须收到有效 `local_odom` 后才开始任务。 |
| 结束动作 | 完成后 `RETURN`，是否自动降落由 `return_with_land` 决定。 |
| 实机建议 | 修改高度时优先小幅调整，并确认定位 z 轴正常。 |

#### circle_velocity

启动：

```bash
roslaunch examples/sunray_uav_control_example/launch/MOVE_VELOCITY/circle_velocity.launch \
  use_private_agent_key:=true agent_name:=uav agent_id:=1
```

用途：用速度接口追踪一圈圆形轨迹。

代码做了两件事：

1. 先飞到圆上的起点。
2. 按时间采样圆轨迹，生成期望位置和切向前馈速度，再用 P 项纠偏。

关键参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `kCircleRadius` | `1.0` | 圆半径。 |
| `kCircleAngularSpeed` | `0.2` | 圆轨迹角速度。 |
| `kControlDt` | `0.02` | 发布周期，约 50Hz。 |
| `kp` | `0.3` | 位置误差到速度的比例系数。 |
| `max_vel/min_vel` | `0.5/-0.5` | 速度限幅。 |

安全边界：

| 项目 | 当前行为 |
| --- | --- |
| 轨迹范围 | 以起飞后当前位置为圆心，半径 `1.0m`。 |
| 高度 | 使用进入任务时的当前高度。 |
| 速度规模 | 切向前馈速度约 `0.2m/s`，反馈速度限幅 `0.5m/s`。 |
| 依赖输入 | 必须收到有效 `local_odom` 后才计算圆心。 |
| 结束动作 | 绕圆一圈并回到起点附近后 `RETURN`。 |
| 实机建议 | 首次测试可把 `kCircleRadius` 改成 `0.3`。 |

#### circle_velocity_yaw

启动：

```bash
roslaunch examples/sunray_uav_control_example/launch/MOVE_VELOCITY/circle_velocity_yaw.launch \
  use_private_agent_key:=true agent_name:=uav agent_id:=1
```

用途：在圆形速度轨迹基础上，同时控制 yaw，让机头沿圆轨迹切线方向。

关键字段：

```cpp
uav_cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
uav_cmd.desired_yaw = desired_yaw;
```

适合学习：速度控制任务中如何同时给 yaw 角目标。

安全边界：

| 项目 | 当前行为 |
| --- | --- |
| 轨迹范围 | 以起飞后当前位置为圆心，半径 `1.0m`。 |
| yaw 行为 | 期望机头沿圆轨迹切线方向变化。 |
| 速度规模 | 切向前馈速度约 `0.2m/s`，反馈速度限幅 `0.5m/s`。 |
| 依赖输入 | 必须收到有效 `local_odom` 后才计算圆心和 yaw。 |
| 结束动作 | 绕圆一圈并回到起点附近后 `RETURN`。 |
| 实机建议 | 首次测试先确认 yaw 方向，再缩小半径。 |

#### lemniscate_velocity

启动：

```bash
roslaunch examples/sunray_uav_control_example/launch/MOVE_VELOCITY/lemniscate_velocity.launch \
  use_private_agent_key:=true agent_name:=uav agent_id:=1
```

用途：用速度接口追踪 8 字轨迹。

它和 `circle_velocity` 的结构类似，区别是轨迹采样函数从圆变成双纽线。适合学习如何把任意二维参数曲线转换成：

```text
期望位置 desired_pos
前馈速度 desired_vel_ff
反馈修正 kp * position_error
最终 MOVE_VELOCITY
```

安全边界：

| 项目 | 当前行为 |
| --- | --- |
| 轨迹范围 | 以起飞后当前位置为中心，x 方向峰峰值约 `1m`，y 方向峰峰值约 `2m`。 |
| 高度 | 使用进入任务时的当前高度。 |
| 速度规模 | 沿轨迹期望速度约 `0.2m/s`，反馈速度限幅 `0.5m/s`。 |
| 依赖输入 | 必须收到有效 `local_odom` 后才计算中心点。 |
| 结束动作 | 跑完一圈 8 字并回中心附近后 `RETURN`。 |
| 实机建议 | 首次测试可减小 `kLemniscateXSpan/YSpan`。 |

### MOVE_VELOCITY_BODY 示例

#### move_velocity_body

启动：

```bash
roslaunch examples/sunray_uav_control_example/launch/MOVE_VELOCITY_BODY/move_velocity_body.launch \
  use_private_agent_key:=true agent_name:=uav agent_id:=1
```

用途：演示 `MOVE_VELOCITY_BODY`，即机体系水平速度 + 固定高度。

关键字段：

```cpp
uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_VELOCITY_BODY;
uav_cmd.desired_body_xy_vel.x = body_vx;
uav_cmd.desired_body_xy_vel.y = body_vy;
uav_cmd.fixed_height = fixed_height;
```

新手注意：示例内部的航点仍用惯性系位置表达，代码先根据位置误差算出惯性系速度目标，再根据当前 yaw 转换成机体系 `desired_body_xy_vel`。这正是 `MOVE_VELOCITY_BODY` 和普通 `MOVE_VELOCITY` 的主要区别。

安全边界：

| 项目 | 当前行为 |
| --- | --- |
| 目标点 | 惯性系 `(+/-1m, +/-1m, 0.6m)` 矩形四角。 |
| 输出命令 | 发布机体系水平速度 `desired_body_xy_vel`。 |
| 固定高度 | `fixed_height = 0.6m`。 |
| 速度限制 | `max_vel/min_vel = 0.5/-0.5 m/s`。 |
| 依赖输入 | 必须收到有效 `local_odom`，并依赖 odom 中的 yaw。 |
| 结束动作 | 完成后 `RETURN`。 |
| 实机建议 | yaw 估计不稳定时不要直接测试机体系速度。 |

### MOVE_TRAJECTORY 示例

轨迹类示例发布的是 `MOVE_TRAJECTORY`，每个周期都给出完整参考：

```text
desired_pos
desired_vel
desired_acc
desired_jerk
desired_yaw / desired_yaw_rate
```

这类接口更适合接规划器，比速度接口更适合做平滑轨迹跟踪。

#### move_trajectory

启动：

```bash
roslaunch examples/sunray_uav_control_example/launch/MOVE_TRAJECTORY/move_trajectory.launch \
  use_private_agent_key:=true agent_name:=uav agent_id:=1
```

用途：生成四个矩形角点之间的直线轨迹，并持续发布 `MOVE_TRAJECTORY`。

关键参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `Point_1..Point_4` | 矩形四角 | 目标航点。 |
| `kLineSpeed` | `0.3` | 直线段速度。 |
| `kControlDt` | `0.02` | 轨迹发布周期。 |

适合学习：如何从航点生成连续的 `pos/vel/acc/jerk` 参考。

安全边界：

| 项目 | 当前行为 |
| --- | --- |
| 目标点 | 惯性系 `(+/-1m, +/-1m, 0.6m)` 矩形四角。 |
| 轨迹速度 | 直线段期望速度 `0.3m/s`。 |
| 发布频率 | `kControlDt = 0.02`，约 `50Hz`。 |
| 依赖输入 | 必须收到有效 `local_odom` 后才开始生成轨迹。 |
| 结束动作 | 完成四段轨迹后 `RETURN`。 |
| 实机建议 | 接入真实规划器前，先确认轨迹参考连续且范围足够小。 |

#### circle_trajectory

启动：

```bash
roslaunch examples/sunray_uav_control_example/launch/MOVE_TRAJECTORY/circle_trajectory.launch \
  use_private_agent_key:=true agent_name:=uav agent_id:=1
```

用途：用轨迹接口追踪圆形轨迹。

和 `circle_velocity` 的区别：

| 示例 | 发布内容 | 控制思路 |
| --- | --- | --- |
| `circle_velocity` | 速度命令 | 示例节点自己做 P 外环，输出速度。 |
| `circle_trajectory` | 位置、速度、加速度、jerk | 控制器根据完整轨迹参考做跟踪。 |

如果你已经有规划器，通常更接近 `circle_trajectory` 这种写法。

安全边界：

| 项目 | 当前行为 |
| --- | --- |
| 轨迹范围 | 以起飞后当前位置为圆心，半径 `1.0m`。 |
| 轨迹速度 | 切向速度约 `0.2m/s`。 |
| 发布内容 | 持续发布 `pos/vel/acc/jerk`。 |
| 依赖输入 | 必须收到有效 `local_odom` 后才计算圆心。 |
| 结束动作 | 绕圆一圈并回到起点附近后 `RETURN`。 |
| 实机建议 | 首次测试可把半径减小到 `0.3m`。 |

#### lemniscate_trajectory

启动：

```bash
roslaunch examples/sunray_uav_control_example/launch/MOVE_TRAJECTORY/lemniscate_trajectory.launch \
  use_private_agent_key:=true agent_name:=uav agent_id:=1
```

用途：用轨迹接口追踪 8 字轨迹。

适合学习：如何对复杂参数曲线生成 `desired_pos/vel/acc/jerk`。代码里用相邻采样点近似计算加速度和 jerk，对新手理解比较直观，但正式项目中建议由规划器直接输出连续可导的参考量。

安全边界：

| 项目 | 当前行为 |
| --- | --- |
| 轨迹范围 | 以起飞后当前位置为中心，x 方向峰峰值约 `1m`，y 方向峰峰值约 `2m`。 |
| 轨迹速度 | 沿轨迹期望速度约 `0.2m/s`。 |
| 发布内容 | 持续发布 `pos/vel/acc/jerk`。 |
| 依赖输入 | 必须收到有效 `local_odom` 后才计算中心点。 |
| 结束动作 | 跑完一圈 8 字并回中心附近后 `RETURN`。 |
| 实机建议 | 首次测试先缩小 `kLemniscateXSpan/YSpan`。 |

### 二次开发该从哪个例程改

| 你的需求 | 建议复制 |
| --- | --- |
| 只想起飞、降落、验证链路 | `src/basic/takeoff_land.cpp` |
| 想用 Python 写最小任务节点 | `scripts/takeoff_land.py` |
| 飞到一个固定点 | `src/MOVE_POINT/move_point.cpp` |
| 飞多个航点 | `src/MOVE_POINT/move_multipoint.cpp` |
| 到点后返航 | `src/MOVE_POINT/move_point_and_return.cpp` |
| 想自己写外环，输出速度 | `src/MOVE_VELOCITY/move_velocity.cpp` |
| 想保持固定高度做水平速度控制 | `src/MOVE_VELOCITY/move_velocity_fixed_height.cpp` |
| 想接规划器轨迹 | `src/MOVE_TRAJECTORY/move_trajectory.cpp` |
| 想控制 yaw | `src/MOVE_VELOCITY/circle_velocity_yaw.cpp` |

修改目标点时，优先改源码顶部的 `Point_X/Point_Y/Point_Z` 或 `Point_1..Point_4`。修改轨迹形状时，优先改 `sample_circle_reference()`、`sample_lemniscate_reference()` 或对应的轨迹采样函数。

### 常见问题

#### 一启动示例就报 agent_name 为空

原因通常是 `use_private_agent_key=false`，但全局参数 `/agent_name` 没有设置。

解决方式：

```bash
roslaunch examples/sunray_uav_control_example/launch/basic/takeoff_land.launch \
  use_private_agent_key:=true agent_name:=uav agent_id:=1
```

或者先用 `uav_control.launch` 启动控制器，并保持 `set_global_params:=true`。

#### 程序一直等待 INIT

说明 `sunray_uav_control` 没有进入可接收任务的初始化状态。检查：

```bash
rostopic echo /uav1/sunray/uav_control/control_state
rosnode list | grep uav_control
```

同时确认 MAVROS、定位、配置文件和 airframe 参数正常。

#### 速度或轨迹示例飞一下就不动

速度和轨迹命令必须持续发布。如果你改代码时只发布了一次 `MOVE_VELOCITY` 或 `MOVE_TRAJECTORY`，行为就不对。参考本包速度和轨迹示例中的 `while (ros::ok())` 循环。

速度和轨迹示例在起飞完成后会先等待一帧有效的 `local_odom`，再开始计算圆心、轨迹起点、速度误差或到达判定。如果一直打印 `waiting for valid local odom...`，应先检查定位模块和 `/uav1/sunray/localization/local_odom` 话题。

#### MOVE_POINT_WGS84 为什么不能启动

当前 `MOVE_POINT_WGS84` 示例源码和 launch 都是空文件，CMake 也没有编译目标。它只是目录占位，不是完整例程。

#### 实机测试前要注意什么

先在仿真中确认轨迹方向、速度限幅和高度正确。实机第一次测试时，把目标点、半径、速度和高度调小，并保证遥控器接管、急停、降落流程可用。

</section>
