<!-- title: ROS -->

<section id="appendix-ros">

## ROS

ROS 是机器人软件开发中常用的通信和工程组织框架。它提供节点、话题、服务、参数、launch 启动文件、消息定义、bag 录制回放等能力，让感知、定位、规划、控制和驱动模块可以用统一方式连接起来。

Sunray_v2 当前按 ROS 1/catkin 工作空间组织，默认环境以 ROS Noetic 为主。普通二次开发者通常不需要修改 ROS 本身，只需要理解 ROS 的运行模型，并掌握查看话题、启动 launch、编写节点和排查环境的基本方法。

在 Sunray 中，可以把 ROS 理解为模块之间的“连接层”：

```text
传感器/飞控/仿真器
  -> ROS driver node
  -> localization / perception / planning node
  -> sunray_uav_control 或 sunray_ugv_control
  -> MAVROS / 底盘驱动 / 仿真接口
```

### ROS 在 Sunray 中的角色

| ROS 概念 | 在 Sunray 中的表现 | 例子 |
| --- | --- | --- |
| Workspace | 一个可编译的 catkin 工程根目录。 | `Sunray_v2/` |
| Package | 一个功能包，包含源码、launch、msg、配置等。 | `sunray_uav_control`、`localization_fusion`、`sunray_mavros` |
| Node | 运行中的进程，负责某个功能。 | 控制节点、定位融合节点、感知节点 |
| Topic | 节点之间持续传输数据的通道。 | `/uav1/sunray/px4_state`、`/uav1/mavros/state` |
| Service | 请求-响应式调用。 | `/uav1/mavros/cmd/arming`、`/uav1/mavros/set_mode` |
| Message | topic/service 的数据类型定义。 | `sunray_msgs/UAVControlCMD`、`nav_msgs/Odometry` |
| Launch | 批量启动节点并设置参数。 | `roslaunch sunray_uav_control uav_control.launch` |
| Parameter | ROS 参数服务器中的配置值。 | `agent_name`、`agent_id`、定位源选择 |
| Bag | ROS 数据录制文件。 | 记录定位、控制和传感器数据用于复盘 |

Sunray 的模块文档中大量命令都依赖这些基础概念。看不懂某个 launch 或 topic 时，先回到 ROS 这一层检查：环境是否 source、package 是否能找到、节点是否启动、topic 是否存在、消息类型是否匹配。

### 环境准备

每个新终端都需要先加载 ROS 和 Sunray 工作空间环境：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

如果系统中设置了 `ROS_DISTRO`，也可以写成：

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source devel/setup.bash
```

常用检查：

```bash
echo $ROS_DISTRO
echo $ROS_PACKAGE_PATH
rospack find sunray_uav_control
```

如果 `rospack find` 找不到 Sunray 包，通常是当前终端没有执行 `source devel/setup.bash`，或者工作空间还没有成功编译。

### 编译工作空间

Sunray_v2 是 catkin workspace，常见编译方式：

```bash
cd ~/Sunray_v2
catkin_make
source devel/setup.bash
```

仓库也提供了封装脚本：

```bash
cd ~/Sunray_v2
./build.sh
source devel/setup.bash
```

编译完成后，`devel/setup.bash` 会把当前工作空间的 package、消息类型、可执行文件和库路径加入终端环境。只编译不 source，`roslaunch` 仍然可能找不到新包或新消息。

### ROS Master

ROS 1 依赖 ROS Master 维护节点注册、topic 发现和 service 发现。通常有两种启动方式：

手动启动：

```bash
roscore
```

通过 launch 自动启动：

```bash
roslaunch sunray_mavros mavros.launch
```

如果当前没有 ROS Master，`roslaunch` 会自动拉起一个。多个终端协同调试时，只要它们使用同一个 `ROS_MASTER_URI`，就能看到同一套节点和话题。

本机调试常见设置：

```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
```

多机通信时，需要把 `ROS_MASTER_URI` 指向主机 IP，并确认所有机器之间网络可达、主机名或 `ROS_IP` 配置正确。多机问题优先用 `ping`、`rostopic list`、`rosnode list` 和 `rostopic echo` 分层检查。

### 常用命令

| 命令 | 作用 |
| --- | --- |
| `roscore` | 启动 ROS Master。 |
| `roslaunch <pkg> <file.launch>` | 启动一个 launch 文件。 |
| `rosrun <pkg> <node>` | 直接运行某个节点。 |
| `rosnode list` | 查看当前节点列表。 |
| `rosnode info <node>` | 查看节点订阅、发布和服务信息。 |
| `rostopic list` | 查看当前 topic 列表。 |
| `rostopic echo <topic>` | 打印 topic 数据。 |
| `rostopic hz <topic>` | 查看 topic 发布频率。 |
| `rostopic info <topic>` | 查看 topic 类型和连接节点。 |
| `rosmsg show <msg>` | 查看消息字段。 |
| `rosservice list` | 查看服务列表。 |
| `rosservice call <service> <args>` | 调用服务。 |
| `rosparam list` | 查看参数列表。 |
| `rosparam get <name>` | 读取参数。 |
| `rosparam set <name> <value>` | 写入参数。 |
| `rosbag record <topic...>` | 录制 topic 数据。 |
| `rosbag play <file.bag>` | 回放 bag 数据。 |
| `rqt_graph` | 图形化查看节点和 topic 连接关系。 |

### Topic 入门

Topic 适合连续数据流，例如定位、速度、图像、点云、状态和控制指令。

查看当前 Sunray 相关 topic：

```bash
rostopic list | grep sunray
rostopic list | grep /uav1
```

查看无人机状态：

```bash
rostopic echo /uav1/sunray/px4_state
```

查看 topic 类型：

```bash
rostopic info /uav1/sunray/px4_state
rosmsg show sunray_msgs/Px4State
```

查看发布频率：

```bash
rostopic hz /uav1/mavros/local_position/odom
```

调试时要同时关注“有没有 topic”和“有没有数据”。`rostopic list` 只能说明 topic 已注册，不能说明发布频率正常，也不能说明数据字段有效。

### Service 入门

Service 适合一次性请求，例如解锁、切模式、读取参数、启动或停止某个功能。

查看 MAVROS 服务：

```bash
rosservice list | grep /uav1/mavros
```

查看服务类型：

```bash
rosservice type /uav1/mavros/cmd/arming
rossrv show mavros_msgs/CommandBool
```

服务调用需要谨慎。真机上不要在不了解状态机和安全条件的情况下直接调用解锁、切模式、起飞或降落服务。Sunray 用户层控制应优先通过 `sunray_msgs/UAVControlCMD` 和项目提供的示例节点完成。

### Launch 入门

`roslaunch` 用于启动一组节点、加载参数并设置命名空间。Sunray 的大多数模块都通过 launch 启动。

基础格式：

```bash
roslaunch <package> <launch_file> arg_name:=value
```

示例：

```bash
roslaunch sunray_mavros mavros.launch agent_name:=uav agent_id:=1
roslaunch localization_fusion localization_fusion.launch agent_name:=uav agent_id:=1 source_id:=3
roslaunch sunray_uav_control uav_control.launch agent_name:=uav agent_id:=1
```

常见 launch 参数：

| 参数 | 含义 |
| --- | --- |
| `agent_name` | 智能体名前缀，例如 `uav` 或 `ugv`。 |
| `agent_id` | 智能体编号，例如 `1`。 |
| `source_id` | 定位源编号，具体含义见定位模块文档。 |
| `fcu_url` | MAVROS 连接 PX4 的串口或 UDP 地址。 |

命名空间通常由 `agent_name` 和 `agent_id` 组成。例如 `agent_name:=uav agent_id:=1` 会生成 `/uav1/...` 这一组 topic 和 service。多机运行时，每架飞机必须使用不同命名空间，并配合不同 PX4 system id、串口或 UDP 端口。

### 编写一个最小 ROS 节点

下面是一个只订阅 Sunray 状态并打印模式的 Python 节点示例。实际工程中建议放在某个 catkin package 的 `scripts/` 目录下，并赋予可执行权限。

```python
#!/usr/bin/env python3
import rospy
from sunray_msgs.msg import Px4State


def state_cb(msg):
    rospy.loginfo_throttle(
        1.0,
        "connected=%s armed=%s flight_mode=%s",
        msg.connected,
        msg.armed,
        msg.flight_mode,
    )


def main():
    rospy.init_node("px4_state_listener")
    rospy.Subscriber("/uav1/sunray/px4_state", Px4State, state_cb, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    main()
```

运行前确认：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosmsg show sunray_msgs/Px4State
```

如果 `rosmsg show` 找不到消息，先重新编译并 source 工作空间。

### rosbag 录制与回放

出现定位跳变、控制异常或感知结果不稳定时，建议录制关键 topic，避免只靠终端截图排查。

录制示例：

```bash
rosbag record -O debug_uav1.bag \
  /uav1/sunray/px4_state \
  /uav1/mavros/local_position/odom \
  /uav1/sunray/uav_control/control_state
```

查看 bag 内容：

```bash
rosbag info debug_uav1.bag
```

回放：

```bash
rosbag play debug_uav1.bag
```

注意：回放 bag 会重新发布里面的 topic。不要在真机控制链路仍然运行时随意回放控制或定位相关 bag，避免旧数据进入实时系统。

### 日常排错路径

| 现象 | 优先检查 |
| --- | --- |
| `roslaunch` 找不到 package | 是否 source 了 `/opt/ros/noetic/setup.bash` 和 `devel/setup.bash`，package 是否编译成功。 |
| 找不到消息类型 | 是否重新编译，是否 source 最新 `devel/setup.bash`。 |
| `rostopic list` 为空 | ROS Master 是否启动，`ROS_MASTER_URI` 是否正确。 |
| topic 存在但没有数据 | 发布节点是否运行，传感器/飞控/仿真器是否正常，是否有命名空间写错。 |
| topic 频率很低或抖动 | 节点负载、网络、串口、传感器驱动、日志输出是否异常。 |
| 多机互相串数据 | `agent_id`、ROS namespace、MAVLink system id、UDP 端口是否冲突。 |
| service 调用失败 | service 是否存在，请求字段格式是否正确，后端节点是否返回错误。 |
| 真机无法进入控制状态 | 先看 Sunray 状态 topic，再看 MAVROS state、定位 odom、setpoint 和 PX4 failsafe。 |

### 对 Sunray 开发者的建议

- 新写任务节点时，优先使用 Sunray 已定义的消息和控制接口，不要直接向 MAVROS raw setpoint 发送命令。
- 先用 `rostopic info` 和 `rosmsg show` 确认数据类型，再写发布或订阅代码。
- 节点参数尽量放到 launch 或 YAML 中，不要把机号、话题名、速度上限等写死在代码里。
- 多机系统中坚持使用命名空间，例如 `/uav1`、`/uav2`，不要把公共 topic 写成无前缀的固定名。
- 真机调试先录 bag，再复现实验；定位、控制、飞控状态至少保留一份同步日志。
- 不确定链路是否正常时，从下到上检查：ROS 环境、节点、topic/service、消息频率、数据字段、Sunray 状态机、PX4/MAVROS 状态。

### 官方资料

- ROS Wiki：http://wiki.ros.org/
- ROS Noetic 安装：http://wiki.ros.org/noetic/Installation/Ubuntu
- ROS Tutorials：http://wiki.ros.org/ROS/Tutorials
- catkin 教程：http://wiki.ros.org/catkin/Tutorials
- roslaunch：http://wiki.ros.org/roslaunch
- rosbag：http://wiki.ros.org/rosbag

</section>
