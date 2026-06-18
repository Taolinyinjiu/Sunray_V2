<!-- title: sunray_navrl_adapter -->

<section id="sunray-navrl-adapter">

## 1. sunray_navrl_adapter

`planning/third_party_planner_examples/navrl_planner_example/sunray_navrl_adapter` 是 NAVRL 接入 Sunray 控制链路的适配包。它不运行 NAVRL 神经网络本体，而是负责启动 NAVRL 所需的占据地图服务，并把 `navigation_runner` 发布的控制话题转换为 Sunray 可消费的控制输入。

### 1.1 适配包功能

核心节点是 `NavRL2Sunray_node`：

```text
navigation_runner/navigation_node.py
  -> /CERLAB/quadcopter/cmd_vel
  -> NavRL2Sunray_node
  -> /uav1/sunray/uav_control/control_cmd
  -> sunray_uav_control
```

如果 NAVRL 发布 `/CERLAB/quadcopter/setpoint_pose`，适配器会转换为 Sunray `MOVE_POINT`。如果 NAVRL 发布 `/CERLAB/quadcopter/cmd_vel`，适配器会转换为 Sunray `MOVE_VELOCITY` 定高速度控制。

单无人车仿真使用 `NavRL2SunrayUGV_node`：

```text
navigation_runner/navigation_node.py
  -> /CERLAB/quadcopter/cmd_vel  世界系速度
  -> NavRL2SunrayUGV_node
  -> /ugv1/sunray/ugv_control/cmd_vel  车体系速度
  -> sunray_sim UgvPlant
```

UGV 版本不启动 `localization_fusion`，也不启动 `sunray_ugv_control`；它直接把 NavRL 输出接到 `sunray_sim` 的麦克纳姆轮模型。由于 `sunray_sim` 的 UGV `cmd_vel` 是车体系速度，适配器会根据 `/ugv1/sunray_sim/odom` 当前 yaw 把 NavRL 世界系速度转换到车体系后再发布。

### 1.2 主要文件

| 文件 | 作用 |
| --- | --- |
| `src/NavRL2Sunray.cpp` | 订阅 NAVRL 速度/位置话题，转换为 Sunray 控制指令，并打印输入输出状态。 |
| `src/NavRL2SunrayUGV.cpp` | 订阅 NAVRL 世界系速度和 UGV odom，转换为 `sunray_sim` UGV 车体系 `cmd_vel`。 |
| `src/NavRLTerminalControl.cpp` | 手动目标点和控制命令终端工具。交互式节点建议手动 `rosrun` 启动，不建议通过 `roslaunch` 脚本管理器启动。 |
| `launch/NavRL2Sunray_sim.launch` | 仿真入口，读取 `sunray_sim` 的 odom 和传感器系局部点云。 |
| `launch/NavRL2SunrayUGV_sim.launch` | 单 UGV 仿真入口，读取 `sunray_sim` UGV odom 和传感器系局部点云，直接发布 UGV `cmd_vel`。 |
| `launch/NavRL2Sunray.launch` | 真机入口，读取 Sunray 定位 odom 和局部点云。 |
| `launch/NavRLTerminalControl.launch` | 终端交互工具的 launch 入口。 |
| `config/sim_config.yaml` | 仿真占据地图参数。 |
| `config/real_config.yaml` | 真机占据地图参数。 |
| `rviz/sim.rviz`、`rviz/real.rviz` | NAVRL 调试 RViz 配置。 |

### 1.3 话题接口

#### 1.3.1 仿真默认话题

| 方向 | 话题 | 消息 | 说明 |
| --- | --- | --- | --- |
| 输入 | `/uav1/sunray_sim/odom` | `nav_msgs/Odometry` | 仿真里程计。 |
| 输入 | `/uav1/sunray_sim/cloud_sensor_frame` | `sensor_msgs/PointCloud2` | 仿真局部点云，供 `map_manager` 建占据地图。 |
| 输入 | `/CERLAB/quadcopter/cmd_vel` | `geometry_msgs/TwistStamped` | NAVRL 输出速度。 |
| 输入 | `/CERLAB/quadcopter/setpoint_pose` | `geometry_msgs/PoseStamped` | NAVRL 输出姿态/定点保持。 |
| 输入 | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz 或终端发布的导航目标点。 |
| 输出 | `/uav1/sunray/uav_control/control_cmd` | `sunray_msgs/UAVControlCMD` | 发给 Sunray 控制器。 |

#### 1.3.2 单 UGV 仿真默认话题

| 方向 | 话题 | 消息 | 说明 |
| --- | --- | --- | --- |
| 输入 | `/ugv1/sunray_sim/odom` | `nav_msgs/Odometry` | sunray_sim UGV 里程计，用于获取当前位置和 yaw。 |
| 输入 | `/ugv1/sunray_sim/cloud_sensor_frame` | `sensor_msgs/PointCloud2` | 仿真局部点云，供 `map_manager` 建占据地图。 |
| 输入 | `/CERLAB/quadcopter/cmd_vel` | `geometry_msgs/TwistStamped` | NAVRL 输出的世界系速度。 |
| 输入 | `/CERLAB/quadcopter/setpoint_pose` | `geometry_msgs/PoseStamped` | NAVRL 输出的目标朝向；UGV 适配器将其转换为原地 yaw 角速度。 |
| 输入 | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz 或终端发布的导航目标点。 |
| 输出 | `/ugv1/sunray/ugv_control/cmd_vel` | `geometry_msgs/Twist` | 发给 `sunray_sim` UGV 麦克纳姆轮模型的车体系速度。 |

#### 1.3.3 真机默认话题

| 方向 | 话题 | 消息 | 说明 |
| --- | --- | --- | --- |
| 输入 | `/uav1/sunray/localization/local_odom` | `nav_msgs/Odometry` | Sunray 定位融合输出。 |
| 输入 | `/cloud_registered_body` | `sensor_msgs/PointCloud2` | Fast-LIO 当前帧机体系点云，供 `map_manager` 建局部占据地图。 |
| 输入 | `/CERLAB/quadcopter/cmd_vel` | `geometry_msgs/TwistStamped` | NAVRL 输出速度。 |
| 输入 | `/CERLAB/quadcopter/setpoint_pose` | `geometry_msgs/PoseStamped` | NAVRL 输出姿态/定点保持。 |
| 输入 | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 目标点。 |
| 输出 | `/uav1/sunray/uav_control/control_cmd` | `sunray_msgs/UAVControlCMD` | 发给 Sunray 控制器。 |

注意：NAVRL 的 `map_manager` 会根据 odom 把输入点云转换到地图坐标系，因此真机应优先使用当前帧局部点云，例如 `/cloud_registered_body`。不要把已经在世界系或全局地图系下的点云直接作为 `point_cloud_topic`，否则可能出现二次变换。

### 1.4 仿真分终端启动

以下流程以单机 `/uav1` 为例。每一步在新的终端中执行，不使用一键启动。

#### 1.4.1 终端1：启动 ROS 环境

如果没有 `roscore`，先启动：

```bash
roscore
```

后续每个终端都先执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/Sunray_v2/devel/setup.bash
```

#### 1.4.2 终端2：启动 sunray_sim

```bash
roslaunch sunray_sim sunray_sim.launch
```

该仿真节点会发布：

```text
/uav1/sunray_sim/odom
/uav1/sunray_sim/cloud_sensor_frame
/uav1/sunray_sim/cloud_world_frame
/uav1/mavros/*
```

#### 1.4.3 终端3：启动 Sunray 控制器

```bash
roslaunch sunray_uav_control uav_control.launch \
  agent_name:=uav \
  agent_id:=1 \
  enable_single_monitor:=true
```

如果只需要先验证 NAVRL 输出转换，也可以暂时不启动控制器，只查看 `/uav1/sunray/uav_control/control_cmd` 是否有消息。

#### 1.4.4 终端4：启动 NAVRL 适配和占据地图

```bash
roslaunch sunray_navrl_adapter NavRL2Sunray_sim.launch \
  agent_name:=uav \
  agent_id:=1 \
  rviz:=true \
  use_safety_shield:=false
```

这个 launch 会启动：

```text
occupancy_map_node
NavRL2Sunray_sim_1
rviz
```

其中 `occupancy_map_node` 读取 `/uav1/sunray_sim/cloud_sensor_frame` 和 `/uav1/sunray_sim/odom`，发布 NAVRL 使用的占据地图和 raycast 服务。

#### 1.4.5 终端5：启动 NAVRL 官方导航节点

NAVRL 官方节点需要 Conda 环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/Sunray_v2/devel/setup.bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate NavRL
rosrun navigation_runner navigation_node.py
```

`navigation_node.py` 会订阅目标点 `/move_base_simple/goal`，调用 `occupancy_map/raycast`，并发布：

```text
/CERLAB/quadcopter/cmd_vel
/CERLAB/quadcopter/setpoint_pose
```

仿真中 `navigation.py` 的非 PX4 模式会同时订阅 `/uav1/sunray_sim/odom`、`/uav1/sunray/localization/local_odom` 和 `/CERLAB/quadcopter/odom`。当前仿真流程主要依赖 `/uav1/sunray_sim/odom`。

#### 1.4.6 终端6：起飞并发送目标点

可以手动启动终端控制工具：

```bash
rosrun sunray_navrl_adapter NavRLTerminalControl_node
```

建议先选择 `2 - 起飞 TAKEOFF`，等待 Sunray 控制器进入 HOVER 后，再选择 `1 - NavRL 目标点` 输入 `x y`。

也可以直接用 `rostopic pub` 发送目标点：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose:
  position:
    x: 5.0
    y: 0.0
    z: 0.0
  orientation:
    w: 1.0"
```

NAVRL 收到目标点后会把目标高度覆盖为起飞/定高高度，因此目标点通常只需要关注 `x` 和 `y`。

如果用 `rostopic pub` 直接发目标点，需要先通过其他工具让无人机完成 TAKEOFF 并进入 HOVER。目标点消息本身不会向 Sunray 控制器发送起飞指令。

### 1.5 真机分终端启动

真机启动顺序建议保持“底层链路先稳定，上层规划后启动”。

#### 1.5.1 终端1：启动 MAVROS

根据实际串口选择：

```bash
roslaunch sunray_mavros mavros.launch \
  fcu_url:=/dev/ttyACM0:921600 \
  gcs_ip:=192.168.2.99:14551 \
  agent_name:=uav \
  agent_id:=1
```

确认：

```bash
rostopic echo -n 1 /uav1/mavros/state
```

#### 1.5.2 终端2：启动定位

以 Fast-LIO + localization_fusion 为例：

```bash
roslaunch localization_fusion localization_fusion.launch \
  source_id:=6 \
  agent_name:=uav \
  agent_id:=1 \
  enable_monitor:=true \
  enable_vision_pose:=true
```

确认 odom 和局部点云：

```bash
rostopic hz /uav1/sunray/localization/local_odom
rostopic hz /cloud_registered_body
```

`/cloud_registered_body` 来自 Fast-LIO 当前帧 body/IMU 系点云。若该话题没有数据，需要检查 Fast-LIO 配置中是否打开 body cloud 发布。

#### 1.5.3 终端3：启动 Sunray 控制器

```bash
roslaunch sunray_uav_control uav_control.launch \
  agent_name:=uav \
  agent_id:=1 \
  enable_single_monitor:=true
```

确认控制器状态稳定后，再启动规划链路。

#### 1.5.4 终端4：启动 NAVRL 适配和占据地图

```bash
roslaunch sunray_navrl_adapter NavRL2Sunray.launch \
  agent_name:=uav \
  agent_id:=1 \
  rviz:=false \
  use_safety_shield:=false
```

如果需要打开 RViz 调试：

```bash
roslaunch sunray_navrl_adapter NavRL2Sunray.launch \
  agent_name:=uav \
  agent_id:=1 \
  rviz:=true
```

#### 1.5.5 终端5：启动 NAVRL 官方导航节点

```bash
source /opt/ros/noetic/setup.bash
source ~/Sunray_v2/devel/setup.bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate NavRL
rosrun navigation_runner navigation_node.py
```

启动后先观察是否持续打印等待 odom、takeoff 完成、收到目标点等信息。

#### 1.5.6 终端6：起飞并发送目标点

手动终端工具：

```bash
rosrun sunray_navrl_adapter NavRLTerminalControl_node
```

真机建议先选择 `2 - 起飞 TAKEOFF`，确认无人机稳定 HOVER 后，再选择 `1 - NavRL 目标点` 输入 `x y`。如果现场已经通过遥控器或其他工具完成起飞并切到 Sunray 控制器 HOVER，也可以只发布目标点。

或直接发布目标点：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose:
  position:
    x: 3.0
    y: 0.0
    z: 0.0
  orientation:
    w: 1.0"
```

真机发送目标点前，应确认 `/uav1/sunray/localization/local_odom`、`/cloud_registered_body`、`/occupancy_map/inflated_voxel_map` 和 `/uav1/sunray/uav_control/control_cmd` 的状态。

如果用 `rostopic pub` 直接发目标点，需要先确认无人机已经处于可接受规划控制指令的 HOVER 状态。目标点消息只触发 NAVRL 规划，不负责解锁、起飞或切换飞控模式。

### 1.6 常用检查命令

检查 NAVRL 算法输出：

```bash
rostopic hz /CERLAB/quadcopter/cmd_vel
rostopic echo -n 1 /CERLAB/quadcopter/cmd_vel
rostopic echo -n 1 /CERLAB/quadcopter/setpoint_pose
```

检查适配器输出：

```bash
rostopic echo -n 1 /uav1/sunray/uav_control/control_cmd
```

检查占据地图：

```bash
rostopic hz /occupancy_map/inflated_voxel_map
rosservice list | grep occupancy_map
```

`/occupancy_map/inflated_voxel_map` 默认只显示局部可视化范围，由 `local_map_size` 控制。无人机走到远处后仍只显示当前位置附近的小范围地图是正常现象；如果需要显示全局占据地图，可调整 `visualize_global_map`，但真机调试时不建议为了可视化牺牲实时性。

### 1.7 常见问题

#### 1.7.1 NavRLTerminalControl 不能通过 roslaunch 交互

`roslaunch` 的 `output="screen"` 只保证输出显示到终端，不保证键盘输入会传给节点。因此 `NavRLTerminalControl_node` 这种需要 `std::cin` 的交互式程序，建议用：

```bash
rosrun sunray_navrl_adapter NavRLTerminalControl_node
```

不要把它放到脚本管理器或普通 `roslaunch` 中作为交互入口。

#### 1.7.2 速度和位置指令为什么都会出现

NAVRL 控制流程中会先发布 `/CERLAB/quadcopter/setpoint_pose` 调整机头朝向目标方向；朝向稳定后发布 `/CERLAB/quadcopter/cmd_vel` 执行速度控制；到目标点附近后再发布 `/CERLAB/quadcopter/setpoint_pose` 做定点保持。适配器会分别转换为 Sunray `MOVE_POINT` 和 `MOVE_VELOCITY`。

#### 1.7.3 真机点云话题如何选择

`map_manager` 需要当前帧局部点云，并会结合 odom 做坐标变换。Fast-LIO 的 `/cloud_registered_body` 是较合适的默认输入。如果使用其他雷达或定位模块，优先选择传感器系/机体系当前帧点云，并同步检查 `real_config.yaml` 中的 `body_to_camera` 外参。

</section>
