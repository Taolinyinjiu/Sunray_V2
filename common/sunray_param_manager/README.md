# sunray_param_manager

`sunray_param_manager` 是一个独立的 ROS 参数管理演示包，用来模拟 Sunray 这类大系统中“公共参数 + 模块私有参数 + 运行时调参”的组织方式。

当前包不会改动或依赖其他 Sunray 功能包，只用于单独测试参数管理思路。

## 演示目标

这个包用 A/B/C 三个节点模拟一条简化的数据链路：

```text
A 定位模块
  -> 发布模拟里程计
  -> /{agent_name}{agent_id}/sunray/localization/local_odom

B 规划模块
  -> 订阅定位里程计
  -> 根据目标点和当前位置计算速度命令
  -> /{agent_name}{agent_id}/sunray/planning/mock_velocity_cmd

C 控制模块
  -> 订阅定位里程计和规划速度命令
  -> 根据控制参数计算模拟执行速度
  -> /{agent_name}{agent_id}/sunray/control/mock_state
```

例如 `agent_name=uav`、`agent_id=1` 时，前缀是 `uav1`。如果启动时改成 `agent_id:=2`，所有话题和 frame 都会自动变成 `uav2` 前缀，不在代码里写死 `/uav1`。

## 编译

```bash
cd ~/Sunray_v2
catkin_make --source common --build build/common --pkg sunray_param_manager
source build/common/devel/setup.bash
```

如果你使用的是整个工程统一编译，也可以按自己的工作空间方式 source 对应的 `devel/setup.bash`。

## 启动

默认启动 `uav1`：

```bash
roslaunch sunray_param_manager param_manager_demo.launch
```

指定其他编号：

```bash
roslaunch sunray_param_manager param_manager_demo.launch agent_name:=uav agent_id:=2
```

启动后会有三个节点：

| 节点 | 模拟角色 | 主要输入 | 主要输出 | 参数生效方式 |
| --- | --- | --- | --- | --- |
| `module_a_localization` | 定位模块 | 私有定位参数 | `local_odom` | 启动时读取一次 |
| `module_b_planning` | 规划模块 | `local_odom`、目标点和规划增益 | `mock_velocity_cmd` | 周期刷新 |
| `module_c_control` | 控制模块 | `local_odom`、`mock_velocity_cmd`、控制增益 | `mock_state` | 周期刷新 |

## 共享参数

共享参数由 `config/common_params.yaml` 加载到：

```text
/{agent_name}{agent_id}/sunray
```

例如默认 `uav1`：

```text
/uav1/sunray/agent/name
/uav1/sunray/agent/id
/uav1/sunray/agent/key
/uav1/sunray/frames/local_suffix
/uav1/sunray/frames/base_link_suffix
/uav1/sunray/topics/localization_odom
/uav1/sunray/topics/planning_cmd
/uav1/sunray/topics/control_state
```

这里重点演示一个规则：共享参数里只保存后缀和通用规则，完整前缀由 `agent_name + agent_id` 推导。

例如：

```text
agent_name = uav
agent_id   = 1
agent_key  = uav1
topics/localization_odom = localization/local_odom
```

节点最终发布：

```text
/uav1/sunray/localization/local_odom
```

frame 也是同样规则：

```text
frames/local_suffix = sunray_local
frames/base_link_suffix = base_link
```

节点最终使用：

```text
uav1/sunray_local -> uav1/base_link
```

## 模块私有参数

模块参数分别来自：

```text
config/module_a_params.yaml
config/module_b_params.yaml
config/module_c_params.yaml
```

它们会加载到三个节点自己的私有命名空间。

默认 `uav1` 下的实际路径是：

```text
/uav1/module_a_localization/module_a/publish_rate_hz
/uav1/module_a_localization/module_a/simulated_vx

/uav1/module_b_planning/module_b/target_x
/uav1/module_b_planning/module_b/kp_position
/uav1/module_b_planning/module_b/max_cmd_vel

/uav1/module_c_control/module_c/kp_velocity
/uav1/module_c_control/module_c/max_applied_vel
/uav1/module_c_control/module_c/stop_speed_threshold
```

推荐理解方式：

- 多个模块都要知道的 agent、topic、frame 规则，放共享命名空间。
- 只属于某个模块自己的频率、增益、限幅、阈值，放节点私有命名空间。
- 需要运行时生效的参数，节点必须主动重新读取，参数服务器不会自动推送。

## A 模块：模拟定位

A 模块发布 `nav_msgs/Odometry`：

```text
/uav1/sunray/localization/local_odom
```

它使用这些私有参数：

| 参数 | 作用 | 是否运行时刷新 |
| --- | --- | --- |
| `module_a/publish_rate_hz` | odom 发布频率 | 否 |
| `module_a/start_x` | 初始 x | 否 |
| `module_a/start_y` | 初始 y | 否 |
| `module_a/height` | 模拟飞行高度 | 否 |
| `module_a/simulated_vx` | 模拟 x 方向速度 | 否 |

A 模块故意只在启动时读取一次参数，用来演示传感器外参、初始配置这类参数的常见处理方式。运行中修改下面的参数不会立刻生效：

```bash
rosparam set /uav1/module_a_localization/module_a/simulated_vx 0.2
```

需要重启 `module_a_localization` 或重新 launch 才会使用新值。

## B 模块：模拟规划

B 模块订阅：

```text
/uav1/sunray/localization/local_odom
```

发布：

```text
/uav1/sunray/planning/mock_velocity_cmd
```

它会读取当前 odom 与目标点之间的位置误差，然后用一个简单比例控制生成速度命令：

```text
cmd_vel = clamp(kp_position * position_error, max_cmd_vel)
```

常用运行时调参：

```bash
rosparam set /uav1/module_b_planning/module_b/target_x 5.0
rosparam set /uav1/module_b_planning/module_b/max_cmd_vel 0.3
rosparam set /uav1/module_b_planning/module_b/kp_position 1.5
```

B 模块默认会周期重新读取这些参数，因此不需要重启就能看到规划速度变化。

## C 模块：模拟控制

C 模块订阅：

```text
/uav1/sunray/localization/local_odom
/uav1/sunray/planning/mock_velocity_cmd
```

发布：

```text
/uav1/sunray/control/mock_state
```

它会用控制参数处理规划速度，并输出一个字符串状态，便于直接用 `rostopic echo` 查看：

```bash
rostopic echo /uav1/sunray/control/mock_state
```

常用运行时调参：

```bash
rosparam set /uav1/module_c_control/module_c/kp_velocity 2.0
rosparam set /uav1/module_c_control/module_c/max_applied_vel 0.5
rosparam set /uav1/module_c_control/module_c/stop_speed_threshold 0.1
```

C 模块默认会周期重新读取这些参数，因此可以模拟控制参数在线调整。

## 查看话题

```bash
rostopic list | grep sunray
rostopic echo /uav1/sunray/localization/local_odom
rostopic echo /uav1/sunray/planning/mock_velocity_cmd
rostopic echo /uav1/sunray/control/mock_state
```

如果启动的是 `agent_id:=2`，对应话题应改成：

```text
/uav2/sunray/localization/local_odom
/uav2/sunray/planning/mock_velocity_cmd
/uav2/sunray/control/mock_state
```

## 学习重点

1. `ros::NodeHandle nh;` 适合读公共命名空间或发布/订阅公共话题。
2. `ros::NodeHandle pnh("~");` 适合读节点私有参数。
3. `agent_name + agent_id` 应该作为 agent 前缀的唯一来源，避免在代码中写死 `/uav1`。
4. 参数服务器只负责保存参数，不会自动通知节点变量变化。
5. 静态参数适合启动时读取一次，动态参数需要节点周期读取、service 更新，或者使用 `dynamic_reconfigure`。
6. 大系统里建议把“公共命名规则”和“模块控制参数”分开管理，否则多机、多型号、多仿真环境会很容易混乱。
