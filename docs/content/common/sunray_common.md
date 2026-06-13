<!-- title: sunray_common -->

<section id="common-sunray-common">

## sunray_common

`common/sunray_common` 是 Sunray 的公共 C++ 头文件工具包。它不定义 ROS 节点，主要提供多个包都会用到的 agent key、字符串替换、Eigen/ROS 类型转换和常用消息头聚合。

### 目录结构

```text
common/sunray_common/
├── include/
│   ├── WGS84.h
│   ├── agent_key_helper.hpp
│   ├── eigen_helper.hpp
│   ├── ros_msg_utils.h
│   └── string_uav_namespace_utils.hpp
├── CMakeLists.txt
└── package.xml
```

### agent_key_helper.hpp

推荐新代码优先使用这个文件。它解决 Sunray 多智能体命名中最常见的问题：如何统一得到 `/uav1`、`/ugv2` 这类 agent key，以及如何替换配置文件里的 `${agent_key}` 占位符。

主要函数：

| 函数 | 作用 |
| --- | --- |
| `normalize_agent_key(std::string agent_key)` | 规范化 agent key，确保开头有 `/`，结尾没有多余 `/`。 |
| `get_agent_key_from_global()` | 从全局参数 `agent_name`、`agent_id` 读取并组合 agent key。 |
| `get_agent_key_from_private()` | 从私有参数 `~agent_name`、`~agent_id` 读取并组合 agent key。 |
| `replace_agent_key(input_string, agent_key)` | 把字符串中的 `${agent_key}` 替换成实际 agent key。 |

示例：

```cpp
bool use_private_agent_key = false;
private_nh.param("use_private_agent_key", use_private_agent_key, false);

std::string agent_key = use_private_agent_key
    ? sunray_common::get_agent_key_from_private()
    : sunray_common::get_agent_key_from_global();

std::string odom_topic = sunray_common::replace_agent_key(
    "${agent_key}/sunray/localization/local_odom",
    agent_key);
```

常见结果：

```text
agent_name=uav, agent_id=1 -> /uav1
${agent_key}/sunray/localization/local_odom -> /uav1/sunray/localization/local_odom
```

### string_uav_namespace_utils.hpp

这是较早的命名空间工具头，功能和 `agent_key_helper.hpp` 基本重复，也包含：

- `normalize_agent_key`
- `get_agent_key_from_global`
- `get_agent_key_from_private`
- `replace_agent_key`
- `replace_string`

后续新代码建议优先使用 `agent_key_helper.hpp`。这个文件可以视为历史兼容或旧代码参考。若后续整理公共库，可以考虑逐步收敛到一个头文件，避免重复定义带来的维护成本。

### eigen_helper.hpp

`eigen_helper.hpp` 提供 Eigen 与 ROS geometry 消息之间的常用转换，以及姿态角计算。

主要函数：

| 函数 | 作用 |
| --- | --- |
| `get_yaw_from_orientation(Eigen::Quaterniond)` | 从四元数计算 yaw，单位 rad。 |
| `get_euler_from_orientation(Eigen::Quaterniond)` | 从四元数计算 roll/pitch/yaw，单位 rad。 |
| `wrap_angle(double)` | 把角度归一到 `[-pi, pi]` 附近。 |
| `to_ros_point(Eigen::Vector3d)` | 转成 `geometry_msgs::Point`。 |
| `to_ros_vector3(Eigen::Vector3d)` | 转成 `geometry_msgs::Vector3`。 |
| `to_ros_quaternion(Eigen::Quaterniond)` | 转成 `geometry_msgs::Quaternion`。 |
| `rot2Quaternion(Eigen::Matrix3d)` | 旋转矩阵转四元数向量。 |

典型使用场景：

- 控制器从 odom 姿态中提取 yaw。
- 规划器或可视化工具把 Eigen 轨迹点转成 ROS 消息。
- 状态机处理 yaw 误差时用 `wrap_angle()` 防止跨 `pi/-pi` 跳变。

### ros_msg_utils.h

`ros_msg_utils.h` 是一个“常用消息头聚合文件”。它把 Sunray 示例和控制节点常用的大量 ROS 消息头集中 include，方便示例程序少写 include。

包含内容包括：

- `sunray_msgs` 下的控制、定位、规划、集群消息。
- `std_msgs` 常用基础消息。
- `sensor_msgs` 中的电池、测距、IMU、GPS。
- `geometry_msgs` 中的点、位姿、速度、TF。
- `mavros_msgs` 中的状态、setpoint、服务、GPS、RC、参数。
- `nav_msgs/Odometry`、`nav_msgs/Path`。
- `visualization_msgs/Marker`、`MarkerArray`。
- `tf`、`tf2`、`gazebo_msgs/ModelState`。

使用建议：

- 示例程序可以直接 include 它，降低新手上手成本。
- 正式库代码建议只 include 自己真正需要的头文件，减少编译依赖和隐藏耦合。

### WGS84.h

当前文件存在但内容为空。它显然是为 WGS84 经纬高相关工具预留的头文件。

后续如果要完善 WGS84 支持，建议把这些能力放在这里：

- WGS84 经纬高到本地 ENU 的转换。
- 本地 ENU 到 WGS84 经纬高的反算。
- home/origin 管理工具。
- 与 `UAVControlCMD.desired_wgs84_pos`、`UGVControlCMD.desired_wgs84_pos` 的转换示例。

### 二次开发建议

- 需要拼接 `/uav1`、`/ugv1` 这类命名空间时，使用 `agent_key_helper.hpp`。
- 需要写 YAML 或 launch 中可复用的话题名时，推荐使用 `${agent_key}` 占位符，再用 `replace_agent_key()` 替换。
- 需要姿态角和 Eigen/ROS 转换时，使用 `eigen_helper.hpp`。
- 不要在多个包里重复实现 agent key 规则，否则多机、仿真和真机切换时很容易出现话题不一致。

</section>
