# sunray_sim 轻量仿真 TODO

本文档用于整理 `sunray_sim` 后续修改方向。目标不是复刻 Gazebo 的全部物理和传感器细节，而是做一个更轻、更容易启动、更适合规划/控制闭环调试的 MAVROS 风格仿真器。

## 本轮替换 pengyu_sim 后待确认

- [ ] 机型 overlay 命名：`uav_control/sunray_uav_control/config/airframes/pengyu_sim.yaml` 和 `ugv_control/sunray_ugv_control/config/airframes/pengyu_sim.yaml` 仍保留旧文件名。启动器已不再引用它们，是否删除或重命名需要确认。
- [ ] 第三方规划器旧入口命名：`planning/third_party_planner_examples/*/run_pengyusim_lio.launch` 仍保留历史文件名和注释。本轮没有修改这些入口，避免影响外部规划器兼容性；后续可确认是否新增/改名为 `run_sunray_sim_lio.launch`。

## 总体目标

- [ ] 明确本包定位：提供全局点云、局部 MID360 点云、四旋翼 odom/imu/navsat、fake MAVROS 接口，用于替代 Gazebo 做算法联调。
- [ ] 保持启动方式简单：一个 launch、一个 node，内部按模块组合。
- [ ] 保持运行轻量：默认 CPU 可跑，不依赖 Gazebo、PX4 SITL、GPU。
- [ ] 保持接口稳定：上层控制、规划、定位模块尽量通过 MAVROS 风格话题/服务接入。
- [ ] 保持物理模型“够用且可解释”：动力学、控制转换、传感器仿真都要明确简化边界。

## P0：先确认的架构问题

### 1. 明确模块边界

- 当前状态：
  - `sunray_sim_node.cpp`：唯一入口，负责读取通用参数、创建地图，并按 `agent_ids` 创建一架或多架无人机仿真。
  - `SingleUavSimulator`：持有单架无人机的局部感知、动力学、PX4 控制转换、fake MAVROS。
  - `GlobalMapServer`：读取 PCD，发布全局点云，并给局部雷达提供内存点云。
  - `LocalMid360Simulator`：用全局点云和 odom 生成局部点云。
  - `QuadrotorSimulator` + `QuadrotorDynamics`：动力学积分、odom/imu/navsat 发布。
  - `SimVisualizer`：订阅仿真状态并发布 RViz MarkerArray。
  - `Px4ControlSim`：把 MAVROS setpoint 转成电机 RPM。
  - `FakeMavrosBridge`：发布假的 MAVROS 状态、odom、imu 和常用服务。
- 问题：
  - 模块边界已经基本清楚，但参数仍有跨模块读取，例如 `Px4ControlSim` 读取 `dynamics/*` 和 `motor/*`。
  - `SingleUavSimulator` 当前是单架无人机组合器，主节点通过循环创建多个实例支持集群。
- 建议：
  - 保留当前“一个 node + 多个类”的结构。
  - 明确 `SingleUavSimulator` 是单机组合层，不承担具体算法逻辑。
  - 新增统一的 `VehicleParams` 或 `AirframeParams` 读取结果，避免动力学和控制转换分别从 ROS 参数读同一批机体参数。
- 收益：
  - 集群支持可以复用单机组合层。
  - 避免参数重复读取导致控制器和动力学不一致。

### 3. 确认感知链路开关策略

- 当前状态：
  - `enable_sensing=false` 时不会启动 `GlobalMapServer` 和 `LocalMid360Simulator`。
- 问题：
  - 现在 `local_mid360_simulator.yaml` 和 `global_map_server.yaml` 仍会被 launch 加载，只是代码不创建模块。
- 建议：
  - 保持当前写法，原因是 launch 简单统一。
  - 后续可以在状态面板中明确显示“感知模块未启用”。
- 收益：
  - 不引入复杂 launch 条件。
  - 用户能清楚知道当前是否有全局/局部点云输出。

## P0：动力学和控制闭环

### 7. 机体参数读取改成单一来源

- 当前状态：
  - `QuadrotorSimulator` 和 `Px4ControlSim` 都读取 `dynamics/*`、`motor/*`。
- 问题：
  - 虽然来自同一 YAML，但代码上是两套读取逻辑，后续新增参数时容易漏同步。
- 建议：
  - 新增 `airframe_params.h/.cpp`：
    - `DynamicParams loadDynamicParams(nh)`
    - `MotorParams loadMotorParams(nh)`
  - `QuadrotorSimulator` 和 `Px4ControlSim` 共用这套读取函数。
- 收益：
  - 参数一致性更强。
  - 后续支持不同机型更方便。

## P1：传感器与地图

### 9. 局部点云渲染性能优化

- 当前状态：
  - 每次渲染使用 KD-tree 半径搜索，然后按角度栅格保留最近点。
- 问题：
  - 大地图、高频率、小角分辨率时 CPU 压力会上升。
- 建议：
  - 增加最大参与渲染点数或二次体素降采样参数。
  - 可选：缓存局部子地图，odom 位移/转角超过阈值才重新搜索。
- 收益：
  - 更容易判断 CPU 是否跑满。
  - 保持轻量仿真的实时性。

### 10. 全局地图加载增强

- 当前状态：
  - `GlobalMapServer` 读取 `PointXYZ` PCD，再转成 `PointXYZI`。
- 问题：
  - 如果 PCD 本身有 intensity，会被丢弃。
  - 地图坐标变换只支持平移，不支持 yaw/roll/pitch。
- 建议：
  - 支持 `PointXYZI` PCD，保留原始 intensity。
  - 增加地图 yaw 旋转或完整 `map_pose` 参数。
  - 地图加载失败时状态面板给出更明显提示。
- 收益：
  - 更容易复用真实地图。
  - 减少外部预处理地图的需求。

## P1：ROS 接口与 MAVROS 兼容性

### 14. MAVROS 服务行为更接近真实流程

- 当前状态：
  - fake 服务基本都返回成功。
- 问题：
  - 上层状态机可能依赖 arming/mode 切换结果，但当前没有失败条件。
- 建议：
  - 增加参数控制是否严格模拟：
    - `strict_mavros_services=false`：当前行为。
    - `true`：未 connected、非法 mode、未收到 setpoint 时可返回失败。
- 收益：
  - 默认方便调试，高级模式可测试状态机健壮性。

## P1：配置与启动

### 15. 配置文件继续按模块收敛

- 当前状态：
  - 配置已经拆成 node、single_uav、global_map、local_mid360、quadrotor、imu、px4、fake_mavros。
- 问题：
  - 共享参数仍散落在多个文件描述中。
- 建议：
  - 新增 `airframe.yaml` 或保留 `quadrotor_simulator.yaml` 但明确它是机体参数源。
  - `Px4ControlSim` 不再直接“概念上拥有”机体参数，只引用同一份 airframe 参数。
- 收益：
  - 用户调参入口更清晰。

### 16. launch 支持快速覆盖常用参数

- 当前状态：
  - launch 主要通过 YAML 加载配置。
- 问题：
  - 临时改地图、初始位置、感知开关时需要改 YAML 或传 config。
- 建议：
  - launch 保留 YAML 为主。
  - 可增加少量常用 arg 覆盖：
    - `enable_sensing`
    - `map_name`
    - `agent_name`
    - `agent_id`
  - 注意不要把所有参数都变成 launch arg。
- 收益：
  - 日常测试更方便。

### 17. package 元信息补齐

- 当前状态：
  - `package.xml` 中 `license` 仍是 `TODO`，maintainer 邮箱也是占位。
- 问题：
  - 包元信息不完整。
- 建议：
  - 确认许可证和维护者信息后补齐。
- 收益：
  - 包更规范，后续发布/移植更少问题。

## P2：状态面板与调试能力

### 18. 统一状态面板格式

- 当前状态：
  - 各模块都有彩色 `printStatus()`，但标题、字段命名和布局仍不完全统一。
- 问题：
  - 状态面板信息多时不够紧凑。
- 建议：
  - 定义统一格式：
    - 基本状态
    - 参数摘要
    - 订阅话题
    - 发布话题
    - 最近告警
  - `enable_sensing=false` 时打印一行“感知链路关闭”。
- 收益：
  - 一眼能看出模块是否正常。

### 19. 增加运行统计

- 当前状态：
  - 状态面板主要显示参数和话题。
- 问题：
  - 无法判断实时性，比如点云渲染耗时、动力学更新耗时、控制更新频率是否稳定。
- 建议：
  - 增加轻量统计：
    - 动力学实际更新频率
    - PX4 控制实际更新频率
    - 局部点云渲染耗时
    - 最近一次 setpoint/odom/imu 的延迟
- 收益：
  - 方便判断是否适合作为 Gazebo 替代。

## P2：测试与验证

### 21. 增加参数合法性检查

- 当前状态：
  - 多数参数只做简单 `max()` 或 clamp。
- 问题：
  - 非法参数可能导致仿真行为奇怪但不报错。
- 建议：
  - 对关键参数做启动时检查：
    - 质量、惯量、电机系数必须大于 0。
    - `rpm_max > rpm_min`。
    - 地图文件必须存在。
    - sensing 频率和分辨率不能导致过大的 range image。
- 收益：
  - 减少无效调参时间。

### 22. 建立和 Gazebo/SITL 的对比基准

- 当前状态：
  - 目标是替代 Gazebo，但没有明确“替代到什么程度”。
- 问题：
  - 如果没有基准，后续很难判断模型是否够用。
- 建议：
  - 选 3 个场景对比：
    - 悬停。
    - 阶跃位置指令。
    - 固定速度穿越点云地图。
  - 记录轨迹、速度、姿态、控制输入、CPU 占用。
- 收益：
  - 明确轻量仿真的可信边界。

## P2：代码质量

### 24. 降低头文件耦合

- 当前状态：
  - `single_uav_simulator.h` 直接 include 了多个模块头文件。
- 问题：
  - 编译依赖偏重，改一个模块头文件会触发更多重编译。
- 建议：
  - 使用前向声明，把具体 include 放到 `.cpp`。
  - PCL 点云类型可保留必要声明或集中到公共类型头中。
- 收益：
  - 编译更快，模块边界更清楚。

### 25. 提取公共工具函数

- 当前状态：
  - 多处重复实现颜色常量、角度转换、clamp、四元数/欧拉角转换。
- 问题：
  - 后续维护容易出现不一致。
- 建议：
  - 新增：
    - `status_utils.h`
    - `math_utils.h`
    - `topic_utils.h`
  - 只提取稳定且重复的工具，不做过度封装。
- 收益：
  - 减少重复代码。

### 26. 明确 C++ 标准和 catkin 依赖

- 当前状态：
  - CMake 使用 C++14，`package.xml` 依赖基本齐全。
- 问题：
  - `CMAKE_BUILD_TYPE` 在包内强设为 Release，可能影响整个 workspace 调试体验。
- 建议：
  - 确认是否保留强制 Release。
  - 如果需要调试，改为只在未设置时默认 Release。
- 收益：
  - 调试和发布构建更灵活。

## P3：可选增强

### 27. 增加风场/扰动模型

- 建议：
  - 增加可选的常值风、随机风、外力扰动。
  - 默认关闭。
- 收益：
  - 可测试控制器鲁棒性。

### 28. 增加碰撞/地面约束

- 当前状态：
  - z 小于 0 时被限制到地面。
- 建议：
  - 可选检测与全局点云的简单碰撞。
  - 默认关闭，避免影响性能。
- 收益：
  - 用于规划失败或撞障碍物时的基础反馈。

### 29. 增加多地图/场景配置

- 建议：
  - 提供几套场景 YAML：
    - small_forest
    - yundrone
    - library
    - no_sensing
  - launch 通过 `scene_config` 切换。
- 收益：
  - 用户不用手动改多个 YAML。

## 建议确认顺序

1. 先确认 P0：
   - 是否保留单机为主。
   - 是否做机体参数单一来源。
2. 再处理 P1：
   - 参数和 launch 体验。
3. 最后处理 P2/P3：
   - 状态统计。
   - 自动测试。
   - 风场、碰撞、多场景、RViz 配置。
