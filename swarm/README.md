Swarm (sunray_swarm_control)
====================
面向多机编队控制与 ORCA 避障的 ROS1 包，包含编队策略、状态机、内嵌 ORCA 避障引擎以及交互式编队控制工具。

目录结构
--------
Swarm/
  swarm_control/ # 编队/集群控制主体（内含本地 ORCA 调用）
  ORCA/          # ORCA 纯 C++ 避障内核
  launch/        # 一键启动/仿真启动
  scripts/       # 启动辅助脚本

每个子模块常见目录：
  include/   头文件
  src/       源文件
  config/    参数与YAML配置
  launch/    ROS启动文件
  docs/      说明文档与设计
  tests/     测试

依赖
----
- ROS 1 (catkin)
- roscpp, std_msgs, geometry_msgs, nav_msgs, sensor_msgs
- tf, tf2, tf2_geometry_msgs
- sunray_msgs（自定义消息）

编译
----
在 catkin 工作空间中构建：
```bash
cd ~/catkin_ws
catkin_make --pkg sunray_swarm_control
# 或
catkin build sunray_swarm_control
```

快速运行
--------
多机仿真（示例 6 台）：
```bash
roslaunch sunray_swarm_control swarm_sim.launch agent_num:=6
```

单机集群控制（已内嵌 ORCA，无需再启动 `orca.launch`）：
```bash
roslaunch sunray_swarm_control swarm_control.launch agent_id:=1 agent_num:=6
```

交互式编队控制（TUI）：
```bash
roslaunch sunray_swarm_control formation_tui.launch
```

formation_tui 要点
------------------
- 20x20 网格，中心为 Leader（不可选）；上方为 +x，左侧为 +y
- follower 数量 = agent_num - 1，自定义阵型座位数必须与 follower 数量一致
- 读取 agent_num 顺序：`~agent_num` → `agent_num` → `/agent_num`（默认 1）
  - 仅运行 TUI 时可手动设置：
    ```bash
    rosparam set /agent_num 6
    # 或
    rosrun sunray_swarm_control formation_tui _agent_num:=6
    ```
- 保存/加载自定义阵型：
  - 按 `S`/`L` 后输入路径；直接回车默认 `custom_formation.txt`
  - 默认保存到 **formation_tui 进程的当前工作目录**
    - `roslaunch` 未设置 cwd 时通常是 `~/.ros`
  - 需要固定位置时请输入**绝对路径**

常用话题
--------
- `/sunray/formation_cmd`（`sunray_msgs::Formation`）
- `/sunray/swarm/uav_swarm_cmd` 中 `SWARM_FORMATION + CUSTOM + custom_offsets` 可下发自定义阵型
- `/sunray/swarm/uav_swarm_cmd` 中 `SWARM_FORMATION + KEEP_FORMATION` 可保持当前物理阵型移动虚拟 leader
- `/{agent_name}{id}/orca/setup`（`sunray_msgs::OrcaSetup`，用于各 agent 间共享目标）

更多细节
--------
详见：`swarm_control/docs/Swarm_Architecture.md`
