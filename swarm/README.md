sunray_swarm_control
====================

`swarm/` 是 Sunray 的集群编队控制模块。它面向 `UAV` 和 `UGV` 两类平台，负责接收集群指令、计算期望阵型、调用 ORCA 做多智能体避障，并把结果分别下发给 `uav_control` 和 `ugv_control`。

核心职责
--------
- 接收集群级控制指令：起飞、降落、悬停、返航、切换阵型、保持当前阵型。
- 基于虚拟 leader 和阵型策略，计算每个 agent 的期望目标点。
- 在 agent 之间交换 `/{agent}/orca/setup`，形成分布式编队目标同步。
- 使用内嵌 ORCA 引擎，结合全体里程计状态计算局部避障速度。
- 将 ORCA 输出映射到底层控制消息：
  - `UAV` 下发到 `/{uav}/sunray/uav_control_cmd`
  - `UGV` 下发到 `/{ugv}/sunray/ugv_control/control_cmd`

与其他包的关系
--------------
- `localization_fusion`：为每个 agent 提供 `/{agent}/sunray/localization/local_odom`
- `sunray_uav_control`：执行无人机底层控制
- `sunray_ugv_control`：执行无人车底层控制
- `sunray_msgs`：提供 `UAVSwarmCMD / UGVSwarmCMD / OrcaSetup / OrcaCmd` 等消息定义

运行链路
--------
```text
formation_switch / formation_tui
        |
        v
/sunray/swarm/{uav,ugv}_swarm_cmd
        |
        v
uav_swarm_node / ugv_swarm_node
        |
        +--> FormationPolicy 计算期望阵型目标
        +--> GoalDispatcher 发布 /{agent}/orca/setup
        +--> AgentStateCache 汇总全体 local_odom
        +--> OrcaEngine 计算避障结果
        |
        v
ControlCommandMapper
        |
        +--> /{uav}/sunray/uav_control_cmd
        +--> /{ugv}/sunray/ugv_control/control_cmd
```

目录结构
--------
```text
swarm/
├── ORCA/                  # ORCA / RVO2 避障内核与封装
├── docs/                  # 架构说明与功能梳理文档
├── launch/                # 多机启动入口与调试 launch
├── scripts/               # 联调辅助脚本
├── swarm_control/
│   ├── include/           # 编队策略、状态机、缓存、控制映射接口
│   ├── launch/            # 单 agent 启动片段
│   ├── src/               # UAV / UGV 集群主节点与公共实现
│   └── utils/             # formation_switch / formation_tui 等工具
├── CMakeLists.txt
└── package.xml
```

主要可执行节点
--------------
- `uav_swarm_node`：无人机集群控制主节点，入口文件是 `swarm_control/src/swarm_uav_control_fsm.cpp`
- `ugv_swarm_node`：无人车集群控制主节点，入口文件是 `swarm_control/src/swarm_ugv_control_fsm.cpp`
- `uav_swarm_cmd_pub_terminal`：命令行交互发布 UAV 集群指令
- `ugv_formation_switch`：命令行发布 UGV 集群指令
- `formation_tui`：基于 ncurses 的阵型编辑和发送工具

常用启动文件
------------
- [swarm_sim.launch](/home/amov/Sunray_v2/swarm/launch/swarm_sim.launch)：UAV 集群仿真启动
- [swarm_sim_ugv.launch](/home/amov/Sunray_v2/swarm/launch/swarm_sim_ugv.launch)：UGV 集群仿真启动
- [swarm_control.launch](/home/amov/Sunray_v2/swarm/swarm_control/launch/swarm_control.launch)：单 agent 集群控制节点启动片段
- [uav_swarm_cmd_pub_terminal.launch](/home/amov/Sunray_v2/swarm/launch/uav_swarm_cmd_pub_terminal.launch)：UAV 指令终端发布工具
- [ugv_formation_switch.launch](/home/amov/Sunray_v2/swarm/launch/ugv_formation_switch.launch)：UGV 指令发布工具
- [formation_tui.launch](/home/amov/Sunray_v2/swarm/launch/formation_tui.launch)：阵型 TUI 工具

编译
----
在当前仓库内推荐使用：
```bash
./build.sh -y sunray_swarm_control
```

快速使用
--------
UAV 集群仿真：
```bash
roslaunch sunray_swarm_control swarm_sim.launch agent_num:=6 agent_name:=uav
```

UGV 集群仿真：
```bash
roslaunch sunray_swarm_control swarm_sim_ugv.launch agent_num:=6 agent_name:=ugv
```

启动交互式阵型工具：
```bash
roslaunch sunray_swarm_control formation_tui.launch
```

进一步阅读
----------
- [Swarm_Architecture.md](/home/amov/Sunray_v2/swarm/docs/Swarm_Architecture.md)：详细架构说明
- [Swarm_Function_Overview.md](/home/amov/Sunray_v2/swarm/docs/Swarm_Function_Overview.md)：按目录和文件整理的功能总览
