# Sunray_v2 项目阅读报告（重点：规划模块）

## 1. 结论

我已经完成了两层阅读：

1. **项目整体结构阅读**：明确了 `Sunray_v2` 是一个基于 **ROS1 / catkin** 的无人机/无人车平台代码仓，按功能域拆分为控制、定位、规划、仿真、驱动、通信、集群等模块。
2. **规划模块重点阅读**：明确了 `planning/` 不是单一算法，而是一个**集成式规划子仓**，内部包含多套上游规划器源码、Sunray 适配层、消息定义和桥接工具。

后续如果我们在对话里说“**规划模块**”，我将默认它指代整个 `planning/` 目录及其内部子系统。

---

## 2. 项目整体结构

仓库根目录是一个典型的 catkin 工作区，能看到：

- `.catkin_workspace`
- `build/`
- `devel/`
- 多个 ROS package 的 `CMakeLists.txt` 与 `package.xml`

这说明当前仓库不是“单包项目”，而是**多包单仓（monorepo）**形式。

### 2.1 顶层目录职责概览

| 目录 | 作用 |
|---|---|
| `common/` | 公共消息、公共库、桥接与日志 |
| `control/` | 无人机 / 无人车控制相关 |
| `localization/` | 定位、里程计、建图输入来源 |
| `drivers/` | 雷达、相机、MAVROS、云台等驱动 |
| `simulation/` | 仿真器、Gazebo 插件、比赛仿真场景 |
| `swarm/` | 集群控制与避障 |
| `communication/` | 通信模块 |
| `planning/` | 规划算法与规划适配层 |
| `examples/` | 使用示例 |
| `scripts/` | 启动/辅助脚本 |
| `tools/` | 构建与辅助工具 |
| `test/` | 测试或实验工具 |

### 2.2 从 README 得到的整体定位

根目录 `README.md` 表明：

- 项目名称是 **Sunray - 开源无人机仿真与实践平台**
- 面向对象是 **无人机智能化/自主化飞行**
- 同时覆盖：
  - 真机控制
  - RViz / Gazebo 仿真
  - 多种上游开源算法接入
  - 二次开发接口

因此它更像一个**平台型集成工程**，而不是只提供某一个算法实现。

---

## 3. 规划模块总览

`planning/` 当前可以分成三层：

```text
planning/
├── adaptor/            # 各规划器到 Sunray 的适配层、消息层、点云重映射
├── planner_adaptor/    # SUPER 专用适配器
└── source/             # 多套上游规划器源码
```

### 3.1 规划模块内的三大部分

#### A. `planning/source/`
这是**规划算法源码主体**。不是一套代码，而是直接并列集成了多套不同来源的规划器。

#### B. `planning/adaptor/`
这是**Sunray 接入层**。核心任务是把上游规划器自己的输出消息，转换成 Sunray 控制模块能直接消费的消息格式。

#### C. `planning/planner_adaptor/`
这是一个额外的**统一/专用适配层**，目前主要用于 **SUPER** 的命令输出转换。

### 3.2 规模感知

我核实到：

- `planning/` 下有 **51 个** `package.xml`
- 说明它本身就是一个相当大的 ROS 多包系统
- `planning/source/` 里主要包含 **4 套核心规划体系**

这意味着后续做任何修改时，必须先分清你改的是：

1. 上游规划器本体
2. Sunray 适配层
3. 仿真/地图/消息层
4. 启动与集成层

否则很容易改错位置。

---

## 4. 规划模块内部的 4 套核心规划体系

`planning/source/` 下目前主要包含：

1. `ego_planner_swarm/`
2. `diff_planner/`
3. `FUEL/`
4. `SUPER/`

它们不是同一套算法的不同版本，而是**设计目标不同、消息体系不同、状态机不同、地图表达不同**的四个独立系统。

---

## 5. `ego_planner_swarm/` 阅读结果

## 5.1 定位

这是对 **EGO-Planner / EGO-Planner-Swarm** 的集成，偏向：

- 局部重规划
- B-spline 轨迹优化
- 多机/集群轨迹交换

## 5.2 目录结构

核心子包包括：

- `bspline_opt/`：B-spline 优化器
- `plan_manage/`：状态机、规划主流程、轨迹服务器
- `plan_env/`：地图/光线投射/环境表示
- `path_searching/`：动态 A* 搜索
- `traj_utils/`：轨迹消息、可视化、容器
- `drone_detect/`：检测相关
- `rosmsg_tcp_bridge/`：ROS 消息桥接

## 5.3 关键入口与控制核心

我重点看了：

- `planning/source/ego_planner_swarm/plan_manage/include/plan_manage/ego_replan_fsm.h`

从这个头文件可以确认它的控制核心是一个 FSM：

### FSM 状态

- `INIT`
- `WAIT_TARGET`
- `GEN_NEW_TRAJ`
- `REPLAN_TRAJ`
- `EXEC_TRAJ`
- `EMERGENCY_STOP`
- `SEQUENTIAL_START`

### FSM 关键职责

- 接收 odom / target / 触发消息
- 生成新轨迹
- 在执行中重规划
- 遇到风险时紧急停车
- 发布 swarm 轨迹给其他无人机

### 关键成员说明

- `EGOPlannerManager::Ptr planner_manager_`
- `PlanningVisualization::Ptr visualization_`
- `traj_utils::MultiBsplines multi_bspline_msgs_buf_`

这说明它的主干是：

**FSM -> PlannerManager -> B-spline Trajectory -> 可视化/广播/执行**

## 5.4 我对它在 Sunray 中角色的判断

`ego_planner_swarm` 在这个仓库里是一个**相对完整、可以独立运行的局部规划器实现**，Sunray 对它主要做了两类接入：

1. 消息定义接入：`ego_planner_msgs/`
2. 控制命令接入：`ego_planner_adaptor/`

也就是说，Sunray 不是重写了 EGO-Planner，而是把它“嵌进来并接到自己的控制总线上”。

---

## 6. `diff_planner/` 阅读结果

## 6.1 定位

这是另一套局部规划器，与 EGO 路线不同，它更偏向：

- 多项式 / MINCO / differential flatness 轨迹表示
- 动态 A* 前端
- 多项式轨迹优化
- 多机桥接能力

## 6.2 目录结构

核心子包包括：

- `plan_manage/`
- `plan_env/`
- `path_searching/`
- `traj_opt/`
- `traj_utils/`
- `swarm_bridge/`
- `drone_detect/`

## 6.3 关键控制核心

我重点看了：

- `planning/source/diff_planner/plan_manage/include/plan_manage/diff_replan_fsm.h`

它的 FSM 状态几乎与 EGO 体系平行：

- `INIT`
- `WAIT_TARGET`
- `GEN_NEW_TRAJ`
- `REPLAN_TRAJ`
- `EXEC_TRAJ`
- `EMERGENCY_STOP`
- `SEQUENTIAL_START`

但它的内部对象与轨迹类型不同。

### 关键对象

- `DiffPlannerManager::Ptr planner_manager_`
- `traj_utils::PolyTraj`
- `traj_utils::MINCOTraj`

说明它的轨迹主表示不是 B-spline，而是 **多项式 / MINCO**。

## 6.4 额外特征

这个 FSM 里还看到一些更工程化的项：

- stuck detection（卡死检测）
- ground height measurement（地面高度测量）
- mandatory stop（强制停止）
- heartbeat / ground height publisher

说明它相比 EGO 版本加入了更多工程安全措施和运行时诊断逻辑。

## 6.5 我对它在 Sunray 中角色的判断

`diff_planner` 也是被当作一套**完整规划器内核**接入进来的，Sunray 对它同样主要负责：

- 自定义消息层 `diff_planner_msgs/`
- 输出命令转换 `diff_planner_adaptor/`

因此它和 EGO 在项目里的地位是平行的，区别主要在**轨迹表示方式**和**内部规划实现**。

---

## 7. `FUEL/` 阅读结果

## 7.1 定位

FUEL 不是普通的点到点局部避障器，它更偏向：

- **自主探索（exploration）**
- Frontier-based information structure
- 分层规划（前沿发现 -> 视角优化 -> 轨迹生成）

从 `planning/source/FUEL/README.md` 可以确认：

- 它的核心目标是 **Fast UAV Exploration**
- 论文重点是 **incremental frontier structure + hierarchical planning**
- 项目自带独立仿真器与 demo

## 7.2 目录结构

FUEL 的体量很大，分成两层：

### A. `fuel_planner/`
包含真正的探索规划与运动规划：

- `exploration_manager/`
- `active_perception/`
- `plan_manage/`
- `plan_env/`
- `path_searching/`
- `bspline/`
- `bspline_opt/`
- `poly_traj/`
- `traj_utils/`
- `utils/lkh_tsp_solver/`

### B. `uav_simulator/`
包含它自己的仿真栈：

- `local_sensing/`
- `map_generator/`
- `so3_control/`
- `so3_quadrotor_simulator/`
- `poscmd_2_odom/`
- `Utils/*`

## 7.3 关键控制核心

我重点看了：

- `planning/source/FUEL/fuel_planner/exploration_manager/include/exploration_manager/fast_exploration_fsm.h`

它的 FSM 状态是：

- `INIT`
- `WAIT_TRIGGER`
- `PLAN_TRAJ`
- `PUB_TRAJ`
- `EXEC_TRAJ`
- `FINISH`

这和 EGO / Diff 的“持续重规划局部跟踪 FSM”不一样。

这里的工作模式更像：

**收到探索触发 -> 规划探索轨迹 -> 发布 -> 执行 -> 继续下一轮探索 / 结束**

因此 FUEL 在本项目里更适合被理解为：

> 一套“自主探索任务系统”，而不仅仅是一个普通 local planner。

## 7.4 我对它在 Sunray 中角色的判断

FUEL 更像是一个**相对独立的研究型探索系统**，在当前仓库里的集成深度，明显不像 EGO / Diff 那样已经针对 Sunray 控制链做了专门适配。

也就是说：

- 它被纳入了规划模块
- 但它更偏“上游原始工程 + 自带仿真链”
- 当前没有看到像 `ego_planner_adaptor` / `diff_planner_adaptor` 那样明显的 Sunray 专属输出桥接层

如果后面要把 FUEL 真正接到 Sunray 控制链，预计需要额外补一层适配。

---

## 8. `SUPER/` 阅读结果

## 8.1 定位

从 `planning/source/SUPER/README.md` 可以确认：

- 这是 HKU MaRS 的 **SUPER: Safety-assured High-speed Navigation for MAVs**
- 目标是 **高速度、安全保证、复杂环境自主导航**
- 支持 ROS1 / ROS2 双版本
- 仓库里同时带：
  - planner
  - map
  - mission planner
  - simulator

## 8.2 目录结构

### A. `super_planner/`
核心规划系统：

- FSM
- SuperPlanner 主类
- corridor generation
- A* path search
- exp / backup trajectory optimization
- yaw optimization
- logging / tuning tools

### B. `rog_map/`
这是 SUPER 里的地图系统：

- occupancy / prob map
- ESDF / inf map
- sliding map
- raycaster
- ROS1 / ROS2 封装

### C. `mission_planner/`
偏上层任务与 demo 启动：

- waypoint mission
- benchmark launch
- click demo

### D. `mars_uav_sim/`
它自己的仿真器体系：

- `perfect_drone_sim/`
- `marsim_render/`
- `mars_quadrotor_msgs/`

## 8.3 关键控制核心

我重点看了：

- `planning/source/SUPER/super_planner/include/fsm/fsm.h`

它的执行状态为：

- `INIT`
- `WAIT_GOAL`
- `YAWING`
- `GENERATE_TRAJ`
- `FOLLOW_TRAJ`
- `EMER_STOP`

这个 FSM 的结构比 EGO / Diff 更偏“系统化飞行执行器”。

### 关键对象

- `super_planner::SuperPlanner::Ptr planner_ptr_`
- `ros_interface::RosInterface::Ptr ros_ptr_`
- `rog_map::RobotState robot_state_`

这说明 SUPER 是：

**FSM + Planner Core + ROS Interface + ROG-Map**

一个强耦合但完整度很高的导航系统。

## 8.4 我对它在 Sunray 中角色的判断

SUPER 在 `planning/` 内不是一个简单算法子包，而是一个**近乎完整的独立导航栈**。

当前仓库中对 SUPER 的 Sunray 接入方式是：

- 保留其原始 `quadrotor_msgs::PositionCommand` 输出风格
- 再通过 `planning/planner_adaptor/super_adaptor.cpp` 做转接

这与 EGO / Diff 的“分散在 adaptor/ 目录下的适配方式”略有不同。

---

## 9. 适配层（Adaptor）阅读结果

这是当前规划模块里最关键的 Sunray 集成部分。

---

## 9.1 `ego_planner_adaptor/`

关键文件：

- `planning/adaptor/ego_planner_adaptor/src/positionCmd_to_sunray.cpp`

### 功能

将：

- `ego_planner_msgs::PositionCommand`

转换为：

- `sunray_msgs::UAVControlCMD`

### 转换内容

它映射了：

- position -> `desired_pos`
- velocity -> `desired_vel`
- acceleration -> `desired_acc`
- yaw -> `desired_yaw`
- yaw_dot -> `desired_yaw_rate`

并根据 `yaw_dot` 是否接近零选择：

- `SET_YAW`
- `SET_YAWRATE`

同时会检查 `trajectory_flag`，若轨迹非法则直接丢弃，不下发控制指令。

### 接入方式特征

它通过 ROS 参数读取：

- `cmd_sub_topic`
- `control_pub_topic`

这说明它是**可配置的适配节点**，便于接到不同无人机实例的话题上。

---

## 9.2 `diff_planner_adaptor/`

关键文件：

- `planning/adaptor/diff_planner_adaptor/src/positionCmd_to_sunray.cpp`

### 功能

将：

- `sunray_planner_msgs::DiffPositionCommand`

转换为：

- `sunray_msgs::UAVControlCMD`

### 与 EGO 适配器的区别

Diff 版本额外映射了：

- `jerk -> desired_jerk`

说明 Diff 体系的输出消息比 EGO 更完整一些。

### 接入方式特征

同样支持 ROS 参数化配置：

- `cmd_sub_topic`
- `control_pub_topic`

因此这两个适配器在工程风格上是统一的。

---

## 9.3 `planner_adaptor/super_adaptor.cpp`

关键文件：

- `planning/planner_adaptor/src/super_adaptor.cpp`

### 功能

将：

- `quadrotor_msgs::PositionCommand`

转换为：

- `sunray_msgs::UAVControlCMD`

### 当前实现特征

这个文件与 EGO / Diff 适配器的差异很明显：

1. **输入输出话题名是硬编码的**
   - `sub_topic_ = "/position_cmd_in"`
   - `pub_topic_ = "/uav_control_cmd_out"`
2. 没有像 EGO / Diff 那样通过参数服务器读取话题
3. `desired_jerk` 被直接置零，而不是来自输入消息

### 结论

SUPER 的适配层目前更像一个**临时或基础版桥接器**，工程化程度弱于 EGO / Diff 的 adaptor。

如果后续要在 Sunray 里正式稳定地使用 SUPER，这个文件大概率是优先需要改造的位置之一。

---

## 9.4 `pointcloud_remap/`

这个包位于：

- `planning/adaptor/pointcloud_remap/`

它的价值不是轨迹命令转换，而是**感知输入重映射**：

- 点云直接 remap
- 或基于 TF 做点云坐标变换

这个包在实际接入中很关键，因为：

- 规划器通常对输入点云坐标系有严格要求
- 特别是 SUPER README 明确提到，ROG-Map 默认按世界系理解输入点云

所以如果规划看起来“算法没问题但避障异常”，优先怀疑：

1. 点云 frame 不对
2. odom frame 不对
3. remap / tf 链路不对

---

## 10. 规划模块的数据流理解

综合这些代码与目录，我认为当前规划模块在 Sunray 中的典型数据流是：

```text
传感器 / 仿真器
  -> 点云 / 深度 / odom
  -> 地图层（GridMap / SDF / ROG-Map / PlanEnv）
  -> 前端搜索（A* / kinodynamic A* / frontier search / corridor）
  -> 轨迹优化（B-spline / MINCO / polynomial / yaw）
  -> PositionCommand / Bspline / PolyTraj 等规划器自有消息
  -> Adaptor 转换为 sunray_msgs::UAVControlCMD
  -> Sunray 控制模块执行
```

如果换成按目录理解：

```text
planning/source/*           负责“算出轨迹”
planning/adaptor/*          负责“把轨迹命令翻译成 Sunray 控制命令”
control/*                   负责“执行控制命令”
localization/* + drivers/*  负责“提供规划所需状态与传感器输入”
simulation/*                负责“提供测试环境”
```

---

## 11. 我对规划模块的整体判断

## 11.1 本质判断

`planning/` 的本质不是“Sunray 自研的一套单体规划器”，而是：

> **一个集成了多套上游规划系统，并通过适配层接入 Sunray 控制栈的规划平台。**

## 11.2 结构上的关键现实

这个事实会直接影响后续开发策略：

### 现实 A：上游源码很多，且相互独立

- EGO / Diff / FUEL / SUPER 的内部风格差异非常大
- 不适合一上来横向统一重构
- 更适合在各自边界上做集成和小范围改造

### 现实 B：真正和 Sunray 强耦合的地方主要在适配层

如果目标是：

- 跑通某个规划器
- 接到 Sunray 控制
- 修复输出命令不对
- 对接某个无人机实例话题

优先看：

- `planning/adaptor/*`
- `planning/planner_adaptor/*`

### 现实 C：地图/坐标系问题会是高频问题

尤其是：

- 点云 frame
- odom frame
- 世界系 vs 机体系
- TF 是否正确

这类问题不一定出在规划器本体，而可能出在：

- `pointcloud_remap/`
- 上游 map package
- launch 参数
- 传感器/定位链路

### 现实 D：SUPER 的集成成熟度和 EGO/Diff 不同

SUPER 很强，但在当前仓库里的 Sunray 专属桥接看起来还比较薄，说明：

- 它可能是较新的接入项
- 后续如果用 SUPER 做正式集成，适配层和启动流程大概率还要补工程化工作

---

## 12. 如果后续继续深入，建议的阅读优先级

后续若我们围绕“规划模块”继续工作，我建议按问题类型来选入口。

### 12.1 如果目标是“改输出控制命令 / 对接 Sunray 控制”
优先读：

- `planning/adaptor/ego_planner_adaptor/src/positionCmd_to_sunray.cpp`
- `planning/adaptor/diff_planner_adaptor/src/positionCmd_to_sunray.cpp`
- `planning/planner_adaptor/src/super_adaptor.cpp`

### 12.2 如果目标是“改局部重规划逻辑”
优先读：

- `planning/source/ego_planner_swarm/plan_manage/*`
- `planning/source/diff_planner/plan_manage/*`
- 各自的 `planner_manager.*`
- 各自的 `traj_server.*`

### 12.3 如果目标是“改探索逻辑”
优先读：

- `planning/source/FUEL/fuel_planner/exploration_manager/*`
- `planning/source/FUEL/fuel_planner/active_perception/*`

### 12.4 如果目标是“改 SUPER 主流程 / 高速导航”
优先读：

- `planning/source/SUPER/super_planner/include/fsm/fsm.h`
- `planning/source/SUPER/super_planner/src/super_core/*`
- `planning/source/SUPER/rog_map/*`
- `planning/source/SUPER/mission_planner/*`

### 12.5 如果目标是“查点云 / frame / 地图输入异常”
优先读：

- `planning/adaptor/pointcloud_remap/*`
- `planning/source/*/plan_env/*`
- `planning/source/SUPER/rog_map/*`
- 对应 launch 文件

---

## 13. 后续对话的术语约定

从现在开始：

- **“规划模块”** = `planning/` 整个目录
- 默认包括：
  - `planning/source/`
  - `planning/adaptor/`
  - `planning/planner_adaptor/`

如果你后续说：

- “看一下规划模块的输出链路”
- “改一下规划模块和控制模块的接口”
- “排查规划模块为什么不出轨迹”

我会默认按这个范围处理。

---

## 14. 当前我已经建立的工作认知

为了方便后续协作，这里给出一个简化版认知图：

### 规划模块不是 1 套算法，而是 4 套核心体系

- **EGO**：B-spline 局部重规划
- **Diff**：多项式 / MINCO 局部重规划
- **FUEL**：自主探索系统
- **SUPER**：高速安全导航系统

### Sunray 真正接它们的地方主要是 adaptor

- EGO -> `ego_planner_adaptor`
- Diff -> `diff_planner_adaptor`
- SUPER -> `planner_adaptor/super_adaptor.cpp`
- FUEL -> 当前没看到同等成熟的 Sunray 专属命令适配层

### 后续最常改的高概率位置

1. `planning/adaptor/*`
2. `planning/source/*/plan_manage/*`
3. `planning/source/SUPER/super_planner/*`
4. `planning/adaptor/pointcloud_remap/*`

---

## 15. 补充说明

这次阅读是为了建立**整体结构与后续开发的正确入口**，因此我优先做了：

- 项目级结构识别
- 规划模块分层识别
- 每套规划器的角色判定
- 关键 FSM / 适配器 / 输出链路识别

而没有继续把每个子系统所有 cpp 逐个细抠到底。

如果你下一步给出更具体目标，例如：

- 统一规划器输出接口
- 接通 SUPER 到 Sunray 控制
- 排查某个 planner 不发布控制命令
- 改某个 FSM 的状态流
- 比较 EGO 与 Diff 的轨迹输出差异

我现在已经有足够的上下文，能够直接进入对应子系统继续做。