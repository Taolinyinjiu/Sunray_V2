<!-- title: 仿真模块 -->

<section id="simulation">

## 仿真模块

仿真用于在实机前验证控制、规划、集群和感知逻辑。Sunray_v2 同时包含 Sunray 主仿真环境、Gazebo 插件、quadrotor simulator、比赛 demo、评分工具和 RViz/启动面板集成。

对新手开发者来说，仿真模块的价值不是“替代实机”，而是先把上层任务链路跑通：

```text
仿真器/地图/传感器
  -> 仿真 odom / 点云 / 图像 / 障碍物
  -> localization_fusion
  -> sunray_uav_control / sunray_ugv_control
  -> planning / swarm / examples
```

只要上层节点都使用 Sunray 统一消息和话题，后续从仿真迁移到实机时，通常只需要替换定位源、控制参数、airframe 配置和硬件驱动。

### 目录说明

```text
simulation/
├── 2025_uav_competition_demo/   # 2025 无人机竞赛示例，包含 A*、APF、位置发布等
├── future_2025_score/           # 比赛评分脚本和任务评估相关代码
├── gazebo_plugin/               # Gazebo 插件集合，包含雷达、风场、随机障碍等
├── simulator_utils/             # 通用仿真工具，包含地图、控制、可视化和四旋翼动力学
├── sunray_simulator/            # Sunray 仿真主场景、模型、启动文件和接口适配
└── sysu_competition/            # SYSU 竞赛相关环境与源码
```

### 环境依赖

推荐环境：

| 依赖 | 说明 |
| --- | --- |
| Ubuntu 20.04 | 当前 ROS Noetic 和 PX4/Gazebo 链路最常见组合。 |
| ROS Noetic | Sunray_v2 主要面向 ROS 1。 |
| Gazebo | Noetic 默认 Gazebo 版本即可。 |
| catkin 工具链 | 可使用 `catkin_make` 或 `catkin build`。 |
| MAVROS | PX4 SITL 或 MAVROS 接口仿真时需要。 |
| RViz / TF | 可视化仿真状态、定位、规划轨迹和集群 marker。 |
| PCL / OpenCV | 点云、地图、视觉相关 demo 可能依赖。 |

如果是第一次配置工作空间，建议先用 `rosdep` 补齐依赖：

```bash
cd /home/amov/Sunray_v2
rosdep install --from-paths src --ignore-src -r -y
```

如果当前仓库不是标准 `src` 工作空间布局，可以按实际 catkin 工作空间调整 `--from-paths` 路径。

### 构建方式

在 Sunray_v2 根目录构建：

```bash
cd /home/amov/Sunray_v2
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

如果只调试仿真相关包，可以用单独 build 目录减少编译范围：

```bash
cd /home/amov/Sunray_v2
catkin_make --source simulation --build build/simulation -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

修改消息定义、控制接口或定位接口后，应先编译 `common`，再编译依赖这些消息的仿真、控制、规划或集群包。

### 快速运行

运行 Sunray 基础仿真：

```bash
roslaunch sunray_simulator sunray_px4_basic.launch
```

运行无人机规划仿真：

```bash
roslaunch sunray_simulator sunray_sim_uav_planning.launch
```

运行 2025 竞赛 demo：

```bash
roslaunch 2025_uav_competition_demo Astar.launch
roslaunch 2025_uav_competition_demo APF.launch
```

这些 launch 的侧重点不同：

| 启动方式 | 适合验证 |
| --- | --- |
| `sunray_px4_basic.launch` | PX4/Gazebo/MAVROS 基础链路是否能跑通。 |
| `sunray_sim_uav_planning.launch` | 规划、控制、定位仿真接口串联。 |
| `Astar.launch` | 栅格/图搜索类路径规划 demo。 |
| `APF.launch` | 人工势场类局部避障 demo。 |

如果启动后提示找不到包，先确认已经：

```bash
source /home/amov/Sunray_v2/devel/setup.bash
rospack find sunray_simulator
```

### `simulation/sunray_simulator`

Sunray 主仿真入口，通常是新手最先接触的仿真包。它负责组织 Gazebo 世界、模型、PX4/SITL 或仿真动力学、传感器插件、Sunray 标准话题之间的关系。

| 路径 | 作用 |
| --- | --- |
| `config/` | 仿真参数、模型配置、地图或场景配置。 |
| `launch/` | 常用仿真入口，建议优先从这里读启动链路。 |
| `models/` / `worlds/` | Gazebo 模型和世界文件，具体目录以当前包实际结构为准。 |
| `include/`、`src/` | 仿真节点和工具代码。 |
| `scripts/` | 仿真辅助脚本。 |

二次开发时通常会改：

| 目标 | 常见改动 |
| --- | --- |
| 换场景 | 修改 world、模型路径或 launch 中的 world 参数。 |
| 换机型 | 修改模型、PX4 airframe、控制参数和传感器配置。 |
| 加传感器 | 在模型 SDF/URDF 中增加 Gazebo plugin，并确认输出话题。 |
| 接入控制 | 确认仿真输出的 odom 能进入 `localization_fusion`，控制命令能回到仿真器。 |

### `simulation/simulator_utils`

仿真基础工具集合。这里更偏底层，不一定每个新手都需要改，但理解它能帮助你知道仿真数据从哪里来。

| 子包 | 作用 |
| --- | --- |
| `quadrotor_simulator` | 四旋翼动力学仿真，接收控制输入并输出无人机状态。 |
| `quadrotor_control` | 仿真控制器，适合验证轨迹跟踪和低层控制算法。 |
| `map_generator` | 随机/静态地图生成，常用于规划算法测试。 |
| `local_map` | 局部地图构建或局部环境表达。 |
| `Utils/quadrotor_msgs` | 仿真内部消息。不要和 `common/sunray_msgs` 混淆，Sunray 跨包接口仍以 `sunray_msgs` 为主。 |
| `Utils/odometry_visualization` | odom 可视化工具。 |
| `Utils/uav_utils` | 仿真工具函数。 |
| `Utils/simulator_cmake_utils` | 仿真 CMake 工具。 |

使用建议：

- 做控制/规划二次开发时，优先依赖 Sunray 标准话题，不要直接依赖仿真内部消息。
- 做仿真器本体开发时，再深入 `quadrotor_simulator`、`quadrotor_control` 和地图工具。
- 修改内部仿真消息后，要检查所有依赖该消息的 demo 是否需要同步改。

### Gazebo plugins

| 子包 | 作用 |
| --- | --- |
| `livox_laser_simulation` | Livox 雷达 Gazebo 仿真插件。 |
| `realsense_gazebo_plugin` | RealSense 相机 Gazebo 插件。 |
| `random_cylinder_plugin` | 随机圆柱障碍物插件。 |
| `wind_zone_plugin` | 风场插件。 |

插件通常通过 Gazebo 模型或 world 文件加载。排查插件是否生效时，优先看三件事：

```bash
rostopic list
gz topic -l
rosnode list
```

如果 Gazebo 能启动但传感器话题没有出现，常见原因是插件没有编译、模型路径没有加入 `GAZEBO_MODEL_PATH`，或者 SDF/URDF 中 plugin 名称与实际库名不一致。

### 比赛/任务 demo

| 功能包 | 说明 |
| --- | --- |
| `simulation/2025_uav_competition_demo` | 2025 无人机比赛 demo。 |
| `simulation/future_2025_score` | 比赛计分/脚本相关。 |
| `simulation/sysu_competition` | SYSU 比赛仿真。 |

这类 demo 更适合学习完整任务链路：

```text
任务目标
  -> 地图/障碍物/评分规则
  -> 规划或策略节点
  -> Sunray 控制接口
  -> 仿真器反馈
```

如果你要基于比赛 demo 做二次开发，建议先只改上层策略或规划节点，保持仿真场景、控制接口和评分脚本不变。等任务逻辑跑通后，再调整场景参数或评分规则。

### 作为 Git 子模块使用

旧文档中提到过把 `simulation` 作为独立仓库维护的方式。当前 Sunray_v2 已经把它作为项目内目录使用；只有当团队确实需要单独维护仿真仓库时，才建议改成子模块。

独立初始化仿真仓库：

```bash
cd /home/amov/Sunray_v2/simulation
git init
git add .
git commit -m "init simulation repository"
git branch -M main
git remote add origin <your-simulation-repo-url>
git push -u origin main
```

在主仓库中改为子模块：

```bash
cd /home/amov/Sunray_v2
git rm -r simulation
git commit -m "移除本地 simulation 目录"
git submodule add <your-simulation-repo-url> simulation
git commit -m "添加 simulation 子模块"
```

团队成员拉取时：

```bash
git clone --recurse-submodules <main-repo-url>
git submodule update --init --recursive
```

对子模块不熟悉的团队，不建议轻易采用这个方案。子模块能让仿真仓库独立演进，但也会增加版本指针、分支同步和新手拉取成本。

### 仿真到实机迁移

| 环节 | 仿真 | 实机 |
| --- | --- | --- |
| 定位 | Gazebo/Pengyu 真值或仿真 odom | 动捕、VIO、FAST-LIO、GPS/RTK |
| 控制 | 同样发布 Sunray 控制消息 | 同样发布 Sunray 控制消息 |
| 底层 | 仿真 MAVROS/模型/底盘 | 真实 PX4/MAVROS/底盘驱动 |
| 安全 | 可快速试错 | 必须限速、限高、设置围栏和急停 |

迁移原则：任务节点尽量不改，只替换 launch、airframe、定位源和硬件驱动。

典型迁移步骤：

1. 确认仿真中任务节点只发布 Sunray 标准控制命令。
2. 把定位源从 Gazebo/Pengyu 切到动捕、VIO、FAST-LIO 或其他实机定位。
3. 把控制参数从仿真 airframe 切到真实机型 airframe。
4. 先限速、限高、限航点范围，在空旷环境做单机验证。
5. 单机稳定后，再验证规划、集群、感知等上层功能。

### 常见问题

找不到包或话题：

```text
确认已经 source devel/setup.bash
确认 rospack find <package> 能找到目标包
确认 ROS_PACKAGE_PATH 指向当前工作空间
确认 launch 中 package 名和实际 package.xml 名称一致
```

Gazebo 模型加载失败：

```text
检查模型路径是否加入 GAZEBO_MODEL_PATH
检查 world/SDF/URDF 中的 model:// 路径是否正确
检查插件是否已经编译生成 .so
检查 Gazebo 终端输出中是否有 plugin load error
```

脚本无执行权限：

```bash
chmod +x simulation/**/scripts/*.py
```

如果 shell 不支持 `**`，可以进入具体功能包的 `scripts/` 目录后执行：

```bash
chmod +x *.py
```

仿真启动后无人机不动：

```text
检查控制命令话题是否有数据
检查定位话题是否进入 localization_fusion
检查 sunray_uav_control 的 control_state 是否有效
检查 PX4/MAVROS 模式是否进入 OFFBOARD 或仿真控制器是否接收命令
检查控制命令发布频率是否满足控制模式要求
```

规划 demo 没有路径：

```text
检查地图话题是否存在
检查起点/终点是否在地图范围内
检查障碍物是否完全堵住路径
检查 RViz Fixed Frame 是否和地图 frame 一致
```

### 维护建议

- 新增功能优先放入对应子包，保持包边界清晰。
- 启动配置变更时同步更新 `launch/` 与手册。
- 新增传感器插件时，同时写清输入、输出话题、Gazebo 加载方式和实机对应硬件。
- 仿真 demo 不要直接绕过 Sunray 标准消息，否则后续迁移到实机时改动会变大。
- 如果作为子模块使用，主仓库需要及时提交子模块指针，避免团队成员拉到旧版本。

</section>
