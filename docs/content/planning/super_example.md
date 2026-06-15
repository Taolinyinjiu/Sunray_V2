<!-- title: SUPER 第三方规划示例 -->

<section id="super-example">

## SUPER 第三方规划示例

`planning/third_party_planner_examples/super_example/SUPER` 是第三方 SUPER 高速安全导航规划源码。Sunray 当前状态机中保留了 SUPER 类型枚举，但还没有接入可用 adapter；因此它目前更适合作为算法研究和后续适配对象。

### Sunray 中的位置

```text
planning/third_party_planner_examples/super_example/SUPER/
├── mars_uav_sim
├── mission_planner
├── rog_map
├── scripts
└── super_planner
```

主要子目录：

| 子目录 | 说明 |
| --- | --- |
| `super_planner` | SUPER planner 核心。 |
| `rog_map` | ROG-Map 占据栅格地图。 |
| `mission_planner` | 任务规划和 benchmark/click demo。 |
| `mars_uav_sim` | MARS UAV 仿真相关包。 |
| `scripts` | ROS 版本选择等脚本。 |

### 上游项目信息

SUPER 全称 Safety-assured High-speed Navigation for MAVs，是面向微型飞行器高速安全导航的第三方规划系统。上游论文发表于 Science Robotics，项目同时提供 ROS1/ROS2 版本、benchmark demo、click demo、ROG-Map 和日志工具。

上游项目资料：

| 项 | 地址 |
| --- | --- |
| SUPER 论文 | https://www.science.org/doi/10.1126/scirobotics.ado6187 |
| Bilibili 视频 | https://www.bilibili.com/video/BV1BSFgeJEJn/ |
| YouTube 视频 | https://youtu.be/GPHuzG0ANmI?si=npW-FNp1rkQQ5YaF |
| SUPER-Hardware | https://github.com/hku-mars/SUPER-Hardware |

本仓库保留了上游图片和徽标资源，路径通常位于：

```text
planning/third_party_planner_examples/super_example/SUPER/misc/
```

手册页面不直接嵌入这些相对路径图片，避免迁移到网站后出现破图。

The upstream repository also provides a paper PDF under `planning/third_party_planner_examples/super_example/SUPER/misc/scirobotics.ado6187.pdf`. If this repository supports your academic projects, please cite the upstream work.

```tex
@article{ren2025safety,
  title={Safety-assured high-speed navigation for MAVs},
  author={Ren, Yunfan and Zhu, Fangcheng and Lu, Guozheng and Cai, Yixi and Yin, Longji and Kong, Fanze and Lin, Jiarong and Chen, Nan and Zhang, Fu},
  journal={Science Robotics},
  volume={10},
  number={98},
  pages={eado6187},
  year={2025},
  publisher={American Association for the Advancement of Science}
}

@article{lu2025autonomous,
  title={Autonomous Tail-Sitter Flights in Unknown Environments},
  author={Lu, Guozheng and Ren, Yunfan and Zhu, Fangcheng and Li, Haotian and Xue, Ruize and Cai, Yixi and Lyu, Ximin and Zhang, Fu},
  journal={IEEE Transactions on Robotics},
  year={2025},
  publisher={IEEE}
}

@inproceedings{ren2024rog,
  title={Rog-map: An efficient robocentric occupancy grid map for large-scene and high-resolution lidar-based motion planning},
  author={Ren, Yunfan and Cai, Yixi and Zhu, Fangcheng and Liang, Siqi and Zhang, Fu},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={8119--8125},
  year={2024},
  organization={IEEE}
}
```

### Highlights

#### Autonomous Navigation in Challenging Environments

Video demo: https://youtu.be/GPHuzG0ANmI?si=W83mDMxqfgWReWPF

#### Applications: Object Tracking & Autonomous Exploration

SUPER has been successfully deployed in various applications, including large-scale autonomous exploration in an ongoing project by [@jackykongfz](https://github.com/jackykongfz) and [@ZbyLGsc](https://github.com/ZbyLGsc) from STAR Lab, among others, as well as object tracking under both day and night conditions.

Demo assets in the upstream source tree include `planning/third_party_planner_examples/super_example/SUPER/misc/exp.gif` and `planning/third_party_planner_examples/super_example/SUPER/misc/tracking.gif`.

> This segment is from an unpublished work by Kong [[@jackykongfz](https://github.com/jackykongfz) ] et al., conducted in collaboration with STAR Lab, using SUPER.



#### Supported Projects

Autonomous Tail-Sitter (TRO '25):

Building on SUPER, a similar planning system has been successfully validated in [Autonomous Navigation for Tail-Sitter UAVs](https://github.com/hku-mars/EFOPT)  by [@genegzl](https://github.com/genegzl)  et al.

Demo asset path: `planning/third_party_planner_examples/super_example/SUPER/misc/tailsitter.gif`.

FAST-LIVO2 (TRO '24):

SUPER serves as the flight platform and navigation system in the video demonstration of [FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry](https://github.com/hku-mars/FAST-LIVO2) by [@xuankuzcr](https://github.com/xuankuzcr) et al.

Image asset path: `planning/third_party_planner_examples/super_example/SUPER/misc/image-20250130031404057.png`.

### Quick Start

#### Installation

Install dependencies

```bash
# for MARSIM example
sudo apt-get install libglfw3-dev libglew-dev libncurses5-dev libncursesw5-dev
# Eigen [version testd: 3.3.7-2] and soft link
sudo apt-get install libeigen3-dev
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
# dw for backward cpp
sudo apt-get install libdw-dev
# for ROS dependency
sudo apt-get install ros-${YOUR-ROS-VERSION}-mavros* ros-${YOUR-ROS-VERSION}-pcl* ros-${YOUR-ROS-VERSION}-rosfmt
```

Before building the code, select the appropriate ROS version:

```bash
# Use ROS1-noetic
bash ${PATH-TO-SUPER}/SUPER/scripts/select_ros_version.sh ROS1
# Use ROS2
bash ${PATH-TO-SUPER}/SUPER/scripts/select_ros_version.sh ROS2
```

Tested Environments:

* Ubuntu 20.04 + ROS1 Noetic
* Ubuntu 20.04 + ROS2 foxy
* ...

Currently, **ROS1 Noetic** serves as the **Tier 1** supported platform for SUPER. The ROS2 version is still under development and may be unstable, with some issues such as imperfect visualization. We are actively working on improvements.

### Known Build issues

* ...

#### ROS1 (Noetic) Installation
```bash
mkdir -p super_ws/src && cd super_ws/src
git clone https://github.com/hku-mars/SUPER.git
cd ..
catkin_make -DBUILD_TYPE=Release
```

To test, use one of the following commands:

1. **High-Speed Navigation**

```bash
cd ${PATH-TO-WS}
source devel/setup.bash
roslaunch mission_planner benchmark_high_speed.launch
```

2. **Agile Flight in Dense Environments**

```bash
cd ${PATH-TO-WS}
source devel/setup.bash
roslaunch mission_planner benchmark_dense.launch
```

3. **Click and Go Demo**

```
roslaunch mission_planner click_demo.launch
```
In the click demo, press `G` to enable the `2D Goal Pose` plugin, then click a position in RViz to set the goal.
#### ROS2


```bash
mkdir -p super_ws/src && cd super_ws/src
git clone https://github.com/hku-mars/SUPER.git
cd ..
colcon build --symlink-install
# add to debug:  --event-handlers console_direct+
```

To test, run:

1. **High-speed Navigation**

```bash
cd ${PATH-TO-WS}
source install/local_setup.bash
ros2 launch mission_planner benchmark_high_speed.launch.py
```

2. **Agile flights in dense enviroment**

```bash
cd ${PATH-TO-WS}
source install/local_setup.bash
ros2 launch mission_planner benchmark_dense.launch.py
```

3. **Click demo**

```
ros2 launch mission_planner click_demo.launch.py
```

### Real-world deployment

A detailed guide for deploying SUPER on real-world hardware will be available soon. In the meantime, you can refer to [issue #5](https://github.com/hku-mars/SUPER/issues/5) for some helpful hints.

#### Use Your Own Map

SUPER allows users to load their own **.pcd** maps as simulation environments. To do so:

1. Place your **.pcd** file in `planning/third_party_planner_examples/super_example/SUPER/mars_uav_sim/perfect_drone_sim/pcd/`.
2. Modify the `pcd_name` parameter in the corresponding YAML file under `planning/third_party_planner_examples/super_example/SUPER/mars_uav_sim/perfect_drone_sim/config/`.

This enables seamless integration of custom maps for simulation.

#### Logging System

SUPER includes a built-in logging system that records each run automatically. Logs are saved in:

- `planning/third_party_planner_examples/super_example/SUPER/super_planner/log/cmd_logs`
- `planning/third_party_planner_examples/super_example/SUPER/super_planner/log/replan_logs`

After stopping the program with `Ctrl + C`, the latest log will be saved. Users can evaluate **trajectory quality** by running:

```bash
# Install dependencies
pip3 install numpy pandas matplotlib

# Plot the command log
python3 plotCmdLog.py
```

For advanced usage, refer to:

- `planning/third_party_planner_examples/super_example/SUPER/super_planner/Apps/read_replan_log.cpp`
- `planning/third_party_planner_examples/super_example/SUPER/super_planner/Apps/traj_opt_tuning.cpp`

We are actively working on improving the logging system, and updates will be available soon!

#### Tuning

To maximize performance, parameter tuning is crucial. The current version of SUPER has a large number of parameters (maybe TOOOO MUCH), requiring careful adjustment. Users can refer to the provided examples for guidance. We plan to provide detailed tuning instructions soon. In the meantime, feedback and issue reports are welcome.

#### Notable Known Issues
* [#10]: When using SUPER with your own simulator (e.g., Gazebo) or a LiDAR odometry system other than FAST-LIO2, ensure that the input point cloud is provided in the world frame. ROG-Map does not utilize `frame_id` or `/tf` information and assumes by default that all input point clouds are in the world frame rather than the body frame.

### Acknowledgments

SUPER is built upon several outstanding open-source projects. We extend our gratitude to the developers of the following repositories:

* **[FAST_LIO](https://github.com/hku-mars/FAST_LIO)**, **[Swarm-LIO2](https://github.com/hku-mars/Swarm-LIO2)** and  **[LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init)**  for their excellent localization solutions.
* **[ROG-Map](https://github.com/hku-mars/ROG-Map)** - A high-performance mapping framework that influenced our approach to map representation and optimization.
* **[MARSIM](https://github.com/hku-mars/MARSIM)** - A simulation environment that played a key role in testing and evaluating our algorithms in virtual scenarios.
* **[GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER)** – A valuable resource that efficiently performs differentiable trajectory optimization and serves as the foundation of our trajectory optimization method.

  **[FIRI](https://github.com/ZJU-FAST-Lab/GCOPTER/blob/main/gcopter/include/gcopter/firi.hpp)** – An extremely efficient safe flight corridor generation method upon which our CIRI is built.
* [**FASTER**](https://github.com/mit-acl/faster) - Introduces the initial concept of a two-trajectory optimization framework.
* **[DecompUtil](https://github.com/sikang/DecompUtil)** - A convex decomposition tool that was instrumental in implementing our algorithms.
* **[Mockamap](https://github.com/HKUST-Aerial-Robotics/mockamap)** - A simple ROS-based map generator that assisted in our development and testing.
* [**Nxt-FC**](https://github.com/HKUST-Aerial-Robotics/Nxt-FC) – A compact yet powerful hardware platform for the PX4 flight controller.

We sincerely appreciate the efforts of these communities in advancing robotics research.


### 接入 Sunray 的建议

如果后续要把 SUPER 接入 `sunray_planning`，建议新增 `SuperPlanner` adapter，而不是直接改 `sunray_uav_control`：

1. 让 SUPER 接收 Sunray 的目标点、odom 和点云输入。
2. 将 SUPER 输出轨迹转换成 `PlannerPositionCommand`。
3. 由 `sunray_planning` 继续统一发布 `UAVControlCMD::MOVE_TRAJECTORY`。
4. 明确 SUPER 对点云 frame 的要求。上游说明中提到 ROG-Map 默认假设输入点云在 world frame，不使用 `frame_id` 或 `/tf` 自动判断。
5. 把 SUPER 自身日志和 Sunray `UAVPlanningState` 对齐，方便调试。

</section>
