<!-- title: FUEL 第三方规划示例 -->

<section id="fuel-example">

## FUEL 第三方规划示例

`planning/third_party_planner_examples/fuel_example/FUEL` 是第三方 FUEL 探索规划源码。Sunray 当前状态机中保留了 FUEL 类型枚举，但还没有接入可用 adapter；因此它目前更适合作为算法源码和后续适配对象，而不是默认规划链路的一部分。

### Sunray 中的位置

```text
planning/third_party_planner_examples/fuel_example/FUEL/
├── fuel_planner/
└── uav_simulator/
```

主要子目录：

| 子目录 | 说明 |
| --- | --- |
| `fuel_planner/active_perception` | 主动感知和探索相关逻辑。 |
| `fuel_planner/exploration_manager` | FUEL 探索管理器和 demo launch。 |
| `fuel_planner/bspline`、`bspline_opt` | B 样条轨迹表达和优化。 |
| `fuel_planner/path_searching` | 路径搜索。 |
| `fuel_planner/plan_env` | 地图和环境表示。 |
| `fuel_planner/plan_manage` | 规划管理。 |
| `fuel_planner/traj_utils` | 轨迹工具。 |
| `uav_simulator/map_generator` | PCD 环境地图生成和发布。 |
| `uav_simulator/so3_control`、`so3_quadrotor_simulator` | FUEL 自带仿真和控制工具。 |

### 上游项目信息

FUEL 是面向无人机自主探索的第三方规划框架，全称 Fast UAV Exploration。它的核心思想是使用 Frontier Information Structure 维护探索前沿信息，再通过分层规划生成覆盖路径、局部视点和最小时间轨迹。

上游项目资料：

| 项 | 地址 |
| --- | --- |
| FUEL 论文 | https://arxiv.org/abs/2010.11561 |
| FUEL 视频 | https://www.youtube.com/watch?v=_dGgZUrWk-8 |
| RACER 多机探索 | https://github.com/SYSU-STAR/RACER |

本仓库保留了上游示例资源，路径通常位于：

```text
planning/third_party_planner_examples/fuel_example/FUEL/files/
```

手册页面不直接嵌入这些相对路径图片，避免迁移到网站后出现破图。

引用信息：

```
@article{zhou2021fuel,
  title={FUEL: Fast UAV Exploration Using Incremental Frontier Structure and Hierarchical Planning},
  author={Zhou, Boyu and Zhang, Yichen and Chen, Xinyi and Shen, Shaojie},
  journal={IEEE Robotics and Automation Letters},
  volume={6},
  number={2},
  pages={779--786},
  year={2021},
  publisher={IEEE}
}
```

### Quick Start

This project has been tested on Ubuntu 16.04(ROS Kinetic) and 18.04(ROS Melodic). Take Ubuntu 18.04 as an example, run the following commands to install required tools:

```
  sudo apt-get install libarmadillo-dev ros-melodic-nlopt
```

<!-- To simulate the depth camera, we use a simulator based on CUDA Toolkit. Please install it first following the [instruction of CUDA](https://developer.nvidia.com/zh-cn/cuda-toolkit).

After successful installation, in the **local_sensing** package in **uav_simulator**, remember to change the 'arch' and 'code' flags in CMakelist.txt according to your graphics card devices. You can check the right code [here](https://github.com/tpruvot/ccminer/wiki/Compatibility). For example:

```
  set(CUDA_NVCC_FLAGS
    -gencode arch=compute_61,code=sm_61;
  )
``` -->

Then simply clone and compile our package (using ssh here):

```
  cd ${YOUR_WORKSPACE_PATH}/src
  git clone git@github.com:HKUST-Aerial-Robotics/FUEL.git
  cd ../
  catkin_make
```

After compilation you can start a sample exploration demo. Firstly run ```Rviz``` for visualization:

```
  source devel/setup.bash && roslaunch exploration_manager rviz.launch
```
then run the simulation (run in a new terminals):
```
  source devel/setup.bash && roslaunch exploration_manager exploration.launch
```

By default you can see an office-like environment. Trigger the quadrotor to start exploration by the ```2D Nav Goal``` tool in ```Rviz```. Unexplored structures are shown in grey, explored areas are shown as colored voxels, and the FoV and trajectories are displayed in RViz.

### Exploring Different Environments

The exploration environments in our simulator are represented by [.pcd files](https://pointclouds.org/documentation/tutorials/pcd_file_format.html).
We provide several sample environments, which can be selected in `planning/third_party_planner_examples/fuel_example/FUEL/fuel_planner/exploration_manager/launch/simulator.xml`:


```xml
  <!-- Change office.pcd to specify the exploration environment -->
  <!-- We provide office.pcd, office2.pcd, office3.pcd and pillar.pcd in this repo -->
  <node pkg ="map_generator" name ="map_pub" type ="map_pub" output = "screen" args="$(find map_generator)/resource/office.pcd"/>
```

Example maps include `office.pcd`, `office2.pcd`, `office3.pcd` and `pillar.pcd`.

If you want to use your own environments, place the `.pcd` files in `planning/third_party_planner_examples/fuel_example/FUEL/uav_simulator/map_generator/resource`, and follow the comments above to specify it.
You may also need to change the bounding box of explored space in `planning/third_party_planner_examples/fuel_example/FUEL/fuel_planner/exploration_manager/launch/exploration.launch`:

```xml
    <arg name="box_min_x" value="-10.0"/>
    <arg name="box_min_y" value="-15.0"/>
    <arg name="box_min_z" value=" 0.0"/>
    <arg name="box_max_x" value="10.0"/>
    <arg name="box_max_y" value="15.0"/>
    <arg name="box_max_z" value=" 2.0"/>
```

### Creating a `.pcd` Environment

We provide a simple tool to create .pcd environments.
First, run:

```
  rosrun map_generator click_map
```

Then in ```Rviz```, use the ```2D Nav Goal``` tool (shortcut G) to create your map. Two consecutively clicked points form a wall.

After you've finished, run the following node to save the map in another terminal:

```
  rosrun map_generator map_recorder ~/
```

Normally, a file named __tmp.pcd__ will be saved at ```~/```. You may replace ```~/``` with any locations you want.
Lastly, use this file as the exploration map by editing the map path in the FUEL launch/config files.

### Known issues

### Compilation issue

When running this project on Ubuntu 20.04, C++14 is required. Please add the following line in all CMakelists.txt files:

```
set(CMAKE_CXX_STANDARD 14)
```

### Unexpected crash

If the ```exploration_node``` dies after triggering a 2D Nav Goal, it is possibly caused by the ros-nlopt library. In this case, we recommend to uninstall it and [install nlopt following the official document](https://nlopt.readthedocs.io/en/latest/NLopt_Installation/). Then in the [CMakeLists.txt of bspline_opt package](https://github.com/HKUST-Aerial-Robotics/FUEL/blob/main/fuel_planner/bspline_opt/CMakeLists.txt), change the associated lines to link the nlopt library:

```
find_package(NLopt REQUIRED)
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})

...

include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${NLOPT_INCLUDE_DIR}
)

...

add_library( bspline_opt
    src/bspline_optimizer.cpp
    )
target_link_libraries( bspline_opt
    ${catkin_LIBRARIES}
    ${NLOPT_LIBRARIES}
    # /usr/local/lib/libnlopt.so
    )

```

## Acknowledgements
  We use **NLopt** for non-linear optimization and use **LKH** for travelling salesman problem.

### 接入 Sunray 的建议

如果后续要把 FUEL 接入 `sunray_planning`，建议不要直接让控制器订阅 FUEL 私有话题，而是新增 `FuelPlanner` adapter：

1. 在 `planner_datatypes.hpp` 中确认或补齐 FUEL planner type。
2. 新建 `include/planner_interface/fuel_planner.hpp` 和 `src/planner_interface/fuel_planner.cpp`。
3. 实现 `send_goal()`，把 Sunray 的 local/global goal 转成 FUEL 目标。
4. 订阅 FUEL 输出轨迹，转成 `PlannerPositionCommand`。
5. 在 `PlanningFSM::init()` 中按 `planner_type=fuel` 创建 adapter。
6. 在 `sunray_planning.launch` 中 include FUEL 启动文件并传入 odom/cloud/topic 参数。

</section>
