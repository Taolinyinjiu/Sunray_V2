<!-- title: NAVRL规划器源码架构 -->

<section id="navrl-example">

## 1. NAVRL规划器源码架构

`planning/third_party_planner_examples/navrl_planner_example` 是 NAVRL 第三方规划器示例目录。本页重点介绍检测、地图和导航运行三个源码功能包；Sunray 侧适配包见同级页面 `sunray_navrl_adapter`。

### 1.1 源码功能包结构

#### 1.1.1 onboard_detector

`onboard_detector` 提供动态障碍物检测节点和 `GetDynamicObstacles` 服务，是 NAVRL 感知侧输入的一部分。

#### 1.1.2 map_manager

`map_manager` 提供占据地图、ESDF、动态地图以及碰撞/射线/静态障碍物查询服务。

#### 1.1.3 navigation_runner

`navigation_runner` 提供安全动作和策略推理相关服务，是 NAVRL 的导航运行入口。

### 1.2 目录结构

```text
planning/third_party_planner_examples/navrl_planner_example/
├── onboard_detector/       # 动态障碍物检测和检测服务
├── map_manager/            # 占据地图、ESDF、动态地图和地图查询服务
├── navigation_runner/      # 安全动作、策略推理服务和导航运行脚本
└── sunray_navrl_adapter/   # NAVRL 到 Sunray 控制接口的适配包，详见同级 adapter 页面
```

| 包 | 作用 |
| --- | --- |
| `onboard_detector` | 提供动态障碍物检测节点和 `GetDynamicObstacles` 服务。 |
| `map_manager` | 提供占据地图、ESDF、动态地图以及碰撞/射线/静态障碍物查询服务。 |
| `navigation_runner` | 提供安全动作和策略推理相关服务，是 NAVRL 运行入口。 |
| `sunray_navrl_adapter` | Sunray 侧适配包，详见同级 `sunray_navrl_adapter` 页面。 |

### 1.3 额外依赖

NAVRL 示例依赖部分 ROS 发行版包。当前已确认的额外依赖如下：

```bash
sudo apt update
sudo apt install -y ros-noetic-vision-msgs
```

`vision_msgs` 由 `onboard_detector` 使用。如果编译时报下面错误，说明该依赖还没有安装，或当前 shell 没有正确 source ROS 环境：

```text
Could NOT find vision_msgs
Could not find the required component 'vision_msgs'
```

安装后可用下面命令确认：

```bash
rospack find vision_msgs
```

NAVRL 官方导航节点还需要单独的 Conda 部署环境。如果机器上没有 `conda`，可以先安装 Miniconda：

```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda3.sh
bash /tmp/miniconda3.sh -b -p ~/miniconda3
source ~/miniconda3/etc/profile.d/conda.sh
conda init bash
conda config --set auto_activate_base false
```

创建 NAVRL 部署环境：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
cd ~/Sunray_v2/planning/third_party_planner_examples/navrl_planner_example/isaac-training
bash setup_deployment.sh
```

如果 Conda 新版本提示 channel Terms of Service 未确认，按提示先执行：

```bash
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
```

当前部署脚本固定使用 `torch==2.0.1`。如果脚本在 `pip install` 阶段没有使用到 `NavRL` 环境内的 pip，可先补齐 pip 后继续安装：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate NavRL
conda install -y -c conda-forge pip setuptools wheel
python -m pip install numpy==1.26.4
python -m pip install torch==2.0.1 torchvision==0.15.2 torchaudio==2.0.2
python -m pip install "pydantic!=1.7,!=1.7.1,!=1.7.2,!=1.7.3,!=1.8,!=1.8.1,<2.0.0,>=1.6.2" imageio-ffmpeg==0.4.9 moviepy==1.0.3 hydra-core einops pyyaml rospkg matplotlib tomli cloudpickle
python -m pip install "setuptools<81"

cd ~/Sunray_v2/planning/third_party_planner_examples/navrl_planner_example/isaac-training/third_party/tensordict
python -m pip install -e . --no-build-isolation --no-deps

cd ../rl
python -m pip install -e . --no-build-isolation --no-deps
```

运行官方导航节点前，需要同时 source ROS 和 Sunray 工作空间：

```bash
source /opt/ros/noetic/setup.bash
source ~/Sunray_v2/devel/setup.bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate NavRL
rosrun navigation_runner navigation_node.py
```

### 1.4 编译方式

构建脚本中提供了 `navrl` 预设组，包含四个 NAVRL 相关包：

```text
onboard_detector
map_manager
navigation_runner
sunray_navrl_adapter
```

依赖展开后还会自动带上 `sunray_msgs` 和 `sunray_common`：

```text
onboard_detector
map_manager
navigation_runner
sunray_msgs
sunray_common
sunray_navrl_adapter
```

推荐直接使用构建组：

```bash
./build.sh -y navrl
```

也可以显式指定四个 NAVRL 包：

```bash
./build.sh -y onboard_detector map_manager navigation_runner sunray_navrl_adapter
```

### 1.5 运行边界

`navigation_runner/scripts/navigation_node.py` 是 NAVRL 官方导航节点入口，它负责加载策略网络、订阅目标点、调用 `occupancy_map/raycast`，并发布 `/CERLAB/quadcopter/cmd_vel` 和 `/CERLAB/quadcopter/setpoint_pose`。

`sunray_navrl_adapter` 不修改 NAVRL 的策略网络逻辑，只完成两类工作：

1. 通过 `NavRL2Sunray.launch` 或 `NavRL2Sunray_sim.launch` 启动 `map_manager` 和控制转换节点。
2. 将 NAVRL 输出转换为 Sunray 控制指令，发布到 `/uav1/sunray/uav_control/control_cmd`。

仿真和真机的分终端启动流程见同级页面 `sunray_navrl_adapter`。

### 1.6 编译排错记录

如果 `onboard_detector` 没有先成功编译，后续包会出现级联错误，例如：

```text
Could NOT find onboard_detector
Could NOT find navigation_runner
```

这类错误通常不是 `map_manager`、`navigation_runner` 或 `sunray_navrl_adapter` 自身的首要问题，应先回到最前面的 `onboard_detector` 编译错误处理。

</section>
