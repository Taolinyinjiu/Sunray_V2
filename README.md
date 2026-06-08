<img src="https://pic1.imgdb.cn/item/67ceabf3066befcec6e26c76.png" alt="yundrone logo" align="right" height="90" />

# Sunray - 开源无人机仿真与实践平台

**Sunray** 是云纵科技 “**Sunray 系列科研无人机**” 的配套开源代码，为无人机智能化和自主化飞行提供 **全套解决方案**。

- 支持对无人车、无人机的控制，提供完善的二次开发接口
- 内置丰富的接口使用例程，提供二次开发使用教程
- 接入多个无人机开源项目，如 FAST-LIO2、EGO-planner 等
- 支持 RVIZ 仿真、Gazebo 仿真及真实无人机平台实验，满足不同科研需求

## 快速安装

```bash
# 下载
git clone https://gitee.com/yundrone_sunray2023/Sunray_v2

# 编译，当前测试选择 test 分组编译即可
cd Sunray_v2
./build.sh
```

```bash
# 安装配网工具，方便以后切换wifi
bash <(curl -fsSL https://install.yundrone.cn/ble-wifi-tool.sh)

# 外部PC使用如下网址进行配网
https://tool.yundrone.cn/ble/
```




### 编译问题排查

- Fast-lio2编译不过
  ```text
  可能原因：原有的镜像预先配置的Livox_SDK与Livox_ros_driver2是基于MID360的旧版本，但是Sunray_V2仓库中实用的livox_ros_driver2是MID360s的新版本，如果系统中的Livox_SDK没有更新，则会导致编译过程中产生报错，因此需要更新Livox_SDK
  ```
  ```text
  # 进入对应目录
  cd ~/Sunray_v2/drivers/Livox_SDK2/
  # 创建编译目录
  mkdir build
  cd build
  # 进行编译
  cmake ..
  make 
  sudo make install
  # 回到主目录重新编译
  ```


## 仿真使用

- 启动仿真一键启动面板：

  ```bash
  roslaunch sunray_launcher_panel sunray_launcher_panel.launch
  ```
  - 可以启动仿真简易地面站
  - 可以启动任何已经写好的仿真脚本

- 一键启动面板的配置文件：

  ```text
  tools/sunray_launcher_panel/config/sunray_launch_groups.yaml
  tools/sunray_launcher_panel/config/sunray_quick_launch_groups.yaml
  ```

## 真机测试

- Sunray系统后台任务（建议设置为开机自启）

  ```bash
  cd ~/Sunray_v2
  source devel/setup.bash
  roslaunch sunray_system sunray_system.launch
  ```

  如果需要修改启动任务，请查看：

  ```text
  tools/sunray_system/config/features.yaml
  ```

- Sunray脚本启动器（TUI界面，通过SSH启动）

  ```bash
  cd ~/Sunray_v2
  source devel/setup.bash
  roslaunch scripts_manage scripts_manage_tui.launch
  ```

### 一键修改Mavros参数
- 如果想确认或修改 PX4 参数，请确认 Mavros 已启动，然后执行：

  ```bash
  cd ~/Sunray_v2/tools/px4_param_check

  # 检查 PX4 参数，config指PX4参数的配置文件，fcu-url为飞控端口地址
  ./check_px4_param.sh --config px4_params_default.yaml --fcu-url /dev/ttyACM0:921600
  ./check_px4_param.sh --config px4_params_s150.yaml --fcu-url /dev/ttyACM0:921600
  ./check_px4_param.sh --config px4_params_s300.yaml --fcu-url /dev/ttyACM0:921600

  # 设置 PX4 参数，config指PX4参数的配置文件，fcu-url为飞控端口地址
  ./set_px4_param.sh --config px4_params_default.yaml --fcu-url /dev/ttyACM0:921600
  ./set_px4_param.sh --config px4_params_s150.yaml --fcu-url /dev/ttyACM0:921600
  ./set_px4_param.sh --config px4_params_s300.yaml --fcu-url /dev/ttyACM0:921600
  ```

- 参数存放在：

  ```text
  ~/Sunray_v2/tools/px4_param_check/config
  ```

  目前包含三份参数配置：

  - `px4_params_default.yaml`
  - `px4_params_s150.yaml`
  - `px4_params_s300.yaml`

## 版权声明

- 本开源项目仅限个人使用，请勿用于商业用途。
- 如利用本项目进行营利活动，云纵科技将追究侵权行为。
