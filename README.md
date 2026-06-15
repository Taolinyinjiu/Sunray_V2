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

## 二次开发与使用手册

本仓库已经提供新版手册，入口在：

```text
docs/index.html
```

手册按功能包和开发链路组织，建议新手按“项目总览 -> Common/sunray_msgs -> 定位 -> UAV 控制或 UGV 控制 -> 规划/集群/驱动/仿真/工具”的顺序阅读。示例程序已经并入对应模块，例如 UAV 控制示例在 `uav_control` 章节内。

### 直接打开

```bash
cd ~/Sunray_v2
xdg-open docs/index.html
```

### 通过本地网页服务查看

如果浏览器限制本地文件访问，或希望用更接近网站部署的方式查看：

```bash
cd ~/Sunray_v2/docs
python3 -m http.server 8080
```

然后在浏览器打开：

```text
http://127.0.0.1:8080
```

### 修改手册后同步

正文 Markdown 位于 `docs/content/`。修改 Markdown 后，请重新生成 `docs/assets/doc-data.js`，这样双击打开 `docs/index.html` 时也能看到最新内容：

```bash
cd ~/Sunray_v2
python3 docs/assets/sync-doc-data.py
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
  # 启动文件中需配置机型
  roslaunch sunray_system sunray_system.launch airframe_type:=sunray_150
  roslaunch sunray_system sunray_system.launch airframe_type:=sunray_300
  ```

  系统会自动加载 `tools/sunray_system/config/features_<机型>.yaml`。

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


## 编译问题排查

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

- open3d_loc 编译找不到 Open3D
  ```text
  open3d_loc 依赖 Open3D 的 C++ 开发库，需要系统中存在 Open3DConfig.cmake 或 open3d-config.cmake。
  Ubuntu 20.04 默认 apt 源通常没有 Open3D C++ 开发包，可使用 Open3D 官方预编译包。
  ```
  ```bash
  # 安装基础依赖。官方预编译包依赖 libc++ / libc++abi。
  sudo apt update
  sudo apt install -y git cmake build-essential python3-dev libc++-dev libc++abi-dev

  # 下载并安装 Open3D C++ 开发包，版本可按需要调整
  cd ~
  mkdir -p open3d_install
  curl -L -o /tmp/open3d-devel-linux-x86_64-cxx11-abi-0.19.0.tar.xz \
    https://github.com/isl-org/Open3D/releases/download/v0.19.0/open3d-devel-linux-x86_64-cxx11-abi-0.19.0.tar.xz
  tar -xf /tmp/open3d-devel-linux-x86_64-cxx11-abi-0.19.0.tar.xz \
    -C $HOME/open3d_install --strip-components=1

  # 让 open3d_loc 找到 Open3D
  cd ~/Sunray_v2
  cmake -S localization/open3d_loc -B build/open3d_loc \
    -DOpen3D_ROOT=$HOME/open3d_install \
    -DCATKIN_DEVEL_PREFIX=$HOME/Sunray_v2/devel
  cmake --build build/open3d_loc --target global_localization_node -j$(nproc)
  ```

  ```bash
  # 也可以写入 shell 配置，后续构建自动生效
  echo 'export Open3D_ROOT=$HOME/open3d_install' >> ~/.bashrc
  ```
  ```bash
  # 如果没有 sudo 权限安装 libc++，可以只下载并解包到用户目录
  mkdir -p /tmp/libcxx_debs $HOME/open3d_libcxx
  cd /tmp/libcxx_debs
  apt-get download libc++1-10 libc++-10-dev libc++abi1-10 libc++abi-10-dev
  for f in *.deb; do dpkg-deb -x "$f" $HOME/open3d_libcxx; done

  cd ~/Sunray_v2
  cmake -S localization/open3d_loc -B build/open3d_loc \
    -DOpen3D_ROOT=$HOME/open3d_install \
    -DLIBCXX_ROOT=$HOME/open3d_libcxx \
    -DCATKIN_DEVEL_PREFIX=$HOME/Sunray_v2/devel
  cmake --build build/open3d_loc --target global_localization_node -j$(nproc)
  ```
