# RealSense D400 FW 5.16.0.1 使用报告

本文档适用于本仓库中的 RealSense 方案：

- 相机固件：D400 系列 FW `5.16.0.1`
- SDK：`librealsense 2.55.1`
- ROS 驱动：`drivers/realsense2_camera`
- 目标行为：深度帧使用激光投射器，双目灰度帧避开激光光斑

当前驱动默认使用 `RS2_OPTION_EMITTER_ON_OFF` 开启相机端逐帧切换 emitter。由于本机测试中 `RS2_FRAME_METADATA_FRAME_EMITTER_MODE` 没有稳定返回，默认不依赖 metadata，而是使用同步 `frameset` 的统一交替相位分流：

- depth / aligned depth：发布带激光的帧
- infra1 / infra2：发布无激光的帧

## 1. 卸载旧版驱动

先停止所有占用相机的进程：

```bash
pkill -f realsense-viewer || true
pkill -f realsense2_camera || true
pkill -f nodelet || true
```

可检查是否仍有进程占用 `/dev/video*`：

```bash
fuser -v /dev/video0 /dev/video1 /dev/video2 /dev/video3 /dev/video4 /dev/video5
```

如果之前通过 apt 安装过 RealSense SDK 或 ROS 驱动，建议清理：

```bash
sudo apt purge 'librealsense2*'
sudo apt purge ros-noetic-realsense2-camera ros-noetic-realsense2-description
sudo apt autoremove
sudo ldconfig
```

如果旧版 librealsense 是从源码 `cmake --install` 安装的，优先使用旧 build 目录里的安装清单卸载：

```bash
cd /path/to/old/librealsense
sudo xargs -r -a build/install_manifest.txt rm -v
sudo ldconfig
```

如果旧 build 目录已经不存在，可手动检查并清理 `/usr/local` 下的旧安装。执行前先确认路径：

```bash
ls /usr/local/lib/librealsense2*
ls /usr/local/include/librealsense2*
ls /usr/local/lib/cmake/realsense2*
ls /usr/local/lib/pkgconfig/realsense2*
```

常见手动清理命令如下：

```bash
sudo rm -f /usr/local/lib/librealsense2*
sudo rm -rf /usr/local/include/librealsense2 /usr/local/include/librealsense2-gl
sudo rm -rf /usr/local/lib/cmake/realsense2 /usr/local/lib/cmake/realsense2-gl
sudo rm -f /usr/local/lib/pkgconfig/realsense2.pc /usr/local/lib/pkgconfig/realsense2-gl.pc
sudo ldconfig
```

udev 规则也可以先移除，后续会重新安装：

```bash
sudo rm -f /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 2. 安装 librealsense 2.55.1

进入仓库：

```bash
cd /home/taolin/Documents/GitHub/Sunray_V2_PR
```

确认 SDK 源码目录：

```bash
cd drivers/librealsense
```

如果该目录保留了内部 git 信息，可确认版本：

```bash
git describe --tags --exact-match HEAD
```

期望输出：

```text
v2.55.1
```

安装常用依赖：

```bash
sudo apt update
sudo apt install build-essential cmake pkg-config libusb-1.0-0-dev libssl-dev libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
```

编译并安装：

```bash
cd /home/taolin/Documents/GitHub/Sunray_V2_PR/drivers/librealsense
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
cmake --build build -j$(nproc)
sudo cmake --install build
sudo ldconfig
```

安装 udev 规则：

```bash
cd /home/taolin/Documents/GitHub/Sunray_V2_PR/drivers/librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔相机，或重启系统。

验证 SDK：

```bash
realsense-viewer --version
rs-enumerate-devices | grep -E 'Firmware Version|Product Id|Serial Number'
```

如果系统 PATH 中的 `realsense-viewer` 仍是旧版，可直接使用本仓库编译出的 viewer：

```bash
cd /home/taolin/Documents/GitHub/Sunray_V2_PR/drivers/librealsense
LD_LIBRARY_PATH=$PWD/build/Release ./build/Release/realsense-viewer
```

## 3. 编译 ROS 驱动

回到仓库根目录：

```bash
cd /home/taolin/Documents/GitHub/Sunray_V2_PR
```

编译 `realsense2_camera`：

```bash
catkin_make --source drivers/realsense2_camera --build build/realsense2_camera
source devel/setup.bash
```

确认 ROS 驱动链接到 librealsense 2.55：

```bash
ldd devel/lib/librealsense2_camera.so | grep librealsense
```

期望看到：

```text
librealsense2.so.2.55 => /usr/local/lib/librealsense2.so.2.55
```

## 4. 默认 launch 使用方式

启动：

```bash
cd /home/taolin/Documents/GitHub/Sunray_V2_PR
source devel/setup.bash
roslaunch realsense2_camera rs_camera.launch
```

当前 `rs_camera.launch` 的关键默认参数：

```text
enable_depth=true
enable_infra1=true
enable_infra2=true
depth_width=640
depth_height=480
infra_width=640
infra_height=480
depth_fps=60
infra_fps=60
enable_emitter=true
emitter_on_off=true
emitter_on_off_depth_phase=0
emitter_on_off_use_metadata=false
enable_sync=true
align_depth=true
enable_color=false
```

注意：`emitter_on_off=true` 时，相机会逐帧切换激光状态，驱动只发布其中一半的 depth 帧和另一半的 infrared 帧。因此虽然相机配置为 `60 FPS`，ROS 中 depth 和 infra 的实际输出约为 `30 FPS`。

## 5. 主要话题

常用图像话题：

```text
/camera/depth/image_rect_raw
/camera/infra1/image_rect_raw
/camera/infra2/image_rect_raw
/camera/aligned_depth_to_infra1/image_raw
/camera/aligned_depth_to_infra2/image_raw
```

检查话题频率：

```bash
rostopic hz /camera/depth/image_rect_raw
rostopic hz /camera/infra1/image_rect_raw
rostopic hz /camera/infra2/image_rect_raw
```

查看图像：

```bash
rqt_image_view
```

期望现象：

- `/camera/depth/image_rect_raw` 有稳定输出，深度效果受激光增强
- `/camera/infra1/image_rect_raw` 和 `/camera/infra2/image_rect_raw` 没有明显激光光斑
- aligned depth 话题有输出时，内容来自被保留的 depth 帧

## 6. 常用参数调整

如果双目灰度仍有光斑，说明 fallback 相位可能和具体相机状态相反，切换 phase：

```bash
roslaunch realsense2_camera rs_camera.launch emitter_on_off_depth_phase:=1
```

如果要测试官方 metadata 判断路径：

```bash
roslaunch realsense2_camera rs_camera.launch emitter_on_off_use_metadata:=true
```

若出现如下日志：

```text
emitter_on_off metadata unavailable ... falling back to alternating phase
```

说明当前设备 / 固件 / kernel backend 没有返回 `RS2_FRAME_METADATA_FRAME_EMITTER_MODE`。这不是启动失败，驱动会自动 fallback 到同步 frameset 交替相位。由于本机已验证 fallback 效果正确，所以默认关闭 metadata，避免运行时刷 warning。

如果要关闭交替功能，使用普通深度 / 红外输出：

```bash
roslaunch realsense2_camera rs_camera.launch emitter_on_off:=false
```

如果要关闭激光：

```bash
roslaunch realsense2_camera rs_camera.launch emitter_on_off:=false enable_emitter:=false
```

## 7. 常见问题

### 7.1 realsense-viewer 版本不对

现象：

```bash
realsense-viewer --version
```

显示旧版本，例如 `2.30`。

处理：

```bash
which realsense-viewer
ldd $(which realsense-viewer) | grep librealsense
sudo ldconfig
```

也可直接运行源码 build 目录中的 viewer：

```bash
cd /home/taolin/Documents/GitHub/Sunray_V2_PR/drivers/librealsense
LD_LIBRARY_PATH=$PWD/build/Release ./build/Release/realsense-viewer
```

### 7.2 udev 规则提示过期

现象：

```text
RealSense UDEV-Rules file is not up-to date
```

处理：

```bash
cd /home/taolin/Documents/GitHub/Sunray_V2_PR/drivers/librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

然后重新插拔相机。

### 7.3 Device or resource busy

说明相机被其他进程占用：

```bash
fuser -v /dev/video0 /dev/video1 /dev/video2 /dev/video3 /dev/video4 /dev/video5
```

关闭占用进程后重新启动 launch。

### 7.4 metadata unavailable

本机 FW `5.16.0.1` + librealsense `2.55.1` 测试中，`RS2_FRAME_METADATA_FRAME_EMITTER_MODE` 没有稳定返回。因此默认：

```text
emitter_on_off_use_metadata=false
```

这是预期配置。驱动会使用同步 frameset fallback，相机效果已经验证：

- depth 使用激光增强
- binocular grayscale 无激光光斑

### 7.5 `/camera/depth/image_rect_raw` 没有输出

优先确认：

```bash
rostopic list | grep camera
rostopic hz /camera/depth/image_rect_raw
```

如果使用了自定义参数，确认：

```text
enable_depth=true
depth_fps == infra_fps
emitter_on_off_depth_phase=0 或 1
```

若 depth 无输出而 infra 有输出，尝试翻转相位：

```bash
roslaunch realsense2_camera rs_camera.launch emitter_on_off_depth_phase:=1
```

## 8. 推荐生产启动命令

当前相机上推荐直接使用默认 launch：

```bash
cd /home/taolin/Documents/GitHub/Sunray_V2_PR
source devel/setup.bash
roslaunch realsense2_camera rs_camera.launch
```

等价显式写法：

```bash
roslaunch realsense2_camera rs_camera.launch \
  enable_depth:=true \
  enable_infra1:=true \
  enable_infra2:=true \
  depth_fps:=60 \
  infra_fps:=60 \
  enable_emitter:=true \
  emitter_on_off:=true \
  emitter_on_off_depth_phase:=0 \
  emitter_on_off_use_metadata:=false \
  enable_sync:=true \
  align_depth:=true
```
