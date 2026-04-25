# Jetson Orin NX: 当前 Sunray_V2 仓库内使用 OpenCV 4.6 CUDA 编译 VINS-Fusion-GPU

## 目标

系统 ROS Noetic 保持 OpenCV 4.2，避免污染 `/usr` 或 `/usr/local`。

单独编译一份 OpenCV 4.6 CUDA：

```bash
/opt/opencv-4.6.0-cuda
```

然后在当前 `Sunray_V2` 仓库内编译：

```text
cv_bridge + vins_fusion_gpu
```

本方案不改变 VINS-Fusion-GPU 的源码位置：

```bash
localization/vins-fusion-gpu
```

也不改变编译输出位置：

```bash
build/vins-fusion-gpu
devel
```

关键原因：系统 ROS 的 `/opt/ros/noetic/lib/libcv_bridge.so` 链接 OpenCV 4.2。如果 VINS 链接 OpenCV 4.6，而 `cv_bridge` 仍然加载 OpenCV 4.2，同一个进程会混两个 OpenCV 版本，容易崩溃。因此必须在当前仓库内重编一份本地 `cv_bridge`，并让它和 VINS 一起链接 `/opt/opencv-4.6.0-cuda`。

## 1. 确认系统 ROS OpenCV 已稳定在 4.2

```bash
pkg-config --modversion opencv4
ls -l /usr/lib/aarch64-linux-gnu/libopencv_core.so
ldd /opt/ros/noetic/lib/libcv_bridge.so | grep opencv
```

期望：

```text
pkg-config -> 4.2.0
libopencv_core.so -> libopencv_core.so.4.2
cv_bridge -> libopencv_*.so.4.2
```

这一步表示系统 ROS 环境是稳定的。后续 OpenCV 4.6 CUDA 只给 VINS/YOLO 临时使用。

## 2. 编译独立 OpenCV 4.6 CUDA

进入当前仓库：

```bash
cd ~/Documents/Sunray_V2
```

执行：

```bash
JOBS=6 ./scripts/build_opencv46_cuda.sh
```

如需重新干净编译：

```bash
CLEAN_BUILD=1 JOBS=6 ./scripts/build_opencv46_cuda.sh
```

脚本会安装到：

```bash
/opt/opencv-4.6.0-cuda
```

成功后应存在：

```bash
/opt/opencv-4.6.0-cuda/setup.sh
/opt/opencv-4.6.0-cuda/lib/cmake/opencv4/OpenCVConfig.cmake
/opt/opencv-4.6.0-cuda/include/opencv4/opencv2/cudaoptflow.hpp
/opt/opencv-4.6.0-cuda/include/opencv4/opencv2/cudaimgproc.hpp
/opt/opencv-4.6.0-cuda/include/opencv4/opencv2/cudaarithm.hpp
```

验证：

```bash
source /opt/opencv-4.6.0-cuda/setup.sh
pkg-config --modversion opencv4
pkg-config --libs opencv4 | grep cuda
```

期望：

```text
4.6.0
```

并且能看到 CUDA 相关 OpenCV 库。

## 3. 确认当前仓库已包含本地 cv_bridge 源码

当前仓库已经固化了 `vision_opencv` 源码：

```bash
localization/vins-fusion-gpu/third_party/vision_opencv
```

该目录不是 git submodule，而是普通源码。固定版本：

```text
vision_opencv 1.16.2
commit 08b012c038e575d7fe1d538f11235a994159dc93
```

这与目标系统上的 `ros-noetic-cv-bridge 1.16.2` 保持一致。

当前构建只需要 `cv_bridge`。`image_geometry`、`opencv_tests` 和 `vision_opencv` metapackage 目录保留源码上下文，但已加入 `CATKIN_IGNORE`，不会被当前 VINS 构建扫描。

确认源码存在：

```bash
cd ~/Documents/Sunray_V2

test -f localization/vins-fusion-gpu/third_party/vision_opencv/cv_bridge/package.xml
test -f localization/vins-fusion-gpu/third_party/vision_opencv/VENDOR_VERSION.md
```

这个操作不会移动 VINS-Fusion-GPU。VINS 仍然在：

```bash
localization/vins-fusion-gpu/vins_estimator
localization/vins-fusion-gpu/camera_models
localization/vins-fusion-gpu/loop_fusion
localization/vins-fusion-gpu/global_fusion
```

只是把 `cv_bridge` 源码放到同一个 catkin source space 下，让它和 VINS 一起编译。

## 4. 清理旧的 VINS 构建缓存

只清理 VINS 相关 build 输出和旧库，保留当前仓库结构：

```bash
cd ~/Documents/Sunray_V2

rm -rf build/vins-fusion-gpu
rm -f devel/lib/vins/vins_node
rm -f devel/lib/libvins_lib.so
rm -f devel/lib/libcamera_models.so
rm -f devel/lib/libcv_bridge.so
```

如果你希望更彻底地清理当前仓库的 devel overlay 中旧 `cv_bridge` 痕迹，可以再执行：

```bash
rm -rf devel/include/cv_bridge
rm -rf devel/share/cv_bridge
rm -f devel/lib/pkgconfig/cv_bridge.pc
```

## 5. 在当前仓库原位置编译 VINS + cv_bridge

环境顺序很重要：

```bash
source /opt/ros/noetic/setup.bash
source /opt/opencv-4.6.0-cuda/setup.sh
```

然后编译：

```bash
cd ~/Documents/Sunray_V2

catkin_make \
  --source localization/vins-fusion-gpu \
  --build build/vins-fusion-gpu \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR=/opt/opencv-4.6.0-cuda/lib/cmake/opencv4 \
  -DVINS_REQUIRE_OPENCV_CUDA=ON
```

这个命令保持：

```bash
source space: localization/vins-fusion-gpu
build space:  build/vins-fusion-gpu
devel space:  devel
```

CMake 成功时应看到：

```text
OpenCV CUDA feature tracker support: enabled
```

如果报错：

```text
OpenCV CUDA feature tracker support is required but missing cudaoptflow/cudaimgproc/cudaarithm headers
```

说明没有真正使用 `/opt/opencv-4.6.0-cuda`，或者 OpenCV 4.6 CUDA 编译不完整。

## 6. 验证是否全部链接到 OpenCV 4.6 CUDA

编译完成后检查：

```bash
ldd devel/lib/libcv_bridge.so | grep opencv
ldd devel/lib/vins/vins_node | grep opencv
ldd devel/lib/libvins_lib.so | grep opencv
ldd devel/lib/libcamera_models.so | grep opencv
```

期望全部指向：

```bash
/opt/opencv-4.6.0-cuda/lib/libopencv_*.so
```

不能出现：

```bash
/lib/aarch64-linux-gnu/libopencv_*.so.4.2
/usr/lib/aarch64-linux-gnu/libopencv_*.so.4.2
```

如果 `vins_node` 或 `libvins_lib.so` 是 4.6，但 `libcv_bridge.so` 是 4.2，不能使用该构建结果。

## 7. 运行 VINS

每次运行前使用这个环境顺序：

```bash
source /opt/ros/noetic/setup.bash
source /opt/opencv-4.6.0-cuda/setup.sh
source ~/Documents/Sunray_V2/devel/setup.bash
```

然后启动：

```bash
roslaunch vins vins_sunray_150.launch
```

或使用你的实际 launch：

```bash
roslaunch vins <your_launch_file>.launch
```

运行前再次确认当前 shell 看到的是仓库里的 VINS：

```bash
which roslaunch
rospack find vins
rospack find cv_bridge
```

期望 `vins` 和 `cv_bridge` 都来自当前 `Sunray_V2/devel` overlay，而不是只有 `/opt/ros/noetic`。

## 8. VINS 配置要求

配置文件中需要打开 GPU：

```yaml
use_gpu: 1
use_gpu_acc_flow: 1
```

如果 OpenCV CUDA 模块存在，VINS 会使用 GPU 光流和 GPU 特征点检测。

当前仓库的 VINS CMake 支持：

```bash
-DVINS_REQUIRE_OPENCV_CUDA=ON
```

开启后如果 CUDA OpenCV 模块缺失，会直接配置失败，而不是静默退回 CPU。

## 9. YOLO 使用 OpenCV 4.6 CUDA

YOLO 如果是独立 CMake 项目，可以直接：

```bash
source /opt/opencv-4.6.0-cuda/setup.sh
```

编译时指定：

```bash
-DOpenCV_DIR=/opt/opencv-4.6.0-cuda/lib/cmake/opencv4
```

编译后检查：

```bash
ldd <your_yolo_binary> | grep opencv
```

期望全部指向：

```bash
/opt/opencv-4.6.0-cuda/lib/libopencv_*.so
```

## 10. 常见问题

### 为什么不使用 build_vins_opencv46_overlay.sh？

`scripts/build_vins_opencv46_overlay.sh` 会创建独立工作区：

```bash
~/vins_opencv46_ws
```

这适合完全隔离的部署方式，但不满足“VINS 仍在当前 Sunray_V2 仓库、编译输出仍在当前仓库”的要求。

本指南使用当前仓库原位置方案：

```bash
localization/vins-fusion-gpu
build/vins-fusion-gpu
devel
```

### 为什么要在 localization/vins-fusion-gpu 下放 vision_opencv？

因为 `catkin_make --source localization/vins-fusion-gpu` 只会扫描这个 source space 下的包。

把 `vision_opencv` 放到：

```bash
localization/vins-fusion-gpu/third_party/vision_opencv
```

可以让 `cv_bridge` 和 VINS 在同一个 source space 里一起编译，最终输出仍然进入当前仓库的 `devel`。

### 为什么固定 vision_opencv 1.16.2？

`1.16.2` 是 ROS Noetic 发布的 `vision_opencv/cv_bridge` 版本，和目标系统中的 `ros-noetic-cv-bridge 1.16.2` 对齐。

使用固定 tag 的源码可以保证可复现，不会因为上游 noetic 分支后续变化导致构建行为变化。

### 会不会影响系统 ROS？

不会修改 `/opt/ros/noetic`。

只有在当前 shell 中执行：

```bash
source /opt/opencv-4.6.0-cuda/setup.sh
source ~/Documents/Sunray_V2/devel/setup.bash
```

才会使用 OpenCV 4.6 CUDA 和本地 `cv_bridge`。

### JOBS=6 是什么？

`JOBS=6` 表示使用 6 个并行任务编译 OpenCV。

Jetson Orin NX 编 OpenCV 很吃内存和温度，`JOBS=6` 比默认用满所有核心更稳。

内存不足时：

```bash
JOBS=4 ./scripts/build_opencv46_cuda.sh
```

散热和 swap 足够时：

```bash
JOBS=8 ./scripts/build_opencv46_cuda.sh
```

## 11. 整体流程

```text
build_opencv46_cuda.sh
        ↓
/opt/opencv-4.6.0-cuda
        ↓
使用仓库内已固化的 localization/vins-fusion-gpu/third_party/vision_opencv
        ↓
catkin_make --source localization/vins-fusion-gpu --build build/vins-fusion-gpu
        ↓
devel/lib/libcv_bridge.so
devel/lib/vins/vins_node
devel/lib/libvins_lib.so
        ↓
当前 Sunray_V2 仓库内运行 VINS CUDA 版本
```
