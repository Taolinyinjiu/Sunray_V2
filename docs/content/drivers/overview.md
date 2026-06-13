<!-- title: 驱动总览 -->

<section id="drivers-overview">

## 驱动总览

`drivers` 目录负责把飞控、雷达、相机、云台、动捕、底盘等硬件或第三方设备接入 ROS。它位于 Sunray 架构的最底层，主要任务是把设备协议转换成 ROS 话题、服务和参数。

对二次开发者来说，驱动模块通常不是业务逻辑入口。优先修改 launch 参数、配置文件、设备端口、话题 remap；只有确认设备协议或驱动行为不满足需求时，才进入驱动源码。

### 架构位置

```text
硬件设备 / 第三方 SDK
  -> drivers/*
  -> ROS 标准话题或 MAVROS 话题
  -> localization / control / perception
  -> sunray_msgs 统一接口
```

典型关系：

- PX4 飞控通过 `sunray_mavros` 接入，控制模块通过 MAVROS 话题和服务与飞控交互。
- Livox 点云通过 `livox_ros_driver2` 接入，FAST-LIO、建图、避障会订阅点云和 IMU。
- RealSense 或 USB 相机通过 `realsense2_camera`、`web_cam` 接入，视觉感知和 VINS 会订阅图像。
- VRPN 动捕通过 `vrpn_client_ros` 接入，再由 `sunray_mocap` 转成 Sunray odom。
- 轮趣底盘通过 `turn_on_wheeltec_robot` 接入，UGV 控制最终输出 `cmd_vel`。

### 功能包列表

| 功能包 | 作用 | 主要输入 | 主要输出 |
| --- | --- | --- | --- |
| `sunray_mavros` | Sunray 对 MAVROS 启动和 PX4 配置的封装 | 飞控串口/UDP、MAVLink | `/uavX/mavros/*` 话题和服务 |
| `livox_ros_driver2` | Livox 雷达 ROS 驱动 | Livox 设备或 lvx 文件 | 点云、IMU、Livox 自定义消息 |
| `realsense2_camera` | Intel RealSense ROS 驱动 | RealSense 相机 | 彩色图、深度图、红外图、点云、IMU |
| `sunray_gimbal` | SIYI/云台控制与图像发布 | 云台 UDP/RTSP | 云台控制服务、状态、图像 |
| `vrpn_client_ros` | VRPN 动捕客户端 | VRPN server | pose、twist、TF |
| `web_cam` | USB 摄像头驱动 | V4L 摄像头 | `sensor_msgs/Image`、CameraInfo |
| `turn_on_wheeltec_robot` | 轮趣底盘驱动和导航相关 launch | 串口、`cmd_vel` | 底盘 odom、电压、TF |
| `librealsense2` | RealSense 底层库 | RealSense USB 设备 | 被 `realsense2_camera` 调用 |

### 开发建议

- 接新硬件时，先让驱动单独跑通，再接入定位、控制或感知。
- 驱动输出尽量保持 ROS 标准消息，例如 `sensor_msgs/Image`、`sensor_msgs/PointCloud2`、`nav_msgs/Odometry`。
- 如果某个驱动原始输出不符合 Sunray 约定，优先新建适配节点，不要让上层控制器直接依赖设备私有协议。
- 真机排错先看设备权限、串口名、IP、波特率、话题是否发布，再看上层模块。

</section>
