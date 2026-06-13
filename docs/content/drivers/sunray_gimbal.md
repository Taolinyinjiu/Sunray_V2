<!-- title: sunray_gimbal -->

<section id="driver-sunray-gimbal">

## sunray_gimbal

`drivers/sunray_gimbal` 用于云台控制和云台图像接入。当前启动文件会同时启动控制节点和 RTSP 图像发布节点。

### 启动方式

```bash
roslaunch sunray_gimbal gimbal_core.launch
```

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gimbal_ip` | `192.168.2.25` | 云台 IP。 |
| `gimbal_port` | `37260` | 云台控制端口。 |
| `rtsp_url` | `rtsp://<ip>:8554/main.264` | 云台视频流地址。 |

### 主要文件

| 文件 | 说明 |
| --- | --- |
| `src/gimbal_control.py` | 云台控制节点。 |
| `src/image_publish.cpp` | RTSP 图像发布节点。 |
| `msg/GimbalParams.msg` | 云台参数消息。 |
| `srv/GimbalReboot.srv` | 云台重启服务。 |
| `srv/GimbalStatus.srv` | 云台状态服务。 |
| `demo/*.cpp` | 角度控制、拍照、扫描、目标锁定等示例。 |

### 二次开发边界

- 若只改云台 IP、端口或视频地址，修改 launch 参数即可。
- 需要把视觉检测和云台联动时，建议任务节点订阅感知结果，再调用云台接口；不要把飞行控制逻辑写进驱动。
- 云台图像可作为 `sunray_perception` 的输入源，注意同步相机内参和图像话题。

</section>
