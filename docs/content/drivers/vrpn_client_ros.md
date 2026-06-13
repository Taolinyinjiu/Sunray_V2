<!-- title: vrpn_client_ros -->

<section id="driver-vrpn-client-ros">

## vrpn_client_ros

`drivers/vrpn_client_ros` 是 VRPN 动捕客户端，兼容 VICON、OptiTrack 等系统。它负责从 VRPN server 读取刚体位姿，并发布 ROS pose/twist/TF。

### 启动方式

```bash
roslaunch vrpn_client_ros sample.launch server:=192.168.20.15
```

关键参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `server` | `192.168.20.15` | VRPN server IP。 |
| `port` | `3883` | VRPN 端口。 |
| `update_frequency` | `100.0` | 更新频率。 |
| `frame_id` | `world` | 发布位姿的坐标系。 |
| `broadcast_tf` | `true` | 是否广播 TF。 |

### 和 Sunray 定位链路的关系

典型动捕数据流：

```text
VRPN server
  -> vrpn_client_ros
  -> /vrpn_client_node/uav1/pose, /twist
  -> sunray_mocap
  -> /uav1/sunray/odometry
  -> localization_fusion source_id:=1
```

控制模块一般不直接订阅 VRPN 原始话题，而是使用 `localization_fusion` 的统一输出。

### 二次开发边界

- 刚体名、server IP、更新频率是最常改的内容。
- 多机动捕时，要保证 VRPN 刚体名和 Sunray `agent_name/agent_id` 能对应。
- 坐标轴方向异常时，优先检查动捕系统坐标系和 `sunray_mocap` 适配，不要在控制器里临时取反。

</section>
