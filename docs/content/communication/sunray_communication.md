<!-- title: sunray_communication -->

<section id="communication-sunray-communication">

## sunray_communication

`communication/sunray_communication` 当前主要存放 YunLink 相关通信库代码，不是一个独立 ROS 功能包。实际 ROS 节点入口在 `yunlink_ros_bridge`。

### 目录关系

```text
communication/sunray_communication/lib
communication/yunlink_ros_bridge
```

可以把它理解为通信协议和底层库，把 `yunlink_ros_bridge` 理解为 ROS 适配层。

### 二次开发边界

- 如果只是增加 ROS 话题映射、Sunray 控制命令或系统服务桥接，优先修改 `yunlink_ros_bridge`。
- 如果要修改 YunLink 协议本身、序列化格式、网络传输或认证机制，才进入 `sunray_communication/lib`。
- 协议层修改通常需要同步地面站或外部系统，不能只改 ROS 侧。

</section>
