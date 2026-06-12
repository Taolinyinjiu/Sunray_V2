# sunray_ugv_control_example 示例说明

这个包集中演示如何通过 `sunray_msgs/UGVControlCMD` 调用 `sunray_ugv_control` 的无人车控制接口。相比 `control/sunray_ugv_control/README.md`，本文更关注每个示例程序怎么启动、会发布什么命令、代码里哪些字段最重要，以及新手二次开发时应该复制哪一段。

这些示例不是底盘驱动，也不是定位模块。它们的角色是“上层任务节点”：

```text
示例节点
  发布 /ugv1/sunray/ugv_control/control_cmd

sunray_ugv_control
  订阅 control_cmd 和 local_odom
  运行状态机和控制器
  发布 /ugv1/sunray/ugv_control/cmd_vel
```

## 1. 运行前准备

运行任意示例前，至少需要先启动：

1. UGV 底盘驱动或仿真环境。
2. 定位模块，保证 `/<agent_key>/sunray/localization/local_odom` 有数据。
3. `sunray_ugv_control`，保证 `/<agent_key>/sunray/ugv_control/control_state` 正常发布。

常用启动方式：

```bash
cd /home/amov/Sunray_v2
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch sunray_ugv_control ugv_control.launch \
  agent_name:=ugv \
  agent_id:=1 \
  airframe:=mecanum_default
```

差速底盘示例：

```bash
roslaunch sunray_ugv_control ugv_control.launch \
  agent_name:=ugv \
  agent_id:=1 \
  airframe:=differential_default
```

启动后建议先看状态：

```bash
rostopic echo /ugv1/sunray/ugv_control/control_state
rostopic echo /ugv1/sunray/ugv_control/cmd_vel
```

如果 `control_state.odom_valid=false`，说明控制器还没有收到定位，`MOVE_POINT` 无法正常闭环。

## 2. 示例 launch 参数

所有示例都有：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `agent_name` | `ugv` | 无人车名字前缀 |
| `agent_id` | `1` | 无人车编号 |
| `node_name` | 各示例自己的节点名 | ROS 节点名 |

代码会把 `agent_name + agent_id` 组合成话题前缀。例如：

```text
agent_name:=ugv
agent_id:=1
```

得到：

```text
/ugv1/sunray/ugv_control/control_cmd
```

## 3. 命令发布频率规则

UGV 控制命令分两类：

| 命令 | 发布方式 | 说明 |
| --- | --- | --- |
| `HOLD` | 发一次 | 停车；控制器收到后会持续发布零速度 |
| `MOVE_POINT` | 发一次 | 点位目标；控制器内部持续跟踪目标直到到点 |
| `MOVE_VELOCITY` | 持续发布 | 世界系速度命令，适合麦克纳姆轮；差速轮默认拒绝 |
| `MOVE_VELOCITY_BODY` | 持续发布 | 车体系速度命令，语义和底盘 `/cmd_vel` 一致 |
| `MOVE_WGS84` | 暂不建议使用 | 当前 UGV 控制器仅保留接口，不执行 |

速度类命令必须持续发布，并且每次发布都要更新：

```cpp
cmd.header.stamp = ros::Time::now();
```

如果忘记填写 `header.stamp`，`sunray_ugv_control` 会拒绝速度命令并切换 `HOLD`，同时在 `UGVControlState.diagnostic_msg` 中提示原因。

## 4. 示例总览

| 示例 | launch | 源码 | 用途 |
| --- | --- | --- | --- |
| HOLD | `launch/basic/hold.launch` | `src/basic/hold.cpp` | 发布一次停车命令 |
| MOVE_POINT | `launch/MOVE_POINT/move_point.launch` | `src/MOVE_POINT/move_point.cpp` | 发布一次本地目标点 |
| MOVE_VELOCITY_BODY | `launch/MOVE_VELOCITY_BODY/move_velocity_body.launch` | `src/MOVE_VELOCITY_BODY/move_velocity_body.cpp` | 按固定频率持续发布车体系速度 |

## 5. HOLD 示例

启动：

```bash
roslaunch examples/sunray_ugv_control_example/launch/basic/hold.launch \
  agent_name:=ugv \
  agent_id:=1
```

用途：发布一次 `HOLD`，让 `sunray_ugv_control` 进入停车状态。

核心代码：

```cpp
sunray_msgs::UGVControlCMD cmd;
cmd.header.stamp = ros::Time::now();
cmd.cmd_source = sunray_msgs::UGVControlCMD::EXAMPLE_DEMO;
cmd.control_cmd = sunray_msgs::UGVControlCMD::HOLD;
cmd_pub.publish(cmd);
```

二次开发时，如果你的任务结束或异常退出，建议主动发布一次 `HOLD`。

## 6. MOVE_POINT 示例

启动：

```bash
roslaunch examples/sunray_ugv_control_example/launch/MOVE_POINT/move_point.launch \
  agent_name:=ugv \
  agent_id:=1 \
  target_x:=1.0 \
  target_y:=0.0 \
  target_yaw_deg:=0.0
```

用途：发布一次 `MOVE_POINT`，控制车移动到本地世界坐标系目标点。

参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `target_x` | `1.0` | 本地世界坐标系 x，单位 m |
| `target_y` | `0.0` | 本地世界坐标系 y，单位 m |
| `target_yaw_deg` | `0.0` | 目标 yaw，单位 deg，源码中转换成 rad |

核心代码：

```cpp
cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_POINT;
cmd.desired_pos.x = target_x;
cmd.desired_pos.y = target_y;
cmd.desired_pos.z = 0.0;
cmd.desired_yaw = degToRad(target_yaw_deg);
cmd_pub.publish(cmd);
```

`MOVE_POINT` 发一次即可。到点判定由 `sunray_ugv_control` 配置中的 `point_pos_tolerance` 和 `point_yaw_tolerance` 决定。

## 7. MOVE_VELOCITY_BODY 示例

启动：

```bash
roslaunch examples/sunray_ugv_control_example/launch/MOVE_VELOCITY_BODY/move_velocity_body.launch \
  agent_name:=ugv \
  agent_id:=1 \
  vx:=0.2 \
  vy:=0.0 \
  wz_deg_s:=0.0 \
  duration:=3.0 \
  publish_hz:=20.0
```

用途：按固定频率持续发布车体系速度，持续 `duration` 秒后自动发布 `HOLD`。

参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `vx` | `0.2` | 车体前向速度，单位 m/s |
| `vy` | `0.0` | 车体横向速度，单位 m/s；差速车会忽略 |
| `wz_deg_s` | `0.0` | yaw 角速度，单位 deg/s，源码中转换成 rad/s |
| `duration` | `3.0` | 持续发布时间，单位 s |
| `publish_hz` | `20.0` | 发布频率，单位 Hz |

核心代码：

```cpp
while (ros::ok() && ros::Time::now() < end_time) {
  sunray_msgs::UGVControlCMD cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.cmd_source = sunray_msgs::UGVControlCMD::EXAMPLE_DEMO;
  cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY;
  cmd.cmd_vel.linear.x = vx;
  cmd.cmd_vel.linear.y = vy;
  cmd.cmd_vel.angular.z = degToRad(wz_deg_s);
  cmd_pub.publish(cmd);
  rate.sleep();
}
```

这个示例最适合学习“速度命令为什么要持续发布”。如果你把循环删掉只发布一次，控制器会在 `wait_velcmd_time` 后自动切换 `HOLD`。

## 8. 差速轮和麦克纳姆轮差异

麦克纳姆轮：

- 支持 `MOVE_POINT`
- 支持 `MOVE_VELOCITY`
- 支持 `MOVE_VELOCITY_BODY`
- 支持 `cmd_vel.linear.y`

差速轮：

- 支持 `MOVE_POINT`
- 支持 `MOVE_VELOCITY_BODY` 中的 `linear.x` 和 `angular.z`
- 不支持 `MOVE_VELOCITY`
- 不支持 `cmd_vel.linear.y`

如果差速轮收到 `linear.y`，控制器会忽略它，并在 `control_state.diagnostic_msg` 中提示。

## 9. 常见问题

### 9.1 示例启动后车不动

先检查：

```bash
rostopic echo /ugv1/sunray/localization/local_odom
rostopic echo /ugv1/sunray/ugv_control/control_state
rostopic echo /ugv1/sunray/ugv_control/cmd_vel
```

如果 `odom_valid=false`，说明没有定位输入。
如果 `cmd_vel` 有输出但底盘不动，说明问题可能在底盘驱动、仿真订阅话题或 remap。

### 9.2 速度示例很快停车

确认示例是否在持续运行，并查看：

```bash
rostopic echo /ugv1/sunray/ugv_control/control_state/diagnostic_msg
```

如果提示速度命令超时，说明发布频率太低或示例已经结束。
如果提示 `header.stamp is zero`，说明你自己的二次开发代码没有填写 `cmd.header.stamp`。

### 9.3 差速车设置 vy 没效果

这是正常行为。差速车无法横移，`MOVE_VELOCITY_BODY.linear.y` 会被忽略。差速车建议只使用：

```text
cmd_vel.linear.x
cmd_vel.angular.z
```

## 10. 二次开发建议

如果你要写自己的 UGV 任务节点：

1. 从 `move_point.cpp` 或 `move_velocity_body.cpp` 复制最接近的示例。
2. 保留 `agent_name/agent_id` 参数，避免代码写死 `/ugv1`。
3. 事件型命令发一次，速度型命令持续发布。
4. 速度型命令每帧都填 `header.stamp`。
5. 订阅 `UGVControlState`，优先查看 `diagnostic_level` 和 `diagnostic_msg` 做排错。
