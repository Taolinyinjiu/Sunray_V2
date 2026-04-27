# sunray_ugv_control 技术文档

本文档面向已经离开这部分代码一段时间、希望快速重新建立理解的开发者。重点说明 `sunray_ugv_control` 当前实现的模块结构、数据流、状态机逻辑、控制器细节、配置参数以及几个容易踩坑的行为。

## 1. 模块定位

`sunray_ugv_control` 是一个基于 ROS 的无人车局部控制模块，职责可以概括为两层：

1. `UGVControlFSM`
负责接入外部控制指令、接收定位状态、执行安全监督、决定当前处于哪种控制状态，并发布到底盘的 `cmd_vel`。

2. `UGVController`
负责把“当前应该怎么动”转换成具体的 `geometry_msgs/Twist` 控制量。它不关心安全策略，只关心根据当前状态和目标计算输出。

这个包当前只做局部坐标系控制，不做全局路径规划。`MOVE_WGS84` 只是保留了消息接口，控制上没有实际执行。

## 2. 代码结构

核心文件如下：

- `src/ugv_control_node.cpp`
节点入口，创建 `UGVControlFSM`，启动 `AsyncSpinner(2)`，主线程周期调用 `fsm.process()`。

- `src/ugv_control_fsm.cpp`
状态机主体，负责参数加载、话题收发、安全检查、状态切换、控制输出调度。

- `src/ugv_controller.cpp`
底层控制器实现，支持点位控制、速度控制、车体系速度直通控制。

- `include/sunray_ugv_control/ugv_control_fsm.h`
FSM 相关配置结构体、成员变量、接口声明。

- `include/sunray_ugv_control/ugv_controller.h`
控制器配置、PID 结构、状态缓存、控制接口声明。

- `include/sunray_ugv_control/ugv_control_utils.h`
与 ROS 无关的工具函数，主要是坐标变换、限幅、角度归一化、地理围栏判断。

- `include/sunray_ugv_control/ugv_param_utils.h`
读取 `ugv_name/ugv_id` 或 `ugv_ns`，组装命名空间。

- `config/sunray_control_config.yaml`
运行参数，包括话题名、频率、围栏、返航点、超时阈值、PID 参数和限幅。

- `launch/ugv_control.launch`
启动入口。

- `test/ugv_controller_test.cpp`
控制器测试，覆盖坐标转换后的速度输出、HOLD 输出和底盘限幅等行为。

- `test/ugv_control_fsm_test.cpp`
FSM 级 rostest，覆盖 odom 状态门控、home 初始化、外部命令切状态、`MOVE_POINT` 到点自动 HOLD 等行为。

- `test/ugv_control_test_config.yaml`
测试专用配置。它会把 `wait_poscmd_time` 放宽到 30s，避免点位到达测试被命令超时抢先触发。

## 3. 节点与线程模型

程序入口在 `ugv_control_node`。

启动后会有三条主要执行路径：

1. ROS 回调线程
由 `ros::AsyncSpinner(2)` 驱动，处理里程计、定位状态、控制指令回调。

2. FSM 监督主循环
主线程按 `supervisor_update_frequency` 运行 `UGVControlFSM::process()`，默认 20Hz。

3. 控制定时器
`UGVControlFSM::control_timer_callback()` 按 `controller_update_frequency` 运行，默认 100Hz，负责真正发布控制指令。

另外 `UGVController` 内部还有一个 100Hz 定时器，发布控制器状态调试消息。

这套设计的核心思想是：

- 回调线程只更新共享状态，不直接做复杂控制。
- 低频主循环专门做安全监督。
- 高频定时器专门做控制计算和 `cmd_vel` 输出。

因此，FSM 中很多成员都受 `mutex_` 保护。

## 4. 启动与参数装载

启动文件见 [ugv_control.launch](/home/yyf/Sunray_v2/control/sunray_ugv_control/launch/ugv_control.launch)。

默认传入：

- `ugv_name=ugv`
- `ugv_id=1`
- `config_yamlfile_path=$(find sunray_ugv_control)/config/sunray_control_config.yaml`

命名空间解析逻辑在 [ugv_param_utils.h](/home/yyf/Sunray_v2/control/sunray_ugv_control/include/sunray_ugv_control/ugv_param_utils.h)：

- 优先读 `~ugv_ns` 或 `/ugv_ns`
- 如果没有，再读 `~ugv_name + ~ugv_id` 或 `/ugv_name + /ugv_id`
- 最终得到规范化命名空间，例如 `ugv1`

YAML 加载逻辑在 `UGVControlFSM::load_config()`，会把诸如 `${ugv_ns}/cmd_vel` 这样的占位符替换成实际命名空间。

## 5. 外部接口

### 5.1 输入话题

1. 里程计

- 默认话题：`${ugv_ns}/sunray/localization/local_odom`
- 类型：`nav_msgs/Odometry`
- 用途：提供当前位置、速度、姿态 yaw

2. 定位健康状态

- 默认话题：`${ugv_ns}/sunray/localization/odom_status`
- 类型：`sunray_msgs/OdomStatus`
- 用途：辅助判断定位是否可用

3. 控制指令

- 默认话题：`${ugv_ns}/sunray/ugv_control/control_cmd`
- 类型：`sunray_msgs/UGVControlCMD`
- 用途：外部模块下发控制模式和目标

### 5.2 输出话题

1. 底盘速度指令

- 默认话题：`${ugv_ns}/cmd_vel`
- 类型：`geometry_msgs/Twist`
- 用途：真正输出到底盘驱动

2. FSM 状态

- 默认话题：`${ugv_ns}/sunray/ugv_control/ugv_control_fsm_state`
- 类型：`sunray_msgs/UGVControlFSMState`
- 用途：给 UI、调试、系统监控看当前状态

3. 控制器状态

- 默认话题：`${ugv_ns}/sunray/ugv_control/ugv_controller_state`
- 类型：`sunray_msgs/UGVControllerState`
- 用途：给调试使用，查看期望值、当前值、误差和最后输出

## 6. 消息语义

### 6.1 UGVControlCMD

定义见 [UGVControlCMD.msg](/home/yyf/Sunray_v2/common/sunray_msgs/msg/UGVControlCMD.msg)。

关键字段：

- `cmd_source`
表示指令来源，例如 RC、地面站、规划器等。

当前实现里，这个字段只是语义标记，`sunray_ugv_control` 没有基于它做权限判断、优先级仲裁或控制分支。

- `control_cmd`
控制模式枚举：
`HOLD`、`RETURN`、`MOVE_POINT`、`MOVE_VELOCITY`、`MOVE_VELOCITY_BODY`、`MOVE_WGS84`

- `desired_pos`
点位控制目标

- `desired_vel`
ENU 坐标系下的速度目标

- `desired_linear`、`desired_angular`
车体系速度目标

- `desired_yaw`
目标航向

- `desired_wgs84_pos`
预留接口，当前不执行

### 6.2 UGVControllerState

定义见 [UGVControllerState.msg](/home/yyf/Sunray_v2/common/sunray_msgs/msg/UGVControllerState.msg)。

它主要是控制器内部快照，包含：

- 控制模式
- 期望位置、速度、车体系速度、yaw
- 当前位置、速度、yaw
- 位置误差、速度误差、yaw 误差
- 最近一次输出的 `cmd_vel`

### 6.3 UGVControlFSMState

定义见 [UGVControlFSMState.msg](/home/yyf/Sunray_v2/common/sunray_msgs/msg/UGVControlFSMState.msg)。

它把两类信息拼在一起：

- FSM 侧的安全和模式信息
- Controller 侧的目标和状态信息

因此它适合做总览调试，不适合做控制闭环输入。

## 7. 总体数据流

可以把整个模块理解成下面这条链路：

```text
外部模块发布 UGVControlCMD
        |
        v
UGVControlFSM::control_cmd_callback
        |
        | 仅缓存 active_cmd_ 并切换 state_
        v
UGVControlFSM::control_timer_callback (100Hz)
        |
        | 读取 last_odom_ / active_cmd_ / state_
        | 调用 UGVController
        v
UGVController::move_xxx()
        |
        v
生成 geometry_msgs/Twist
        |
        v
发布到 ${ugv_ns}/cmd_vel
```

与这条主链并行的还有两条辅助链：

```text
Odometry -> odom_callback -> 更新 last_odom_ / 围栏状态 / 尝试初始化 home
OdomStatus -> odom_status_callback -> 更新 odom_status_received_ / odom_status_valid_
```

其中自动 home 初始化比普通 odom 有效性更严格：必须已经收到有效 `OdomStatus`，并且本地 odom 未超时，`odom_callback()` 才会使用当前里程计初始化 home。

以及一条安全监督链：

```text
UGVControlFSM::process() (20Hz)
    -> 检查里程计超时
    -> 检查围栏
    -> 检查控制指令超时
    -> 必要时 enter_hold()
    -> 发布 UGVControlFSMState
```

## 8. FSM 设计

### 8.1 状态枚举

FSM 状态定义在 [ugv_control_fsm.h](/home/yyf/Sunray_v2/control/sunray_ugv_control/include/sunray_ugv_control/ugv_control_fsm.h)：

- `INIT`
初始化状态

- `HOLD`
静止安全状态

- `RETURN`
返航状态

- `MOVE`
统一承接所有外部运动指令

注意，FSM 没有把 `MOVE_POINT`、`MOVE_VELOCITY`、`MOVE_VELOCITY_BODY` 分成多个子状态，而是统一放在 `MOVE` 下，再由 `active_cmd_.control_cmd` 决定具体执行哪条控制分支。

### 8.2 控制指令回调怎样切状态

逻辑在 `UGVControlFSM::control_cmd_callback()`：

- 收到 `HOLD`，直接切到 `HOLD`
- 收到 `RETURN`，切到 `RETURN`
- 收到 `MOVE_POINT`、`MOVE_VELOCITY`、`MOVE_VELOCITY_BODY`、`MOVE_WGS84`，切到 `MOVE`
- 收到未知命令，退回 `HOLD`

如果指令时间戳是零，会被补成 `ros::Time::now()`。

### 8.3 高频控制输出

真正的控制动作发生在 `UGVControlFSM::control_timer_callback()`。

执行顺序如下：

1. 从共享状态中复制快照
2. 调用 `controller_->set_current_state(current_state)`
3. 如果 odom 无效，直接发布零速度并返回
4. 根据 FSM 状态选择控制分支

各分支行为：

1. `INIT`
执行 `controller_->hold()` 并发布零速度 `cmd_vel`

2. `HOLD`
执行 `controller_->hold()` 并发布零速度 `cmd_vel`

3. `RETURN`
如果 home 尚未初始化，执行 `hold()` 并发布零速度。home 可用时，把 home 点包装成一个 `MOVE_POINT` 命令，然后调用 `controller_->move_point(return_cmd)`，发布到 `cmd_vel`。如果 `controller_->reached_point(return_cmd)` 返回真，则切回 `HOLD`。

4. `MOVE`
根据 `active_cmd_.control_cmd` 进一步分发：

- `MOVE_POINT` -> `controller_->move_point(active_cmd)`，到点后自动切回 `HOLD`
- `MOVE_VELOCITY` -> `controller_->move_velocity(active_cmd)`
- `MOVE_VELOCITY_BODY` -> `controller_->move_velocity_body(active_cmd)`
- `MOVE_WGS84` -> 当前不执行，执行 `hold()` 并发布零速度

### 8.4 低频安全监督

`UGVControlFSM::process()` 负责三件事：

1. 定位有效性检查
如果当前状态是 `MOVE` 或 `RETURN`，且 odom 无效，则 `enter_hold("odometry timeout")`

2. 围栏检查
如果越界，则 `enter_hold("geo fence violated")`

3. 控制指令时效检查
如果当前状态是 `MOVE`，且是运动指令，但命令超时，则 `enter_hold("control command timeout")`

最后发布 FSM 状态消息。

### 8.5 `enter_hold()` 的作用

`enter_hold()` 做三件事：

1. 把 FSM 状态切成 `HOLD`
2. 把 `active_cmd_` 替换成内部构造的 `HOLD` 命令
3. 立即发布零速度 `cmd_vel`

因此它是整个模块统一的安全回退入口。

## 9. home 点与返航逻辑

home 点支持两种来源：

1. YAML 固定配置
当 `use_current_pose_as_home=false` 时，直接用 `home_x/home_y/home_z/home_yaw`

2. 首帧有效定位自动记录
当 `use_current_pose_as_home=true` 时，必须先收到有效的 `OdomStatus`，随后 `odom_callback()` 才会把第一帧有效里程计的位置和 yaw 记为 home。只有收到普通 `Odometry` 但没有收到有效 `OdomStatus` 时，home 不会初始化。

返航逻辑很简单：

- `RETURN` 状态下，不走特殊控制器
- 只是把 home 点转换成一个内部 `MOVE_POINT`
- 达到阈值后切回 `HOLD`
- 如果 home 尚未初始化，则保持 `HOLD` 输出，不会盲目向默认 home 运动

## 10. 定位有效性判断

控制环路里的 odom 有效性不是只看一个地方，而是两级判断：

1. 本地超时判断
`now - last_odom_stamp_ <= odom_timeout`

2. 外部 `OdomStatus` 判断
如果收到了 `OdomStatus`，则要求：

- `has_odometry == true`
- `odom_timeout == false`

最终逻辑在 `odom_is_valid_locked()`：

- 如果收到了 `OdomStatus`，优先相信它，但仍叠加本地超时判断
- 如果没收到 `OdomStatus`，只用本地超时判断

默认 `odom_timeout=0.5s`。

需要注意：这是“能不能控制运动”的一般 odom 有效性判断。自动 home 初始化额外要求 `odom_status_received_ == true`，所以系统启动后必须等到有效 `OdomStatus` 之后，才允许把当前里程计记录为 home。

## 11. 地理围栏

围栏是轴对齐长方体，参数在 `geo_fence_param` 中：

- `x_min/x_max`
- `y_min/y_max`
- `z_min/z_max`

判断函数在 [ugv_control_utils.h](/home/yyf/Sunray_v2/control/sunray_ugv_control/include/sunray_ugv_control/ugv_control_utils.h) 的 `check_geo_fence()`。

围栏状态在每次 `odom_callback()` 中更新为 `inside_geo_fence_`，随后由低频监督循环决定是否切 `HOLD`。

## 12. 控制器实现细节

### 12.1 控制器的边界

`UGVController` 不负责：

- 订阅任何输入话题
- 决定状态机该切到哪
- 判断指令是否过期
- 判断围栏越界

它只负责：

- 接收当前状态 `set_current_state()`
- 根据控制模式计算一个 `Twist`
- 对外发布调试状态

### 12.2 PIDAxis

每个控制轴都对应一个简单 PID：

- `kp`
- `ki`
- `kd`

`update(error, dt)` 内部维护积分和微分项。

`set_mode()` 在控制模式切换时会把所有 PID 清零，避免不同模式之间积分串扰。

### 12.3 HOLD

`hold()` 的行为：

- 切换控制模式到 `CONTROL_HOLD`
- 把当前位置冻结为 `desired_pos_`
- 把当前 yaw 冻结为 `desired_yaw_`
- 清空期望速度
- 输出零 `Twist`

这能保证调试状态消息看起来稳定，也便于 UI 看到“当前静止参考点”。

### 12.4 点位控制 `move_point()`

这是一个二维平面位置闭环：

1. 先计算 ENU 下的位置误差
2. 用 `enu_to_body()` 变到车体系
3. 分别对车体系前向误差、侧向误差、yaw 误差做 PID
4. 得到 `linear.x`、`linear.y`、`angular.z`
5. 经过平台限幅和底盘类型适配

这样做的好处是：

- 控制量和底盘运动方向直接对齐
- 对麦轮和差速底盘都更直观

### 12.5 ENU 速度控制 `move_velocity()`

这是速度闭环，不是速度直通。

执行流程：

1. 将 `desired_vel` 从 ENU 转到车体系
2. 将当前速度也从 ENU 转到车体系
3. 将车体系期望速度作为前馈，再叠加速度误差 PID 修正
4. yaw 单独闭环到 `desired_yaw`
5. 输出 `cmd_vel`

因此 `MOVE_VELOCITY` 的语义是：

- 输入是世界系期望速度
- 控制内部转成车体系，输出为速度前馈 + 闭环误差修正

### 12.6 车体系速度控制 `move_velocity_body()`

这个模式不做 PID 整形，基本是直接下发：

- `desired_linear` 直接给 `Twist.linear`
- `desired_angular` 直接给 `Twist.angular`

只会做平台限幅和底盘类型处理。

同时，控制器会把这组车体系速度再转成 ENU 速度，填到 `desired_vel_`，便于状态消息统一展示。

### 12.7 到点判断 `reached_point()`

判断逻辑：

- 平面位置误差 `hypot(dx, dy) <= goal_pos_tolerance`
- 偏航误差 `abs(normalize_angle(dyaw)) <= goal_yaw_tolerance`

默认阈值来自 YAML：

- `goal_pos_tolerance = 0.15`
- `goal_yaw_tolerance = 0.20`

注意，这个函数当前在 `RETURN` 和外部 `MOVE_POINT` 分支里被 FSM 用来自动切回 `HOLD`。

## 13. 麦轮与差速底盘差异

平台类型由 `ugv_types` 决定：

- `0` 对应 `MECANUM`
- `1` 对应 `DIFFERENTIAL`

底层差异集中在 `apply_platform_limits()`：

1. 麦轮
保留 `linear.x`、`linear.y`、`angular.z`

2. 差速
不能直接执行 `linear.y`

当前处理方式是：

- 若是差速底盘，`linear.y` 被置零
- 非车体直通模式下，把侧向需求按 `lateral_to_yaw_gain` 折算到 `angular.z`
- 车体直通模式下，不额外补偿侧向速度

这意味着：

- 对差速底盘，`MOVE_POINT` 和 `MOVE_VELOCITY` 会尝试用转向逼近侧向目标
- 对差速底盘，`MOVE_VELOCITY_BODY` 中的 `linear.y` 会被直接丢弃

## 14. 命令时效与当前行为陷阱

### 14.1 命令时效规则

逻辑在 `is_command_fresh()`：

- `MOVE_POINT`、`MOVE_WGS84` 使用 `wait_poscmd_time`
- `MOVE_VELOCITY`、`MOVE_VELOCITY_BODY` 使用 `wait_velcmd_time`

默认值：

- `wait_poscmd_time = 2.0s`
- `wait_velcmd_time = 0.3s`

### 14.2 一个很关键的当前实现特征

`MOVE_POINT` 现在是“单次点位命令，在有效期内持续跟踪；到点自动 HOLD；若过期仍未到点也 HOLD”语义。

具体表现：

- 收到一条 `MOVE_POINT`
- FSM 切到 `MOVE`
- 高频控制器持续按这条命令算 `cmd_vel`
- 如果 `reached_point()` 返回真，控制定时器会 `enter_hold("move point target reached")`
- 如果 `当前时间 - 命令时间戳 > wait_poscmd_time`，低频监督循环会 `enter_hold("control command timeout")`

因此，如果车比较慢、目标比较远，或者控制增益偏保守，机器人仍可能还没走到目标点就因为命令超时进入 `HOLD`；正常到点时则会立即自动 HOLD。

### 14.3 `RETURN` 和 `MOVE_POINT` 的差异

当前代码里，`RETURN` 和 `MOVE_POINT` 都会调用 `reached_point()`，真正到目标点后切回 `HOLD`。差异在于 `RETURN` 的目标来自 home 点，`MOVE_POINT` 的目标来自外部控制指令。

## 15. 状态消息如何理解

### 15.1 `UGVControllerState`

适合看：

- 控制器当前使用哪种模式
- 期望值和当前值差多少
- 最近一次发了什么 `cmd_vel`

### 15.2 `UGVControlFSMState`

适合看：

- 当前 FSM 状态是 `INIT/HOLD/RETURN/MOVE`
- 当前正在执行哪条 `active_control_cmd`
- 定位是否有效
- 控制命令是否仍有效
- 是否在围栏内
- 当前 home 点是什么

如果只看一个调试话题，优先看 `UGVControlFSMState`。

## 16. 配置参数速查

配置文件见 [sunray_control_config.yaml](/home/yyf/Sunray_v2/control/sunray_ugv_control/config/sunray_control_config.yaml)。

### 16.1 `basic_param`

- `ugv_types`
底盘类型，`0` 麦轮，`1` 差速

- `controller_update_frequency`
高频控制输出频率，默认 100Hz

- `supervisor_update_frequency`
低频监督循环频率，默认 20Hz

- `controller_state_pub_frequency`
控制器状态发布频率，默认 100Hz

- 各类 `*_topic_name`
输入输出话题名模板，支持 `${ugv_ns}` 占位符

### 16.2 `timeout_param`

- `wait_poscmd_time`
点位类命令允许陈旧多久

- `wait_velcmd_time`
速度类命令允许陈旧多久

- `odom_timeout`
本地里程计超时阈值

### 16.3 `geo_fence_param`

三维轴对齐围栏范围

### 16.4 `home_param`

- `use_current_pose_as_home`
是否将第一帧满足条件的定位作为 home。这里的“满足条件”指已经收到有效 `OdomStatus`，并且本地 odom 未超时；单独收到 `Odometry` 不会初始化 home。

- `home_x/home_y/home_z/home_yaw`
固定 home 配置

### 16.5 `controller_param`

- `point_x_*`、`point_y_*`、`point_yaw_*`
点位控制 PID

- `vel_x_*`、`vel_y_*`、`vel_yaw_*`
速度控制 PID

- `lateral_to_yaw_gain`
差速底盘侧向需求折算为角速度的增益

- `max_linear_x`
- `max_linear_y`
- `max_angular_z`
输出限幅

- `goal_pos_tolerance`
- `goal_yaw_tolerance`
到点阈值

## 17. 测试与验证

当前包已经接入 catkin/rostest 测试：

- `ugv_controller_test`
覆盖控制器输出语义，包括 `MOVE_VELOCITY` 的“车体系期望速度前馈 + PID 修正”、`MOVE_VELOCITY_BODY` 的直通限幅、`HOLD` 零速度输出等。

- `ugv_control_fsm_test`
覆盖 FSM 行为，包括有效 `OdomStatus` 之前不初始化 home、无效 odom 进入 HOLD、外部 `HOLD` 立即零速度、`MOVE_POINT` 到点自动切回 HOLD 等。

这个仓库不是标准 `src/` 工作空间。单独构建本包时应从仓库根目录使用：

```bash
catkin_make --source control/sunray_ugv_control --build build/sunray_ugv_control
```

运行本包测试时使用：

```bash
ROS_LOG_DIR=/tmp/sunray_ros_logs catkin_make --source control/sunray_ugv_control --build build/sunray_ugv_control run_tests_sunray_ugv_control
catkin_test_results build/sunray_ugv_control/test_results
```

## 18. 重新看代码时的推荐阅读顺序

如果过了很久再回来，建议按这个顺序看：

1. [ugv_control.launch](/home/yyf/Sunray_v2/control/sunray_ugv_control/launch/ugv_control.launch)
先知道节点怎么起、参数从哪里来

2. [sunray_control_config.yaml](/home/yyf/Sunray_v2/control/sunray_ugv_control/config/sunray_control_config.yaml)
先知道话题和参数默认值

3. [ugv_control_node.cpp](/home/yyf/Sunray_v2/control/sunray_ugv_control/src/ugv_control_node.cpp)
知道程序有哪些执行线程

4. [ugv_control_fsm.h](/home/yyf/Sunray_v2/control/sunray_ugv_control/include/sunray_ugv_control/ugv_control_fsm.h)
先看配置结构和核心成员

5. [ugv_control_fsm.cpp](/home/yyf/Sunray_v2/control/sunray_ugv_control/src/ugv_control_fsm.cpp)
重点看 `load_config()`、`control_cmd_callback()`、`control_timer_callback()`、`process()`

6. [ugv_controller.h](/home/yyf/Sunray_v2/control/sunray_ugv_control/include/sunray_ugv_control/ugv_controller.h)
知道控制器内部缓存了哪些状态

7. [ugv_controller.cpp](/home/yyf/Sunray_v2/control/sunray_ugv_control/src/ugv_controller.cpp)
重点看 `move_point()`、`move_velocity()`、`move_velocity_body()`、`apply_platform_limits()`

8. [UGVControlCMD.msg](/home/yyf/Sunray_v2/common/sunray_msgs/msg/UGVControlCMD.msg)、[UGVControllerState.msg](/home/yyf/Sunray_v2/common/sunray_msgs/msg/UGVControllerState.msg)、[UGVControlFSMState.msg](/home/yyf/Sunray_v2/common/sunray_msgs/msg/UGVControlFSMState.msg)
最后再看消息定义，对照调试话题理解字段含义

## 19. 当前实现的边界与后续改进点

当前实现已经可用，但有几个明确边界：

1. `cmd_source` 未参与实际控制逻辑
没有优先级仲裁，也没有来源鉴权

2. `MOVE_WGS84` 仅保留接口
当前收到该命令不会执行全局导航

3. `MOVE_POINT` 是“单次点位命令，到点自动 HOLD；若长期未到点，过期也 HOLD”语义

4. 没有轨迹规划层
只支持直接跟踪单目标点或速度指令

如果后续要改功能，最值得优先关注的通常是：

- 是否要对 `cmd_source` 做仲裁
- 是否要增加更丰富的调试信息和控制模式

## 20. 一句话总结

`sunray_ugv_control` 当前本质上是一个“FSM 安全壳 + 二维局部控制器”的 ROS 包：FSM 负责决定能不能动、该按哪种模式动，Controller 负责把目标变成 `cmd_vel`，所有调试信息再通过状态消息统一暴露出来。
