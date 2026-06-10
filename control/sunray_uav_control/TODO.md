# sunray_uav_control 优化建议

这份 TODO 来自对当前代码结构、接口和配置的阅读，重点放在二次开发稳定性、可维护性和新手上手成本上。不是所有条目都需要马上做，建议按优先级逐步处理。

## P0：安全与行为一致性

- [ ] 明确 `KILL`、里程计超时、MAVROS 超时的安全策略。
  - 现在 `check_rosmsg_timeout()` 检测到里程计超时时没有真正触发保护动作，只保留了注释。
  - 建议把 `msg_timeout_operate`、`lost_with_rc`、`low_voltage_operate` 这类保护策略接入一个明确的 `system_check` 模块，再由 FSM 消费统一事件。

- [ ] 统一速度命令超时阈值。
  - `check_rosmsg_timeout()` 中控制命令超时写死为 `0.2s`，没有使用 YAML 的 `msg_timeout_param`。
  - 建议配置成 `control_cmd` 超时参数，并在 README 和配置文件中说明。

- [ ] 明确 `MOVE_POINT_WGS84` 当前状态。
  - 消息和 FSM 已有接口，但控制执行中基本未实现。
  - 建议要么完整实现经纬高到 local 目标的转换，要么在收到该命令时明确拒绝并发布错误状态，避免上层误以为支持。

## P1：架构与接口可维护性

- [ ] 拆分 YAML 参数读取，避免 FSM 和控制器重复解析同一个文件。
  - 目前 FSM 读 `basic_param/takeoff_land_param/...`，控制器又分别读取 `basic_param/arrival_judge_param/velocity_param/geometric_controller_param`。
  - 建议建立统一的 `ControlConfig` 结构，启动时读取一次，再传给 FSM 和 Controller。

- [ ] 给 `Controller_Interface` 增加能力声明。
  - 两个控制器都继承同一接口，但不同控制器对速度、body、WGS84、轨迹的支持程度不同。
  - 建议增加 `supports(CommandType)` 或能力位，FSM 在接到命令时能提前拒绝不支持的命令。

- [ ] 给 `UAVControlState` 增加错误码/拒绝原因。
  - 现在状态话题能看到当前状态和最后命令，但看不到“为什么没有转移”或“为什么控制器拒绝命令”。
  - 建议增加 `last_error_code`、`last_error_text` 或结构化诊断字段。

- [ ] 梳理 agent key 获取方式。
  - 现在 `Sunray_FSM` 和 `MavrosHelper` 都根据 `use_private_agent_key` 自己取 agent key。
  - 建议在主节点解析一次，显式传给 FSM、Controller、MavrosHelper，减少全局参数/私有参数混用带来的调试成本。

- [ ] 把状态机事件处理从“每轮只处理一个事件”改成有边界的批处理。
  - 当前 `process_fsm_event_queue()` 每次只 pop 一个事件。
  - 如果短时间内收到多个事件，状态响应会被 `supervisor_update_frequency` 拉长。
  - 建议每轮最多处理 N 个事件，并记录被丢弃/合并的事件。

## P2：可测试性

- [ ] 增加 FSM 状态转移单元测试。
  - 当前测试更多是编译测试和工具测试。
  - 建议覆盖 `OFF->INIT->TAKEOFF->HOVER->MOVE->LAND->INIT`、`KILL` 全局事件、非法状态命令拒绝等。

- [ ] 增加 `UAVControlCMD` 到 controller 调用的映射测试。
  - 可以用 mock controller 验证 `MOVE_POINT`、`MOVE_VELOCITY`、`MOVE_TRAJECTORY`、body 命令是否调用了正确接口。

- [ ] 增加参数校验测试。
  - YAML 字段很多，且有重复读取逻辑。
  - 建议准备最小合法配置和若干非法配置，测试异常信息是否明确。

- [ ] 增加 MAVROS helper 的离线封装测试。
  - 重点验证坐标、mask、type_mask、姿态四元数、时间戳和 `PositionTarget/AttitudeTarget` 转换。

## P3：新手体验与二次开发体验

- [ ] 提供最小控制 demo 包或节点。
  - README 已给出片段，但建议仓库内加入 `simple_takeoff_move_land.cpp`，演示等待 `INIT/HOVER`、发送 `TAKEOFF/MOVE_POINT/LAND` 的完整流程。

- [ ] 给 `control_state` 增加任务进度字段。
  - 例如当前目标点、位置误差、速度误差、yaw 误差、到达判定剩余时间。
  - 这会显著降低调参和二次开发排错成本。

- [ ] 清理历史 TODO 注释。
  - 代码内有一些已经过期或不清晰的 TODO，例如 Raptor、system_check、参数检查迁移等。
  - 建议把仍然有效的 TODO 移到本文件或 issue，把无效 TODO 删除。

## P4：代码质量小项

- [ ] 统一命名风格。
  - 当前存在 `Sunray_FSM`、`Geometric_Controller`、`PX4_OriginController`、`uav_ns_` 等混合风格。
  - 不建议大规模一次性重命名，但新增代码应遵循同一命名约定。

- [ ] 减少 `switch` 分散。
  - 命令枚举到字符串、命令到事件、命令到控制器调用分散在多个文件。
  - 建议抽出轻量工具函数，至少保证新增命令时不会漏改监控和可视化。

- [ ] 统一日志节流策略。
  - 高频控制循环中应避免普通 `ROS_WARN/ROS_INFO`。
  - 建议统一用 `ROS_WARN_THROTTLE`，并标明 agent key 和当前状态。

- [ ] 去掉或配置化 `DEBUG` 编译宏。
  - `CMakeLists.txt` 当前对 `sunray_uav_control_lib` 默认定义 `DEBUG`。
  - 建议改成 CMake option，避免发布版本产生过多日志或性能影响。

- [ ] 检查未使用配置项。
  - `protect_param`、`local_fence_param`、部分 `velocity_param.max_velocity_with_rc` 当前没有完整接入 FSM。
  - 建议要么实现，要么在配置注释中明确“当前未使用”。
