# sunray_ugv_control TODO

这个文件记录 `sunray_ugv_control` 后续可以继续优化的方向，主要用于团队开发排期和二次开发参考。当前优先级不是强制要求，只表示建议处理顺序。

## P0 安全和行为一致性

- [x] 地理围栏只发布状态，没有保护动作。
  - 已增加 `enable_geo_fence_protection` 参数。
  - 默认保持旧行为，只发布 `inside_geo_fence`；开启后越界自动切换 HOLD 并发布零速度。
  - 后续仍可扩展更精细策略：仅告警、拒绝继续向外运动、触发上层安全回调。

- [ ] `odom_state_callback()` 已订阅但没有实际逻辑。
  - 当前控制主要依赖是否收到 `local_odom`。
  - 建议根据 `sunray_msgs/OdomState` 判断定位是否健康，定位丢失或状态异常时进入 HOLD。

- [x] 删除底层 `RETURN` 返航接口。
  - `RETURN` 属于任务层/导航层语义，不应由 `sunray_ugv_control` 持有返航点。
  - 已删除 `UGVControlCMD::RETURN`、`UGVControlState::FSM_RETURN`、FSM 返航状态和工具端按钮/显示。
  - 上层需要返航时，应自行保存 home 点，并向本包发布普通 `MOVE_POINT`。

- [ ] `MOVE_WGS84` 只是预留接口。
  - 当前消息和控制面板可以表达该命令，但控制器不执行。
  - 建议二选一：实现经纬高到本地坐标转换并执行，或在状态机和 UI 中明确禁用并给出拒绝原因。

- [x] 速度命令超时依赖 `header.stamp`，缺少明确错误反馈。
  - 已在 `UGVControlState` 增加 `diagnostic_level` 和 `diagnostic_msg`。
  - 已覆盖速度命令 stamp 为 0、速度命令超时、差速底盘不支持世界系速度、差速底盘忽略横向速度、围栏保护触发、未知命令等诊断。
  - 控制面板、终端监控和总监控面板已显示诊断信息。

## P1 接口和状态反馈

- [ ] `UGVControlState` 可以增加任务进度字段。
  - 建议字段：位置误差、yaw 误差、目标距离、速度命令剩余有效时间、最近一次拒绝原因。
  - 这样控制面板、终端监控和二次开发程序能更容易判断“为什么没动”或“是否快到点”。

- [ ] 底盘能力应该显式暴露给上层。
  - 当前差速底盘不支持 `MOVE_VELOCITY`，也不支持 `linear.y`，但上层只能从 warning 或文档知道。
  - 建议在状态消息或服务中暴露 `supports_world_velocity`、`supports_lateral_velocity` 等能力字段。

- [x] 命令拒绝和状态切换原因不够清晰。
  - 已通过 `diagnostic_level` / `diagnostic_msg` 发布最近一次命令拒绝、降级执行或自动 HOLD 原因。
  - 后续仍可继续把命令校验逻辑抽成统一函数，减少 `process_move()` 内部分支。

- [x] `control_cmd_valid` 只能表示是否收到过非 `UNDEFINE` 命令。
  - 暂不拆分为 `last_cmd_received`、`last_cmd_accepted`、`last_cmd_active`。
  - 当前已通过 `diagnostic_level` / `diagnostic_msg` 覆盖命令拒绝、超时和降级执行等主要排错场景。
  - 后续如果引入统一命令校验器或任务管理器，再考虑拆分更细的命令生命周期字段。

## P2 配置和多车型支持

- [x] 控制参数缺少车型预设机制。
  - 已拆分为 `config/ugv_control_base.yaml` 和 `config/airframes/*.yaml`。
  - 已新增 `mecanum_default`、`differential_default`、`gazebo_sim`、`pengyu_sim` 四套预设。
  - 默认 launch 通过 `airframe` 参数选择车型，旧 `ugv_control_config.yaml` 已删除。

- [x] 参数命名可以进一步区分通用参数和控制器专用参数。
  - `kp_linear`、`kp_angular`、速度限制和到点阈值保持平铺，表示所有控制器共用。
  - 差速倒车参数已移动到 `differential_controller:` 分组，降低麦克纳姆车型误改风险。

- [x] launch 中的参数覆盖关系需要持续保持清晰。
  - 当前优先级为 `ugv_control_base.yaml < airframes/<airframe>.yaml < launch agent_name/agent_id`。
  - README 和 YAML 注释已说明覆盖顺序。

## P3 测试和验证

- [ ] 增加状态机单元测试。
  - 覆盖 INIT、HOLD、MOVE 的切换。
  - 覆盖速度命令超时、点位到达后切换 HOLD、差速底盘拒绝世界系速度命令。

- [ ] 增加控制器数学测试。
  - 麦克纳姆轮：世界系误差到车体系速度的转换、限速逻辑。
  - 差速轮：目标在前方、侧方、后方、允许倒车和禁止倒车等情况。

- [ ] 增加 launch/config 基础检查。
  - 检查 YAML 参数是否能被节点读取。
  - 检查多车启动时不同车辆的参数命名空间不会互相覆盖。

- [ ] 增加仿真回归例程。
  - 用固定轨迹测试 `MOVE_POINT` 和 `MOVE_VELOCITY_BODY`。
  - 输出最终位置误差和状态机状态，方便改控制器后快速验证。

## P4 工具和新手体验

- [x] 增加 UGV 控制示例包或示例目录。
  - 已新增 `examples/sunray_ugv_control_example`。
  - 覆盖 `HOLD`、`MOVE_POINT`、`MOVE_VELOCITY_BODY` 三个基础示例。
  - README 已说明启动方式、命令频率规则、差速/麦克纳姆差异和二次开发建议。

- [x] 控制面板根据底盘类型禁用不支持的命令。
  - 差速底盘下已禁用 `MOVE_VELOCITY` 按钮和 `linear.y` 输入，并将 `linear.y` 清零。
  - 麦克纳姆轮下保留世界系速度和横向速度输入。
  - 后端 FSM 仍保留能力校验，防止其他程序直接发错命令。

- [x] 终端监控工具可以显示更多诊断信息。
  - 已显示 `diagnostic_level` 和 `diagnostic_msg`。
  - 命令年龄、速度命令剩余有效时间、位置误差、yaw 误差属于任务进度字段，当前暂不增加。

- [x] README 中的示例可以配套 roslaunch。
  - 已新增 `examples/sunray_ugv_control_example`，包含可直接启动的 launch。
  - `sunray_ugv_control/README.md` 已引导开发者参考该示例包。

- [x] RViz 可视化可以显示命令合法性和拒绝原因。
  - 已新增独立 `ugv_diagnostic_text` marker。
  - 根据 `diagnostic_level` 使用 OK/WARN/ERROR 颜色显示 `diagnostic_msg`。

## P5 代码质量

- [x] 控制器参数读取可以收敛到统一配置结构。
  - 已新增 `UGVControlConfig` 和 `UGVControllerParams`。
  - FSM 身份、底盘类型、超时、到点阈值和地理围栏参数统一由 `UGVControlConfig::loadFromRos()` 读取。
  - 通用控制器增益和限速参数统一由 `UGVControllerParams::loadFromRos()` 读取。
  - 控制节点启动时会打印最终生效的关键参数，方便核对 airframe 覆盖是否正确。

- [ ] 状态机命令处理可以拆分为“校验”和“执行”两层。
  - 当前 `process_move()` 中同时处理超时、能力判断、控制器调用和状态切换。
  - 拆分后更容易测试，也更容易给 UI 返回明确原因。

- [x] 日志输出需要统一节流和格式。
  - 已统一 FSM 运行期 warning 日志格式为 `[ugv1][COMMAND] reason...`。
  - 速度命令超时、stamp 为空、差速底盘拒绝世界系速度、差速底盘忽略横向速度、地理围栏保护都包含车辆名、命令类型和原因。
  - 重复触发的运行期 warning 继续使用 `ROS_WARN_THROTTLE(1.0, ...)`，避免刷屏。

- [ ] 包元信息可以进一步完善。
  - 检查 `package.xml` 中的维护者、许可证、依赖和导出信息。
  - 方便后续发布、安装和团队协作。
