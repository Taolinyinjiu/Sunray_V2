# sunray_uav_control_example TODO

这个文件记录阅读 `sunray_uav_control_example` 后发现的改进建议。优先级按“会不会误导新手、会不会影响运行、会不会影响二次开发复用”排序。

## 高优先级

- [ ] 补齐 `MOVE_POINT_WGS84` 示例。
  - 当前 `src/MOVE_POINT_WGS84/move_point_wgs84.cpp` 和 `launch/MOVE_POINT_WGS84/move_point_wgs84.launch` 都是空文件。
  - `CMakeLists.txt` 里也没有 `move_point_wgs84_node` 编译目标。
  - 建议补充经纬高目标输入、GPS/全局原点检查、高度基准说明，以及失败保护逻辑。

- [x] 修复 `circle_velocity_yaw_node` 安装遗漏。
  - `CMakeLists.txt` 已经 `add_executable(circle_velocity_yaw_node ...)`，也有 launch 文件。
  - 但 `install(TARGETS ...)` 列表里没有 `circle_velocity_yaw_node`。
  - 源码空间能运行，安装后可能找不到该节点。

- [x] 处理 `install(DIRECTORY config/)` 与实际目录不一致的问题。
  - 当前包内没有 `config/` 目录，但 `CMakeLists.txt` 末尾仍安装 `config/*.yaml`。
  - 如果执行安装流程，可能因为目录不存在导致问题。
  - 可以删除这段安装规则，或者真正新增示例专用配置目录。

- [ ] 给每个 launch 增加更稳妥的启动入口。
  - 当前推荐用文件路径启动，例如 `roslaunch examples/sunray_uav_control_example/launch/basic/takeoff_land.launch`。
  - `roslaunch sunray_uav_control_example basic/takeoff_land.launch` 在当前环境下不能解析子目录文件。
  - 建议在 `launch/` 根目录增加扁平化入口，或在安装后验证包名启动方式。

- [x] 清理源码中的占位注释。
  - `move_multipoint.cpp` 和 `move_point_and_return.cpp` 里还有 `[补充一句：这个文件用于教学/演示什么内容]`。
  - 这些占位文字会降低示例代码可信度，建议替换成正式说明。

## 中优先级

- [ ] 把硬编码目标点、速度、半径、`kp`、`fixed_height` 改成 ROS 参数。
  - 当前很多示例需要改源码才能调整目标点和速度。
  - 建议每个 launch 暴露常用参数，例如 `target_x/y/z`、`fixed_height`、`max_vel`、`kp`、`circle_radius`。
  - 这样小白用户可以先改 launch，不必一开始就改 C++。

- [ ] 抽出公共等待和发布工具函数。
  - 多数示例重复了等待 `INIT`、发布 `TAKEOFF`、等待 `HOVER`、发布 `RETURN/LAND` 的代码。
  - 建议新增一个小型 helper，例如 `waitForState()`、`publishTakeoff()`、`publishLand()`。
  - 这样示例更短，也能避免各文件超时逻辑不一致。

- [x] 给所有依赖里程计的示例增加 odom 接收检查。
  - 速度和轨迹示例订阅 `local_odom`，但启动后没有显式确认是否已经收到第一帧。
  - 如果刚进入 HOVER 时 odom 仍是默认值，圆心或目标判断可能出错。
  - 建议增加 `has_odom` 标志，并在进入任务前等待有效里程计。

- [x] 重新校准 `MOVE_VELOCITY_BODY` 示例的坐标系逻辑。
  - 已改为先根据惯性系位置误差计算惯性系速度目标，再根据当前 yaw 旋转到机体系。
  - `desired_body_xy_vel` 现在填入的是机体系速度，避免 yaw 不为 0 时方向错误。

- [x] 统一节点名、目标名和 launch 名。
  - 已将 `move_velocity_fixed_height.cpp` 内部默认节点名改为 `move_velocity_fixed_height_node`。
  - 这样直接 `rosrun` 和通过 launch 启动时看到的节点名更一致。

- [x] 用 `ros::Rate` 替代分散的 `ros::Duration(0.02).sleep()`。
  - 已将速度和轨迹示例主控制循环统一改为 `ros::Rate`。
  - 状态等待中的 `ros::Duration(1.0)`、`ros::Duration(0.1)` 保持不变，避免改变等待语义。

- [x] 明确每个示例的安全边界。
  - 已在 README 中为各示例补充默认飞行范围、速度/高度、结束动作、依赖输入和实机建议。
  - 已在各非空 launch 顶部增加简短安全边界注释，方便运行前快速确认。

## 低优先级

- [x] 清理拼写和注释问题。
  - 已修正明显拼写错误。
  - 已整理部分注释和日志文案，避免 RETURN 示例误写成进入降落模式。
  - 已更新 launch 中关于 `use_private_agent_key` 的说明，避免“参数为空才读全局参数”的误导。

- [x] 完善 `package.xml` 元信息。
  - 已将版本改为 `0.1.0`，描述改为控制接口教学示例说明。
  - 已将维护者改为 `YunDrone Tech <yundrone@yundrone.cn>`。
  - 已按仓库根目录 LICENSE 使用 `MPL-2.0`。

- [x] 给 README 增加流程图或状态时序图。
  - 已新增“状态流程速查”，分别说明起降、位置点、速度/轨迹任务流程。
  - 已强调事件型命令发一次、速度/轨迹连续参考量持续发布。

- [x] 增加一个最小 Python 示例。
  - 已新增 `scripts/takeoff_land.py`，演示 `rospy` 版 `TAKEOFF -> HOVER -> LAND`。
  - 已新增 `launch/basic/takeoff_land_py.launch` 并补充 `catkin_install_python()`。
  - README 已补充 Python 示例启动方式和学习重点。

- [ ] 增加示例自检脚本。
  - 检查 `control_state`、`local_odom`、`control_cmd` 发布者/订阅者是否存在。
  - 在真正发飞行命令前提示用户当前 agent、话题和状态。
