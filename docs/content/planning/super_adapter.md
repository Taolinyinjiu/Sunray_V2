<!-- title: sunray_super_adapter -->

<section id="sunray-super-adapter">

## sunray_super_adapter

`planning/third_party_planner_examples/super_planner_example/sunray_super_adapter` 是 SUPER 到 Sunray 控制接口的预留适配包。

### 功能定位

SUPER 侧重点是高速安全导航，适配层需要解决目标输入、地图/点云 frame、轨迹输出和 Sunray 控制命令之间的接口差异。

### 建议职责

后续实现 `sunray_super_adapter` 时建议聚焦这些工作：

1. 将 Sunray 目标点、odom 和点云话题传给 SUPER。
2. 订阅 SUPER 输出轨迹或控制采样。
3. 转换为 `sunray_msgs/UAVControlCMD` 或 Sunray 统一规划中间消息。
4. 明确 SUPER/ROG-MAP 对点云 frame 的要求。
5. 把 SUPER 自身日志和 Sunray `UAVPlanningState` 对齐，方便调试。

### 相关源码

SUPER planner 源码架构见同级页面“SUPER规划器源码架构”。

</section>
