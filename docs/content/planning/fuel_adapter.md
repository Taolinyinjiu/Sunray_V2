<!-- title: sunray_fuel_adapter -->

<section id="sunray-fuel-adapter">

## sunray_fuel_adapter

`planning/third_party_planner_examples/fuel_planner_example/sunray_fuel_adapter` 是 FUEL 到 Sunray 控制接口的预留适配包。

### 功能定位

FUEL 是探索规划框架，输出语义通常不只是单个局部目标点。适配层需要把 FUEL 的探索目标、轨迹或局部视点输出转换为 Sunray 控制链路可消费的数据。

### 建议职责

后续实现 `sunray_fuel_adapter` 时建议按下面边界拆分：

1. 接收 Sunray 上层任务或 RViz 目标，转换为 FUEL 可用的探索入口。
2. 订阅 FUEL 轨迹/视点输出。
3. 转换为 `sunray_msgs/UAVControlCMD` 或 Sunray 统一规划中间消息。
4. 明确 FUEL 对地图、odom、点云和探索边界参数的要求。
5. 不在 adapter 中修改 FUEL 核心搜索和优化逻辑。

### 相关源码

FUEL planner 源码架构见同级页面“FUEL规划器源码架构”。

</section>
