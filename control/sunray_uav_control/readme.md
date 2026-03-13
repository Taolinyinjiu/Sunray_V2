uav_control是Sunray项目的控制核心部分，主要包含以Base_Controller为基类，延伸出来的几类控制器，分别是

- 基于PX4位置环设计的控制器，位置控制的精度由PX4位置环精度决定，PX4内部的位置环为单P环节，也就是说纯比例环节，因此性能没有其他几个控制器好
- 基于PX4速度环设计的PID控制器，位置控制的精度由PID参数决定，相较于基于PX4位置环设计的控制器，其为完整的PID控制，因此性能会相对较好
- 基于PX4姿态环设计的Sunray自定义控制器，位置控制的精度由控制器的参数决定
- 基于强化学习设计的RAPTOR控制器，位置控制的精度由模型决定

这四种控制器，对应了四个文件夹
- px4_position_controller 基于px4位置环的控制器
- px4_velocity_controller 基于px4速度环的控制器
- sunray_attitude_controller 基于px4姿态环的控制器
- sunray_raptor_controller 基于强化学习RAPTOR模型的控制器

除此之外，还存在一个核心的状态机模块，也就是Sunray_statemachine，通常我们简称为Sunray_fsm（fsm指的是有限状态机）

总的来说sunray_fsm是一个相对封闭的模块，向外暴露出特定的接口，允许用户通过这些接口组合实现自己的算法
sunray_fsm所提供的接口可以分为这几类

1. 与无人机运动控制相关
 - 起飞(先service服务 后topic话题)
    takeoff_request
    takeoff_command
 - 降落
    land_request
    land_command
 - 紧急降落
    emergency_land_request
    emergency_land_command
 - 运动控制
    control_request
    control_command

2. 与无人机状态相关的
 - 状态机状态变量（10hz）
    sunray_fsm_state
 - 无人机运动相关变量
    uav_odom_state
 - 传感器相关状态
    uav_sensor_status

## 状态机图自动生成

`uav_control` 提供了基于源码自动生成状态机图的脚本：

```bash
python3 control/uav_control/scripts/gen_fsm_diagram.py --write
```

生成文件：

- `control/uav_control/state_machine_diagram.md`

一致性检查（适合 CI）：

```bash
python3 control/uav_control/scripts/gen_fsm_diagram.py --check --strict
```
