代码架构如下所示

- include 文件夹存放各类定义 \*hpp
- src 文件夹存放具体实现，\*cpp

- control_data_types 用于存放sunray_uav_control模块中所有的数据类型
- controller 用于存放具体的控制器
- mavros_helper 用于存放mavros_helper抽象类
- statemachine 用于存放sunray_fsm
- system_check 用于存放状态or系统整体检查类

定义
Controller 带有控制句柄
Control 是纯粹的控制算法
