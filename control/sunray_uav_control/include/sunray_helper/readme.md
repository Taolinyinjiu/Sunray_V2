1. 我们需要FSM为我们提供什么？
a. 当前的FSM自身状态
b. 当前的无人机定位数据
c. 


```cpp
// Sunray_FSM以10Hz的频率向外发布状态机状态，数据类型为
struct Sunray_FSM_Status{
	// 当前状态机的状态,起飞，悬停，降落，运动...
	sunray_fsm::FSM_State uav_state;
	// 目标的数据信息？
	Eigen::Vector3d position;	
	Eigen::Vector3d velocity;	
	

	// 当前无人机的状态
	Eigen::Vector3d position;
	Eigen::Vector3d velocity_linear;
	Eigen::Vector3d velocity_angular;
	Eigen::Vector3d attitude_rpy_deg;
	Eigen::Quaterniond attitude_quat;
	// 
}
```