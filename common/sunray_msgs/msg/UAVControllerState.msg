std_msgs/Header header

# 控制器输出坐标系
uint8 FRAME_LOCAL=0
uint8 FRAME_BODY=1

# 控制器类型枚举
uint8 PX4_ORIGINAL_CONTROLLER = 0
uint8 SUNRAY_ATTITUDE_CONTROLLER = 1
uint8 RL_RAPTOR_CONTROLLER = 2

# 枚举信息
uint8 reference_frame
uint8 controller_type

# 期望状态
geometry_msgs/Point   desired_pos # 期望位置
geometry_msgs/Vector3 desired_vel # 期望速度
float64 desired_yaw # 期望偏航角
float64 desired_yawrate # 期望偏航角速度

# 当前状态
geometry_msgs/Point   current_pos # 当前位置
geometry_msgs/Vector3 current_vel # 当前速度
float64 current_yaw # 当前偏航角

# 从px4_setpoint_target中读取的数据
geometry_msgs/Vector3    position_from_px4
geometry_msgs/Vector3    velocity_from_px4
geometry_msgs/Vector3    acceleration_from_px4
float64                  yaw_from_px4
float64                  yawrate_from_px4
geometry_msgs/Quaternion attitude_from_px4
geometry_msgs/Vector3    bodyrate_from_px4
float64                  thrust_from_px4

# 从控制器中读取的数据
geometry_msgs/Vector3    position_from_ctrl
geometry_msgs/Vector3    velocity_from_ctrl
geometry_msgs/Vector3    acceleration_from_ctrl
float64                  yaw_from_ctrl
float64                  yawrate_from_ctrl
geometry_msgs/Quaternion attitude_from_ctrl
geometry_msgs/Vector3    bodyrate_from_ctrl
float64                  thrust_from_ctrl
