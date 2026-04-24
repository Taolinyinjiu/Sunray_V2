std_msgs/Header header

# 控制器类型枚举
uint8 DIFFERENTIAL_DRIVE=0    # 差速轮控制器
uint8 MECANUM_WHEEL=1         # 麦轮控制器

# 当前位置
geometry_msgs/Point pos

# 当前速度
geometry_msgs/Vector3 vel

# 当前偏航角
float64 yaw