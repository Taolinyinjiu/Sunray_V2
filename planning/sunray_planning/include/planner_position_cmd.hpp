// 设计一个position_cmd.hpp的原因在于，有一些planner规划出来的轨迹，求解器拆出来有一些没有必要，比如super的高阶多项式之类的，实际上我们直接使用他输出的position_cmd一样可以达到目的

// 基本的逻辑是 rostopic callback -> 填充PlannerPositionCommand结构体,置位has_used = false -> fsm发送这个控制命令，置位has_used=true防止下次发送旧址

#include <Eigen/Dense>

struct PlannerPositionCommand {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d position{Eigen::Vector3d::Zero()};      // 期望位置，世界坐标系
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};      // 期望速度，世界坐标系
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};  // 期望加速度
    Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};          // 期望加加速度(原生控制器不支持)

    double yaw = 0.0;       // 期望偏航角
    double yaw_rate = 0.0;  // 期望偏航角速度

    bool has_used=false; // 添加这个has_used 的原因是因为，这个是类内进行交互使用的
};