/* clang-format off */
/**
 * @file Desired_State.hpp
 * @author Taolinyinjiu@Yun-Drone Tech
 * @brief 本文件旨在描述Sunray项目中控制器模块所需要遵循的数据类型，Desired_State表示控制器在进行更新状态量时的期望变量
 *	1. 在仔细的分析后，我们认为sunray_attitude_controller承袭了px4_ctrl_controller中的部分数据结构，因此这里对px4_ctrl中的controller做一个简单地分析，该控制器实现了一个级联的PID结构
 *		1.1 外环：根据位置和速度的误差，计算所需要的加速度
 *		1.2 内环：将期望的加速度转换为姿态角和推力
 *	2. 高阶轨迹可以作为统一的上层期望输入，但不能作为所有控制器唯一的输入语义，对于姿态控制器来说，原因有这样几个
 *		2.1 对于轨迹跟踪控制器，高阶轨迹很合适，比如位置控制、速度-位置级联、微分平坦控制、几何控制，这些控制器天然就吃 pos/vel/acc，有些还会用 jerk，所以用高阶轨迹做统一输入很舒服
 *		2.2 对于内环或者非轨迹控制器，比如姿态控制器，角速度控制器，推力分配控制器，他们的输入量更像是q/bodyrate/thrust，高阶轨迹只能作为他们的参考，而不是本质的输入
 *		2.3 综上，我们认为高阶轨迹作为参考层次，每个控制器从参考层映射到自己的输入中去，也就是从高阶轨迹总取出自己所需要的那部分
 *	3. 由于后续我们并不准备实现SE3 Control以及对应的求解器，因此选择使用微分平坦轨迹作为后续所有控制器的参考输入，也就是期望状态，并修改命名，将Desired_State_t修改为FlatTrajectoryPoint
 *  4. 使用命名空间包裹，和sunray_uav_control现有语义一致
		5. 由于引入了C++ 17的语法糖，因此可以不使用mask掩码来判断是否有值被填入
 *
 * @version 0.1
 * @date 2026-03-16
 *
 * @copyright Copyright (c) 2026
 *
 */
/* clang-format on */
#pragma once

#include <Eigen/Dense>

namespace controller_data_types {

// 单个时刻的平坦轨迹参考点，供高层轨迹跟踪控制器使用。
struct TargetTrajectoryPoint_t {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::optional<Eigen::Vector3d> position;      // 期望位置，世界坐标系
    std::optional<Eigen::Vector3d> velocity;      // 期望速度，世界坐标系
    std::optional<Eigen::Vector3d> acceleration;  // 期望加速度
    std::optional<Eigen::Vector3d> jerk;          // 期望加加速度

    std::optional<double> yaw = 0.0;       // 期望偏航角
    std::optional<double> yaw_rate = 0.0;  // 期望偏航角速度
};

struct TargetPoint_t {
    std::optional<Eigen::Vector3d> position;
    std::optional<float> yaw;
    std::optional<float> yaw_rate;
};

struct TargetVelocity_t {
    std::optional<Eigen::Vector3d> velocity;
    std::optional<float> yaw;
    std::optional<float> yaw_rate;
};

}  // namespace controller_data_types
