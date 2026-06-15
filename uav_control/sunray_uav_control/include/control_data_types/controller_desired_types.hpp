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
		4.1. 由于引入了C++ 17的语法糖，因此可以不使用mask掩码来判断是否有值被填入
		4.2. 我们只对yaw以及yaw_rate使用optional特性
	5. 修正,移除std::optional语法特性，我们认为这并没有给控制器带来良好的可维护性，同时反而引入了不必要的复杂性
	6. UAVControl的回调函数，应该结合消息中的掩码，来做一些合理的处理
 *
 * @version 0.2
 * @date 2026-03-16
 *
 * @copyright Copyright (c) 2026
 *
 */
/* clang-format on */
#pragma once

#include <Eigen/Dense>
#include <ros/time.h>

namespace controller_data_types {

// ═══════════════════════════════════════════════════════════
// 微分平坦轨迹点结构体
// ═══════════════════════════════════════════════════════════
struct TargetTrajectoryPoint_t {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d position{Eigen::Vector3d::Zero()};      // 期望位置，世界坐标系
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};      // 期望速度，世界坐标系
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};  // 期望加速度
    Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};          // 期望加加速度(原生控制器不支持)

    double yaw = 0.0;       // 期望偏航角
    double yaw_rate = 0.0;  // 期望偏航角速度
};

// ═══════════════════════════════════════════════════════════
// 期望位置点结构体
// ═══════════════════════════════════════════════════════════
struct TargetPoint_t {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    double yaw{0.0};
};

// ═══════════════════════════════════════════════════════════
// 期望速度结构体
// update: 速度结构体中加入fix_height字段，当该字段值大于零时，认为只在xy方向进行速度控制
// z轴交给sunray控制器进行固定高度控制，并围绕fixed_height进行闭环控制
// ═══════════════════════════════════════════════════════════
struct TargetVelocity_t {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ros::Time stamp{ros::Time(0)};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    double fixed_height{0.0};
    double yaw{0.0};
    double yaw_rate{0.0};
};

// ═══════════════════════════════════════════════════════════
// body系位置控制结构体
// position_xy 字段描述机体系下水平方向位移
// fixed_height 字段描述惯性系下z轴固定高度
// yaw 字段描述惯性系绝对 yaw 角
// ═══════════════════════════════════════════════════════════
struct TargetBodyPoint_t {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector2d position_xy{Eigen::Vector2d::Zero()};
    double fixed_height{0.0};
    double yaw{0.0};
};

// ═══════════════════════════════════════════════════════════
// body系速度控制结构体
// velocity_xy 字段描述机体系下水平方向期望速度
// fixed_height 字段描述惯性系下z轴固定高度
// yaw 字段描述惯性系绝对 yaw 角
// yaw_rate 字段描述 yaw 角速度
// ═══════════════════════════════════════════════════════════
struct TargetBodyVelocity_t {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ros::Time stamp{ros::Time(0)};
    Eigen::Vector2d velocity_xy{Eigen::Vector2d::Zero()};
    double fixed_height{0.0};
    double yaw{0.0};
    double yaw_rate{0.0};
};

}  // namespace controller_data_types
