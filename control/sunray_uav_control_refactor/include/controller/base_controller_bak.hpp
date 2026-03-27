/* clang-format off */
/**
 * @file base_controller.hpp
 * @author Taolinyinjiu@Yun-Drone Tech
 * @brief 本文件旨在抽象出一个Sunray项目中控制器所需要遵循的基类控制器，通过对该基类的继承，我们可以得到多种的控制器
1. 首先我们得到一个基本的问题，控制器需要面临无人机着地-> 无人机离地这一阶段，动力学的突变，
2. 本文件旨在描述两种基本控制器的接口使用，分别是基于px4位置环+速度环的控制器基类以及姿态环+推力环的控制器基类
3. 控制器的输出量是mavros的setpoint类型，而输入量是期望状态与当前状态，考虑到添加px4的target setpoint能够得到更好地反馈控制，因此我们为控制器的入口参数添加了px4的反馈量
	3.1 姿态-推力控制器得到px4的反馈量，可以优化起飞/降落过程中的推力曲线
 * @version 0.1
 * @date 2026-03-16
 *
 * @copyright Copyright (c) 2026
 *
 */
/* clang-format on */
#pragma once

#include <control_data_types/controller_debug_types.hpp>
#include <control_data_types/controller_desired_types.hpp>
#include <control_data_types/controller_output_types.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sunray_common/quad_state_estimate.h>
// 基于px4位置环和速度环的控制器基类设计

namespace controller_data_types {
// 构建输出结构体，包含输出控制量+debug结构体
struct Px4LocalControlResult {
    Px4LocalSetpoint command;
    Px4LocalControlDebug debug;
};

struct Px4AttitudeControlResult {
    Px4AttitudeSetpoint command;
    Px4AttitudeControlDebug debug;
};

}  // namespace controller_data_types

class Px4LocalControlBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit Px4LocalControlBase(ros::NodeHandle& nh)
        : nh_(nh) {}  // 控制器的构造函数使用ros句柄，从ros参数空间中读取参数
    virtual ~Px4LocalControlBase() = default;
    // 我们希望控制器所提供的函数名尽可能的简单，函数接口尽可能的少，因此我们使用is_ready()函数用于表示，控制器是否做好了准备
    // 这里的准备包括但不限于 读取参数，检查增益……
    // 当返回值为true时状态机可以考虑执行任务
    virtual bool is_ready() const;

    // 控制器计算输出函数，参数为 期望轨迹点，当前里程计，输出为控制器的输出
    virtual controller_data_types::Px4LocalControlResult
    calculateControl(const controller_data_types::FlatTrajectoryPoint& des,
                     const sunray_common::QuadStateEstimate& odom);

  protected:
    ros::NodeHandle nh_;
};

class Px4AttitudeControlBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit Px4AttitudeControlBase(ros::NodeHandle& nh);
    virtual ~Px4AttitudeControlBase() = default;

    virtual bool is_ready() const;

    // 控制器计算更新函数，参数为期望轨迹点，当前里程计，当前px4的imu输入，当前的px4_target输入
    virtual controller_data_types::Px4AttitudeControlResult
    calculateControl(const controller_data_types::FlatTrajectoryPoint& des,
                     const sunray_common::QuadStateEstimate& odom,
                     const sensor_msgs::Imu& imu);

  protected:
    ros::NodeHandle nh_;
};
