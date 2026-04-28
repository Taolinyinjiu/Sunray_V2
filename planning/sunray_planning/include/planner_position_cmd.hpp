#pragma once

#include <Eigen/Dense>
#include <ros/time.h>

struct PlannerPositionCommand {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ros::Time stamp;
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};      // 期望位置，世界坐标系
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};      // 期望速度，世界坐标系
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};  // 期望加速度
    Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};          // 期望加加速度(原生控制器不支持)

    double yaw = 0.0;       // 期望偏航角
    double yaw_rate = 0.0;  // 期望偏航角速度
};
