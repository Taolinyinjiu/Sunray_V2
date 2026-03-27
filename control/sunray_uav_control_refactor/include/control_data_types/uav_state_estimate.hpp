/**
 * @file uav_state_estimate.hpp
 * @author taolinyinjiu
 * @brief
 * 设计意图：简化nav_msgs::Odometry类型，去除协方差矩阵,去除坐标系帧,转换为Eigen类型
 * @version 0.1
 * @date 2026-03-19
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

// 考虑到这个结构体可以被各个模块使用，因此不使用命名空间
// 由于一些原因，我认为使用命名空间没有坏处
// 考虑的简单一点，不分析里程计也不分析有效性，也不去关心这个值合不合理

namespace control_common {
struct UAVStateEstimate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 填充时间
    ros::Time timestamp = ros::Time(0);
    // 填充位置与姿态
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    // 填充速度与角速度
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    // 部分算法无法得到角速度，这里可能需要额外考虑？
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Zero();
    // 构造函数
    UAVStateEstimate() = default;
    UAVStateEstimate(const nav_msgs::Odometry& msg);
};

inline UAVStateEstimate::UAVStateEstimate(const nav_msgs::Odometry& msg) {
    // 首先提取时间戳
    timestamp = msg.header.stamp;
    // 提取位置
    position.x() = msg.pose.pose.position.x;
    position.y() = msg.pose.pose.position.y;
    position.z() = msg.pose.pose.position.z;
    // 提取姿态
    orientation.w() = msg.pose.pose.orientation.w;
    orientation.x() = msg.pose.pose.orientation.x;
    orientation.y() = msg.pose.pose.orientation.y;
    orientation.z() = msg.pose.pose.orientation.z;
    // 提取线速度
    velocity.x() = msg.twist.twist.linear.x;
    velocity.y() = msg.twist.twist.linear.y;
    velocity.z() = msg.twist.twist.linear.z;
    // 提取角速度
    bodyrate.x() = msg.twist.twist.angular.x;
    bodyrate.y() = msg.twist.twist.angular.y;
    bodyrate.z() = msg.twist.twist.angular.z;
}

}  // namespace control_common
