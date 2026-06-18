/**
 * @file sunray_fsm_param.hpp
 * @brief sunray_fsm的参数类型
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-27
 * @version 0.1
 *
 */

#pragma once

#include <cstdint>
#include <string>
#include <Eigen/Dense>
#include "odom_filter/odom_kalman_filter.hpp"

namespace sunray_fsm {

// -------------------基本参数-----------------------
struct basic_param_t {
    // 控制器类型
    uint8_t controller_types{0};
    // 控制器更新频率
    double controller_update_frequency{0.0};
    // 状态机检查监督频率
    double supervisor_update_frequency{0.0};
    // 控制模块订阅的里程计话题
    std::string odom_topic_name{"null"};
    // 向px4飞控融合的类型
    uint8_t fuse_odom_type{0};
    // 向px4飞控融合的频率
    double fuse_odom_frequency{0.0};
};

// -----------------ROS消息超时参数-----------------------
// question:超时参数有什么用？
struct msg_timeout_param_t {
    // 里程计超时参数
    double local_odometry{0.0};
    // mavros连接超时参数
    double mavros_connect{0.0};
    // 地面站连接超时参数
    double sunray_station{0.0};
};

// -------------------起飞降落参数-----------------------
struct takeoff_land_param_t {
    // 起飞相对当前位置的高度
    double takeoff_relative_height{0.0};
    // 起飞过程中的最大速度
    double takeoff_max_velocity{0.0};
    // 降落的类型
    uint8_t land_type{0};
    // 降落过程中最大的速度
    double land_max_velocity{0.0};
    // return过程是否自动降落
    bool return_with_land{true};
};

// -------------------电子围栏参数-----------------------
struct local_fence_param_t {
    double x_max{0.0};
    double x_min{0.0};
    double y_max{0.0};
    double y_min{0.0};
    double z_max{0.0};
    double z_min{0.0};
};
// -------------------飞行速度参数-----------------------
struct velocity_param_t {
    // 飞行过程中的最大速度
    Eigen::Vector3d max_velocity{Eigen::Vector3d::Zero()};
    // 使用RC摇杆飞行时的最大速度
    Eigen::Vector3d max_velocity_with_rc{Eigen::Vector3d::Zero()};
    // 飞行过程中yaw角速率限制
    double yaw_rate{0.0};
};
// 一个大而去全的结构体，将上面的结构体全放进来，简化代码逻辑
struct sunray_fsm_config_t {
    basic_param_t basic_param;              // 基础参数
    msg_timeout_param_t msg_timeout_param;  // 消息超时参数
    takeoff_land_param_t takeoff_land_param;  // 起飞降落参数
    local_fence_param_t local_fence_param;    // local系电子围栏参数
    velocity_param_t velocity_param;          // 飞行速度参数
    control_common::OdomKalmanFilterParam_t odom_filter_param;  // 里程计滤波参数
};
}  // namespace sunray_fsm
