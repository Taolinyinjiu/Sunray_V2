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

namespace sunray_fsm {

// -------------------基本参数-----------------------
struct basic_param_t {
    double mass_kg{0.0};
    double gravity{0.0};
    uint8_t controller_types{0};
    double controller_update_frequency{0.0};
    double supervisor_update_frequency{0.0};
    std::string odom_topic_name{"null"};
    bool fuse_odom_to_px4{true};
    bool fuse_odom_type{false};
    double fuse_odom_frequency{0.0};
    bool trajectory_type{false};
};
// -------------------保护参数-----------------------
struct protect_param_t {
    double low_voltage{0.0};
    uint8_t low_voltage_operate{0};
    bool control_with_no_rc{false};
    uint8_t lost_with_rc{0};
    bool arm_with_code{false};
    bool takeoff_with_code{true};
    bool check_flip{false};
    double tilt_angle_max{0.0};
    uint8_t msg_timeout_operate{0};
};
// -------------------消息超时参数-----------------------
struct msg_timeout_param_t {
    double local_odometry{0.0};
    double mavros_connect{0.0};
    double sunray_station{0.0};
};
// -------------------起飞参数-----------------------
struct takeoff_land_param_t {
    double takeoff_relative_height{0.0};
    double takeoff_max_velocity{0.0};
    bool land_type{false};
    double land_max_velocity{0.0};
};
// -------------------电子围栏参数-----------------------
struct local_fence_param_t {
    double x_max{0.0};
    double x_mix{0.0};
    double y_max{0.0};
    double y_mix{0.0};
    double z_max{0.0};
    double z_mix{0.0};
};
// -------------------飞行速度参数-----------------------
struct velocity_param_t {
    Eigen::Vector3d max_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d max_velocity_with_rc{Eigen::Vector3d::Zero()};
    double yaw_rate{0.0};
};
// 一个大而去全的结构体，将上面的结构体全放进来，简化代码逻辑
struct sunray_fsm_config_t {
    basic_param_t basic_param;                // 基础参数
    protect_param_t protect_param;            // 保护措施参数
    msg_timeout_param_t msg_timeout_param;    // 消息超时参数
    takeoff_land_param_t takeoff_land_param;  // 起飞降落参数
    local_fence_param_t local_fence_param;    // local系电子围栏参数
    velocity_param_t velocity_param;          // 飞行速度参数
};
}  // namespace sunray_fsm
