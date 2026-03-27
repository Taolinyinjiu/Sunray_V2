/* clang-format off */
/**
 * @file controller_debug_types.hpp
 * @brief 定义 Sunray 控制器核心层使用的调试数据类型。
 * @details
 * 本文件旨在设计一个各控制器通用的 debug 数据类型，用于状态机向外发布或者记录控制器日志。
 *
 * 设计原则：
 * 1. 调试数据与控制输出解耦，避免控制器接口被 ROS 通信格式污染。
 * 2. 公共字段与控制器专有字段分离，降低后续扩展时的破坏性修改。
 * 3. 使用 std::optional 区分“字段未提供”和“字段有效但数值恰好为 0”。
 *
 * -----------控制器在使用时-----------
 * struct Px4LocalControlResult {
 *   Px4LocalSetpoint command;
 *   Px4LocalControlDebug debug;
 * };
 *
 * Px4LocalControlResult calculateControl(
 *     const FlatTrajectoryPoint& des,
 *     const sunray_common::QuadStateEstimate& odom);
 *
 * -----------状态机在使用时-----------
 * auto result = controller.calculateControl(des, odom);
 *
 * publishSetpoint(result.command);
 * publishDebug(toRosMsg(result.debug));
 *
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-17
 * @version 0.1
 */
/* clang-format on */

#pragma once

#include <optional>

#include <Eigen/Dense>

namespace controller_data_types {

/// 所有控制器都可复用的通用调试信息。
struct ControllerDebugCommon {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::optional<Eigen::Vector3d> position_error;
    std::optional<Eigen::Vector3d> velocity_error;
    std::optional<double> yaw_error;
    std::optional<double> yaw_rate_error;
};

/// 面向 `setpoint_raw/local` 输出路径的调试信息。
struct Px4LocalControlDebug {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ControllerDebugCommon common;

    /// 误差反馈产生的修正加速度。
    std::optional<Eigen::Vector3d> feedback_acceleration;
    /// 限幅前的最终加速度指令。
    std::optional<Eigen::Vector3d> commanded_acceleration;
    /// 限幅后的实际采用加速度。
    std::optional<Eigen::Vector3d> limited_acceleration;

    /// 最终给 local setpoint 的偏航与偏航角速度。
    std::optional<double> commanded_yaw;
    std::optional<double> commanded_yaw_rate;
};

/// 面向 `setpoint_raw/attitude` 输出路径的调试信息。
struct Px4AttitudeControlDebug {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ControllerDebugCommon common;

    std::optional<Eigen::Quaterniond> reference_attitude;
    std::optional<Eigen::Quaterniond> feedback_attitude;
    std::optional<Eigen::Vector3d> attitude_error;

    std::optional<Eigen::Vector3d> reference_body_rate;
    std::optional<Eigen::Vector3d> feedback_body_rate;
    std::optional<Eigen::Vector3d> commanded_body_rate;

    std::optional<double> reference_thrust;
    std::optional<double> commanded_thrust;
};

}  // namespace controller_data_types
