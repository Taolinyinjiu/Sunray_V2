/**
 * @file controller_output_types.hpp
 * @author Taolinyinjiu @YunDrone Tech
 * @brief 说明 Sunray 控制器输出接口的边界设计。
 * @details
 * 在 2026-03-16 的重构讨论中，项目确定了两类控制器的输出能力边界：
 * - `px4_original_controller` 提供位置、速度、轨迹和自定义接口，
 *   其中自定义接口用于按 PX4 setpoint mask 进行转发。
 * - `sunray_attitude_controller` 提供位置和轨迹接口。
 *
 * 之所以做这样的边界划分，是因为姿态-推力接口控制的是加速度/力，而不是速度本身。
 * 如果没有速度误差反馈，系统只能根据给定姿态和推力飞行，无法保证收敛到目标速度。
 * 因此，控制器能够提供哪些接口，取决于其闭环中使用了哪些反馈量。
 * 对于姿态控制器而言，其反馈量是位置和姿态，所以只适合对外提供位置和轨迹接口。
 *
 * 基于上述边界，状态机后续需要根据控制器输出分别处理两类更新路径：
 * - `setpoint_raw/local`
 * - `setpoint_raw/attitude`
 *
 * 上层接口可以是位置、速度、轨迹或自定义掩码，但最终都应归一到这两种输出之一。
 *
 * @version 0.1
 * @date 2026-03-16
 *
 * @copyright Copyright (c) 2026
 */

#pragma once
#include <Eigen/Dense>
#include <cstdint>

namespace controller_data_types {

/// `setpoint_raw/local` 使用的坐标系，与 MAVROS PositionTarget 保持一致。
enum class Px4LocalFrame : uint8_t {
    kLocalNed = 1,
    kLocalOffsetNed = 7,
    kBodyNed = 8,
    kBodyOffsetNed = 9
};

/// 对应 `setpoint_raw/local` 的控制器输出。
struct Px4LocalSetpoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// 与 MAVROS PositionTarget::type_mask 对齐。
    enum Mask : uint16_t {
        kIgnorePx = 1u,
        kIgnorePy = 2u,
        kIgnorePz = 4u,
        kIgnoreVx = 8u,
        kIgnoreVy = 16u,
        kIgnoreVz = 32u,
        kIgnoreAfx = 64u,
        kIgnoreAfy = 128u,
        kIgnoreAfz = 256u,
        kForceSetpoint = 512u,
        kIgnoreYaw = 1024u,
        kIgnoreYawRate = 2048u
    };

    /// setpoint 中位置/速度/加速度字段的解释坐标系。
    Px4LocalFrame frame = Px4LocalFrame::kLocalNed;
    /// 指定哪些字段参与控制，哪些字段被忽略。
    uint16_t mask = 0;

    /// 目标位置。
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    /// 目标速度。
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    /// 目标加速度，或在 `kForceSetpoint` 置位时表示目标力。
    Eigen::Vector3d accel_or_force = Eigen::Vector3d::Zero();

    /// 目标偏航角。
    double yaw = 0.0;
    /// 目标偏航角速度。
    double yaw_rate = 0.0;
};

/// 对应 `setpoint_raw/attitude` 的控制器输出。
struct Px4AttitudeSetpoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// 与 MAVROS AttitudeTarget::type_mask 对齐。
    enum Mask : uint8_t {
        kIgnoreRollRate = 1u,
        kIgnorePitchRate = 2u,
        kIgnoreYawRate = 4u,
        kIgnoreThrust = 64u,
        kIgnoreAttitude = 128u
    };

    /// 指定姿态、角速度、推力中哪些字段参与控制。
    uint8_t mask = 0;

    /// 目标姿态。
    Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
    /// 机体系目标角速度。
    Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();
    /// 归一化总推力。
    double thrust = 0.0;
};

}  // namespace controller_data_types
