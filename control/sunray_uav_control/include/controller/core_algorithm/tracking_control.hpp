/**
 * @ Nonlinear Quadrocopter Attitude Control
 * @see https://nrotella.github.io/assets/pdf/px4_attitude_control.pdf
 * @see https://github.com/Zhefan-Xu/tracking_controller#
 */

#pragma once

#include "control_data_types/controller_desired_types.hpp"
#include "control_data_types/uav_state_estimate.hpp"
#include <Eigen/Dense>
#include <cstdint>

// ═══════════════════════════════════════════════════════════
// 参数结构体
// ═══════════════════════════════════════════════════════════
struct Tracking_Control_Param_t {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 枚举输出量类型
    enum class ControlType : uint8_t { Body_Rate_ = 0, Attitude_, Acceleration_, Undefined_ };
    // 控制器输出量类型
    ControlType control_type = ControlType::Undefined_;
    // 位置环PID增益参数
    Eigen::Vector3f position_p = Eigen::Vector3f::Zero();
    Eigen::Vector3f position_i = Eigen::Vector3f::Zero();
    Eigen::Vector3f position_d = Eigen::Vector3f::Zero();
    // 速度环PID增益参数
    Eigen::Vector3f velocity_p = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity_i = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity_d = Eigen::Vector3f::Zero();
    // 角速率环控制参数
    double attctrl_tau = 0.0;   // 角速度环时间常数
    double hover_thrust = 0.0;  // 悬停推力
};

// ═══════════════════════════════════════════════════════════
// 输出结构体
// ═══════════════════════════════════════════════════════════
struct Tracking_Control_Output_t {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double thrust{0.0};  // 归一化推力

    // ControlType = Attitude_ -> attitude+thrust
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};  // 期望姿态
    // ControlType = Body_Rate_ -> bodyrates+thrust
    Eigen::Vector3d bodyrates{Eigen::Vector3d::Zero()};  // 机体角速度 [ωx, ωy, ωz] (rad/s)
    // ControlType = Acceleration_ -> TargetLocal: Acceleration
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};  // 机体加速度 [ax, ay, az] (m/s^2)
};

// ═══════════════════════════════════════════════════════════
// 主控制器类
// ═══════════════════════════════════════════════════════════
class Tracking_Control {
  public:
    Tracking_Control() = default;
    ~Tracking_Control() = default;
    // 加载控制器对应参数
    void load_param(Tracking_Control_Param_t param);
    // 请注意，以下三个更新函数，对应了三个运动模式，分别是轨迹追踪，位置追踪，速度追踪
    // 输入期望的轨迹, 当前里程计数据, 计算当前需要的输出
    Tracking_Control_Output_t
    compute_output(const controller_data_types::TargetTrajectoryPoint_t& des_state,
                   const control_common::UAVStateEstimate& current_odom);
    // 输入期望的位置，然后计算当前需要的输出
    Tracking_Control_Output_t
    compute_output(const controller_data_types::TargetPoint_t& des_state,
                   const control_common::UAVStateEstimate& current_odom);
    // 输入期望的速度，然后计算当前需要的输出
    Tracking_Control_Output_t
    compute_output(const controller_data_types::TargetVelocity_t& des_state,
                   const control_common::UAVStateEstimate& current_odom);

  private:
    Tracking_Control_Param_t param_;
};
