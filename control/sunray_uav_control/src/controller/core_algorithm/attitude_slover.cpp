#include "controller/core_algorithm/attitude_slover.hpp"
#include <cmath>

// ═══════════════════════════════════════════════════════════════════════════
// 使用四元数来表示姿态的误差
// Attitude error is defined as in Brescianini, Dario, Markus Hehn, and Raffaello D'Andrea.
// Nonlinear quadrocopter attitude control: Technical report. ETH Zurich, 2013.
// ═══════════════════════════════════════════════════════════════════════════

Quaternion_Solver::Quaternion_Solver(double attctrl_tau) : attctrl_tau_(attctrl_tau) {}

void Quaternion_Solver::update(const Eigen::Quaterniond& curr_att,
                               const Eigen::Quaterniond& ref_att,
                               const Eigen::Vector3d& ref_acc) {

    // 计算姿态误差四元数：q_err = q_curr^{-1} * q_ref
    Eigen::Quaterniond q_err = curr_att.inverse() * ref_att;

    // 期望角速度：(2 / tau) * sign(q_err.w) * q_err.xyz
    // sign(q_err.w) 确保取短弧，避免绕远路旋转
    desired_rate_(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, q_err.w()) * q_err.x();
    desired_rate_(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, q_err.w()) * q_err.y();
    desired_rate_(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, q_err.w()) * q_err.z();
}

// ═══════════════════════════════════════════════════════════════════════════
// 使用SO3空间表示姿态的误差
// Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch.
// "Geometric tracking control of a quadrotor UAV on SE (3)."
// 49th IEEE conference on decision and control (CDC). IEEE, 2010.
// ═══════════════════════════════════════════════════════════════════════════

SO3_Solver::SO3_Solver(double attctrl_tau) : attctrl_tau_(attctrl_tau) {}

void SO3_Solver::update(const Eigen::Quaterniond& curr_att,
                        const Eigen::Quaterniond& ref_att,
                        const Eigen::Vector3d& ref_acc) {
    // 内部传入约定为 Vector4d(w, x, y, z)，转换为 Eigen::Quaterniond

    // 计算当前姿态与期望姿态的旋转矩阵
    Eigen::Matrix3d R = curr_att.toRotationMatrix();
    Eigen::Matrix3d R_d = ref_att.toRotationMatrix();

    // SO3 姿态误差：e_R_hat = 0.5 * (R^T * R_d - R_d^T * R)
    // 这样 vee(e_R_hat) 与后续 desired_rate_ = +(2/tau) * e_R 的反馈方向一致。
    Eigen::Matrix3d e_R_hat = 0.5 * (R.transpose() * R_d - R_d.transpose() * R);

    // vee 映射：从反对称矩阵中提取向量 [x, y, z]^T
    Eigen::Vector3d e_R(e_R_hat(2, 1), e_R_hat(0, 2), e_R_hat(1, 0));

    // 期望角速度
    desired_rate_ = (2.0 / attctrl_tau_) * e_R;
}
