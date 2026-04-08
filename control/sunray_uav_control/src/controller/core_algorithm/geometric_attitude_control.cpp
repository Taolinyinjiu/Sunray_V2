/**
 * @file geometric_attitude_control.cpp
 * @brief 几何控制器核心算法实现
 *
 * 符号约定：
 *   - 位置/速度误差：target - current（误差为正代表无人机落后于目标）
 *   - 位置/速度增益：param 中存储为正值
 *   - 内部四元数：Vector4d(qw, qx, qy, qz)
 *   - 重力向量：(0, 0, -g)，z 轴向上为正
 */

#include "controller/core_algorithm/geometric_attitude_control.hpp"
#include "controller/core_algorithm/geometric_ctrl_math.hpp"
#include <algorithm>
#include <cmath>

// ═══════════════════════════════════════════════════════════════════════════
// QuaternionAttitudeCtrl 实现
// 四元数误差法，参考：Brescianini et al., ETH Zurich, 2013
// ═══════════════════════════════════════════════════════════════════════════

QuaternionAttitudeCtrl::QuaternionAttitudeCtrl(double attctrl_tau) : attctrl_tau_(attctrl_tau) {}

void QuaternionAttitudeCtrl::update(const Eigen::Vector4d& curr_att,
                                    const Eigen::Vector4d& ref_att,
                                    const Eigen::Vector3d& ref_acc,
                                    const Eigen::Vector3d& /*ref_jerk*/) {
    // 计算姿态误差四元数：q_err = q_curr^{-1} ⊗ q_ref
    // q_curr 的共轭（即逆，因为单位四元数）通过对 xyz 分量取反得到
    const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
    const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
    const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att);

    // 期望角速度：(2 / tau) * sign(qe_w) * qe_xyz
    // sign(qe_w) 确保取短弧，避免绕远路旋转
    desired_rate_(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
    desired_rate_(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
    desired_rate_(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);

    // 推力方向：期望加速度在当前机体 z 轴上的投影
    const Eigen::Matrix3d rotmat = quat2RotMatrix(curr_att);
    const Eigen::Vector3d zb = rotmat.col(2);
    desired_thrust_(0) = 0.0;
    desired_thrust_(1) = 0.0;
    desired_thrust_(2) = ref_acc.dot(zb);
}

// ═══════════════════════════════════════════════════════════════════════════
// SO3AttitudeCtrl 实现
// 几何 SO3 法，参考：Lee et al., CDC 2010
// ═══════════════════════════════════════════════════════════════════════════

SO3AttitudeCtrl::SO3AttitudeCtrl(double attctrl_tau) : attctrl_tau_(attctrl_tau) {}

void SO3AttitudeCtrl::update(const Eigen::Vector4d& curr_att,
                             const Eigen::Vector4d& ref_att,
                             const Eigen::Vector3d& ref_acc,
                             const Eigen::Vector3d& /*ref_jerk*/) {
    // 计算当前姿态与期望姿态的旋转矩阵
    const Eigen::Matrix3d rotmat = quat2RotMatrix(curr_att);
    const Eigen::Matrix3d rotmat_d = quat2RotMatrix(ref_att);

    // SO3 姿态误差：e_R = 0.5 * vee(R_d^T * R - R^T * R_d)
    // vee() 即 matrix_hat_inv()，从反对称矩阵中提取向量
    const Eigen::Vector3d error_att =
        0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat_d);

    // 期望角速度
    desired_rate_ = (2.0 / attctrl_tau_) * error_att;

    // 推力方向：期望加速度在当前机体 z 轴上的投影
    const Eigen::Vector3d zb = rotmat.col(2);
    desired_thrust_(0) = 0.0;
    desired_thrust_(1) = 0.0;
    desired_thrust_(2) = ref_acc.dot(zb);
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometric_AttitudeControl 实现
// ═══════════════════════════════════════════════════════════════════════════

void Geometric_AttitudeControl::load_param(const Geometric_AttitudeControl_Param_t& param) {
    param_ = param;

    // 根据 ctrl_mode 实例化对应的姿态子控制器
    if (param_.ctrl_mode == 0) {
        att_controller_ = std::make_shared<QuaternionAttitudeCtrl>(param_.attctrl_tau);
    } else {
        att_controller_ = std::make_shared<SO3AttitudeCtrl>(param_.attctrl_tau);
    }
}

Geometric_AttitudeControl_Output_t Geometric_AttitudeControl::calculateControl(
    const controller_data_types::TargetTrajectoryPoint_t& des_state,
    const control_common::UAVStateEstimate& current_odom) {

    // ── 提取当前状态 ──────────────────────────────────────────────────────
    const Eigen::Vector3d mav_pos = current_odom.position;
    const Eigen::Vector3d mav_vel = current_odom.velocity;
    // Eigen::Quaterniond → Vector4d(qw, qx, qy, qz)（内部数学约定）
    const Eigen::Quaterniond& q_eig = current_odom.orientation;
    const Eigen::Vector4d curr_att_vec(q_eig.w(), q_eig.x(), q_eig.y(), q_eig.z());

    // ── 提取期望状态 ──────────────────────────────────────────────────────
    const Eigen::Vector3d target_pos = des_state.position;
    const Eigen::Vector3d target_vel = des_state.velocity;
    const Eigen::Vector3d target_acc = des_state.acceleration;
    const Eigen::Vector3d target_jerk = des_state.jerk;
    // 若上层显式设置了 yaw 则更新缓存；否则沿用上次的期望 yaw，避免默认归零
    if (des_state.yaw.has_value()) {
        last_desired_yaw_ = static_cast<double>(des_state.yaw.value());
    }
    const double target_yaw = last_desired_yaw_;

    // ── Z 轴积分：消除推力模型未标定导致的稳态悬停误差 ───────────────────
    // 仅在接近目标时积分（误差 < 1m），防止大幅运动时积分饱和
    const double pos_error_z = target_pos.z() - mav_pos.z();
    if (param_.pos_ki_z > 0.0 && std::abs(pos_error_z) < 1.0) {
        const double dt = 1.0 / param_.controller_hz;
        integral_z_ += pos_error_z * dt;
        // 限幅：将积分折算为加速度后不超过 int_max_z
        const double int_limit = param_.int_max_z / std::max(param_.pos_ki_z, 1e-6);
        integral_z_ = std::max(-int_limit, std::min(int_limit, integral_z_));
    }

    // ── 位置环：轨迹点 → 期望加速度 ──────────────────────────────────────
    const Eigen::Vector3d a_des =
        controlPosition(target_pos, target_vel, target_acc, mav_pos, mav_vel, target_yaw);

    // ── 姿态环：期望加速度 → body rate + 归一化推力 ───────────────────────
    Eigen::Vector4d bodyrate_cmd;
    computeBodyRateCmd(bodyrate_cmd, a_des, curr_att_vec, target_yaw, target_jerk);

    // ── 填充输出 ──────────────────────────────────────────────────────────
    Geometric_AttitudeControl_Output_t output;
    output.bodyrates = bodyrate_cmd.head(3);
    output.thrust = bodyrate_cmd(3);
    // 期望姿态由期望加速度方向 + 偏航角决定（主要用于可视化/调试）
    const Eigen::Vector4d q_des_vec = acc2quaternion(a_des, target_yaw);
    output.orientation = Eigen::Quaterniond(q_des_vec(0), q_des_vec(1), q_des_vec(2), q_des_vec(3));
    return output;
}

// ─────────────────────────────────────────────────────────────────────────
// poscontroller
// 位置 PD 控制器，输入误差均为 target - current（正值代表落后于目标）
// ─────────────────────────────────────────────────────────────────────────
Eigen::Vector3d Geometric_AttitudeControl::poscontroller(const Eigen::Vector3d& pos_error,
                                                         const Eigen::Vector3d& vel_error) {

    // Z 轴积分补偿叠加到加速度反馈上
    Eigen::Vector3d a_fb =
        param_.Kpos.asDiagonal() * pos_error + param_.Kvel.asDiagonal() * vel_error;
    a_fb.z() += param_.pos_ki_z * integral_z_;

    // 对输出加速度进行球形限幅，保持方向不变
    if (a_fb.norm() > param_.max_acc) {
        a_fb = (param_.max_acc / a_fb.norm()) * a_fb;
    }
    return a_fb;
}

// ─────────────────────────────────────────────────────────────────────────
// controlPosition
// 综合位置 PD 反馈、前馈加速度、转子阻力补偿，生成期望加速度向量
// ─────────────────────────────────────────────────────────────────────────
Eigen::Vector3d Geometric_AttitudeControl::controlPosition(const Eigen::Vector3d& target_pos,
                                                           const Eigen::Vector3d& target_vel,
                                                           const Eigen::Vector3d& target_acc,
                                                           const Eigen::Vector3d& mav_pos,
                                                           const Eigen::Vector3d& mav_vel,
                                                           double mav_yaw) {

    // 重力向量，z 轴向上为正
    const Eigen::Vector3d gravity_vec(0.0, 0.0, -param_.gravity);

    // 参考旋转矩阵：由参考加速度方向（去除重力）和偏航角构成
    // 用于计算转子阻力补偿项
    const Eigen::Vector4d q_ref = acc2quaternion(target_acc - gravity_vec, mav_yaw);
    const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

    // 位置和速度误差（Sunray 约定：target - current）
    const Eigen::Vector3d pos_error = target_pos - mav_pos;
    const Eigen::Vector3d vel_error = target_vel - mav_vel;

    // PD 反馈加速度
    const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);

    // 转子阻力补偿（D 为零向量时此项为零，不影响基本功能）
    const Eigen::Vector3d a_rd = R_ref * param_.D.asDiagonal() * R_ref.transpose() * target_vel;

    // 合成期望加速度：反馈 + 前馈 - 阻力补偿 - 重力（即加上重力大小的 z 分量）
    return a_fb + target_acc - a_rd - gravity_vec;
}

// ─────────────────────────────────────────────────────────────────────────
// computeBodyRateCmd
// 期望加速度 → 期望姿态 → 调用姿态子控制器 → body rate + 归一化推力
// ─────────────────────────────────────────────────────────────────────────
void Geometric_AttitudeControl::computeBodyRateCmd(Eigen::Vector4d& bodyrate_cmd,
                                                   const Eigen::Vector3d& a_des,
                                                   const Eigen::Vector4d& curr_att_vec,
                                                   double mav_yaw,
                                                   const Eigen::Vector3d& target_jerk) {

    // 由期望加速度方向 + 偏航角计算期望姿态四元数
    const Eigen::Vector4d q_des = acc2quaternion(a_des, mav_yaw);

    // 调用姿态子控制器，计算期望角速度和推力分量
    att_controller_->update(curr_att_vec, q_des, a_des, target_jerk);

    // 填充 body rate（前三分量）
    bodyrate_cmd.head(3) = att_controller_->get_desired_rate();

    // 推力归一化：thrust_norm = clamp(k * (m * acc_z) + offset, 0.0, 0.95)
    // norm_thrust_const 和 norm_thrust_offset 为机型相关的标定值
    const double thrust_acc = att_controller_->get_desired_thrust().z();
    bodyrate_cmd(3) =
        std::max(0.0,
                 std::min(0.95,
                          param_.norm_thrust_const * (param_.drone_mass * thrust_acc) +
                              param_.norm_thrust_offset));
}

// ─────────────────────────────────────────────────────────────────────────
// acc2quaternion
// 由期望加速度方向（即期望推力方向）和偏航角构造期望姿态四元数
// ─────────────────────────────────────────────────────────────────────────
Eigen::Vector4d Geometric_AttitudeControl::acc2quaternion(const Eigen::Vector3d& vector_acc,
                                                          double yaw) {

    // 保护：加速度向量模过小时（接近自由落体）无法定义推力方向，回退到竖直向上
    const double acc_norm = vector_acc.norm();
    const Eigen::Vector3d zb_des =
        (acc_norm > 1e-3) ? (vector_acc / acc_norm) : Eigen::Vector3d(0.0, 0.0, 1.0);

    // 期望 x 轴方向在水平面的参考投影（由偏航角决定）
    const Eigen::Vector3d proj_xb_des(std::cos(yaw), std::sin(yaw), 0.0);

    // 奇异性检测：zb_des 与 proj_xb_des 近似平行时叉积趋零（例如 yaw=0 且需要大 x 向推力）
    // 此时改用 y 方向作为参考向量，绕开奇异点
    Eigen::Vector3d cross_check = zb_des.cross(proj_xb_des);
    Eigen::Vector3d yb_des;
    if (cross_check.norm() > 1e-3) {
        yb_des = cross_check / cross_check.norm();
    } else {
        // 退化情况：用全局 y 轴作为备用参考重新构造
        const Eigen::Vector3d proj_yb_fallback(-std::sin(yaw), std::cos(yaw), 0.0);
        Eigen::Vector3d cross_fallback = zb_des.cross(proj_yb_fallback);
        yb_des = (cross_fallback.norm() > 1e-6) ? (cross_fallback / cross_fallback.norm())
                                                : Eigen::Vector3d(0.0, 1.0, 0.0);
    }
    const Eigen::Vector3d xb_des = (yb_des.cross(zb_des)).normalized();

    // 组装旋转矩阵（列向量为机体坐标系的三个轴在世界系下的方向）
    Eigen::Matrix3d rotmat;
    rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2),
        yb_des(2), zb_des(2);

    // 旋转矩阵 → 四元数（Shepperd 方法，避免数值奇异）
    return rot2Quaternion(rotmat);
}
