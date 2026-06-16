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
#include "eigen_helper.hpp"
#include <ros/console.h>
#include <algorithm>
#include <cmath>

// ═══════════════════════════════════════════════════════════════════════════
// Geometric_AttitudeControl 实现
// ═══════════════════════════════════════════════════════════════════════════

void Geometric_AttitudeControl::load_param(const Geometric_AttitudeControl_Param_t& param) {
    param_ = param;

    // 悬停推力估计器由 core 持有，0=RLS，1=EKFAccel。
    switch (param_.hover_thrust_estimator_type) {
    case 1:
        hover_thrust_estimator_ = std::make_unique<thrust_estimator::EKF_HoverThrustEstimator>();
        break;
    case 0:
    default:
        hover_thrust_estimator_ = std::make_unique<thrust_estimator::RLS_HoverThrustEstimator>();
        break;
    }

    thrust_estimator::Param_t estimator_param;
    estimator_param.gravity = param_.gravity;
    estimator_param.hover_thrust = param_.hover_thrust_init;
    estimator_param.hover_thrust_min = param_.hover_thrust_min;
    estimator_param.hover_thrust_max = param_.hover_thrust_max;
    estimator_param.ekf_onlyhover_estimate = param_.hover_thrust_ekf_onlyhover_estimate;
    estimator_param.ekf_Q = param_.hover_thrust_ekf_Q;
    estimator_param.ekf_R = param_.hover_thrust_ekf_R;
    estimator_param.ekf_P0 = param_.hover_thrust_ekf_P0;
    estimator_param.ekf_P_min = param_.hover_thrust_ekf_P_min;
    estimator_param.ekf_P_max = param_.hover_thrust_ekf_P_max;
    estimator_param.ekf_delay_min_s = param_.hover_thrust_ekf_delay_min_s;
    estimator_param.ekf_delay_max_s = param_.hover_thrust_ekf_delay_max_s;
    estimator_param.ekf_innovation_gate = param_.hover_thrust_ekf_innovation_gate;
    estimator_param.ekf_min_thrust_cmd = param_.hover_thrust_ekf_min_thrust_cmd;
    estimator_param.ekf_max_thrust_cmd = param_.hover_thrust_ekf_max_thrust_cmd;
    estimator_param.ekf_min_tilt_cos_hover = param_.hover_thrust_ekf_min_tilt_cos_hover;
    estimator_param.ekf_min_tilt_cos_move = param_.hover_thrust_ekf_min_tilt_cos_move;
    estimator_param.ekf_max_abs_acc_z_hover = param_.hover_thrust_ekf_max_abs_acc_z_hover;
    estimator_param.ekf_max_abs_acc_z_move = param_.hover_thrust_ekf_max_abs_acc_z_move;
    estimator_param.ekf_convergence_p_threshold =
        param_.hover_thrust_ekf_convergence_p_threshold;
    estimator_param.ekf_convergence_hold_s = param_.hover_thrust_ekf_convergence_hold_s;
    estimator_param.ekf_adaptive_R_enabled = param_.hover_thrust_ekf_adaptive_R_enabled;
    estimator_param.ekf_R_min = param_.hover_thrust_ekf_R_min;
    estimator_param.ekf_R_max = param_.hover_thrust_ekf_R_max;
    hover_thrust_estimator_->load_param(estimator_param);

    last_debug_state_ = Geometric_AttitudeControl_DebugState_t{};
    accepted_hover_thrust_ = param_.hover_thrust_init;
    last_selected_hover_anchor_ = param_.hover_thrust_init;
}

void Geometric_AttitudeControl::feed_thrust_estimator(const thrust_estimator::Input_t& input) {
    if (!hover_thrust_estimator_) {
        return;
    }
    if (!std::isfinite(input.thrust_cmd) || input.thrust_cmd <= 0.0) {
        return;
    }
    hover_thrust_estimator_->update(input);
}

void Geometric_AttitudeControl::seed_hover_thrust_estimator(double hover_thrust) {
    if (!hover_thrust_estimator_) {
        return;
    }
    if (!std::isfinite(hover_thrust) || hover_thrust <= 0.0) {
        return;
    }
    hover_thrust_estimator_->seed_hover_thrust(hover_thrust);
    accepted_hover_thrust_ =
        std::clamp(hover_thrust, param_.hover_thrust_min, param_.hover_thrust_max);
}

bool Geometric_AttitudeControl::thrust_estimator_should_estimate_onlyhover() const {
    return hover_thrust_estimator_ ? hover_thrust_estimator_->should_estimate_onlyhover() : true;
}

void Geometric_AttitudeControl::set_fixed_anchor_override(double anchor) {
    if (!std::isfinite(anchor) || anchor <= 0.0) {
        fixed_anchor_override_ = -1.0;
        return;
    }
    fixed_anchor_override_ =
        std::clamp(anchor, param_.hover_thrust_min, param_.hover_thrust_max);
}

double Geometric_AttitudeControl::get_accepted_hover_thrust() const {
    // 优先返回 estimator 当前值,跨 RLS/EKF 统一行为:
    //   - RLS (type=0): 没有 converged 概念,只要值合法就采纳
    //   - EKF (type=1): 还要求 converged() == true
    // 估计器未就绪或值非法时回退到 accepted_hover_thrust_(= hover_thrust_init 初值)。
    if (hover_thrust_estimator_) {
        const double h = hover_thrust_estimator_->get_hover_thrust();
        const bool valid =
            std::isfinite(h) && h >= param_.hover_thrust_min && h <= param_.hover_thrust_max;
        if (valid) {
            const bool need_converged = (param_.hover_thrust_estimator_type == 1);
            if (!need_converged || hover_thrust_estimator_->converged()) {
                return h;
            }
        }
    }
    return accepted_hover_thrust_;
}

void Geometric_AttitudeControl::refresh_dt(const ros::Time& stamp) {
    const double nominal_dt = 1.0 / std::max(1.0, param_.controller_hz);
    // 用实测时间戳差替代固定 dt：上限取 5 倍标称步长以抑制大跳变，
    // 时间戳无效或回退时退化为标称步长。
    if (stamp.isZero() || last_call_stamp_.isZero()) {
        current_dt_ = nominal_dt;
    } else {
        const double dt = (stamp - last_call_stamp_).toSec();
        if (dt > 0.0 && dt < 5.0 * nominal_dt) {
            current_dt_ = dt;
        } else {
            current_dt_ = nominal_dt;
        }
    }
    last_call_stamp_ = stamp;
}


Geometric_AttitudeControl_Output_t Geometric_AttitudeControl::calculateControl(
    const controller_data_types::TargetTrajectoryPoint_t& des_state,
    const control_common::UAVStateEstimate& current_odom,
    ThrustCommandPolicy thrust_policy) {

    refresh_dt(current_odom.timestamp);

    if (last_control_type_ == Control_Type::Velocity_) {
        reset_integral();
    }
    last_control_type_ = Control_Type::Trajectory_;

    // ── 提取当前状态 ──────────────────────────────────────────────────────
    const Eigen::Vector3d mav_pos = current_odom.position;
    const Eigen::Vector3d mav_vel = current_odom.velocity;
    const Eigen::Quaterniond& curr_att = current_odom.orientation;

    // ── 提取期望状态 ──────────────────────────────────────────────────────
    const Eigen::Vector3d target_pos = des_state.position;
    const Eigen::Vector3d target_vel = des_state.velocity;
    const Eigen::Vector3d target_acc = des_state.acceleration;
    // TargetTrajectoryPoint_t 已改为原始类型，直接使用 yaw
    // 同步缓存值，兼容其他路径依赖 last_desired_yaw_ 的语义
    last_desired_yaw_ = des_state.yaw;
    const double target_yaw = des_state.yaw;

    // ── 位置环：轨迹点 → 期望加速度 ──────────────────────────────────────
    const Eigen::Vector3d a_des =
        controlPosition(target_pos, target_vel, target_acc, mav_pos, mav_vel, target_yaw);

    // ── 姿态环：期望加速度 → body rate + 归一化推力 ───────────────────────
    Eigen::Vector4d bodyrate_cmd;
    computeBodyRateCmd(bodyrate_cmd, a_des, curr_att, target_yaw);

    // ── 填充输出 ──────────────────────────────────────────────────────────
    Geometric_AttitudeControl_Output_t output;
    output.bodyrates = bodyrate_cmd.head(3);
    output.thrust = compose_thrust_command(bodyrate_cmd(3), thrust_policy);
    // 期望姿态由期望加速度方向 + 偏航角决定（主要用于可视化/调试）
    output.orientation = acc2quaternion(a_des, target_yaw);
    update_debug_state(
        des_state, current_odom, a_des, output, Control_Type::Trajectory_, thrust_policy);
    return output;
}

Geometric_AttitudeControl_Output_t Geometric_AttitudeControl::calculateVelocityControl(
    const controller_data_types::TargetVelocity_t& des_state,
    const control_common::UAVStateEstimate& current_odom,
    ThrustCommandPolicy thrust_policy) {

    refresh_dt(current_odom.timestamp);

    const bool fixed_height_active = des_state.fixed_height > 0.0;
    if (last_control_type_ != Control_Type::Velocity_) {
        reset_integral();
    } else if (fixed_height_active != last_velocity_fixed_height_active_) {
        // fixed_height 模式切换时只清理 z 轴积分，避免高度环残留影响下一段速度控制。
        reset_vertical_integral();
    }
    last_control_type_ = Control_Type::Velocity_;
    last_velocity_fixed_height_active_ = fixed_height_active;

    const Eigen::Quaterniond& curr_att = current_odom.orientation;

    Eigen::Vector3d target_vel = des_state.velocity;
    if (fixed_height_active) {
        target_vel.z() = 0.0;
    }
    last_desired_yaw_ = des_state.yaw;
    const double target_yaw = des_state.yaw;

    const Eigen::Vector3d a_des = controlVelocity(des_state, current_odom, target_yaw);

    Eigen::Vector4d bodyrate_cmd;
    computeBodyRateCmd(bodyrate_cmd, a_des, curr_att, target_yaw);

    Geometric_AttitudeControl_Output_t output;
    output.bodyrates = bodyrate_cmd.head(3);
    output.thrust = compose_thrust_command(bodyrate_cmd(3), thrust_policy);
    output.orientation = acc2quaternion(a_des, target_yaw);
    controller_data_types::TargetTrajectoryPoint_t debug_state;
    debug_state.position = current_odom.position;
    if (fixed_height_active) {
        debug_state.position.z() = des_state.fixed_height;
    }
    debug_state.velocity = target_vel;
    debug_state.acceleration = Eigen::Vector3d::Zero();
    debug_state.jerk = Eigen::Vector3d::Zero();
    debug_state.yaw = target_yaw;
    debug_state.yaw_rate = des_state.yaw_rate;
    update_debug_state(
        debug_state, current_odom, a_des, output, Control_Type::Velocity_, thrust_policy);
    return output;
}

double Geometric_AttitudeControl::select_hover_anchor(ThrustCommandPolicy thrust_policy) const {
    if (thrust_policy == ThrustCommandPolicy::UseFixedAnchor) {
        // 优先使用上层 snapshot 的覆盖值(降落入口处会从 estimator 取一次),
        // 否则回退到 yaml 静态 hover_thrust_init。
        if (fixed_anchor_override_ > 0.0 && std::isfinite(fixed_anchor_override_)) {
            return std::clamp(fixed_anchor_override_,
                              param_.hover_thrust_min,
                              param_.hover_thrust_max);
        }
        return param_.hover_thrust_init;
    }

    if (!hover_thrust_estimator_) {
        return accepted_hover_thrust_;
    }

    const double hover_thrust = hover_thrust_estimator_->get_hover_thrust();
    const bool valid_estimate = std::isfinite(hover_thrust) &&
                                hover_thrust >= param_.hover_thrust_min &&
                                hover_thrust <= param_.hover_thrust_max;

    if (param_.hover_thrust_estimator_type != 1) {
        if (valid_estimate) {
            return hover_thrust;
        }
        return accepted_hover_thrust_;
    }

    if (!valid_estimate || !hover_thrust_estimator_->converged()) {
        return accepted_hover_thrust_;
    }

    const double max_step =
        std::max(0.0, param_.accepted_hover_thrust_rate) * std::max(current_dt_, 1e-3);
    accepted_hover_thrust_ =
        std::clamp(hover_thrust,
                   accepted_hover_thrust_ - max_step,
                   accepted_hover_thrust_ + max_step);
    accepted_hover_thrust_ =
        std::clamp(accepted_hover_thrust_, param_.hover_thrust_min, param_.hover_thrust_max);
    return accepted_hover_thrust_;
}

double Geometric_AttitudeControl::normalize_collective_acc(double collective_acc,
                                                           double hover_anchor) const {
    const double gravity = std::max(param_.gravity, 1e-6);
    return std::clamp(hover_anchor * (collective_acc / gravity), 0.0, 0.95);
}

double Geometric_AttitudeControl::compose_thrust_command(
    double collective_acc,
    ThrustCommandPolicy thrust_policy) const {
    last_selected_hover_anchor_ = select_hover_anchor(thrust_policy);
    return normalize_collective_acc(collective_acc, last_selected_hover_anchor_);
}

// ─────────────────────────────────────────────────────────────────────────
// poscontroller
// 位置 PD 控制器，输入误差均为 target - current（正值代表落后于目标）
// ─────────────────────────────────────────────────────────────────────────
Eigen::Vector3d Geometric_AttitudeControl::poscontroller(const Eigen::Vector3d& pos_error,
                                                         const Eigen::Vector3d& vel_error) {
    const double dt = current_dt_;

    // 三轴微分项（D）
    // 第一帧（reset 后）跳过差分，避免 last_*_error_ 为零导致的虚假大冲击。
    Eigen::Vector3d pos_error_dot = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_error_dot = Eigen::Vector3d::Zero();
    if (!first_run_) {
        pos_error_dot = (pos_error - last_pos_error_) / dt;
        vel_error_dot = (vel_error - last_vel_error_) / dt;
    }
    last_pos_error_ = pos_error;
    last_vel_error_ = vel_error;
    first_run_ = false;

    // D 项输出单独限幅，防止速度前馈引入后误差变化率过大导致冲击。
    const Eigen::Vector3d d_term_raw = param_.pos_kd.asDiagonal() * pos_error_dot +
                                       param_.vel_kd.asDiagonal() * vel_error_dot;
    const double d_term_norm = d_term_raw.norm();
    const Eigen::Vector3d d_term = (d_term_norm > param_.max_d_acc && d_term_norm > 1e-6)
                                       ? d_term_raw * (param_.max_d_acc / d_term_norm)
                                       : d_term_raw;

    // 用上一拍的积分先算一次未饱和输出，判定本拍是否处于饱和区（D 项已限幅）
    const Eigen::Vector3d a_fb_unsat = param_.pos_kp.asDiagonal() * pos_error +
                                       param_.pos_ki.asDiagonal() * integral_pos_ +
                                       d_term +
                                       param_.vel_kp.asDiagonal() * vel_error +
                                       param_.vel_ki.asDiagonal() * integral_vel_;
    const bool saturated = a_fb_unsat.norm() > param_.max_acc;
    debug_position_error_dot_ = pos_error_dot;
    debug_velocity_error_dot_ = vel_error_dot;
    debug_derivative_term_raw_ = d_term_raw;
    debug_derivative_term_ = d_term;
    debug_pid_feedback_acceleration_unsaturated_ = a_fb_unsat;
    debug_pid_accel_saturated_ = saturated;

    // 条件积分 anti-windup：饱和时按轴判断，只允许"误差与积分异号"的方向继续积分
    // 即放电方向允许、充电方向冻结，避免饱和后的过冲与滞回。
    // 单轴硬限幅作为兜底保留。
    for (int i = 0; i < 3; ++i) {
        const bool allow_pos = !saturated || (pos_error[i] * integral_pos_[i] < 0.0);
        if (allow_pos) {
            integral_pos_[i] += pos_error[i] * dt;
        }
        const bool allow_vel = !saturated || (vel_error[i] * integral_vel_[i] < 0.0);
        if (allow_vel) {
            integral_vel_[i] += vel_error[i] * dt;
        }

        if (std::abs(param_.pos_ki[i]) > 1e-6) {
            const double lim = param_.int_max_pos[i] / std::max(std::abs(param_.pos_ki[i]), 1e-6);
            integral_pos_[i] = std::max(-lim, std::min(lim, integral_pos_[i]));
        } else {
            integral_pos_[i] = 0.0;
        }

        if (std::abs(param_.vel_ki[i]) > 1e-6) {
            const double lim = param_.int_max_vel[i] / std::max(std::abs(param_.vel_ki[i]), 1e-6);
            integral_vel_[i] = std::max(-lim, std::min(lim, integral_vel_[i]));
        } else {
            integral_vel_[i] = 0.0;
        }
    }

    // 用更新后的积分重新合成反馈加速度（D 项已限幅）
    Eigen::Vector3d a_fb = param_.pos_kp.asDiagonal() * pos_error +
                           param_.pos_ki.asDiagonal() * integral_pos_ +
                           d_term +
                           param_.vel_kp.asDiagonal() * vel_error +
                           param_.vel_ki.asDiagonal() * integral_vel_;
    debug_integral_term_ =
        param_.pos_ki.asDiagonal() * integral_pos_ +
        param_.vel_ki.asDiagonal() * integral_vel_;

    // 对输出加速度进行球形限幅，保持方向不变
    if (a_fb.norm() > param_.max_acc) {
        a_fb = (param_.max_acc / a_fb.norm()) * a_fb;
        debug_pid_accel_saturated_ = true;
    }
    debug_pid_feedback_acceleration_ = a_fb;
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

    // 参考姿态：由"期望加速度 + 重力补偿"构成的期望推力方向，再叠加偏航角。
    // 即 q_ref 表示稳态下推力轴应指向的方向，用于将转子阻力补偿投影到机体系。
    const Eigen::Quaterniond q_ref = acc2quaternion(target_acc - gravity_vec, mav_yaw);
    const Eigen::Matrix3d R_ref = q_ref.toRotationMatrix();

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

Eigen::Vector3d Geometric_AttitudeControl::controlVelocity(
    const controller_data_types::TargetVelocity_t& des_state,
    const control_common::UAVStateEstimate& current_odom,
    double target_yaw) {

    const Eigen::Vector3d gravity_vec(0.0, 0.0, -param_.gravity);
    Eigen::Vector3d target_vel = des_state.velocity;
    const Eigen::Vector3d mav_vel = current_odom.velocity;
    const bool fixed_height_active = des_state.fixed_height > 0.0;

    Eigen::Vector3d a_fb;
    if (fixed_height_active) {
        target_vel.z() = 0.0;
        const Eigen::Vector3d pos_error(0.0, 0.0, des_state.fixed_height - current_odom.position.z());
        const Eigen::Vector3d vel_error = target_vel - mav_vel;
        a_fb = poscontroller(pos_error, vel_error);
    } else {
        const Eigen::Vector3d vel_error = target_vel - mav_vel;
        // 纯速度模式不引入位置环：传入零位置误差，由 poscontroller 内部统一处理积分/D 项/限幅。
        // 同时清掉位置积分残留，避免下一次切回轨迹模式时被旧值污染。
        integral_pos_.setZero();
        last_pos_error_.setZero();
        a_fb = poscontroller(Eigen::Vector3d::Zero(), vel_error);
    }

    // 参考姿态与 controlPosition 对齐：用"反馈加速度 + 重力补偿"作为期望推力方向。
    // 速度控制场景下没有显式 target_acc，用 a_fb 作为期望加速度的最佳近似。
    const Eigen::Quaterniond q_ref = acc2quaternion(a_fb - gravity_vec, target_yaw);
    const Eigen::Matrix3d R_ref = q_ref.toRotationMatrix();
    const Eigen::Vector3d a_rd = R_ref * param_.D.asDiagonal() * R_ref.transpose() * target_vel;
    return a_fb - a_rd - gravity_vec;
}

// ─────────────────────────────────────────────────────────────────────────
// computeBodyRateCmd
// 期望加速度 → 期望姿态 → SO3 姿态误差解算 → body rate + thrust_acc
// ─────────────────────────────────────────────────────────────────────────
void Geometric_AttitudeControl::computeBodyRateCmd(Eigen::Vector4d& bodyrate_cmd,
                                                   const Eigen::Vector3d& a_des,
                                                   const Eigen::Quaterniond& curr_att,
                                                   double mav_yaw) {

    // 由期望加速度方向 + 偏航角计算期望姿态四元数
    const Eigen::Quaterniond q_des = acc2quaternion(a_des, mav_yaw);

    // 使用 SO3 姿态误差解算期望 body rate。
    bodyrate_cmd.head(3) = solve_so3_bodyrate(curr_att, q_des);

    // 输出 collective thrust 对应的总加速度需求。
    // 这里优先匹配世界系 z 轴加速度：u * zb.z = a_des.z。
    // 相比 a_des·zb，在横向机动且姿态尚未到位时能更好抑制先升后降的高度瞬态。
    // 适用倾角范围：约 ±acos(zb_z_min)（默认 ~60°），超出后该近似误差显著。
    // 保护：zb_z 下限避免除零；a_des.z() 取下限避免大倾角下指令为负后被 clamp 到 0。
    const Eigen::Vector3d zb = curr_att.toRotationMatrix().col(2);
    const double zb_z_min = std::clamp(param_.zb_z_min, 0.1, 1.0);
    const double zb_z = std::clamp(zb.z(), zb_z_min, 1.0);
    const double a_des_z = std::max(a_des.z(), 0.0);
    bodyrate_cmd(3) = a_des_z / zb_z;
}

// ─────────────────────────────────────────────────────────────────────────
// acc2quaternion
// 由期望加速度方向（即期望推力方向）和偏航角构造期望姿态四元数
// ─────────────────────────────────────────────────────────────────────────
Eigen::Quaterniond Geometric_AttitudeControl::acc2quaternion(const Eigen::Vector3d& vector_acc,
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
    const Eigen::Vector4d q_vec = eigen_helper::rot2Quaternion(rotmat);
    return Eigen::Quaterniond(q_vec(0), q_vec(1), q_vec(2), q_vec(3));
}


// ═══════════════════════════════════════════════════════════════════════════
// Debug 误差记录
// ═══════════════════════════════════════════════════════════════════════════
Eigen::Vector3d Geometric_AttitudeControl::compute_attitude_error(
    const Eigen::Quaterniond& curr_att,
    const Eigen::Quaterniond& ref_att) const {
    return so3_attitude_error(curr_att, ref_att);
}

void Geometric_AttitudeControl::update_debug_state(
    const controller_data_types::TargetTrajectoryPoint_t& des_state,
    const control_common::UAVStateEstimate& current_odom,
    const Eigen::Vector3d& desired_acc,
    const Geometric_AttitudeControl_Output_t& output,
    Control_Type control_type,
    ThrustCommandPolicy thrust_policy) {
    const Eigen::Quaterniond& q = current_odom.orientation;
    const double current_yaw =
        std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    double hover_thrust_estimate = accepted_hover_thrust_;
    if (hover_thrust_estimator_) {
        const double estimate = hover_thrust_estimator_->get_hover_thrust();
        if (std::isfinite(estimate)) {
            hover_thrust_estimate = estimate;
        }
    }

    last_debug_state_.valid = true;
    last_debug_state_.stamp = current_odom.timestamp.isZero() ? ros::Time::now() : current_odom.timestamp;
    last_debug_state_.control_type = control_type;
    last_debug_state_.reference = des_state;
    last_debug_state_.odom = current_odom;
    last_debug_state_.position_error = des_state.position - current_odom.position;
    last_debug_state_.velocity_error = des_state.velocity - current_odom.velocity;
    last_debug_state_.yaw_error = eigen_helper::wrap_angle(des_state.yaw - current_yaw);
    last_debug_state_.attitude_error = compute_attitude_error(current_odom.orientation, output.orientation);
    last_debug_state_.current_dt = current_dt_;
    last_debug_state_.integral_pos = integral_pos_;
    last_debug_state_.integral_vel = integral_vel_;
    last_debug_state_.last_pos_error = last_pos_error_;
    last_debug_state_.last_vel_error = last_vel_error_;
    last_debug_state_.position_error_dot = debug_position_error_dot_;
    last_debug_state_.velocity_error_dot = debug_velocity_error_dot_;
    last_debug_state_.derivative_term_raw = debug_derivative_term_raw_;
    last_debug_state_.derivative_term = debug_derivative_term_;
    last_debug_state_.integral_term = debug_integral_term_;
    last_debug_state_.pid_feedback_acceleration_unsaturated =
        debug_pid_feedback_acceleration_unsaturated_;
    last_debug_state_.pid_feedback_acceleration = debug_pid_feedback_acceleration_;
    last_debug_state_.pid_accel_saturated = debug_pid_accel_saturated_;
    last_debug_state_.desired_acceleration = desired_acc;
    last_debug_state_.desired_orientation = output.orientation;
    last_debug_state_.desired_bodyrates = output.bodyrates;
    last_debug_state_.desired_thrust = output.thrust;
    last_debug_state_.thrust_policy = thrust_policy;
    last_debug_state_.hover_thrust_estimate = hover_thrust_estimate;
    last_debug_state_.accepted_hover_thrust = accepted_hover_thrust_;
    last_debug_state_.selected_hover_anchor = last_selected_hover_anchor_;
    last_debug_state_.fixed_anchor_override = fixed_anchor_override_;
    last_debug_state_.fixed_anchor_active =
        thrust_policy == ThrustCommandPolicy::UseFixedAnchor;
}
