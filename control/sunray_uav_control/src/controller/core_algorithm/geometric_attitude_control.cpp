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
#include <ros/console.h>
#include <algorithm>
#include <cmath>

// ═══════════════════════════════════════════════════════════════════════════
// Geometric_AttitudeControl 实现
// ═══════════════════════════════════════════════════════════════════════════

void Geometric_AttitudeControl::load_param(const Geometric_AttitudeControl_Param_t& param) {
    param_ = param;

    // 根据 attitude_type 实例化对应的姿态子控制器
    if (param_.attitude_type == 0) {
        att_controller_ = std::make_shared<Quaternion_Solver>(param_.attitude_tau);
    } else {
        att_controller_ = std::make_shared<SO3_Solver>(param_.attitude_tau);
    }

    // 悬停推力估计器由 core 持有，默认沿用 controller 侧现有配置语义。
    switch (param_.hover_thrust_estimator_type) {
    case 0:
        hover_thrust_estimator_ = std::make_unique<thrust_estimator::LowPass_HoverThrustEstimator>();
        break;
    case 2:
        hover_thrust_estimator_ = std::make_unique<thrust_estimator::Kalman_HoverThrustEstimator>();
        break;
    case 1:
    default:
        hover_thrust_estimator_ = std::make_unique<thrust_estimator::RLS_HoverThrustEstimator>();
        break;
    }

    thrust_estimator::Param_t estimator_param;
    estimator_param.gravity = param_.gravity;
    estimator_param.hover_thrust = param_.hover_thrust_init;
    hover_thrust_estimator_->load_param(estimator_param);

    last_thrust_cmd_ = 0.0;
    pending_thrust_cmd_ = 0.0;
    has_pending_thrust_cmd_ = false;
}

void Geometric_AttitudeControl::feed_thrust_estimator(const thrust_estimator::Input_t& input) {
    if (!hover_thrust_estimator_) {
        return;
    }

    thrust_estimator::Input_t estimator_input = input;
    estimator_input.thrust_cmd = (last_thrust_cmd_ > 0.0)
                                     ? last_thrust_cmd_
                                     : (has_pending_thrust_cmd_ ? pending_thrust_cmd_ : 0.0);
    hover_thrust_estimator_->update(estimator_input);
}

Geometric_AttitudeControl_Output_t Geometric_AttitudeControl::calculateControl(
    const controller_data_types::TargetTrajectoryPoint_t& des_state,
    const control_common::UAVStateEstimate& current_odom,
    ThrustCommandPolicy thrust_policy) {

    if (last_control_type_ == Control_Type::Velocity_) {
        reset_integral();
    }
    last_control_type_ = Control_Type::Trajectory_;

    advance_thrust_command_history();

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
    pending_thrust_cmd_ = output.thrust;
    has_pending_thrust_cmd_ = true;
    // 期望姿态由期望加速度方向 + 偏航角决定（主要用于可视化/调试）
    output.orientation = acc2quaternion(a_des, target_yaw);
#ifdef DEBUG
    const double axy_norm = a_des.head<2>().norm();
    const double pos_xy_err = (target_pos - mav_pos).head<2>().norm();
    const double vel_xy_err = (target_vel - mav_vel).head<2>().norm();
    if (axy_norm > 0.3 || pos_xy_err > 0.05 || vel_xy_err > 0.05) {
        const double tilt_cur = curr_att.toRotationMatrix()(2, 2);
        const double tilt_des = output.orientation.toRotationMatrix()(2, 2);
        ROS_INFO_THROTTLE(0.5,
                          "ctrl ez=%.2f vz=%.2f adz=%.2f axy=%.2f tc=%.3f td=%.3f ta=%.2f u=%.3f",
                          target_pos.z() - mav_pos.z(),
                          target_vel.z() - mav_vel.z(),
                          a_des.z(),
                          axy_norm,
                          tilt_cur,
                          tilt_des,
                          bodyrate_cmd(3),
                          output.thrust);
    }
#endif
    return output;
}

Geometric_AttitudeControl_Output_t Geometric_AttitudeControl::calculateVelocityControl(
    const controller_data_types::TargetVelocity_t& des_state,
    const control_common::UAVStateEstimate& current_odom,
    ThrustCommandPolicy thrust_policy) {

    if (last_control_type_ != Control_Type::Velocity_) {
        reset_integral();
    }
    last_control_type_ = Control_Type::Velocity_;

    advance_thrust_command_history();

    const Eigen::Vector3d mav_vel = current_odom.velocity;
    const Eigen::Quaterniond& curr_att = current_odom.orientation;

    const Eigen::Vector3d target_vel = des_state.velocity;
    last_desired_yaw_ = des_state.yaw;
    const double target_yaw = des_state.yaw;

    const Eigen::Vector3d a_des = controlVelocity(target_vel, mav_vel, target_yaw);

    Eigen::Vector4d bodyrate_cmd;
    computeBodyRateCmd(bodyrate_cmd, a_des, curr_att, target_yaw);

    Geometric_AttitudeControl_Output_t output;
    output.bodyrates = bodyrate_cmd.head(3);
    output.thrust = compose_thrust_command(bodyrate_cmd(3), thrust_policy);
    pending_thrust_cmd_ = output.thrust;
    has_pending_thrust_cmd_ = true;
    output.orientation = acc2quaternion(a_des, target_yaw);
#ifdef DEBUG
    const double vel_err = (target_vel - mav_vel).norm();
    if (vel_err > 0.05) {
        ROS_INFO_THROTTLE(0.5,
                          "ctrl vel ex=%.2f ey=%.2f ez=%.2f adz=%.2f axy=%.2f ta=%.2f u=%.3f",
                          target_vel.x() - mav_vel.x(),
                          target_vel.y() - mav_vel.y(),
                          target_vel.z() - mav_vel.z(),
                          a_des.z(),
                          a_des.head<2>().norm(),
                          bodyrate_cmd(3),
                          output.thrust);
    }
#endif
    return output;
}

void Geometric_AttitudeControl::advance_thrust_command_history() {
    if (!has_pending_thrust_cmd_) {
        return;
    }
    last_thrust_cmd_ = pending_thrust_cmd_;
}

double Geometric_AttitudeControl::map_thrust_acc_to_hover_anchor(double thrust_acc) const {
    double hover_anchor = param_.hover_thrust_init;
    if (hover_thrust_estimator_) {
        const double hover_thrust = hover_thrust_estimator_->get_hover_thrust();
        if (std::isfinite(hover_thrust) && hover_thrust > 0.0) {
            hover_anchor = hover_thrust;
        }
    }

    const double gravity = std::max(param_.gravity, 1e-6);
    return std::clamp(hover_anchor * (thrust_acc / gravity), 0.0, 0.95);
}

Geometric_AttitudeControl::ThrustModelMode Geometric_AttitudeControl::get_hover_estimator_mode() const {
    switch (param_.hover_thrust_estimator_type) {
    case 0:
        return ThrustModelMode::LowPassEstimator;
    case 2:
        return ThrustModelMode::KalmanEstimator;
    case 1:
    default:
        return ThrustModelMode::RLSEstimator;
    }
}

const char* Geometric_AttitudeControl::thrust_model_mode_name(ThrustModelMode mode) {
    switch (mode) {
    case ThrustModelMode::LinearFallback:
        return "LIN_FB";
    case ThrustModelMode::LinearForced:
        return "LIN_FORCE";
    case ThrustModelMode::LowPassEstimator:
        return "LP";
    case ThrustModelMode::KalmanEstimator:
        return "KF";
    case ThrustModelMode::RLSEstimator:
    default:
        return "RLS";
    }
}

void Geometric_AttitudeControl::log_thrust_model_usage(ThrustModelMode mode,
                                                       double thrust_cmd,
                                                       double hover_thrust) {
    const char* mode_name = thrust_model_mode_name(mode);

    if (!thrust_model_mode_initialized_ || last_thrust_model_mode_ != mode) {
#ifdef DEBUG
        if (mode == ThrustModelMode::LinearFallback) {
            ROS_WARN("thrust->%s u=%.3f", mode_name, thrust_cmd);
        } else if (mode == ThrustModelMode::LinearForced) {
            ROS_INFO("thrust->%s u=%.3f", mode_name, thrust_cmd);
        } else {
            ROS_INFO("thrust->%s ht=%.3f u=%.3f", mode_name, hover_thrust, thrust_cmd);
        }
#else
        if (mode == ThrustModelMode::LinearFallback) {
            ROS_WARN("thrust->%s", mode_name);
        } else if (mode == ThrustModelMode::LinearForced) {
            ROS_INFO("thrust->%s", mode_name);
        } else {
            ROS_INFO("thrust->%s", mode_name);
        }
#endif
        last_thrust_model_mode_ = mode;
        thrust_model_mode_initialized_ = true;
    }

#ifdef DEBUG
    if (mode == ThrustModelMode::LinearFallback) {
        ROS_WARN_THROTTLE(2.0, "thrust:%s u=%.3f", mode_name, thrust_cmd);
    } else if (mode == ThrustModelMode::LinearForced) {
        ROS_INFO_THROTTLE(2.0, "thrust:%s u=%.3f", mode_name, thrust_cmd);
    } else {
        ROS_INFO_THROTTLE(2.0, "thrust:%s ht=%.3f u=%.3f", mode_name, hover_thrust, thrust_cmd);
    }
#else
    if (mode == ThrustModelMode::LinearFallback) {
        ROS_WARN_THROTTLE(2.0, "thrust:%s", mode_name);
    } else if (mode == ThrustModelMode::LinearForced) {
        ROS_INFO_THROTTLE(2.0, "thrust:%s", mode_name);
    } else {
        ROS_INFO_THROTTLE(2.0, "thrust:%s", mode_name);
    }
#endif
}

double Geometric_AttitudeControl::compose_thrust_command(double thrust_acc,
                                                         ThrustCommandPolicy thrust_policy) {
    const double linear_fallback = map_thrust_acc_to_hover_anchor(thrust_acc);
    if (thrust_policy == ThrustCommandPolicy::ForceLinear) {
        log_thrust_model_usage(ThrustModelMode::LinearForced, linear_fallback, 0.0);
        return linear_fallback;
    }

    if (!hover_thrust_estimator_) {
        log_thrust_model_usage(ThrustModelMode::LinearFallback, linear_fallback, 0.0);
        return linear_fallback;
    }

    const double hover_thrust = hover_thrust_estimator_->get_hover_thrust();
    if (!std::isfinite(hover_thrust) || hover_thrust <= 0.0) {
        log_thrust_model_usage(ThrustModelMode::LinearFallback, linear_fallback, hover_thrust);
        return linear_fallback;
    }

    const double gravity = std::max(param_.gravity, 1e-6);
    const double thrust_cmd = hover_thrust * (thrust_acc / gravity);
    const double clamped_thrust_cmd = std::clamp(thrust_cmd, 0.0, 0.95);
    log_thrust_model_usage(get_hover_estimator_mode(), clamped_thrust_cmd, hover_thrust);
    return clamped_thrust_cmd;
}

// ─────────────────────────────────────────────────────────────────────────
// poscontroller
// 位置 PD 控制器，输入误差均为 target - current（正值代表落后于目标）
// ─────────────────────────────────────────────────────────────────────────
Eigen::Vector3d Geometric_AttitudeControl::poscontroller(const Eigen::Vector3d& pos_error,
                                                         const Eigen::Vector3d& vel_error) {
    const double dt = 1.0 / std::max(1.0, param_.controller_hz);

    // 三轴积分项（I）：支持位置环与速度环全量 PID
    integral_pos_ += pos_error * dt;
    integral_vel_ += vel_error * dt;

    // 将积分状态按“输出加速度上限”做等效限幅，避免积分饱和
    for (int i = 0; i < 3; ++i) {
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

    // 三轴微分项（D）
    const Eigen::Vector3d pos_error_dot = (pos_error - last_pos_error_) / dt;
    const Eigen::Vector3d vel_error_dot = (vel_error - last_vel_error_) / dt;
    last_pos_error_ = pos_error;
    last_vel_error_ = vel_error;

    // 三轴全量 PID 反馈（位置环 + 速度环）
    Eigen::Vector3d a_fb = param_.pos_kp.asDiagonal() * pos_error +
                           param_.pos_ki.asDiagonal() * integral_pos_ +
                           param_.pos_kd.asDiagonal() * pos_error_dot +
                           param_.vel_kp.asDiagonal() * vel_error +
                           param_.vel_ki.asDiagonal() * integral_vel_ +
                           param_.vel_kd.asDiagonal() * vel_error_dot;

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

Eigen::Vector3d Geometric_AttitudeControl::controlVelocity(const Eigen::Vector3d& target_vel,
                                                           const Eigen::Vector3d& mav_vel,
                                                           double target_yaw) {

    const Eigen::Vector3d gravity_vec(0.0, 0.0, -param_.gravity);
    const Eigen::Quaterniond q_ref = acc2quaternion(-gravity_vec, target_yaw);
    const Eigen::Matrix3d R_ref = q_ref.toRotationMatrix();
    const Eigen::Vector3d vel_error = target_vel - mav_vel;
    const double dt = 1.0 / std::max(1.0, param_.controller_hz);

    // 纯速度模式下不使用位置环，避免旧的位置积分残留影响输出。
    integral_pos_.setZero();
    last_pos_error_.setZero();

    integral_vel_ += vel_error * dt;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(param_.vel_ki[i]) > 1e-6) {
            const double lim = param_.int_max_vel[i] / std::max(std::abs(param_.vel_ki[i]), 1e-6);
            integral_vel_[i] = std::max(-lim, std::min(lim, integral_vel_[i]));
        } else {
            integral_vel_[i] = 0.0;
        }
    }

    const Eigen::Vector3d vel_error_dot = (vel_error - last_vel_error_) / dt;
    last_vel_error_ = vel_error;

    Eigen::Vector3d a_fb = param_.vel_kp.asDiagonal() * vel_error +
                           param_.vel_ki.asDiagonal() * integral_vel_ +
                           param_.vel_kd.asDiagonal() * vel_error_dot;

    if (a_fb.norm() > param_.max_acc) {
        a_fb = (param_.max_acc / a_fb.norm()) * a_fb;
    }

    const Eigen::Vector3d a_rd = R_ref * param_.D.asDiagonal() * R_ref.transpose() * target_vel;
    return a_fb - a_rd - gravity_vec;
}

// ─────────────────────────────────────────────────────────────────────────
// computeBodyRateCmd
// 期望加速度 → 期望姿态 → 调用姿态子控制器 → body rate + thrust_acc
// ─────────────────────────────────────────────────────────────────────────
void Geometric_AttitudeControl::computeBodyRateCmd(Eigen::Vector4d& bodyrate_cmd,
                                                   const Eigen::Vector3d& a_des,
                                                   const Eigen::Quaterniond& curr_att,
                                                   double mav_yaw) {

    // 由期望加速度方向 + 偏航角计算期望姿态四元数
    const Eigen::Quaterniond q_des = acc2quaternion(a_des, mav_yaw);

    // 调用姿态求解器，只计算期望角速度
    att_controller_->update(curr_att, q_des, a_des);

    // 填充 body rate（前三分量）
    bodyrate_cmd.head(3) = att_controller_->get_desired_rate();

    // 输出 collective thrust 对应的总加速度需求。
    // 这里优先匹配世界系 z 轴加速度：u * zb.z = a_des.z。
    // 相比 a_des·zb，在横向机动且姿态尚未到位时能更好抑制先升后降的高度瞬态。
    const Eigen::Vector3d zb = curr_att.toRotationMatrix().col(2);
    const double zb_z = std::clamp(zb.z(), 0.1, 1.0);
    bodyrate_cmd(3) = a_des.z() / zb_z;
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
    const Eigen::Vector4d q_vec = rot2Quaternion(rotmat);
    return Eigen::Quaterniond(q_vec(0), q_vec(1), q_vec(2), q_vec(3));
}
