#include "controller/core_algorithm/hoverthrust_estimator.hpp"

#include <ros/console.h>
#include <algorithm>
#include <cmath>

namespace thrust_estimator {

namespace {

bool isFiniteVector(const Eigen::Vector3d& value) {
    return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

bool isFiniteQuaternion(const Eigen::Quaterniond& value) {
    return std::isfinite(value.w()) && std::isfinite(value.x()) && std::isfinite(value.y()) &&
           std::isfinite(value.z()) && value.norm() > 1e-6;
}

double clampPositive(double value, double min_value, double max_value, double fallback) {
    if (!std::isfinite(value)) {
        return fallback;
    }
    const double ordered_min = std::min(min_value, max_value);
    const double ordered_max = std::max(min_value, max_value);
    return std::clamp(value, ordered_min, ordered_max);
}

}  // namespace

void HoverThrustEstimator::load_param(const Param_t& param) {
    hover_thrust_min_ = std::clamp(param.hover_thrust_min, 0.0, 1.0);
    hover_thrust_max_ = std::clamp(param.hover_thrust_max, hover_thrust_min_, 1.0);
    gravity_ = std::max(1e-6, param.gravity);
    hover_thrust_ = std::clamp(param.hover_thrust, hover_thrust_min_, hover_thrust_max_);
}

void HoverThrustEstimator::seed_hover_thrust(double hover_thrust) {
    hover_thrust_ = std::clamp(hover_thrust, hover_thrust_min_, hover_thrust_max_);
}

void RLS_HoverThrustEstimator::seed_hover_thrust(double hover_thrust) {
    HoverThrustEstimator::seed_hover_thrust(hover_thrust);
    thr2acc_ = gravity_ / std::max(hover_thrust_, 1e-3);
    input_history_.clear();
}

void RLS_HoverThrustEstimator::update(const Input_t& input) {
    if (input.stamp.isZero()) {
        return;
    }

    input_history_.push_back(input);
    while (!input_history_.empty()) {
        const Input_t& candidate = input_history_.front();
        const double time_passed = (input.stamp - candidate.stamp).toSec();

        // Keep the same delayed-sample strategy as the original controller logic.
        if (time_passed > 0.045) {
            input_history_.pop_front();
            continue;
        }
        if (time_passed < 0.035) {
            return;
        }

        if (!std::isfinite(thr2acc_) || thr2acc_ <= 1e-6) {
            thr2acc_ = gravity_ / std::max(hover_thrust_, 1e-3);
        }

        const double thrust_cmd = candidate.thrust_cmd;
        if (thrust_cmd <= 0.02 || thrust_cmd >= 0.95) {
            input_history_.pop_front();
            return;
        }

        const Eigen::Vector3d zb = candidate.attitude.toRotationMatrix().col(2);
        const double tilt_cos = std::clamp(zb.dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0);
        const double phi = thrust_cmd * tilt_cos;  // mapped z-axis collective thrust
        if (std::abs(phi) < 1e-3) {
            input_history_.pop_front();
            return;
        }

        // Vertical thrust model in world frame:
        // a_z + g = thr2acc * (u * cos(tilt))
        const double y = candidate.acceleration_w.z() + gravity_;
        const double gamma = 1.0 / (rho2_ + phi * P_ * phi);
        const double K = gamma * P_ * phi;

        thr2acc_ = std::max(thr2acc_ + K * (y - phi * thr2acc_), 1e-3);
        P_ = std::max((1.0 - K * phi) * P_ / rho2_, 1e-9);
        hover_thrust_ = std::clamp(gravity_ / thr2acc_, hover_thrust_min_, hover_thrust_max_);

        input_history_.pop_front();
        while (input_history_.size() > 100) {
            input_history_.pop_front();
        }
        return;
    }
}

double RLS_HoverThrustEstimator::get_hover_thrust() const {
    return hover_thrust_;
}

void EKF_HoverThrustEstimator::load_param(const Param_t& param) {
    HoverThrustEstimator::load_param(param);

    onlyhover_estimate_ = param.ekf_onlyhover_estimate;
    Q_ = std::max(param.ekf_Q, 0.0);
    R_ = std::max(param.ekf_R, 1e-9);
    P0_ = std::max(param.ekf_P0, 1e-9);
    P_ = P0_;
    p_min_ = std::max(param.ekf_P_min, 1e-12);
    p_max_ = std::max(param.ekf_P_max, p_min_);
    P_ = std::clamp(P_, p_min_, p_max_);

    delay_min_s_ = std::max(param.ekf_delay_min_s, 0.0);
    delay_max_s_ = std::max(param.ekf_delay_max_s, delay_min_s_);
    innovation_gate_ = std::max(param.ekf_innovation_gate, 1e-6);

    min_thrust_cmd_ = clampPositive(param.ekf_min_thrust_cmd, 0.0, 0.95, min_thrust_cmd_);
    max_thrust_cmd_ = clampPositive(param.ekf_max_thrust_cmd, min_thrust_cmd_, 0.95, max_thrust_cmd_);
    min_tilt_cos_hover_ = clampPositive(param.ekf_min_tilt_cos_hover, 0.0, 1.0, min_tilt_cos_hover_);
    min_tilt_cos_move_ = clampPositive(param.ekf_min_tilt_cos_move, 0.0, 1.0, min_tilt_cos_move_);
    max_abs_acc_z_hover_ = std::max(param.ekf_max_abs_acc_z_hover, 0.0);
    max_abs_acc_z_move_ = std::max(param.ekf_max_abs_acc_z_move, 0.0);

    convergence_p_threshold_ = std::max(param.ekf_convergence_p_threshold, p_min_);
    convergence_hold_s_ = std::max(param.ekf_convergence_hold_s, 0.0);
    adaptive_R_enabled_ = param.ekf_adaptive_R_enabled;
    R_min_ = std::max(param.ekf_R_min, 1e-9);
    R_max_ = std::max(param.ekf_R_max, R_min_);
    R_ = std::clamp(R_, R_min_, R_max_);

    seed_hover_thrust(hover_thrust_);
}

void EKF_HoverThrustEstimator::seed_hover_thrust(double hover_thrust) {
    HoverThrustEstimator::seed_hover_thrust(hover_thrust);
    input_history_.clear();
    last_input_stamp_ = ros::Time(0);
    last_update_stamp_ = ros::Time(0);
    converged_since_ = ros::Time(0);
    converged_ = false;
    P_ = std::clamp(P0_, p_min_, p_max_);
    reset_debug();
}

void EKF_HoverThrustEstimator::reset_debug() {
    debug_ = UpdateStatus{};
    debug_.hover_thrust = hover_thrust_;
    debug_.P = P_;
    debug_.Q = Q_;
    debug_.R = R_;
    debug_.converged = converged_;
}

void EKF_HoverThrustEstimator::reject_update(const std::string& reason) {
    debug_.updated = false;
    debug_.rejected = true;
    debug_.reject_reason = reason;
    debug_.hover_thrust = hover_thrust_;
    debug_.P = P_;
    debug_.Q = Q_;
    debug_.R = R_;
    debug_.converged = converged_;
}

void EKF_HoverThrustEstimator::update_convergence(const ros::Time& stamp, bool accepted) {
    if (!accepted) {
        converged_since_ = ros::Time(0);
        converged_ = false;
        return;
    }

    const bool stable = P_ < convergence_p_threshold_ && debug_.nis < innovation_gate_;
    if (!stable) {
        converged_since_ = ros::Time(0);
        converged_ = false;
        return;
    }

    if (converged_since_.isZero()) {
        converged_since_ = stamp;
    }

    converged_ = convergence_hold_s_ <= 0.0 ||
                 (!stamp.isZero() && (stamp - converged_since_).toSec() >= convergence_hold_s_);
}

void EKF_HoverThrustEstimator::update_R(double innovation, bool accepted) {
    if (!adaptive_R_enabled_ || !std::isfinite(innovation)) {
        return;
    }

    const double innovation_var_sample = innovation * innovation;
    const double alpha = accepted ? 0.02 : 0.10;
    R_ = std::clamp((1.0 - alpha) * R_ + alpha * innovation_var_sample, R_min_, R_max_);
}

void EKF_HoverThrustEstimator::update(const Input_t& input) {
    reset_debug();
    debug_.stamp = input.stamp;
    debug_.acc_z = input.acceleration_w.z();
    debug_.velocity_z = input.velocity_w.z();
    debug_.input_thrust_cmd = input.thrust_cmd;

    if (input.stamp.isZero()) {
        reject_update("zero_stamp");
        return;
    }
    if (!last_input_stamp_.isZero() && input.stamp <= last_input_stamp_) {
        reject_update("non_monotonic_stamp");
        return;
    }
    last_input_stamp_ = input.stamp;

    if (!std::isfinite(input.thrust_cmd) || !isFiniteVector(input.acceleration_w) ||
        !isFiniteVector(input.velocity_w) || !isFiniteQuaternion(input.attitude)) {
        reject_update("non_finite_input");
        return;
    }

    input_history_.push_back(input);
    while (input_history_.size() > 200) {
        input_history_.pop_front();
    }

    while (!input_history_.empty()) {
        const double time_passed = (input.stamp - input_history_.front().stamp).toSec();
        if (time_passed > delay_max_s_) {
            input_history_.pop_front();
            continue;
        }
        break;
    }

    if (input_history_.empty()) {
        reject_update("empty_history");
        return;
    }

    const Input_t& candidate = input_history_.front();
    const double sample_delay = (input.stamp - candidate.stamp).toSec();
    debug_.sample_delay = sample_delay;

    if (sample_delay < delay_min_s_) {
        debug_.rejected = true;
        debug_.reject_reason = "waiting_delay_window";
        return;
    }

    const double u = candidate.thrust_cmd;
    debug_.delayed_thrust_cmd = u;
    if (!std::isfinite(u) || u <= min_thrust_cmd_ || u >= max_thrust_cmd_) {
        reject_update("thrust_cmd_out_of_range");
        input_history_.pop_front();
        return;
    }

    Eigen::Quaterniond q = candidate.attitude;
    q.normalize();
    const Eigen::Vector3d zb = q.toRotationMatrix().col(2);
    const double tilt_cos = std::clamp(zb.dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0);
    debug_.tilt_cos = tilt_cos;
    if (!std::isfinite(tilt_cos) || tilt_cos <= min_tilt_cos_hover_) {
        reject_update("tilt_cos_out_of_range");
        input_history_.pop_front();
        return;
    }

    if (std::abs(input.acceleration_w.z()) >= max_abs_acc_z_hover_) {
        reject_update("acc_z_out_of_range");
        input_history_.pop_front();
        return;
    }

    if (last_update_stamp_.isZero()) {
        last_update_stamp_ = input.stamp;
        reject_update("initialize_update_stamp");
        input_history_.pop_front();
        return;
    }

    const double dt = (input.stamp - last_update_stamp_).toSec();
    if (dt <= 0.0 || dt > 0.5) {
        last_update_stamp_ = input.stamp;
        reject_update("invalid_dt");
        input_history_.pop_front();
        return;
    }

    const double P_predict = std::clamp(P_ + Q_ * dt, p_min_, p_max_);
    const double x = std::clamp(hover_thrust_, hover_thrust_min_, hover_thrust_max_);
    const double y = input.acceleration_w.z() + gravity_;
    const double h = gravity_ * u * tilt_cos / std::max(x, 1e-6);
    const double H = -gravity_ * u * tilt_cos / std::max(x * x, 1e-12);
    const double innovation = y - h;
    const double S = H * P_predict * H + R_;

    debug_.measurement_y = y;
    debug_.measurement_pred = h;
    debug_.H = H;
    debug_.innovation = innovation;
    debug_.innovation_cov = S;

    if (!std::isfinite(y) || !std::isfinite(h) || !std::isfinite(H) || !std::isfinite(S) ||
        S <= 1e-12) {
        P_ = P_predict;
        reject_update("invalid_measurement_model");
        input_history_.pop_front();
        return;
    }

    const double nis = innovation * innovation / S;
    debug_.nis = nis;
    if (!std::isfinite(nis) || nis >= innovation_gate_) {
        P_ = P_predict;
        update_R(innovation, false);
        update_convergence(input.stamp, false);
        reject_update("innovation_gate");
        input_history_.pop_front();
        return;
    }

    const double K = P_predict * H / S;
    double x_new = x + K * innovation;
    if (!std::isfinite(x_new)) {
        P_ = P_predict;
        reject_update("non_finite_state");
        input_history_.pop_front();
        return;
    }
    x_new = std::clamp(x_new, hover_thrust_min_, hover_thrust_max_);
    const double P_new = (1.0 - K * H) * P_predict;

    hover_thrust_ = x_new;
    P_ = std::clamp(P_new, p_min_, p_max_);
    last_update_stamp_ = input.stamp;
    update_R(innovation, true);

    debug_.updated = true;
    debug_.rejected = false;
    debug_.reject_reason.clear();
    debug_.hover_thrust = hover_thrust_;
    debug_.P = P_;
    debug_.Q = Q_;
    debug_.R = R_;
    debug_.K = K;
    debug_.converged = converged_;
    update_convergence(input.stamp, true);
    debug_.converged = converged_;

    input_history_.pop_front();
}

double EKF_HoverThrustEstimator::get_hover_thrust() const {
    return hover_thrust_;
}

bool EKF_HoverThrustEstimator::converged() const {
    return converged_;
}

}  // namespace thrust_estimator
