#include "controller/core_algorithm/hoverthrust_estimator.hpp"

#include <ros/console.h>
#include <algorithm>
#include <cmath>

namespace thrust_estimator {

void HoverThrustEstimator::load_param(const Param_t& param) {
    gravity_ = std::max(1e-6, param.gravity);
    hover_thrust_ = std::clamp(param.hover_thrust, hover_thrust_min_, hover_thrust_max_);
}

void HoverThrustEstimator::seed_hover_thrust(double hover_thrust) {
    hover_thrust_ = std::clamp(hover_thrust, hover_thrust_min_, hover_thrust_max_);
}

void LowPass_HoverThrustEstimator::seed_hover_thrust(double hover_thrust) {
    HoverThrustEstimator::seed_hover_thrust(hover_thrust);
    last_stamp_ = ros::Time(0);
}

void LowPass_HoverThrustEstimator::update(const Input_t& input) {
    if (input.stamp.isZero()) {
        return;
    }

    if (last_stamp_.isZero()) {
        last_stamp_ = input.stamp;
        return;
    }

    const double dt = (input.stamp - last_stamp_).toSec();
    last_stamp_ = input.stamp;

    if (dt < 1e-4 || dt > 0.1) {
        return;
    }

    const double beta = dt / (adapt_tau_s_ + dt);
    hover_thrust_ = std::clamp((1.0 - beta) * hover_thrust_ + beta * input.thrust_cmd,
                    hover_thrust_min_,
                    hover_thrust_max_);
}

double LowPass_HoverThrustEstimator::get_hover_thrust() const {
    return hover_thrust_;
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

void Kalman_HoverThrustEstimator::seed_hover_thrust(double hover_thrust) {
    HoverThrustEstimator::seed_hover_thrust(hover_thrust);
    last_stamp_ = ros::Time(0);
}

void Kalman_HoverThrustEstimator::update(const Input_t& input) {
    if (input.stamp.isZero()) {
        return;
    }

    if (last_stamp_.isZero()) {
        last_stamp_ = input.stamp;
        return;
    }

    const double dt = (input.stamp - last_stamp_).toSec();
    last_stamp_ = input.stamp;

    if (dt < 1e-4 || dt > 0.1) {
        return;
    }

    P_ += Q_ * dt;

    const double z = input.thrust_cmd;
    const double S = P_ + R_;
    if (std::abs(S) < 1e-9) {
        return;
    }

    const double K = P_ / S;
    hover_thrust_ += K * (z - hover_thrust_);
    hover_thrust_ = std::clamp(hover_thrust_, hover_thrust_min_, hover_thrust_max_);
    P_ = std::max((1.0 - K) * P_, 1e-9);
}

double Kalman_HoverThrustEstimator::get_hover_thrust() const {
    return hover_thrust_;
}

}  // namespace thrust_estimator
