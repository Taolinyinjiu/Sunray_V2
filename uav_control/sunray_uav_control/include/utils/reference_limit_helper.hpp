#pragma once

#include "utils/orientation_utils.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <ros/time.h>

namespace reference_limit_helper {

struct YawReferenceState {
    bool initialized{false};
    double yaw_rad{0.0};
    ros::Time stamp{ros::Time(0)};

    void reset() {
        initialized = false;
        yaw_rad = 0.0;
        stamp = ros::Time(0);
    }
};

inline Eigen::Vector3d clamp_velocity_per_axis(const Eigen::Vector3d& velocity,
                                               const Eigen::Vector3d& max_velocity) {
    return Eigen::Vector3d(std::clamp(velocity.x(), -std::abs(max_velocity.x()), std::abs(max_velocity.x())),
                           std::clamp(velocity.y(), -std::abs(max_velocity.y()), std::abs(max_velocity.y())),
                           std::clamp(velocity.z(), -std::abs(max_velocity.z()), std::abs(max_velocity.z())));
}

inline double clamp_yaw_rate(double yaw_rate_rad_s, double max_yaw_rate_rad_s) {
    if (!std::isfinite(max_yaw_rate_rad_s) || max_yaw_rate_rad_s <= 0.0) {
        return yaw_rate_rad_s;
    }
    return std::clamp(yaw_rate_rad_s, -max_yaw_rate_rad_s, max_yaw_rate_rad_s);
}

inline double update_slewed_yaw_target(YawReferenceState& state,
                                       double target_yaw_rad,
                                       double current_yaw_rad,
                                       double max_yaw_rate_rad_s,
                                       const ros::Time& now) {
    const double target = normalize_angle_rad(target_yaw_rad);
    const double current = normalize_angle_rad(current_yaw_rad);
    if (!state.initialized || state.stamp.isZero()) {
        state.initialized = true;
        state.yaw_rad = current;
        state.stamp = now;
        return state.yaw_rad;
    }

    if (!std::isfinite(max_yaw_rate_rad_s) || max_yaw_rate_rad_s <= 0.0) {
        state.yaw_rad = target;
        state.stamp = now;
        return state.yaw_rad;
    }

    const double dt = std::max(0.0, (now - state.stamp).toSec());
    const double max_step = max_yaw_rate_rad_s * dt;
    const double yaw_error = normalize_angle_rad(target - state.yaw_rad);
    const double yaw_step = std::clamp(yaw_error, -max_step, max_step);

    state.yaw_rad = normalize_angle_rad(state.yaw_rad + yaw_step);
    state.stamp = now;
    return state.yaw_rad;
}

inline double integrate_yaw_rate_command(YawReferenceState& state,
                                         double yaw_rate_cmd_rad_s,
                                         double current_yaw_rad,
                                         double max_yaw_rate_rad_s,
                                         const ros::Time& now) {
    const double current = normalize_angle_rad(current_yaw_rad);
    if (!state.initialized || state.stamp.isZero()) {
        state.initialized = true;
        state.yaw_rad = current;
        state.stamp = now;
        return state.yaw_rad;
    }

    const double dt = std::max(0.0, (now - state.stamp).toSec());
    const double limited_yaw_rate = clamp_yaw_rate(yaw_rate_cmd_rad_s, max_yaw_rate_rad_s);

    state.yaw_rad = normalize_angle_rad(state.yaw_rad + limited_yaw_rate * dt);
    state.stamp = now;
    return state.yaw_rad;
}

inline double compute_point_curve_maxvel(const Eigen::Vector3d& start_position,
                                         const Eigen::Vector3d& goal_position,
                                         const Eigen::Vector3d& max_velocity,
                                         double min_curve_velocity = 0.05) {
    constexpr double kAxisEps = 1e-6;

    const Eigen::Vector3d delta = goal_position - start_position;
    const double distance = delta.norm();
    if (distance <= kAxisEps) {
        return min_curve_velocity;
    }

    const Eigen::Vector3d direction = delta / distance;
    double scalar_speed_limit = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 3; ++i) {
        const double axis_ratio = std::abs(direction[i]);
        if (axis_ratio <= kAxisEps) {
            continue;
        }
        scalar_speed_limit = std::min(scalar_speed_limit, std::abs(max_velocity[i]) / axis_ratio);
    }

    if (!std::isfinite(scalar_speed_limit) || scalar_speed_limit <= 0.0) {
        scalar_speed_limit = max_velocity.cwiseAbs().minCoeff();
    }
    return std::max(min_curve_velocity, scalar_speed_limit);
}

inline bool velocity_exceeds_limits(const Eigen::Vector3d& velocity,
                                    const Eigen::Vector3d& max_velocity,
                                    double eps = 1e-6) {
    return std::abs(velocity.x()) > std::abs(max_velocity.x()) + eps ||
           std::abs(velocity.y()) > std::abs(max_velocity.y()) + eps ||
           std::abs(velocity.z()) > std::abs(max_velocity.z()) + eps;
}

inline bool yaw_rate_exceeds_limit(double yaw_rate_rad_s,
                                   double max_yaw_rate_rad_s,
                                   double eps = 1e-6) {
    return std::isfinite(max_yaw_rate_rad_s) && max_yaw_rate_rad_s > 0.0 &&
           std::abs(yaw_rate_rad_s) > max_yaw_rate_rad_s + eps;
}

inline bool trajectory_reference_exceeds_limits(const Eigen::Vector3d& velocity,
                                                double yaw_rate_rad_s,
                                                const Eigen::Vector3d& max_velocity,
                                                double max_yaw_rate_rad_s) {
    return velocity_exceeds_limits(velocity, max_velocity) ||
           yaw_rate_exceeds_limit(yaw_rate_rad_s, max_yaw_rate_rad_s);
}

}  // namespace reference_limit_helper
