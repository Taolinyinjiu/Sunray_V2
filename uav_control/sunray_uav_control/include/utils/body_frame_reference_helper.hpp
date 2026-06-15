#pragma once

#include "control_data_types/controller_desired_types.hpp"
#include "control_data_types/uav_state_estimate.hpp"
#include "eigen_helper.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace body_frame_reference_helper {

inline controller_data_types::TargetPoint_t to_world_point(
    const control_common::UAVStateEstimate& odom,
    const controller_data_types::TargetBodyPoint_t& body_point) {
    const double yaw = eigen_helper::get_yaw_from_orientation(odom.orientation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    controller_data_types::TargetPoint_t world_point;
    const Eigen::Vector2d p_b = body_point.position_xy;
    world_point.position.x() = odom.position.x() + c * p_b.x() - s * p_b.y();
    world_point.position.y() = odom.position.y() + s * p_b.x() + c * p_b.y();
    world_point.position.z() = body_point.fixed_height;
    world_point.yaw = body_point.yaw;
    return world_point;
}

inline controller_data_types::TargetVelocity_t to_world_velocity(
    const control_common::UAVStateEstimate& odom,
    const controller_data_types::TargetBodyVelocity_t& body_velocity) {
    const double yaw = eigen_helper::get_yaw_from_orientation(odom.orientation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    controller_data_types::TargetVelocity_t world_velocity;
    const Eigen::Vector2d v_b_xy = body_velocity.velocity_xy;
    world_velocity.stamp = body_velocity.stamp;
    world_velocity.velocity.x() = c * v_b_xy.x() - s * v_b_xy.y();
    world_velocity.velocity.y() = s * v_b_xy.x() + c * v_b_xy.y();
    world_velocity.velocity.z() = 0.0;
    world_velocity.fixed_height = body_velocity.fixed_height;
    world_velocity.yaw = body_velocity.yaw;
    world_velocity.yaw_rate = body_velocity.yaw_rate;
    return world_velocity;
}

}  // namespace body_frame_reference_helper
