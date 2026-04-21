#include "controller/geometric_controller.hpp"
#include "control_data_types/mavros_helper_data_types.hpp"
#include "eigen_helper.hpp"
#include <ros/ros.h>
#include <algorithm>
#include <iomanip>
#include <sstream>

namespace {

const char* bool_to_string(bool value) {
    return value ? "true" : "false";
}

const char* control_type_to_string(Control_Type type) {
    switch (type) {
    case Control_Type::Point_:
        return "POINT";
    case Control_Type::Velocity_:
        return "VELOCITY";
    case Control_Type::Trajectory_:
        return "TRAJECTORY";
    case Control_Type::Undefine_:
    default:
        return "UNDEFINE";
    }
}

std::string format_vec3(const Eigen::Vector3d& value, int precision = 3) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << "(" << value.x() << ", " << value.y()
        << ", " << value.z() << ")";
    return oss.str();
}

}  // namespace

void Geometric_Controller::update_log_snapshot() {
    LogSnapshot snapshot;
    const bool landing_active = !landing_state_.start_time.isZero();
    const double height_above_ground =
        std::max(0.0, uav_odometry_.position.z() - ground_height_ref_);
    snapshot.controller_ready = controller_ready_.load(std::memory_order_relaxed);
    snapshot.takeoff_complete = takeoff_complete_.load(std::memory_order_relaxed);
    snapshot.land_complete = land_complete_.load(std::memory_order_relaxed);
    snapshot.point_complete = point_complete_.load(std::memory_order_relaxed);
    snapshot.has_uav_odometry = has_uav_odometry_.load(std::memory_order_relaxed);
    snapshot.has_imu = has_imu_.load(std::memory_order_relaxed);
    snapshot.can_fuse = can_fuse_.load(std::memory_order_relaxed);
    snapshot.land_near_ground =
        landing_active && height_above_ground <= landing_tuning_.near_ground_height_m;
    snapshot.px4_state = mavros_helper_.get_state();
    snapshot.odom = uav_odometry_;
    snapshot.desired_state = desired_state_;
    snapshot.hover_point = hover_point;
    snapshot.hover_yaw = hover_yaw_;
    snapshot.last_setpoint = last_setpoint_;
    snapshot.debug_state = controller_.get_last_debug_state();

    std::lock_guard<std::mutex> lk(log_snapshot_mutex_);
    log_snapshot_ = snapshot;
}

Geometric_Controller::LogSnapshot Geometric_Controller::get_log_snapshot() const {
    std::lock_guard<std::mutex> lk(log_snapshot_mutex_);
    return log_snapshot_;
}

void Geometric_Controller::printf_logs() {
    const LogSnapshot snapshot = get_log_snapshot();
    const auto& debug_state = snapshot.debug_state;
    const auto& desired_state = debug_state.valid ? debug_state.reference : snapshot.desired_state;
    const char* attitude_mode = "UNKNOWN_MODE";
    switch (attitude_command_mode_) {
    case AttitudeCommandMode::Attitude:
        attitude_mode = "ATTITUDE";
        break;
    case AttitudeCommandMode::BodyRate:
        attitude_mode = "BODYRATE";
        break;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "[Controller][GEOMETRIC][" << uav_ns_ << "]"
        << " ready=" << bool_to_string(snapshot.controller_ready)
        << " mode=" << attitude_mode
        << " px4_mode=" << control_common::flightmode_to_string(snapshot.px4_state.flight_mode)
        << " armed=" << bool_to_string(snapshot.px4_state.armed)
        << " landed=" << control_common::landed_to_string(snapshot.px4_state.landed_state)
        << "\n  flags: takeoff_complete=" << bool_to_string(snapshot.takeoff_complete)
        << " land_complete=" << bool_to_string(snapshot.land_complete)
        << " point_complete=" << bool_to_string(snapshot.point_complete)
        << " has_odom=" << bool_to_string(snapshot.has_uav_odometry)
        << " has_imu=" << bool_to_string(snapshot.has_imu)
        << " can_fuse=" << bool_to_string(snapshot.can_fuse)
        << " land_near_ground=" << bool_to_string(snapshot.land_near_ground)
        << "\n  desired: pos=" << format_vec3(desired_state.position)
        << " vel=" << format_vec3(desired_state.velocity)
        << " acc=" << format_vec3(desired_state.acceleration)
        << " yaw=" << desired_state.yaw
        << "\n  hover: pos=" << format_vec3(snapshot.hover_point)
        << " yaw=" << snapshot.hover_yaw
        << " thrust=" << snapshot.last_setpoint.thrust
        << " bodyrate=" << format_vec3(snapshot.last_setpoint.body_rate);

    if (debug_state.valid) {
        oss << "\n  debug[" << control_type_to_string(debug_state.control_type)
            << "]: pos_err=" << format_vec3(debug_state.position_error)
            << " vel_err=" << format_vec3(debug_state.velocity_error)
            << " yaw_err=" << debug_state.yaw_error
            << " att_err=" << format_vec3(debug_state.attitude_error)
            << " thrust=" << debug_state.desired_thrust;
    }

    ROS_INFO_STREAM(oss.str());
}
