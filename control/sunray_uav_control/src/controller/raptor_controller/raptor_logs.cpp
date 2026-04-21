#include "controller/raptor_controller.hpp"
#include "eigen_helper.hpp"
#include <iomanip>
#include <ros/ros.h>
#include <sstream>

namespace {

const char* bool_to_string(bool value) {
    return value ? "true" : "false";
}

std::string format_vec3(const Eigen::Vector3d& value, int precision = 3) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << "(" << value.x() << ", " << value.y()
        << ", " << value.z() << ")";
    return oss.str();
}

}  // namespace

void Raptor_Controller::printf_logs() {
    const LogSnapshot snapshot = get_log_snapshot();
    const control_common::Mavros_SetpointLocal controller_local_output =
        snapshot.last_setpoint.valid ? snapshot.last_setpoint : snapshot.px4_local_target;

    const Eigen::Vector3d pos_error = snapshot.desired_state.position - snapshot.odom.position;
    const Eigen::Vector3d vel_error = snapshot.desired_state.velocity - snapshot.odom.velocity;
    const double current_yaw = eigen_helper::get_yaw_from_orientation(snapshot.odom.orientation);
    const double yaw_error = eigen_helper::wrap_angle(snapshot.desired_state.yaw - current_yaw);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "[Controller][RAPTOR][" << uav_ns_ << "]"
        << " ready=" << bool_to_string(snapshot.controller_ready)
        << " px4_mode=" << control_common::flightmode_to_string(snapshot.px4_state.flight_mode)
        << " armed=" << bool_to_string(snapshot.px4_state.armed)
        << " landed=" << control_common::landed_to_string(snapshot.px4_state.landed_state)
        << "\n  flags: takeoff_complete=" << bool_to_string(snapshot.takeoff_complete)
        << " land_complete=" << bool_to_string(snapshot.land_complete)
        << " point_complete=" << bool_to_string(snapshot.point_complete)
        << " has_odom=" << bool_to_string(snapshot.has_uav_odometry)
        << " can_fuse=" << bool_to_string(snapshot.can_fuse)
        << " setpoint_valid=" << bool_to_string(controller_local_output.valid)
        << "\n  desired: pos=" << format_vec3(snapshot.desired_state.position)
        << " vel=" << format_vec3(snapshot.desired_state.velocity)
        << " acc=" << format_vec3(snapshot.desired_state.acceleration)
        << " yaw=" << snapshot.desired_state.yaw
        << "\n  err: pos=" << format_vec3(pos_error)
        << " vel=" << format_vec3(vel_error)
        << " yaw=" << yaw_error
        << "\n  ctrl_out: pos=" << format_vec3(controller_local_output.position)
        << " vel=" << format_vec3(controller_local_output.velocity)
        << " acc=" << format_vec3(controller_local_output.accel_or_force)
        << " yaw=" << controller_local_output.yaw
        << " yaw_rate=" << controller_local_output.yaw_rate
        << "\n  hover: pos=" << format_vec3(snapshot.hover_point)
        << " yaw=" << snapshot.hover_yaw;

    ROS_INFO_STREAM(oss.str());
}
