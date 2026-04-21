#include "controller/px4_origin_controller.hpp"
#include "eigen_helper.hpp"
#include "control_data_types/mavros_helper_data_types.hpp"
#include <ros/ros.h>
#include <iomanip>
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

std::string format_setpoint_frame(control_common::Mavros_SetpointLocal::Mavros_LocalFrame frame) {
    switch (frame) {
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned:
        return "LOCAL_NED";
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Offset_Ned:
        return "LOCAL_OFFSET_NED";
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Body_Ned:
        return "BODY_NED";
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Body_Offset_Ned:
        return "BODY_OFFSET_NED";
    default:
        return "UNKNOWN_FRAME";
    }
}

}  // namespace

void PX4_OriginController::update_log_snapshot() {
    LogSnapshot snapshot;
    snapshot.controller_ready = controller_ready_.load(std::memory_order_relaxed);
    snapshot.takeoff_complete = takeoff_complete_.load(std::memory_order_relaxed);
    snapshot.land_complete = land_complete_.load(std::memory_order_relaxed);
    snapshot.point_complete = point_complete_.load(std::memory_order_relaxed);
    snapshot.has_uav_odometry = has_uav_odometry_.load(std::memory_order_relaxed);
    snapshot.can_fuse = can_fuse_.load(std::memory_order_relaxed);
    snapshot.px4_state = mavros_helper_.get_state();
    snapshot.odom = uav_odometry_;
    snapshot.px4_local_target = mavros_helper_.get_target_local();
    snapshot.px4_attitude_target = mavros_helper_.get_target_attitude();
    snapshot.last_setpoint = last_setpoint_;
    snapshot.desired_state = desired_state_;
    snapshot.hover_point = hover_point_;
    snapshot.hover_yaw = hover_yaw_;

    std::lock_guard<std::mutex> lk(log_snapshot_mutex_);
    log_snapshot_ = snapshot;
}

PX4_OriginController::LogSnapshot PX4_OriginController::get_log_snapshot() const {
    std::lock_guard<std::mutex> lk(log_snapshot_mutex_);
    return log_snapshot_;
}

void PX4_OriginController::printf_logs() {
    const LogSnapshot snapshot = get_log_snapshot();
    const control_common::Mavros_SetpointLocal controller_local_output =
        snapshot.last_setpoint.valid ? snapshot.last_setpoint : snapshot.px4_local_target;

    const Eigen::Vector3d pos_error = snapshot.desired_state.position - snapshot.odom.position;
    const Eigen::Vector3d vel_error = snapshot.desired_state.velocity - snapshot.odom.velocity;
    const double current_yaw = eigen_helper::get_yaw_from_orientation(snapshot.odom.orientation);
    const double yaw_error = eigen_helper::wrap_angle(snapshot.desired_state.yaw - current_yaw);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "[Controller][PX4_ORIGIN][" << uav_ns_ << "]"
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
        << "\n  ctrl_out[" << format_setpoint_frame(controller_local_output.frame)
        << "]: pos=" << format_vec3(controller_local_output.position)
        << " vel=" << format_vec3(controller_local_output.velocity)
        << " acc=" << format_vec3(controller_local_output.accel_or_force)
        << " yaw=" << controller_local_output.yaw
        << " mask=" << controller_local_output.mask
        << "\n  hover: pos=" << format_vec3(snapshot.hover_point)
        << " yaw=" << snapshot.hover_yaw
        << " px4_thrust=" << snapshot.px4_attitude_target.thrust;

    ROS_INFO_STREAM(oss.str());
}
