#include "controller/geometric_controller.hpp"
#include "eigen_helper.hpp"
#include "utils/controller_panel_log_utils.hpp"
#include <algorithm>

namespace {

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

void Geometric_Controller::printf_logs(uint8_t log_level) {
    using namespace sunray_control_log;

    const LogSnapshot snapshot = get_log_snapshot();
    const uint8_t panel_level = log_level <= 2 ? log_level : 2;
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

    const bool health_ok =
        snapshot.controller_ready && snapshot.has_uav_odometry && snapshot.has_imu;
    const SunrayPanelSeverity severity =
        health_ok ? SunrayPanelSeverity::INFO : SunrayPanelSeverity::ERROR;
    const Eigen::Vector3d pos_error = desired_state.position - snapshot.odom.position;
    const Eigen::Vector3d vel_error = desired_state.velocity - snapshot.odom.velocity;
    const double current_yaw = eigen_helper::get_yaw_from_orientation(snapshot.odom.orientation);
    const double yaw_error = eigen_helper::wrap_angle(desired_state.yaw - current_yaw);

    std::ostringstream body_oss;
    std::ostringstream console_body_oss;
    init_panel_streams(body_oss, console_body_oss);
    std::vector<PanelLine> flight_extra_lines;
    std::vector<PanelLine> flag_extra_lines;
    flag_extra_lines.push_back(make_status_line("IMU状态", snapshot.has_imu, "正常", "异常"));
    if (panel_level >= 2) {
        flight_extra_lines.push_back(
            make_plain_line(" 姿态控制模式: [ " + std::string(attitude_mode) + " ]"));
        flag_extra_lines.push_back(
            make_plain_line(" 近地降落阶段: [ " +
                            bool_to_cn(snapshot.land_near_ground, "是", "否") + " ]"));
    }

    append_common_flight_status(body_oss,
                                console_body_oss,
                                snapshot.controller_ready,
                                snapshot.px4_state,
                                flight_extra_lines);

    append_common_flag_status(body_oss,
                              console_body_oss,
                              snapshot.takeoff_complete,
                              snapshot.land_complete,
                              snapshot.point_complete,
                              snapshot.has_uav_odometry,
                              flag_extra_lines);

    if (panel_level >= 2) {
        append_desired_state_section(body_oss, console_body_oss, " -------- 期望值", desired_state);
    }

    append_tracking_error_section(body_oss,
                                  console_body_oss,
                                  " -------- 控制误差",
                                  pos_error,
                                  vel_error,
                                  yaw_error);

    if (panel_level >= 1) {
        append_attitude_output_section(body_oss,
                                       console_body_oss,
                                       snapshot.last_setpoint.thrust,
                                       snapshot.last_setpoint.body_rate);
    }

    if (panel_level >= 2) {
        append_hover_section(body_oss, console_body_oss, snapshot.hover_point, snapshot.hover_yaw);
    }

    if (panel_level >= 2 && debug_state.valid) {
        append_section_header(body_oss, console_body_oss, " -------- Debug控制误差");
        append_line(body_oss,
                    console_body_oss,
                    " 控制类型: [ " + std::string(control_type_to_string(debug_state.control_type)) +
                        " ]");
        append_line(body_oss,
                    console_body_oss,
                    " 姿态误差: " + format_vec3(debug_state.attitude_error) + " [rad]");
        append_line(body_oss,
                    console_body_oss,
                    " 期望推力: " + format_scalar(debug_state.desired_thrust));
    }

    append_panel_footer(body_oss, console_body_oss);
    write_controller_panel(
        uav_ns_, "GEOMETRIC", severity, body_oss.str(), console_body_oss.str());
}
