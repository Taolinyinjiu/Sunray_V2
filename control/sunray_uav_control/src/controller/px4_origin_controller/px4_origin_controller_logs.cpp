#include "controller/px4_origin_controller.hpp"
#include "eigen_helper.hpp"
#include "utils/controller_panel_log_utils.hpp"

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

void PX4_OriginController::printf_logs(uint8_t log_level) {
    using namespace sunray_control_log;

    const LogSnapshot snapshot = get_log_snapshot();
    const uint8_t panel_level = log_level <= 2 ? log_level : 2;
    const control_common::Mavros_SetpointLocal controller_local_output =
        snapshot.last_setpoint.valid ? snapshot.last_setpoint : snapshot.px4_local_target;

    const Eigen::Vector3d pos_error = snapshot.desired_state.position - snapshot.odom.position;
    const Eigen::Vector3d vel_error = snapshot.desired_state.velocity - snapshot.odom.velocity;
    const double current_yaw = eigen_helper::get_yaw_from_orientation(snapshot.odom.orientation);
    const double yaw_error = eigen_helper::wrap_angle(snapshot.desired_state.yaw - current_yaw);

    SunrayPanelSeverity severity = controller_panel_severity(snapshot.controller_ready,
                                                             snapshot.has_uav_odometry);
    if (!controller_local_output.valid && severity == SunrayPanelSeverity::INFO) {
        severity = SunrayPanelSeverity::WARN;
    }

    std::ostringstream body_oss;
    std::ostringstream console_body_oss;
    init_panel_streams(body_oss, console_body_oss);
    std::vector<PanelLine> flag_extra_lines;
    if (panel_level >= 1) {
        flag_extra_lines.push_back(
            make_status_line("Setpoint有效性", controller_local_output.valid, "有效", "无效"));
    }

    append_common_flight_status(body_oss,
                                console_body_oss,
                                snapshot.controller_ready,
                                snapshot.px4_state);

    append_common_flag_status(body_oss,
                              console_body_oss,
                              snapshot.takeoff_complete,
                              snapshot.land_complete,
                              snapshot.point_complete,
                              snapshot.has_uav_odometry,
                              flag_extra_lines);

    if (panel_level >= 2) {
        append_desired_state_section(
            body_oss, console_body_oss, " -------- 期望值", snapshot.desired_state);
    }

    append_tracking_error_section(body_oss,
                                  console_body_oss,
                                  " -------- 控制误差",
                                  pos_error,
                                  vel_error,
                                  yaw_error);

    if (panel_level >= 1) {
        append_local_output_section(body_oss,
                                    console_body_oss,
                                    controller_local_output,
                                    panel_level >= 2,
                                    panel_level >= 2,
                                    false);
    }

    if (panel_level >= 2) {
        append_hover_section(body_oss,
                             console_body_oss,
                             snapshot.hover_point,
                             snapshot.hover_yaw,
                             {make_plain_line(" PX4推力反馈: " +
                                              format_scalar(snapshot.px4_attitude_target.thrust))});
    }

    append_panel_footer(body_oss, console_body_oss);
    write_controller_panel(
        uav_ns_, "PX4_ORIGIN", severity, body_oss.str(), console_body_oss.str());
}
