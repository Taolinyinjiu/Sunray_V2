#include "controller/raptor_controller.hpp"
#include "eigen_helper.hpp"
#include "utils/controller_panel_log_utils.hpp"

void Raptor_Controller::printf_logs(uint8_t log_level) {
    using namespace sunray_control_log;

    const LogSnapshot snapshot = get_log_snapshot();
    const uint8_t panel_level = log_level <= 2 ? log_level : 2;
    const bool has_controller_setpoint = snapshot.last_setpoint.valid;
    const control_common::Mavros_SetpointLocal controller_local_output =
        has_controller_setpoint ? snapshot.last_setpoint : control_common::Mavros_SetpointLocal{};
    const controller_data_types::TargetTrajectoryPoint_t desired_state =
        has_controller_setpoint ? snapshot.desired_state : make_hold_desired_state(snapshot.odom);

    const Eigen::Vector3d pos_error = desired_state.position - snapshot.odom.position;
    const Eigen::Vector3d vel_error = desired_state.velocity - snapshot.odom.velocity;
    const double current_yaw = eigen_helper::get_yaw_from_orientation(snapshot.odom.orientation);
    const double yaw_error = eigen_helper::wrap_angle(desired_state.yaw - current_yaw);

    SunrayPanelSeverity severity = controller_panel_severity(snapshot.controller_ready,
                                                             snapshot.has_uav_odometry);
    if (!has_controller_setpoint && severity == SunrayPanelSeverity::INFO) {
        severity = SunrayPanelSeverity::WARN;
    }

    std::ostringstream body_oss;
    std::ostringstream console_body_oss;
    init_panel_streams(body_oss, console_body_oss);
    std::vector<PanelLine> flag_extra_lines;
    if (panel_level >= 1) {
        flag_extra_lines.push_back(
            make_status_line("Setpoint有效性", has_controller_setpoint, "有效", "无效"));
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
        append_desired_state_section(body_oss, console_body_oss, " -------- 期望值", desired_state);
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
                                    false,
                                    false,
                                    true);
    }

    if (panel_level >= 2) {
        append_hover_section(
            body_oss, console_body_oss, snapshot.hover_point, snapshot.hover_yaw);
    }

    append_panel_footer(body_oss, console_body_oss);
    write_controller_panel(uav_ns_, "RAPTOR", severity, body_oss.str(), console_body_oss.str());
}
