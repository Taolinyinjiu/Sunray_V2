#pragma once

#include "control_data_types/controller_desired_types.hpp"
#include "control_data_types/mavros_helper_data_types.hpp"
#include "sunray_panel_log_utils.hpp"
#include <Eigen/Dense>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace sunray_control_log {

struct PanelLine {
    std::string plain;
    std::string console;
};

inline const char* bool_to_string(bool value) {
    return value ? "true" : "false";
}

inline std::string bool_to_cn(bool value,
                              const std::string& true_text,
                              const std::string& false_text) {
    return value ? true_text : false_text;
}

inline std::string format_scalar(double value, int precision = 3) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

inline std::string format_vec3(const Eigen::Vector3d& value, int precision = 3) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << "(" << value.x() << ", " << value.y()
        << ", " << value.z() << ")";
    return oss.str();
}

inline std::string format_setpoint_frame(control_common::Mavros_SetpointLocal::Mavros_LocalFrame frame) {
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

inline PanelLine make_plain_line(const std::string& text) {
    return {text, text};
}

inline PanelLine make_status_line(const std::string& label,
                                  const std::string& plain_status,
                                  bool normal) {
    const std::string console_status = sunray_colorize_console_text(
        plain_status, normal ? kSunrayAnsiGreen : kSunrayAnsiRed);
    return {" " + label + ": [ " + plain_status + " ]",
            " " + label + ": [ " + console_status + " ]"};
}

inline PanelLine make_status_line(const std::string& label,
                                  bool value,
                                  const std::string& true_text,
                                  const std::string& false_text,
                                  bool true_is_normal = true) {
    return make_status_line(label,
                            bool_to_cn(value, true_text, false_text),
                            value == true_is_normal);
}

inline void init_panel_streams(std::ostringstream& plain, std::ostringstream& console) {
    plain << std::fixed << std::setprecision(3);
    console << std::fixed << std::setprecision(3);
}

inline void append_line(std::ostringstream& plain,
                        std::ostringstream& console,
                        const std::string& plain_text,
                        const std::string& console_text) {
    plain << plain_text << "\n";
    console << console_text << "\n";
}

inline void append_line(std::ostringstream& plain,
                        std::ostringstream& console,
                        const std::string& text) {
    append_line(plain, console, text, text);
}

inline void append_lines(std::ostringstream& plain,
                         std::ostringstream& console,
                         const std::vector<PanelLine>& lines) {
    for (const auto& line : lines) {
        append_line(plain, console, line.plain, line.console);
    }
}

inline void append_section_header(std::ostringstream& plain,
                                  std::ostringstream& console,
                                  const std::string& title) {
    append_line(plain, console, title);
}

inline SunrayPanelSeverity controller_panel_severity(bool controller_ready,
                                                     bool has_uav_odometry,
                                                     bool extra_ok = true) {
    if (!controller_ready || !has_uav_odometry || !extra_ok) {
        return SunrayPanelSeverity::ERROR;
    }
    return SunrayPanelSeverity::INFO;
}

inline void append_common_flight_status(std::ostringstream& plain,
                                        std::ostringstream& console,
                                        bool controller_ready,
                                        const control_common::Mavros_State& px4_state,
                                        const std::vector<PanelLine>& extra_lines = {}) {
    append_section_header(plain, console, " -------- 飞控状态");
    const auto ready_line = make_status_line("控制器就绪", controller_ready, "已就绪", "未就绪");
    append_line(plain, console, ready_line.plain, ready_line.console);
    append_line(plain,
                console,
                " PX4模式: [ " + control_common::flightmode_to_string(px4_state.flight_mode) + " ]");
    append_line(plain,
                console,
                " 飞控解锁: [ " + std::string(bool_to_string(px4_state.armed)) + " ]");
    append_line(plain,
                console,
                " 落地状态: [ " + control_common::landed_to_string(px4_state.landed_state) + " ]");
    append_lines(plain, console, extra_lines);
}

inline void append_common_flag_status(std::ostringstream& plain,
                                      std::ostringstream& console,
                                      bool takeoff_complete,
                                      bool land_complete,
                                      bool point_complete,
                                      bool has_uav_odometry,
                                      const std::vector<PanelLine>& extra_lines = {}) {
    append_section_header(plain, console, " -------- 控制器标志位");
    append_line(plain,
                console,
                " 起飞完成: " + std::string(bool_to_string(takeoff_complete)) +
                    "  降落完成: " + bool_to_string(land_complete) + "  到点完成: " +
                    bool_to_string(point_complete));
    const auto odom_line = make_status_line("里程计状态", has_uav_odometry, "正常", "异常");
    append_line(plain, console, odom_line.plain, odom_line.console);
    append_lines(plain, console, extra_lines);
}

inline void append_desired_tracking_section(
    std::ostringstream& plain,
    std::ostringstream& console,
    const std::string& title,
    const controller_data_types::TargetTrajectoryPoint_t& desired_state,
    const Eigen::Vector3d* position_error = nullptr,
    const Eigen::Vector3d* velocity_error = nullptr,
    const double* yaw_error = nullptr) {
    append_section_header(plain, console, title);
    append_line(plain,
                console,
                " 期望位置: " + format_vec3(desired_state.position) + " [m]");
    append_line(plain,
                console,
                " 期望速度: " + format_vec3(desired_state.velocity) + " [m/s]");
    append_line(plain,
                console,
                " 期望加速度: " + format_vec3(desired_state.acceleration) + " [m/s^2]");
    append_line(plain,
                console,
                " 期望偏航: " + format_scalar(desired_state.yaw) + " [rad]");
    if (position_error != nullptr) {
        append_line(plain,
                    console,
                    " 位置误差: " + format_vec3(*position_error) + " [m]");
    }
    if (velocity_error != nullptr) {
        append_line(plain,
                    console,
                    " 速度误差: " + format_vec3(*velocity_error) + " [m/s]");
    }
    if (yaw_error != nullptr) {
        append_line(plain,
                    console,
                    " 偏航误差: " + format_scalar(*yaw_error) + " [rad]");
    }
}

inline void append_desired_state_section(
    std::ostringstream& plain,
    std::ostringstream& console,
    const std::string& title,
    const controller_data_types::TargetTrajectoryPoint_t& desired_state) {
    append_desired_tracking_section(plain, console, title, desired_state, nullptr, nullptr, nullptr);
}

inline void append_tracking_error_section(std::ostringstream& plain,
                                          std::ostringstream& console,
                                          const std::string& title,
                                          const Eigen::Vector3d& position_error,
                                          const Eigen::Vector3d& velocity_error,
                                          double yaw_error,
                                          const std::vector<PanelLine>& extra_lines = {}) {
    append_section_header(plain, console, title);
    append_line(plain,
                console,
                " 位置误差: " + format_vec3(position_error) + " [m]");
    append_line(plain,
                console,
                " 速度误差: " + format_vec3(velocity_error) + " [m/s]");
    append_line(plain,
                console,
                " 偏航误差: " + format_scalar(yaw_error) + " [rad]");
    append_lines(plain, console, extra_lines);
}

inline void append_local_output_section(std::ostringstream& plain,
                                        std::ostringstream& console,
                                        const control_common::Mavros_SetpointLocal& output,
                                        bool include_frame = false,
                                        bool include_mask = false,
                                        bool include_yaw_rate = false,
                                        const std::vector<PanelLine>& extra_lines = {}) {
    append_section_header(plain, console, " -------- 控制器输出");
    if (include_frame) {
        append_line(plain,
                    console,
                    " 输出坐标系: [ " + format_setpoint_frame(output.frame) + " ]");
    }
    append_line(plain,
                console,
                " 位置输出: " + format_vec3(output.position) + " [m]");
    append_line(plain,
                console,
                " 速度输出: " + format_vec3(output.velocity) + " [m/s]");
    append_line(plain,
                console,
                " 加速度输出: " + format_vec3(output.accel_or_force) + " [m/s^2]");
    append_line(plain,
                console,
                " 偏航输出: " + format_scalar(output.yaw) + " [rad]");
    if (include_yaw_rate) {
        append_line(plain,
                    console,
                    " 偏航角速度输出: " + format_scalar(output.yaw_rate) + " [rad/s]");
    }
    if (include_mask) {
        append_line(plain, console, " 类型掩码: " + std::to_string(output.mask));
    }
    append_lines(plain, console, extra_lines);
}

inline void append_hover_section(std::ostringstream& plain,
                                 std::ostringstream& console,
                                 const Eigen::Vector3d& hover_point,
                                 double hover_yaw,
                                 const std::vector<PanelLine>& extra_lines = {}) {
    append_section_header(plain, console, " -------- 悬停点");
    append_line(plain,
                console,
                " 悬停位置: " + format_vec3(hover_point) + " [m]");
    append_line(plain,
                console,
                " 悬停偏航: " + format_scalar(hover_yaw) + " [rad]");
    append_lines(plain, console, extra_lines);
}

inline void append_attitude_output_section(std::ostringstream& plain,
                                           std::ostringstream& console,
                                           double thrust,
                                           const Eigen::Vector3d& body_rate,
                                           const std::vector<PanelLine>& extra_lines = {}) {
    append_section_header(plain, console, " -------- 控制器输出");
    append_line(plain, console, " 推力输出: " + format_scalar(thrust));
    append_line(plain,
                console,
                " 角速度输出: " + format_vec3(body_rate) + " [rad/s]");
    append_lines(plain, console, extra_lines);
}

inline void append_panel_footer(std::ostringstream& plain, std::ostringstream& console) {
    append_line(plain, console, " ---------------------------------------------------------");
}

inline void write_controller_panel(const std::string& uav_ns,
                                   const std::string& controller_name,
                                   SunrayPanelSeverity severity,
                                   const std::string& plain_body,
                                   const std::string& console_body) {
    const std::string header_line =
        ">>>>>>>>>>>>>>> 控制器状态 - [ " + uav_ns + " | " + controller_name + " ] <<<<<<<<<<<<<<<";
    const std::string plain_panel = header_line + "\n" + plain_body;
    const std::string console_panel =
        std::string(kSunrayAnsiLightCyan) + header_line + kSunrayAnsiReset + "\n" + console_body;
    sunray_write_panel_log(SunrayLogger::instance().Get(), severity, plain_panel, console_panel);
}

}  // namespace sunray_control_log
