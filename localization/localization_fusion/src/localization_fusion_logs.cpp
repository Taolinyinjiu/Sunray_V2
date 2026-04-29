#include "localization_fusion_logs.hpp"

#include <cmath>
#include <iomanip>
#include <ros/package.h>
#include <sstream>
#include <stdexcept>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "sunray_log.hpp"
#include "sunray_panel_log_utils.hpp"

namespace {

const char* localization_mode_to_string(LocalizationMode mode) {
    switch (mode) {
    case LocalizationMode::LOCAL:
        return "LOCAL";
    case LocalizationMode::GLOBAL:
        return "GLOBAL";
    case LocalizationMode::LOCAL_AND_GLOBAL:
        return "LOCAL_AND_GLOBAL";
    case LocalizationMode::LOCAL_WITH_ARUCO:
        return "LOCAL_WITH_ARUCO";
    default:
        return "UNKNOWN";
    }
}

bool needs_relocalization(LocalizationMode mode) {
    return mode == LocalizationMode::LOCAL_AND_GLOBAL ||
           mode == LocalizationMode::LOCAL_WITH_ARUCO;
}

std::string bool_to_cn(bool value, const std::string& true_text, const std::string& false_text) {
    return value ? true_text : false_text;
}

std::string format_scalar(double value, int precision = 3) {
    if (!std::isfinite(value)) {
        return "nan";
    }
    std::ostringstream oss;
    oss << std::showpos << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

std::string format_vec3(double x, double y, double z, int precision = 3) {
    std::ostringstream oss;
    oss << format_scalar(x, precision) << " " << format_scalar(y, precision) << " "
        << format_scalar(z, precision);
    return oss.str();
}

std::string format_rate_hz(double value, int precision = 2) {
    if (!std::isfinite(value) || value <= 0.0) {
        return "未知";
    }
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

std::string format_odom_rpy_deg(const nav_msgs::Odometry& odom, int precision = 2) {
    const geometry_msgs::Quaternion& q = odom.pose.pose.orientation;
    const bool quat_finite =
        std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
    if (!quat_finite) {
        return "nan nan nan";
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    if (tf_q.length2() < 1e-12) {
        return "nan nan nan";
    }
    tf_q.normalize();

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    constexpr double kRadToDeg = 180.0 / M_PI;
    return format_vec3(roll * kRadToDeg, pitch * kRadToDeg, yaw * kRadToDeg, precision);
}

SunrayPanelSeverity panel_severity(const LocalizationFusionLogSnapshot& snapshot,
                                   std::string& source_status) {
    source_status = "正常";
    if (!snapshot.has_odometry_data) {
        source_status = "未收到里程计";
        return SunrayPanelSeverity::WARN;
    }
    if (snapshot.odometry_data_timeout) {
        source_status = "里程计超时";
        return SunrayPanelSeverity::ERROR;
    }
    if (snapshot.selected_source.localization_mode == LocalizationMode::LOCAL_AND_GLOBAL &&
        !snapshot.has_relocalization_data) {
        source_status = "等待全局里程计";
        return SunrayPanelSeverity::WARN;
    }
    if (snapshot.selected_source.localization_mode == LocalizationMode::LOCAL_WITH_ARUCO &&
        !snapshot.has_relocalization_data) {
        source_status = "正常(未收到重定位)";
        return SunrayPanelSeverity::WARN;
    }
    if (needs_relocalization(snapshot.selected_source.localization_mode) &&
        snapshot.has_relocalization_data && !snapshot.relocalization_data_valid) {
        source_status = "重定位数据无效";
        return SunrayPanelSeverity::ERROR;
    }
    return SunrayPanelSeverity::INFO;
}

}  // namespace

void init_localization_fusion_logger(const std::string& uav_ns,
                                     bool log_save,
                                     std::string& log_file_path) {
    SunrayLogConfig cfg;
    cfg.name = "localization_fusion_" + uav_ns;
    cfg.console_level = SunrayLogLevel::info;
    cfg.file_level = SunrayLogLevel::trace;
    cfg.async = false;

    if (log_save) {
        log_file_path = make_localization_fusion_log_file_path();
        cfg.file_path = log_file_path;
    } else {
        log_file_path.clear();
    }

    SunrayLogger::instance().Init(cfg);
    auto logger = SunrayLogger::instance().Get();
    if (logger && !logger->sinks().empty()) {
        logger->sinks().front()->set_pattern("%v");
    }

    SUNRAY_INFO("localization_fusion logger initialized. log_save={} file_path={}",
                log_save,
                log_save ? log_file_path : "disabled");
}

std::string make_localization_fusion_log_file_path() {
    const std::string package_path = ros::package::getPath("localization_fusion");
    if (package_path.empty()) {
        throw std::runtime_error("failed to locate package path for localization_fusion");
    }

    const std::string localization_root = sunray_parent_directory(package_path);
    const std::string repo_root = sunray_parent_directory(localization_root);
    if (repo_root.empty()) {
        throw std::runtime_error("failed to infer repository root from package path: " +
                                 package_path);
    }

    return repo_root + "/log/localization_fusion/" + sunray_make_startup_timestamp() + ".log";
}

void write_localization_fusion_panel(const LocalizationFusionLogSnapshot& snapshot) {
    std::string source_status;
    const SunrayPanelSeverity severity = panel_severity(snapshot, source_status);

    const std::string odometry_rate_text = format_rate_hz(snapshot.odometry_input_rate_hz);
    const std::string relocalization_rate_text =
        format_rate_hz(snapshot.relocalization_input_rate_hz);
    const std::string odometry_rx_text =
        bool_to_cn(snapshot.has_odometry_data, "已收到", "未收到");
    const std::string odometry_timeout_text =
        bool_to_cn(snapshot.odometry_data_timeout, "超时", "正常");
    const std::string relocalization_rx_text =
        bool_to_cn(snapshot.has_relocalization_data, "已收到", "未收到");
    const std::string relocalization_valid_text =
        bool_to_cn(snapshot.relocalization_data_valid, "有效", "无效/未校验");

    const std::string console_source_status = sunray_colorize_console_text(
        source_status,
        severity == SunrayPanelSeverity::INFO ? kSunrayAnsiGreen : kSunrayAnsiRed);
    const std::string console_odometry_rate = sunray_colorize_console_text(
        odometry_rate_text,
        snapshot.odometry_input_rate_hz > 0.0 ? kSunrayAnsiGreen : kSunrayAnsiRed);
    const std::string console_odometry_rx = sunray_colorize_console_text(
        odometry_rx_text,
        snapshot.has_odometry_data ? kSunrayAnsiGreen : kSunrayAnsiRed);
    const std::string console_odometry_timeout = sunray_colorize_console_text(
        odometry_timeout_text,
        snapshot.odometry_data_timeout ? kSunrayAnsiRed : kSunrayAnsiGreen);
    const std::string console_relocalization_rate = sunray_colorize_console_text(
        relocalization_rate_text,
        snapshot.relocalization_input_rate_hz > 0.0 ? kSunrayAnsiGreen : kSunrayAnsiRed);
    const std::string console_relocalization_rx = sunray_colorize_console_text(
        relocalization_rx_text,
        snapshot.has_relocalization_data ? kSunrayAnsiGreen : kSunrayAnsiRed);
    const std::string console_relocalization_valid = sunray_colorize_console_text(
        relocalization_valid_text,
        snapshot.relocalization_data_valid ? kSunrayAnsiGreen : kSunrayAnsiRed);

    std::ostringstream body_oss;
    std::ostringstream console_body_oss;
    body_oss << std::fixed << std::setprecision(3);
    console_body_oss << std::fixed << std::setprecision(3);
    body_oss << " -------- 外部定位源\n";
    console_body_oss << " -------- 外部定位源\n";
    body_oss << " 外部定位源名称: [ " << snapshot.selected_source.source_name << " ]\n";
    console_body_oss << " 外部定位源名称: [ " << snapshot.selected_source.source_name << " ]\n";
    body_oss << " 外部定位源模式: [ "
             << localization_mode_to_string(snapshot.selected_source.localization_mode)
             << " ]\n";
    console_body_oss << " 外部定位源模式: [ "
                     << localization_mode_to_string(snapshot.selected_source.localization_mode)
                     << " ]\n";
    body_oss << " 外部定位源状态: [ " << source_status << " ]\n";
    console_body_oss << " 外部定位源状态: [ " << console_source_status << " ]\n";

    body_oss << " -------- 当前订阅的里程计相关信息\n";
    console_body_oss << " -------- 当前订阅的里程计相关信息\n";
    body_oss << " odometry话题: " << snapshot.odometry_sub_topic << "\n";
    console_body_oss << " odometry话题: " << snapshot.odometry_sub_topic << "\n";
    body_oss << " odometry上游发布频率: " << odometry_rate_text << " [Hz]\n";
    console_body_oss << " odometry上游发布频率: " << console_odometry_rate << " [Hz]\n";
    body_oss << " odometry接收状态: [ " << odometry_rx_text << " ]\n";
    console_body_oss << " odometry接收状态: [ " << console_odometry_rx << " ]\n";
    body_oss << " odometry超时状态: [ " << odometry_timeout_text << " ]\n";
    console_body_oss << " odometry超时状态: [ " << console_odometry_timeout << " ]\n";
    if (needs_relocalization(snapshot.selected_source.localization_mode)) {
        body_oss << " relocalization话题: " << snapshot.relocalization_sub_topic << "\n";
        console_body_oss << " relocalization话题: " << snapshot.relocalization_sub_topic << "\n";
        body_oss << " relocalization上游发布频率: " << relocalization_rate_text << " [Hz]\n";
        console_body_oss << " relocalization上游发布频率: " << console_relocalization_rate
                         << " [Hz]\n";
        body_oss << " relocalization接收状态: [ " << relocalization_rx_text << " ]\n";
        console_body_oss << " relocalization接收状态: [ " << console_relocalization_rx << " ]\n";
        body_oss << " relocalization数据状态: [ " << relocalization_valid_text << " ]\n";
        console_body_oss << " relocalization数据状态: [ " << console_relocalization_valid
                         << " ]\n";
    }

    body_oss << " -------- 当前发布的里程计相关信息\n";
    console_body_oss << " -------- 当前发布的里程计相关信息\n";
    body_oss << " local_odom发布话题: " << snapshot.local_pub_topic << "\n";
    console_body_oss << " local_odom发布话题: " << snapshot.local_pub_topic << "\n";
    body_oss << " global_odom发布话题: " << snapshot.global_pub_topic << "\n";
    console_body_oss << " global_odom发布话题: " << snapshot.global_pub_topic << "\n";
    body_oss << " odom_status发布话题: " << snapshot.odom_status_pub_topic << "\n";
    console_body_oss << " odom_status发布话题: " << snapshot.odom_status_pub_topic << "\n";

    body_oss << " -------- 当前里程计数据\n";
    console_body_oss << " -------- 当前里程计数据\n";
    if (!snapshot.has_odometry_data) {
        body_oss << " 尚未收到里程计数据\n";
        console_body_oss << " 尚未收到里程计数据\n";
    } else {
        body_oss << " 时间戳: " << snapshot.last_odometry_data.header.stamp.toSec()
                 << " [s]  坐标系: [ " << snapshot.last_odometry_data.header.frame_id << " -> "
                 << snapshot.last_odometry_data.child_frame_id << " ]\n";
        console_body_oss << " 时间戳: " << snapshot.last_odometry_data.header.stamp.toSec()
                         << " [s]  坐标系: [ " << snapshot.last_odometry_data.header.frame_id
                         << " -> " << snapshot.last_odometry_data.child_frame_id << " ]\n";
        body_oss << " 位置[X Y Z]: "
                 << format_vec3(snapshot.last_odometry_data.pose.pose.position.x,
                                snapshot.last_odometry_data.pose.pose.position.y,
                                snapshot.last_odometry_data.pose.pose.position.z)
                 << " [m]\n";
        console_body_oss << " 位置[X Y Z]: "
                         << format_vec3(snapshot.last_odometry_data.pose.pose.position.x,
                                        snapshot.last_odometry_data.pose.pose.position.y,
                                        snapshot.last_odometry_data.pose.pose.position.z)
                         << " [m]\n";
        body_oss << " 速度[X Y Z]: "
                 << format_vec3(snapshot.last_odometry_data.twist.twist.linear.x,
                                snapshot.last_odometry_data.twist.twist.linear.y,
                                snapshot.last_odometry_data.twist.twist.linear.z)
                 << " [m/s]\n";
        console_body_oss << " 速度[X Y Z]: "
                         << format_vec3(snapshot.last_odometry_data.twist.twist.linear.x,
                                        snapshot.last_odometry_data.twist.twist.linear.y,
                                        snapshot.last_odometry_data.twist.twist.linear.z)
                         << " [m/s]\n";
        body_oss << " 姿态[RPY]: " << format_odom_rpy_deg(snapshot.last_odometry_data)
                 << " [deg]\n";
        console_body_oss << " 姿态[RPY]: " << format_odom_rpy_deg(snapshot.last_odometry_data)
                         << " [deg]\n";
    }

    if (needs_relocalization(snapshot.selected_source.localization_mode) &&
        snapshot.has_relocalization_data) {
        body_oss << " -------- 当前重定位数据\n";
        console_body_oss << " -------- 当前重定位数据\n";
        body_oss << " 时间戳: " << snapshot.last_relocalization_data.header.stamp.toSec()
                 << " [s]  坐标系: [ " << snapshot.last_relocalization_data.header.frame_id
                 << " -> " << snapshot.last_relocalization_data.child_frame_id << " ]\n";
        console_body_oss
            << " 时间戳: " << snapshot.last_relocalization_data.header.stamp.toSec()
            << " [s]  坐标系: [ " << snapshot.last_relocalization_data.header.frame_id << " -> "
            << snapshot.last_relocalization_data.child_frame_id << " ]\n";
        body_oss << " 位置[X Y Z]: "
                 << format_vec3(snapshot.last_relocalization_data.pose.pose.position.x,
                                snapshot.last_relocalization_data.pose.pose.position.y,
                                snapshot.last_relocalization_data.pose.pose.position.z)
                 << " [m]\n";
        console_body_oss << " 位置[X Y Z]: "
                         << format_vec3(snapshot.last_relocalization_data.pose.pose.position.x,
                                        snapshot.last_relocalization_data.pose.pose.position.y,
                                        snapshot.last_relocalization_data.pose.pose.position.z)
                         << " [m]\n";
        body_oss << " 姿态[RPY]: "
                 << format_odom_rpy_deg(snapshot.last_relocalization_data) << " [deg]\n";
        console_body_oss << " 姿态[RPY]: "
                         << format_odom_rpy_deg(snapshot.last_relocalization_data) << " [deg]\n";
    }

    body_oss << " ---------------------------------------------------------\n";
    console_body_oss << " ---------------------------------------------------------\n";

    const std::string header_line =
        ">>>>>>>>>>>>>>> 定位融合状态 - [ " + snapshot.uav_ns + " ] <<<<<<<<<<<<<<<";
    const std::string plain_panel = header_line + "\n" + body_oss.str();
    const std::string console_panel =
        std::string(kSunrayAnsiLightCyan) + header_line + kSunrayAnsiReset + "\n" +
        console_body_oss.str();

    sunray_write_panel_log(
        SunrayLogger::instance().Get(), severity, plain_panel, console_panel);
}
