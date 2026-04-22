#include "statemachine/sunray_fsm.hpp"
#include "utils/orientation_utils.hpp"
#include "utils/sunray_panel_log_utils.hpp"
#include <cmath>
#include <iomanip>
#include <sstream>

namespace {

const char* bool_to_string(bool value) {
    return value ? "true" : "false";
}

std::string bool_to_cn(bool value, const std::string& true_text, const std::string& false_text) {
    return value ? true_text : false_text;
}

const char* controller_type_to_string(uint8_t type) {
    switch (type) {
    case 0:
        return "PX4_OriginController";
    case 1:
        return "Geometric_Controller";
    case 2:
        return "Raptor_Controller";
    default:
        return "UnknownController";
    }
}

const char* sunray_state_to_string(sunray_fsm::SunrayState state) {
    switch (state) {
    case sunray_fsm::SunrayState::OFF:
        return "OFF";
    case sunray_fsm::SunrayState::INIT:
        return "INIT";
    case sunray_fsm::SunrayState::TAKEOFF:
        return "TAKEOFF";
    case sunray_fsm::SunrayState::HOVER:
        return "HOVER";
    case sunray_fsm::SunrayState::RETURN:
        return "RETURN";
    case sunray_fsm::SunrayState::LAND:
        return "LAND";
    case sunray_fsm::SunrayState::MOVE:
        return "MOVE";
    case sunray_fsm::SunrayState::EMERGENCY_KILL:
        return "EMERGENCY_KILL";
    default:
        return "UNKNOWN_STATE";
    }
}

const char* cmd_source_to_string(control_common::UavControlCmd::CmdSource source) {
    switch (source) {
    case control_common::UavControlCmd::CmdSource::UNDEFINE:
        return "UNDEFINE";
    case control_common::UavControlCmd::CmdSource::SUNRAY_STATION:
        return "SUNRAY_STATION";
    case control_common::UavControlCmd::CmdSource::RC_CONTROLLER:
        return "RC_CONTROLLER";
    case control_common::UavControlCmd::CmdSource::TERMINAL:
        return "TERMINAL";
    case control_common::UavControlCmd::CmdSource::CONTROL_CMD:
        return "CONTROL_CMD";
    default:
        return "UNKNOWN_SOURCE";
    }
}

const char* control_cmd_to_string(control_common::UavControlCmd::ControlCmd cmd) {
    switch (cmd) {
    case control_common::UavControlCmd::ControlCmd::UNDEFINE:
        return "UNDEFINE";
    case control_common::UavControlCmd::ControlCmd::TAKEOFF:
        return "TAKEOFF";
    case control_common::UavControlCmd::ControlCmd::LAND:
        return "LAND";
    case control_common::UavControlCmd::ControlCmd::RETURN:
        return "RETURN";
    case control_common::UavControlCmd::ControlCmd::KILL:
        return "KILL";
    case control_common::UavControlCmd::ControlCmd::HOVER:
        return "HOVER";
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT:
        return "MOVE_POINT";
    case control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY:
        return "MOVE_VELOCITY";
    case control_common::UavControlCmd::ControlCmd::MOVE_TRAJECTORY:
        return "MOVE_TRAJECTORY";
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT_BODY:
        return "MOVE_POINT_BODY";
    case control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY_BODY:
        return "MOVE_VELOCITY_BODY";
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT_WGS84:
        return "MOVE_POINT_WGS84";
    default:
        return "UNKNOWN_CMD";
    }
}

const char* yaw_mode_to_string(control_common::UavControlCmd::YawMode yaw_mode) {
    switch (yaw_mode) {
    case control_common::UavControlCmd::YawMode::KEEP_YAW:
        return "KEEP_YAW";
    case control_common::UavControlCmd::YawMode::SET_YAW:
        return "SET_YAW";
    case control_common::UavControlCmd::YawMode::SET_YAWRATE:
        return "SET_YAWRATE";
    default:
        return "UNKNOWN_YAW_MODE";
    }
}

std::string format_vec3(const Eigen::Vector3d& value, int precision = 3) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << "(" << value.x() << ", " << value.y()
        << ", " << value.z() << ")";
    return oss.str();
}

std::string format_vec2(const Eigen::Vector2d& value, int precision = 3) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << "(" << value.x() << ", " << value.y()
        << ")";
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

std::string format_age_s(double value, int precision = 3) {
    if (!std::isfinite(value) || value < 0.0) {
        return "未知";
    }
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

std::string format_control_target(const control_common::UavControlCmd& cmd) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    switch (cmd.control_cmd) {
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT:
        oss << "target_pos=" << format_vec3(cmd.position);
        break;
    case control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY:
        oss << "target_vel=" << format_vec3(cmd.velocity);
        break;
    case control_common::UavControlCmd::ControlCmd::MOVE_TRAJECTORY:
        oss << "traj_pos=" << format_vec3(cmd.position) << " traj_vel=" << format_vec3(cmd.velocity)
            << " traj_acc=" << format_vec3(cmd.acceleration);
        break;
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT_BODY:
        oss << "target_body_xy=" << format_vec2(cmd.body_position_xy)
            << " fixed_height=" << cmd.fixed_height;
        break;
    case control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY_BODY:
        oss << "target_body_vel_xy=" << format_vec2(cmd.body_velocity_xy)
            << " fixed_height=" << cmd.fixed_height;
        break;
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT_WGS84:
        oss << "target_wgs84=(" << cmd.wgs84_position.latitude << ", "
            << cmd.wgs84_position.longitude << ", " << cmd.wgs84_position.altitude << ")";
        break;
    default:
        oss << "target=n/a";
        break;
    }

    if (cmd.yaw_mode == control_common::UavControlCmd::YawMode::SET_YAW) {
        oss << " yaw=" << cmd.yaw;
    } else if (cmd.yaw_mode == control_common::UavControlCmd::YawMode::SET_YAWRATE) {
        oss << " yaw_rate=" << cmd.yaw_rate;
    }

    return oss.str();
}

std::string safe_topic(const std::string& runtime_topic, const std::string& fallback_topic) {
    return runtime_topic.empty() ? fallback_topic : runtime_topic;
}

std::string build_localization_source_status(const sunray_msgs::OdomStatus& status) {
    if (!status.has_odometry) {
        return "未收到定位里程计";
    }
    if (status.odom_timeout) {
        return "定位里程计超时";
    }
    if (status.localization_mode == sunray_msgs::OdomStatus::LOCAL_AND_GLOBAL &&
        !status.has_relocalization) {
        return "等待重定位";
    }
    if (status.localization_mode == sunray_msgs::OdomStatus::LOCAL_WITH_ARUCO &&
        !status.has_relocalization) {
        return "正常(未收到重定位)";
    }
    if ((status.localization_mode == sunray_msgs::OdomStatus::LOCAL_AND_GLOBAL ||
         status.localization_mode == sunray_msgs::OdomStatus::LOCAL_WITH_ARUCO) &&
        !status.relocalization_data_valid) {
        return "重定位数据无效";
    }
    return "正常";
}

SunrayPanelSeverity severity_from_status_text(const std::string& status_text) {
    if (status_text == "正常") {
        return SunrayPanelSeverity::INFO;
    }
    if (status_text == "正常(未收到重定位)") {
        return SunrayPanelSeverity::WARN;
    }
    return SunrayPanelSeverity::ERROR;
}

}  // namespace

void Sunray_FSM::show_static_info() {
    SUNRAY_INFO("========== Sunray_FSM Static Info [{}] ==========", uav_ns_);
    SUNRAY_INFO("controller={}  odom_topic={}",
                controller_type_to_string(fsm_config_.basic_param.controller_types),
                fsm_config_.basic_param.odom_topic_name);
    SUNRAY_INFO("rate: controller={:.1f} Hz  supervisor={:.1f} Hz  fuse_odom[type={}, freq={:.1f} Hz]",
                fsm_config_.basic_param.controller_update_frequency,
                fsm_config_.basic_param.supervisor_update_frequency,
                fsm_config_.basic_param.fuse_odom_type,
                fsm_config_.basic_param.fuse_odom_frequency);
    SUNRAY_INFO("vehicle: mass={:.2f} kg  gravity={:.3f}  hover_thrust={:.3f}",
                fsm_config_.basic_param.mass_kg,
                fsm_config_.basic_param.gravity,
                fsm_config_.basic_param.hover_thrust_percent);
    SUNRAY_INFO("takeoff: height={:.2f} m  max_vel={:.2f} m/s | land: type={}  max_vel={:.2f} m/s",
                fsm_config_.takeoff_land_param.takeoff_relative_height,
                fsm_config_.takeoff_land_param.takeoff_max_velocity,
                fsm_config_.takeoff_land_param.land_type,
                fsm_config_.takeoff_land_param.land_max_velocity);
    SUNRAY_INFO("arrival judge: stable_time={:.2f} s  pos_err={:.3f} m  vel_err={:.3f} m/s",
                fsm_config_.arrival_judge_param.judge_stabile_time_s,
                fsm_config_.arrival_judge_param.pos_stabile_err_m,
                fsm_config_.arrival_judge_param.vel_stabile_err_mps);
    SUNRAY_INFO("timeout: odom={:.2f} s  mavros={:.2f} s  station={:.2f} s",
                fsm_config_.msg_timeout_param.local_odometry,
                fsm_config_.msg_timeout_param.mavros_connect,
                fsm_config_.msg_timeout_param.sunray_station);
    SUNRAY_INFO("protect: takeoff_with_code={}  control_with_no_rc={}  arm_with_code={}  check_flip={}  tilt_max={:.1f} deg",
                bool_to_string(fsm_config_.protect_param.takeoff_with_code),
                bool_to_string(fsm_config_.protect_param.control_with_no_rc),
                bool_to_string(fsm_config_.protect_param.arm_with_code),
                bool_to_string(fsm_config_.protect_param.check_flip),
                fsm_config_.protect_param.tilt_angle_max);
    SUNRAY_INFO("velocity limit: xyz={} m/s  rc_xyz={} m/s  yaw_rate={:.3f} rad/s",
                format_vec3(fsm_config_.velocity_param.max_velocity),
                format_vec3(fsm_config_.velocity_param.max_velocity_with_rc),
                fsm_config_.velocity_param.yaw_rate);
    SUNRAY_INFO("local fence: x=[{:.2f}, {:.2f}]  y=[{:.2f}, {:.2f}]  z=[{:.2f}, {:.2f}]",
                fsm_config_.local_fence_param.x_min,
                fsm_config_.local_fence_param.x_max,
                fsm_config_.local_fence_param.y_min,
                fsm_config_.local_fence_param.y_max,
                fsm_config_.local_fence_param.z_min,
                fsm_config_.local_fence_param.z_max);
    SUNRAY_INFO("==================================================");
}

void Sunray_FSM::show_logs() {
    static ros::Time last_log_time = ros::Time(0);
    const ros::Time now = ros::Time::now();
    if (last_log_time != ros::Time(0) && (now - last_log_time).toSec() < 1.0) {
        return;
    }
    last_log_time = now;
    const uint8_t log_level =
        fsm_config_.basic_param.log_level <= 2 ? fsm_config_.basic_param.log_level : 2;

    sunray_fsm::SunrayState state_snapshot;
    control_common::UAVStateEstimate odom_snapshot;
    control_common::UavControlCmd cmd_snapshot;
    sunray_msgs::OdomStatus localization_status_snapshot;
    std::size_t queue_size = 0;
    bool has_valid_odom = false;
    bool has_localization_status = false;
    double odom_frequency_hz = 0.0;
    double localization_status_rate_hz = 0.0;

    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        state_snapshot = fsm_state_;
    }
    {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        odom_snapshot = last_odometry_;
        has_valid_odom = has_valid_odometry_;
        odom_frequency_hz = odom_frequency_hz_;
    }
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        cmd_snapshot = last_control_cmd_;
    }
    {
        std::lock_guard<std::mutex> lk(localization_status_mutex_);
        localization_status_snapshot = last_localization_status_;
        has_localization_status = has_localization_status_;
        localization_status_rate_hz = localization_status_rate_hz_;
    }
    {
        std::lock_guard<std::mutex> lk(event_mutex_);
        queue_size = fsm_event_queue_.size();
    }

    const double now_s = now.toSec();
    const double odom_age_s =
        (odom_snapshot.timestamp == ros::Time(0)) ? -1.0 : now_s - odom_snapshot.timestamp.toSec();
    const double cmd_age_s =
        (cmd_snapshot.timestamp == ros::Time(0)) ? -1.0 : now_s - cmd_snapshot.timestamp.toSec();
    const bool odom_timeout = !has_valid_odom ||
                              (odom_age_s >= 0.0 &&
                               odom_age_s > fsm_config_.msg_timeout_param.local_odometry);
    const bool cmd_received = cmd_snapshot.timestamp != ros::Time(0);
    const bool cmd_timeout = cmd_received && control_msg_lost_;
    const bool yaw_valid = std::isfinite(odom_snapshot.orientation.w()) &&
                           std::isfinite(odom_snapshot.orientation.x()) &&
                           std::isfinite(odom_snapshot.orientation.y()) &&
                           std::isfinite(odom_snapshot.orientation.z());
    const double yaw_deg =
        yaw_valid ? quaternion_to_yaw_rad(odom_snapshot.orientation) * 180.0 / M_PI : 0.0;

    std::string module_status = "正常";
    SunrayPanelSeverity module_severity = SunrayPanelSeverity::INFO;
    if (!controller_ready_) {
        module_status = "控制器未就绪";
        module_severity = SunrayPanelSeverity::ERROR;
    } else if (odom_timeout) {
        module_status = "控制里程计超时";
        module_severity = SunrayPanelSeverity::ERROR;
    } else if (log_level >= 1 && has_localization_status) {
        module_status = build_localization_source_status(localization_status_snapshot);
        module_severity = severity_from_status_text(module_status);
    } else if (log_level >= 1) {
        module_status = "未收到定位状态";
        module_severity = SunrayPanelSeverity::WARN;
    }

    const std::string controller_ready_text = bool_to_cn(controller_ready_, "已就绪", "未就绪");
    const std::string allow_takeoff_text = bool_to_cn(allow_takeoff_, "允许", "不允许");
    const std::string control_msg_text =
        !cmd_received ? "未收到" : (cmd_timeout ? "超时" : "正常");
    const std::string odom_state_text = !has_valid_odom ? "未收到" : (odom_timeout ? "超时" : "正常");
    const std::string localization_state_text =
        has_localization_status ? module_status : "未收到定位状态";
    const std::string localization_odom_text =
        has_localization_status
            ? bool_to_cn(localization_status_snapshot.has_odometry, "已收到", "未收到")
            : "未知";
    const std::string localization_reloc_text =
        has_localization_status
            ? bool_to_cn(localization_status_snapshot.has_relocalization, "已收到", "未收到")
            : "未知";
    const std::string localization_reloc_valid_text =
        has_localization_status
            ? bool_to_cn(localization_status_snapshot.relocalization_data_valid, "有效", "无效")
            : "未知";

    const std::string console_module_status = sunray_colorize_console_text(
        module_status, module_severity == SunrayPanelSeverity::INFO ? kSunrayAnsiGreen : kSunrayAnsiRed);
    const std::string console_controller_ready = sunray_colorize_console_text(
        controller_ready_text, controller_ready_ ? kSunrayAnsiGreen : kSunrayAnsiRed);
    const std::string console_allow_takeoff = sunray_colorize_console_text(
        allow_takeoff_text, allow_takeoff_ ? kSunrayAnsiGreen : kSunrayAnsiRed);
    const std::string console_odom_state = sunray_colorize_console_text(
        odom_state_text, (!has_valid_odom || odom_timeout) ? kSunrayAnsiRed : kSunrayAnsiGreen);
    const std::string console_odom_rate = sunray_colorize_console_text(
        format_rate_hz(odom_frequency_hz), odom_frequency_hz > 0.0 ? kSunrayAnsiGreen : kSunrayAnsiRed);
    const std::string console_localization_state = sunray_colorize_console_text(
        localization_state_text,
        (!has_localization_status || module_severity != SunrayPanelSeverity::INFO) ? kSunrayAnsiRed
                                                                                   : kSunrayAnsiGreen);
    const std::string console_localization_odom = sunray_colorize_console_text(
        localization_odom_text,
        (has_localization_status && localization_status_snapshot.has_odometry) ? kSunrayAnsiGreen
                                                                               : kSunrayAnsiRed);
    const std::string console_localization_reloc = sunray_colorize_console_text(
        localization_reloc_text,
        (has_localization_status && localization_status_snapshot.has_relocalization)
            ? kSunrayAnsiGreen
            : kSunrayAnsiRed);
    const std::string console_localization_reloc_valid = sunray_colorize_console_text(
        localization_reloc_valid_text,
        (has_localization_status && localization_status_snapshot.relocalization_data_valid)
            ? kSunrayAnsiGreen
            : kSunrayAnsiRed);
    const std::string console_localization_rate = sunray_colorize_console_text(
        format_rate_hz(localization_status_rate_hz),
        localization_status_rate_hz > 0.0 ? kSunrayAnsiGreen : kSunrayAnsiRed);
    const std::string console_control_msg = sunray_colorize_console_text(
        control_msg_text, (!cmd_received || cmd_timeout) ? kSunrayAnsiRed : kSunrayAnsiGreen);

    const std::string local_odom_sub_topic =
        safe_topic(local_odom_sub_.getTopic(), fsm_config_.basic_param.odom_topic_name);
    const std::string localization_status_sub_topic =
        safe_topic(localization_state_sub_.getTopic(), uav_ns_ + "/sunray/localization/odom_status");
    const std::string control_cmd_sub_topic =
        safe_topic(uav_control_cmd_sub_.getTopic(), uav_ns_ + "/sunray/uav_control_cmd");
    const std::string fsm_state_pub_topic =
        safe_topic(sunray_fsm_state_pub_.getTopic(), uav_ns_ + "/sunray/fsm/state");
    const std::string debug_odom_pub_topic =
        safe_topic(sunray_odom_debug_pub_.getTopic(), uav_ns_ + "/surnay/debug/odometrry");

    std::ostringstream body_oss;
    std::ostringstream console_body_oss;
    body_oss << std::fixed << std::setprecision(3);
    console_body_oss << std::fixed << std::setprecision(3);

    body_oss << " -------- 控制模块状态\n";
    console_body_oss << " -------- 控制模块状态\n";
    body_oss << " FSM状态: [ " << sunray_state_to_string(state_snapshot) << " ]\n";
    console_body_oss << " FSM状态: [ " << sunray_state_to_string(state_snapshot) << " ]\n";
    body_oss << " 控制器类型: [ "
             << controller_type_to_string(fsm_config_.basic_param.controller_types) << " ]\n";
    console_body_oss << " 控制器类型: [ "
                     << controller_type_to_string(fsm_config_.basic_param.controller_types) << " ]\n";
    body_oss << " 模块状态: [ " << module_status << " ]\n";
    console_body_oss << " 模块状态: [ " << console_module_status << " ]\n";
    body_oss << " 控制器就绪: [ " << controller_ready_text << " ]\n";
    console_body_oss << " 控制器就绪: [ " << console_controller_ready << " ]\n";
    body_oss << " 允许起飞: [ " << allow_takeoff_text << " ]\n";
    console_body_oss << " 允许起飞: [ " << console_allow_takeoff << " ]\n";
    body_oss << " 事件队列长度: " << queue_size << "\n";
    console_body_oss << " 事件队列长度: " << queue_size << "\n";

    if (log_level >= 2) {
        body_oss << " -------- 当前订阅信息\n";
        console_body_oss << " -------- 当前订阅信息\n";
        body_oss << " local_odom话题: " << local_odom_sub_topic << "\n";
        console_body_oss << " local_odom话题: " << local_odom_sub_topic << "\n";
        body_oss << " local_odom上游发布频率: " << format_rate_hz(odom_frequency_hz) << " [Hz]\n";
        console_body_oss << " local_odom上游发布频率: " << console_odom_rate << " [Hz]\n";
        body_oss << " localization_status话题: " << localization_status_sub_topic << "\n";
        console_body_oss << " localization_status话题: " << localization_status_sub_topic << "\n";
        body_oss << " localization_status上游发布频率: "
                 << format_rate_hz(localization_status_rate_hz) << " [Hz]\n";
        console_body_oss << " localization_status上游发布频率: " << console_localization_rate
                         << " [Hz]\n";
        body_oss << " uav_control_cmd话题: " << control_cmd_sub_topic << "\n";
        console_body_oss << " uav_control_cmd话题: " << control_cmd_sub_topic << "\n";

        body_oss << " -------- 当前发布信息\n";
        console_body_oss << " -------- 当前发布信息\n";
        body_oss << " fsm_state发布话题: " << fsm_state_pub_topic << "\n";
        console_body_oss << " fsm_state发布话题: " << fsm_state_pub_topic << "\n";
        body_oss << " debug_odom发布话题: " << debug_odom_pub_topic << "\n";
        console_body_oss << " debug_odom发布话题: " << debug_odom_pub_topic << "\n";
    }

    if (log_level >= 1) {
        body_oss << " -------- 定位健康状态\n";
        console_body_oss << " -------- 定位健康状态\n";
        if (!has_localization_status) {
            body_oss << " 尚未收到定位状态消息\n";
            console_body_oss << " 尚未收到定位状态消息\n";
        } else {
            body_oss << " 外部定位源: [ " << localization_status_snapshot.external_source_name
                     << " ]\n";
            console_body_oss << " 外部定位源: [ " << localization_status_snapshot.external_source_name
                             << " ]\n";
            body_oss << " 定位模式: [ " << localization_status_snapshot.localization_mode_name
                     << " ]\n";
            console_body_oss << " 定位模式: [ " << localization_status_snapshot.localization_mode_name
                             << " ]\n";
            body_oss << " 定位状态: [ " << localization_state_text << " ]\n";
            console_body_oss << " 定位状态: [ " << console_localization_state << " ]\n";
            body_oss << " 定位里程计: [ " << localization_odom_text << " ]\n";
            console_body_oss << " 定位里程计: [ " << console_localization_odom << " ]\n";
            body_oss << " 重定位数据: [ " << localization_reloc_text << " ]\n";
            console_body_oss << " 重定位数据: [ " << console_localization_reloc << " ]\n";
            body_oss << " 重定位有效性: [ " << localization_reloc_valid_text << " ]\n";
            console_body_oss << " 重定位有效性: [ " << console_localization_reloc_valid << " ]\n";
        }
    }

    body_oss << " -------- 当前控制输入\n";
    console_body_oss << " -------- 当前控制输入\n";
    body_oss << " 控制指令: [ " << control_cmd_to_string(cmd_snapshot.control_cmd) << " ]\n";
    console_body_oss << " 控制指令: [ " << control_cmd_to_string(cmd_snapshot.control_cmd) << " ]\n";
    body_oss << " 指令来源: [ " << cmd_source_to_string(cmd_snapshot.cmd_source) << " ]\n";
    console_body_oss << " 指令来源: [ " << cmd_source_to_string(cmd_snapshot.cmd_source) << " ]\n";
    body_oss << " 偏航模式: [ " << yaw_mode_to_string(cmd_snapshot.yaw_mode) << " ]\n";
    console_body_oss << " 偏航模式: [ " << yaw_mode_to_string(cmd_snapshot.yaw_mode) << " ]\n";
    body_oss << " 控制指令状态: [ " << control_msg_text << " ]\n";
    console_body_oss << " 控制指令状态: [ " << console_control_msg << " ]\n";
    body_oss << " 控制指令时间间隔: " << format_age_s(cmd_age_s) << " [s]\n";
    console_body_oss << " 控制指令时间间隔: " << format_age_s(cmd_age_s) << " [s]\n";
    body_oss << " 控制目标: " << format_control_target(cmd_snapshot) << "\n";
    console_body_oss << " 控制目标: " << format_control_target(cmd_snapshot) << "\n";

    body_oss << " -------- 当前里程计\n";
    console_body_oss << " -------- 当前里程计\n";
    body_oss << " 里程计状态: [ " << odom_state_text << " ]\n";
    console_body_oss << " 里程计状态: [ " << console_odom_state << " ]\n";
    if (!has_valid_odom) {
        body_oss << " 尚未收到有效里程计\n";
        console_body_oss << " 尚未收到有效里程计\n";
    } else {
        body_oss << " 位置[X Y Z]: " << format_vec3(odom_snapshot.position) << " [m]\n";
        console_body_oss << " 位置[X Y Z]: " << format_vec3(odom_snapshot.position) << " [m]\n";
        body_oss << " 速度[X Y Z]: " << format_vec3(odom_snapshot.velocity) << " [m/s]\n";
        console_body_oss << " 速度[X Y Z]: " << format_vec3(odom_snapshot.velocity) << " [m/s]\n";
        if (yaw_valid) {
            body_oss << " 偏航角: " << yaw_deg << " [deg]\n";
            console_body_oss << " 偏航角: " << yaw_deg << " [deg]\n";
        } else {
            body_oss << " 偏航角: 未知 [deg]\n";
            console_body_oss << " 偏航角: 未知 [deg]\n";
        }
        body_oss << " 里程计时间间隔: " << format_age_s(odom_age_s) << " [s]\n";
        console_body_oss << " 里程计时间间隔: " << format_age_s(odom_age_s) << " [s]\n";
        body_oss << " Home点[X Y Z]: " << format_vec3(home_point_) << " [m]\n";
        console_body_oss << " Home点[X Y Z]: " << format_vec3(home_point_) << " [m]\n";
    }

    body_oss << " ---------------------------------------------------------\n";
    console_body_oss << " ---------------------------------------------------------\n";

    const std::string header_line =
        ">>>>>>>>>>>>>>> 无人机控制状态 - [ " + uav_ns_ + " ] <<<<<<<<<<<<<<<";
    const std::string plain_panel = header_line + "\n" + body_oss.str();
    const std::string console_panel =
        std::string(kSunrayAnsiLightCyan) + header_line + kSunrayAnsiReset + "\n" +
        console_body_oss.str();

    sunray_write_panel_log(SunrayLogger::instance().Get(),
                           module_severity,
                           plain_panel,
                           console_panel);

    if (sunray_controller_) {
        sunray_controller_->printf_logs(log_level);
    }
}
