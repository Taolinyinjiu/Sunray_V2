#include "planning_fsm.hpp"
#include "sunray_planning_common.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

#include "sunray_log.hpp"

namespace {

std::string bool_to_cn(bool value, const std::string& true_text, const std::string& false_text) {
    return value ? true_text : false_text;
}

const char* bool_to_string(bool value) {
    return value ? "true" : "false";
}

std::string uav_control_cmd_to_string(const uint8_t control_cmd) {
    switch (control_cmd) {
    case sunray_msgs::UAVControlCMD::TAKEOFF:
        return "TAKEOFF";
    case sunray_msgs::UAVControlCMD::LAND:
        return "LAND";
    case sunray_msgs::UAVControlCMD::RETURN:
        return "RETURN";
    case sunray_msgs::UAVControlCMD::KILL:
        return "KILL";
    case sunray_msgs::UAVControlCMD::HOVER:
        return "HOVER";
    case sunray_msgs::UAVControlCMD::MOVE_POINT:
        return "MOVE_POINT";
    case sunray_msgs::UAVControlCMD::MOVE_VELOCITY:
        return "MOVE_VELOCITY";
    case sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY:
        return "MOVE_TRAJECTORY";
    case sunray_msgs::UAVControlCMD::MOVE_POINT_BODY:
        return "MOVE_POINT_BODY";
    case sunray_msgs::UAVControlCMD::MOVE_VELOCITY_BODY:
        return "MOVE_VELOCITY_BODY";
    case sunray_msgs::UAVControlCMD::MOVE_POINT_WGS84:
        return "MOVE_POINT_WGS84";
    case sunray_msgs::UAVControlCMD::UNDEFINE:
    default:
        return "UNDEFINE";
    }
}

std::string format_scalar(double value, int precision = 3) {
    if (!std::isfinite(value)) {
        return "nan";
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

std::string format_stamp_age_s(const ros::Time& now, const ros::Time& stamp) {
    if (stamp.isZero()) {
        return "未知";
    }
    return format_age_s((now - stamp).toSec());
}

std::string format_point3(const geometry_msgs::Point& value, int precision = 3) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << "(" << value.x << ", " << value.y
        << ", " << value.z << ")";
    return oss.str();
}

std::string format_waypoint(const sunray_msgs::PlanningWaypoint& waypoint) {
    std::ostringstream oss;
    oss << "pos=" << format_point3(waypoint.position) << " yaw=" << format_scalar(waypoint.yaw)
        << " hold_time=" << format_scalar(waypoint.hold_time);
    return oss.str();
}

enum class PlanningPanelSeverity {
    INFO,
    WARN,
    ERROR
};

constexpr const char* kPlanningAnsiReset = "\033[0m";
constexpr const char* kPlanningAnsiLightCyan = "\033[1;36m";
constexpr const char* kPlanningAnsiGreen = "\033[32m";
constexpr const char* kPlanningAnsiRed = "\033[31m";

std::string planning_colorize_console_text(const std::string& text, const char* color) {
    return std::string(color) + text + kPlanningAnsiReset;
}

spdlog::level::level_enum planning_panel_level(PlanningPanelSeverity severity) {
    switch (severity) {
    case PlanningPanelSeverity::INFO:
        return spdlog::level::info;
    case PlanningPanelSeverity::WARN:
        return spdlog::level::warn;
    case PlanningPanelSeverity::ERROR:
        return spdlog::level::err;
    }
    return spdlog::level::info;
}

void write_planning_panel_log(const PlanningPanelSeverity severity,
                              const std::string& plain_text,
                              const std::string& console_text) {
    auto logger = SunrayLogger::instance().Get();
    if (!logger) {
        return;
    }

    const spdlog::level::level_enum level = planning_panel_level(severity);
    auto log_to_sink = [&](const spdlog::sink_ptr& sink, const std::string& text) {
        if (!sink || !sink->should_log(level)) {
            return;
        }
        const spdlog::string_view_t payload(text.data(), text.size());
        const spdlog::details::log_msg msg(logger->name(), level, payload);
        sink->log(msg);
        sink->flush();
    };

    auto& sinks = logger->sinks();
    if (!sinks.empty()) {
        log_to_sink(sinks.front(), console_text);
    }
    for (std::size_t i = 1; i < sinks.size(); ++i) {
        log_to_sink(sinks[i], plain_text);
    }
}

PlanningPanelSeverity planning_panel_severity(const PlannerSnapshot& snapshot,
                                              const bool planner_ready,
                                              const bool task_active,
                                              const bool follow_control_fsm,
                                              const bool control_fsm_fresh,
                                              std::string& module_status) {
    module_status = "正常";
    if (!planner_ready || !snapshot.ready) {
        module_status = "规划器未就绪";
        return PlanningPanelSeverity::ERROR;
    }
    if (snapshot.planner_state == PlannerExecState::FAIL) {
        module_status = "规划失败";
        return PlanningPanelSeverity::ERROR;
    }
    if (snapshot.planner_state == PlannerExecState::EMERGENCY_STOP) {
        module_status = "规划器急停";
        return PlanningPanelSeverity::ERROR;
    }
    if (follow_control_fsm && !control_fsm_fresh) {
        module_status = "控制FSM状态超时/未收到";
        return PlanningPanelSeverity::WARN;
    }
    if (task_active && !snapshot.has_valid_output) {
        module_status = "等待规划输出";
        return PlanningPanelSeverity::WARN;
    }
    return PlanningPanelSeverity::INFO;
}

}  // namespace

void PlanningFSM::printf_terminal() {
    if (!planner_) {
        return;
    }

    const ros::Time now = ros::Time::now();
    if (!last_terminal_log_stamp_.isZero() && (now - last_terminal_log_stamp_).toSec() < 1.0) {
        return;
    }
    last_terminal_log_stamp_ = now;

    const PlannerSnapshot snapshot = planner_->get_planner_state();
    const PlanningFsmState effective_state = effective_fsm_state(now);
    const bool control_fsm_fresh = has_fresh_control_fsm_state(now);
    const bool planning_context = sunray_planning::has_planning_context(
        has_last_planning_cmd_, last_planning_cmd_, task_active_, task_arrived_, hover_hold_);
    const bool planner_ready = is_ready();

    std::string module_status;
    const PlanningPanelSeverity module_severity = planning_panel_severity(snapshot,
                                                                          planner_ready,
                                                                          task_active_,
                                                                          follow_control_fsm_,
                                                                          control_fsm_fresh,
                                                                          module_status);

    const std::string planner_ready_text = bool_to_cn(planner_ready, "已就绪", "未就绪");
    const std::string planner_snapshot_ready_text = bool_to_cn(snapshot.ready, "已就绪", "未就绪");
    const std::string goal_active_text = bool_to_cn(snapshot.goal_active, "执行中", "空闲");
    const std::string valid_output_text = bool_to_cn(snapshot.has_valid_output, "有效", "无效");
    const std::string control_follow_text = bool_to_cn(follow_control_fsm_, "启用", "关闭");
    const std::string control_fsm_status_text = follow_control_fsm_
                                                    ? (control_fsm_fresh ? "正常" : "超时/未收到")
                                                    : "未跟随";
    const std::string task_active_text = bool_to_cn(task_active_, "是", "否");
    const std::string task_arrived_text = bool_to_cn(task_arrived_, "是", "否");
    const std::string hover_hold_text = bool_to_cn(hover_hold_, "是", "否");
    const std::string trajectory_ack_text = bool_to_cn(control_fsm_trajectory_ack_, "已确认", "未确认");
    const std::string last_cmd_text = has_last_planning_cmd_
                                          ? sunray_planning::planning_control_cmd_to_string(last_planning_cmd_.control_cmd)
                                          : "UNDEFINE";
    const std::string pending_cmd_text =
        passthrough_control_cmd_ == sunray_msgs::UAVControlCMD::UNDEFINE
            ? "UNDEFINE"
            : uav_control_cmd_to_string(passthrough_control_cmd_);
    const std::string planning_frame_text =
        planning_context
            ? sunray_planning::planning_frame_to_string(
                  sunray_planning::planning_cmd_to_frame(last_planning_cmd_.control_cmd))
            : "UNDEFINE";
    const std::string goal_type_text =
        planning_context
            ? sunray_planning::goal_type_to_string(last_planning_cmd_.waypoints.size() > 1
                                                       ? sunray_msgs::UAVPlanningState::GOAL_MULTI
                                                       : sunray_msgs::UAVPlanningState::GOAL_SINGLE)
            : "UNDEFINE";
    const std::string cmd_source_text = planning_context
                                            ? sunray_planning::cmd_source_to_string(last_planning_cmd_.cmd_source)
                                            : "UNDEFINE";
    const uint32_t waypoint_count =
        planning_context ? static_cast<uint32_t>(last_planning_cmd_.waypoints.size()) : 0;
    const uint32_t waypoint_index = planning_context
                                        ? sunray_planning::clamp_waypoint_index(
                                              last_planning_cmd_, snapshot.current_waypoint_index)
                                        : 0;

    const std::string console_module_status = planning_colorize_console_text(
        module_status, module_severity == PlanningPanelSeverity::INFO ? kPlanningAnsiGreen
                                                                      : kPlanningAnsiRed);
    const std::string console_planner_ready = planning_colorize_console_text(
        planner_ready_text, planner_ready ? kPlanningAnsiGreen : kPlanningAnsiRed);
    const std::string console_snapshot_ready = planning_colorize_console_text(
        planner_snapshot_ready_text, snapshot.ready ? kPlanningAnsiGreen : kPlanningAnsiRed);
    const std::string console_goal_active = planning_colorize_console_text(
        goal_active_text, (!task_active_ || snapshot.goal_active) ? kPlanningAnsiGreen
                                                                  : kPlanningAnsiRed);
    const std::string console_valid_output = planning_colorize_console_text(
        valid_output_text, (!task_active_ || snapshot.has_valid_output) ? kPlanningAnsiGreen
                                                                        : kPlanningAnsiRed);
    const std::string console_control_fsm_status = planning_colorize_console_text(
        control_fsm_status_text,
        (!follow_control_fsm_ || control_fsm_fresh) ? kPlanningAnsiGreen : kPlanningAnsiRed);
    const std::string console_task_active = task_active_text;
    const std::string console_task_arrived = task_arrived_text;
    const std::string console_hover_hold = hover_hold_text;
    const std::string console_trajectory_ack = trajectory_ack_text;

    std::ostringstream body_oss;
    std::ostringstream console_body_oss;
    body_oss << std::fixed << std::setprecision(3);
    console_body_oss << std::fixed << std::setprecision(3);

    body_oss << " -------- 规划模块状态\n";
    console_body_oss << " -------- 规划模块状态\n";
    body_oss << " 模块状态: [ " << module_status << " ]\n";
    console_body_oss << " 模块状态: [ " << console_module_status << " ]\n";
    body_oss << " 规划器类型: [ " << planner_type_to_string(snapshot.planner_type) << " ]\n";
    console_body_oss << " 规划器类型: [ " << planner_type_to_string(snapshot.planner_type) << " ]\n";
    body_oss << " 规划器就绪: [ " << planner_ready_text << " ]\n";
    console_body_oss << " 规划器就绪: [ " << console_planner_ready << " ]\n";
    body_oss << " 规划器快照就绪: [ " << planner_snapshot_ready_text << " ]\n";
    console_body_oss << " 规划器快照就绪: [ " << console_snapshot_ready << " ]\n";
    body_oss << " 规划器状态: [ " << snapshot.planner_state_string << " ]\n";
    console_body_oss << " 规划器状态: [ " << snapshot.planner_state_string << " ]\n";
    body_oss << " 本地FSM状态: [ " << planning_fsm_state_to_string(fsm_state_) << " ]\n";
    console_body_oss << " 本地FSM状态: [ " << planning_fsm_state_to_string(fsm_state_) << " ]\n";
    body_oss << " 生效FSM状态: [ " << planning_fsm_state_to_string(effective_state) << " ]\n";
    console_body_oss << " 生效FSM状态: [ " << planning_fsm_state_to_string(effective_state) << " ]\n";

    body_oss << " -------- 当前订阅/发布信息\n";
    console_body_oss << " -------- 当前订阅/发布信息\n";
    body_oss << " planning_cmd订阅话题: " << planning_cmd_sub_topic_ << "\n";
    console_body_oss << " planning_cmd订阅话题: " << planning_cmd_sub_topic_ << "\n";
    body_oss << " control_fsm_state订阅话题: " << control_fsm_state_sub_topic_ << "\n";
    console_body_oss << " control_fsm_state订阅话题: " << control_fsm_state_sub_topic_ << "\n";
    body_oss << " uav_control_cmd发布话题: " << control_pub_topic_ << "\n";
    console_body_oss << " uav_control_cmd发布话题: " << control_pub_topic_ << "\n";
    body_oss << " planning_state发布话题: " << planning_state_pub_topic_ << "\n";
    console_body_oss << " planning_state发布话题: " << planning_state_pub_topic_ << "\n";

    body_oss << " -------- 控制FSM联动\n";
    console_body_oss << " -------- 控制FSM联动\n";
    body_oss << " 跟随控制FSM: [ " << control_follow_text << " ]\n";
    console_body_oss << " 跟随控制FSM: [ " << control_follow_text << " ]\n";
    body_oss << " 控制FSM状态: [ " << control_fsm_status_text << " ]\n";
    console_body_oss << " 控制FSM状态: [ " << console_control_fsm_status << " ]\n";
    if (control_fsm_fresh) {
        body_oss << " 控制FSM当前状态: [ "
                 << sunray_planning::control_fsm_state_to_string(last_control_fsm_state_.sunray_fsm_state)
                 << " ]\n";
        console_body_oss
            << " 控制FSM当前状态: [ "
            << sunray_planning::control_fsm_state_to_string(last_control_fsm_state_.sunray_fsm_state)
            << " ]\n";
        body_oss << " 控制FSM当前指令: [ "
                 << uav_control_cmd_to_string(last_control_fsm_state_.control_cmd) << " ]\n";
        console_body_oss << " 控制FSM当前指令: [ "
                         << uav_control_cmd_to_string(last_control_fsm_state_.control_cmd)
                         << " ]\n";
        body_oss << " 控制FSM时间间隔: "
                 << format_stamp_age_s(now, last_control_fsm_state_stamp_) << " [s]\n";
        console_body_oss << " 控制FSM时间间隔: "
                         << format_stamp_age_s(now, last_control_fsm_state_stamp_) << " [s]\n";
    } else {
        body_oss << " 尚未收到有效控制FSM状态\n";
        console_body_oss << " 尚未收到有效控制FSM状态\n";
    }

    body_oss << " -------- 当前规划任务\n";
    console_body_oss << " -------- 当前规划任务\n";
    body_oss << " task_id: " << task_id_ << "\n";
    console_body_oss << " task_id: " << task_id_ << "\n";
    body_oss << " 最近规划指令: [ " << last_cmd_text << " ]\n";
    console_body_oss << " 最近规划指令: [ " << last_cmd_text << " ]\n";
    body_oss << " 指令来源: [ " << cmd_source_text << " ]\n";
    console_body_oss << " 指令来源: [ " << cmd_source_text << " ]\n";
    body_oss << " 规划坐标系: [ " << planning_frame_text << " ]\n";
    console_body_oss << " 规划坐标系: [ " << planning_frame_text << " ]\n";
    body_oss << " 目标类型: [ " << goal_type_text << " ]\n";
    console_body_oss << " 目标类型: [ " << goal_type_text << " ]\n";
    body_oss << " 航点数量: " << waypoint_count << "  当前索引: " << waypoint_index << "\n";
    console_body_oss << " 航点数量: " << waypoint_count << "  当前索引: " << waypoint_index << "\n";
    body_oss << " 任务执行中: [ " << task_active_text << " ]\n";
    console_body_oss << " 任务执行中: [ " << console_task_active << " ]\n";
    body_oss << " 任务到达: [ " << task_arrived_text << " ]\n";
    console_body_oss << " 任务到达: [ " << console_task_arrived << " ]\n";
    body_oss << " 悬停保持: [ " << hover_hold_text << " ]\n";
    console_body_oss << " 悬停保持: [ " << console_hover_hold << " ]\n";
    body_oss << " 轨迹接管确认: [ " << trajectory_ack_text << " ]\n";
    console_body_oss << " 轨迹接管确认: [ " << console_trajectory_ack << " ]\n";
    body_oss << " 待转发特殊控制指令: [ " << pending_cmd_text << " ]\n";
    console_body_oss << " 待转发特殊控制指令: [ " << pending_cmd_text << " ]\n";

    if (planning_context && !last_planning_cmd_.waypoints.empty()) {
        body_oss << " 当前目标点: " << format_waypoint(last_planning_cmd_.waypoints[waypoint_index])
                 << "\n";
        console_body_oss << " 当前目标点: "
                         << format_waypoint(last_planning_cmd_.waypoints[waypoint_index]) << "\n";
        body_oss << " 最终目标点: " << format_waypoint(last_planning_cmd_.waypoints.back()) << "\n";
        console_body_oss << " 最终目标点: " << format_waypoint(last_planning_cmd_.waypoints.back())
                         << "\n";
    } else {
        body_oss << " 当前无规划目标点\n";
        console_body_oss << " 当前无规划目标点\n";
    }

    body_oss << " -------- 规划器输出\n";
    console_body_oss << " -------- 规划器输出\n";
    body_oss << " planner_goal_active: [ " << goal_active_text << " ]\n";
    console_body_oss << " planner_goal_active: [ " << console_goal_active << " ]\n";
    body_oss << " planner_valid_output: [ " << valid_output_text << " ]\n";
    console_body_oss << " planner_valid_output: [ " << console_valid_output << " ]\n";
    body_oss << " 最近目标时间间隔: " << format_stamp_age_s(now, snapshot.last_goal_stamp) << " [s]\n";
    console_body_oss << " 最近目标时间间隔: " << format_stamp_age_s(now, snapshot.last_goal_stamp)
                     << " [s]\n";
    body_oss << " 最近输出时间间隔: " << format_stamp_age_s(now, snapshot.last_output_stamp)
             << " [s]\n";
    console_body_oss << " 最近输出时间间隔: " << format_stamp_age_s(now, snapshot.last_output_stamp)
                     << " [s]\n";
    body_oss << " 最近状态时间间隔: " << format_stamp_age_s(now, snapshot.last_state_stamp)
             << " [s]\n";
    console_body_oss << " 最近状态时间间隔: " << format_stamp_age_s(now, snapshot.last_state_stamp)
                     << " [s]\n";

    body_oss << " -------- 运行参数\n";
    console_body_oss << " -------- 运行参数\n";
    body_oss << " process_rate: " << process_rate_hz_ << " [Hz]\n";
    console_body_oss << " process_rate: " << process_rate_hz_ << " [Hz]\n";
    body_oss << " state_pub_rate: " << state_pub_rate_hz_ << " [Hz]\n";
    console_body_oss << " state_pub_rate: " << state_pub_rate_hz_ << " [Hz]\n";
    body_oss << " control_fsm_timeout: " << control_fsm_state_timeout_sec_ << " [s]\n";
    console_body_oss << " control_fsm_timeout: " << control_fsm_state_timeout_sec_ << " [s]\n";
    body_oss << " auto_hover_on_timeout: " << bool_to_string(auto_hover_on_timeout_) << "\n";
    console_body_oss << " auto_hover_on_timeout: " << bool_to_string(auto_hover_on_timeout_) << "\n";

    body_oss << " ---------------------------------------------------------\n";
    console_body_oss << " ---------------------------------------------------------\n";

    const std::string header_line =
        ">>>>>>>>>>>>>>> 规划状态 - [ " + uav_ns_ + " ] <<<<<<<<<<<<<<<";
    const std::string plain_panel = header_line + "\n" + body_oss.str();
    const std::string console_panel = std::string(kPlanningAnsiLightCyan) + header_line +
                                      kPlanningAnsiReset + "\n" + console_body_oss.str();

    write_planning_panel_log(module_severity, plain_panel, console_panel);
}