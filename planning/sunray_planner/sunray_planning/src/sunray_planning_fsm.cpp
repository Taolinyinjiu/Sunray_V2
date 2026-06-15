#include "planning_fsm.hpp"
#include "sunray_planning_common.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <ros/package.h>

#include "agent_key_helper.hpp"
#include "planner_interface/diff_planner.hpp"
#include "planner_interface/ego_planner.hpp"
#include "planner_interface/planner_interface.hpp"
#include "sunray_log.hpp"

namespace {
constexpr double kControlFsmOverrideGraceSec = 2.0;

std::string load_agent_key_or_throw(ros::NodeHandle& private_nh) {
    bool use_private_agent_key = false;
    private_nh.param("use_private_agent_key", use_private_agent_key, false);
    return use_private_agent_key ? sunray_common::get_agent_key_from_private()
                                 : sunray_common::get_agent_key_from_global();
}

std::string expand_topic(const std::string& raw_topic, const std::string& agent_key) {
    if (raw_topic.empty()) {
        return raw_topic;
    }
    return sunray_common::replace_agent_key(raw_topic, agent_key);
}

bool is_special_planning_cmd(const uint8_t plan_cmd) {
    switch (plan_cmd) {
    case sunray_msgs::UAVPlanningCMD::TAKEOFF:
    case sunray_msgs::UAVPlanningCMD::LAND:
    case sunray_msgs::UAVPlanningCMD::PLAN_RETURN:
    case sunray_msgs::UAVPlanningCMD::HOVER:
        return true;
    case sunray_msgs::UAVPlanningCMD::PLAN_LOCAL_GOAL:
    case sunray_msgs::UAVPlanningCMD::PLAN_GLOBAL_GOAL:
    default:
        return false;
    }
}

uint8_t planning_special_cmd_to_uav_control_cmd(const uint8_t plan_cmd) {
    switch (plan_cmd) {
    case sunray_msgs::UAVPlanningCMD::TAKEOFF:
        return sunray_msgs::UAVControlCMD::TAKEOFF;
    case sunray_msgs::UAVPlanningCMD::LAND:
        return sunray_msgs::UAVControlCMD::LAND;
    case sunray_msgs::UAVPlanningCMD::PLAN_RETURN:
        return sunray_msgs::UAVControlCMD::RETURN;
    case sunray_msgs::UAVPlanningCMD::HOVER:
        return sunray_msgs::UAVControlCMD::HOVER;
    default:
        return sunray_msgs::UAVControlCMD::UNDEFINE;
    }
}

std::string parent_directory(const std::string& path) {
    const std::size_t pos = path.find_last_of('/');
    return (pos == std::string::npos) ? "" : path.substr(0, pos);
}

std::string sanitize_path_token(std::string token) {
    std::replace(token.begin(), token.end(), '/', '_');
    return token.empty() ? "default" : token;
}

std::string make_startup_timestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now;
    localtime_r(&now_time, &tm_now);

    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S") << "_" << std::setfill('0') << std::setw(3)
        << milliseconds.count();
    return oss.str();
}

PlanningFsmState passthrough_control_cmd_to_fsm_state(const uint8_t control_cmd) {
    switch (control_cmd) {
    case sunray_msgs::UAVControlCMD::TAKEOFF:
        return PlanningFsmState::TAKEOFF;
    case sunray_msgs::UAVControlCMD::LAND:
        return PlanningFsmState::LAND;
    case sunray_msgs::UAVControlCMD::RETURN:
        return PlanningFsmState::RETURN;
    case sunray_msgs::UAVControlCMD::KILL:
        return PlanningFsmState::EMERGENCY_KILL;
    case sunray_msgs::UAVControlCMD::HOVER:
        return PlanningFsmState::HOVER;
    default:
        return PlanningFsmState::READY;
    }
}

PlanningFsmState active_task_to_fsm_state() {
    return PlanningFsmState::PLANNING;
}
}  // namespace

PlanningFSM::PlanningFSM(ros::NodeHandle& nh) : nh_(nh), private_nh_("~") {}

void PlanningFSM::init() {
    // 读取参数
    load_param();

    // The planner adapter is selected once at node startup from launch/config
    // and kept behind the shared PlannerInterface contract for the whole run.
    switch (planner_type_from_string(selected_planner_type_)) {
    case PlannerType::EGO:
        planner_.reset(new EgoPlanner());
        break;
    case PlannerType::DIFF:
        planner_.reset(new DiffPlanner());
        break;
    case PlannerType::FUEL:
        throw std::runtime_error(
            "planner type 'fuel' is not supported by sunray_planning current adapter; "
            "only ego and diff are enabled");
    case PlannerType::SUPER:
        throw std::runtime_error(
            "planner type 'super' is not supported by sunray_planning current adapter; "
            "only ego and diff are enabled");
    case PlannerType::UNDEFINE:
    default:
        throw std::runtime_error("unsupported planner type '" + selected_planner_type_ + "'");
    }
    planner_->init(private_nh_);

    planning_cmd_sub_ =
        nh_.subscribe(planning_cmd_sub_topic_, 10, &PlanningFSM::planning_cmd_callback, this);
    control_fsm_state_sub_ = nh_.subscribe(
        control_fsm_state_sub_topic_, 10, &PlanningFSM::control_fsm_state_callback, this);
    control_pub_ = nh_.advertise<sunray_msgs::UAVControlCMD>(control_pub_topic_, 10);
    planning_state_pub_ =
        nh_.advertise<sunray_msgs::UAVPlanningState>(planning_state_pub_topic_, 10);

    process_timer_ = nh_.createTimer(
        ros::Duration(1.0 / process_rate_hz_), &PlanningFSM::process_timer_callback, this);
    planning_state_timer_ = nh_.createTimer(
        ros::Duration(1.0 / state_pub_rate_hz_), &PlanningFSM::planning_state_timer_callback, this);

    fsm_state_ = is_ready() ? PlanningFsmState::READY : PlanningFsmState::INIT;

    const PlannerType planner_type = planner_type_from_string(selected_planner_type_);
    SUNRAY_INFO(
        "[sunray_planning] planner={} control_pub={} control_fsm_topic={} follow_control_fsm={}",
        planner_type_to_string(planner_type),
        control_pub_topic_,
        control_fsm_state_sub_topic_,
        follow_control_fsm_ ? 1 : 0);
}

void PlanningFSM::load_param() {
    uav_ns_ = load_agent_key_or_throw(private_nh_);
    private_nh_.param("log_save", log_save_, false);
    init_logger();

    // 规划器类型
    private_nh_.param("planner_type", selected_planner_type_, std::string(""));
    selected_planner_type_ = normalize_planner_type(selected_planner_type_);

    private_nh_.param("process_rate_hz", process_rate_hz_, 50.0);
    private_nh_.param("state_pub_rate_hz", state_pub_rate_hz_, 10.0);
    private_nh_.param("auto_hover_on_timeout", auto_hover_on_timeout_, true);
    private_nh_.param("control_fsm_state_timeout_sec", control_fsm_state_timeout_sec_, 1.0);
    private_nh_.param("follow_control_fsm", follow_control_fsm_, true);

    if (process_rate_hz_ <= 0.0) {
        process_rate_hz_ = 50.0;
    }
    if (state_pub_rate_hz_ <= 0.0) {
        state_pub_rate_hz_ = 10.0;
    }
    if (control_fsm_state_timeout_sec_ <= 0.0) {
        control_fsm_state_timeout_sec_ = 1.0;
    }

    std::string planning_cmd_topic = "${agent_key}/sunray/uav_planning/planning_cmd";
    std::string control_pub_topic = "${agent_key}/sunray/uav_control/control_cmd";
    std::string planning_state_topic = "${agent_key}/sunray/uav_planning/planning_state";
    std::string control_fsm_state_topic = "${agent_key}/sunray/uav_control/control_state";

    private_nh_.param("planning_cmd_sub_topic", planning_cmd_topic, planning_cmd_topic);
    private_nh_.param("control_pub_topic", control_pub_topic, control_pub_topic);
    private_nh_.param("planning_state_pub_topic", planning_state_topic, planning_state_topic);
    private_nh_.param(
        "control_fsm_state_sub_topic", control_fsm_state_topic, control_fsm_state_topic);

    planning_cmd_sub_topic_ = expand_topic(planning_cmd_topic, uav_ns_);
    control_pub_topic_ = expand_topic(control_pub_topic, uav_ns_);
    planning_state_pub_topic_ = expand_topic(planning_state_topic, uav_ns_);
    control_fsm_state_sub_topic_ = expand_topic(control_fsm_state_topic, uav_ns_);
}

void PlanningFSM::init_logger() {
    SunrayLogConfig cfg;
    cfg.name = "sunray_planning_" + uav_ns_;
    cfg.console_level = SunrayLogLevel::info;
    cfg.file_level = SunrayLogLevel::trace;
    cfg.async = false;

    if (log_save_) {
        log_file_path_ = make_log_file_path();
        cfg.file_path = log_file_path_;
    }

    SunrayLogger::instance().Init(cfg);
    auto logger = SunrayLogger::instance().Get();
    if (logger && !logger->sinks().empty()) {
        logger->sinks().front()->set_pattern("%v");
    }

    SUNRAY_INFO("sunray_planning logger initialized. log_save={} file_path={}",
                log_save_,
                log_save_ ? log_file_path_ : "disabled");
}

std::string PlanningFSM::make_log_file_path() const {
    const std::string package_path = ros::package::getPath("sunray_planning");
    if (package_path.empty()) {
        throw std::runtime_error("failed to locate package path for sunray_planning");
    }

    const std::string planning_root = parent_directory(package_path);
    const std::string repo_root = parent_directory(planning_root);
    if (repo_root.empty()) {
        throw std::runtime_error("failed to infer repository root from package path: " +
                                 package_path);
    }

    return repo_root + "/log/sunray_planning/" + sanitize_path_token(uav_ns_) + "_" +
           make_startup_timestamp() + ".log";
}

bool PlanningFSM::is_ready() {
    return planner_ && planner_->is_ready();
}

void PlanningFSM::planning_cmd_callback(const sunray_msgs::UAVPlanningCMD::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    has_last_planning_cmd_ = true;
    last_planning_cmd_ = *msg;
    ++task_id_;

    task_arrived_ = false;
    hover_hold_ = false;

    const uint8_t plan_cmd = last_planning_cmd_.plan_cmd;
    if (is_special_planning_cmd(plan_cmd)) {
        passthrough_control_cmd_ = planning_special_cmd_to_uav_control_cmd(plan_cmd);
        task_active_ = false;
        task_arrived_ = false;
        hover_hold_ = (passthrough_control_cmd_ == sunray_msgs::UAVControlCMD::HOVER);
        fsm_state_ = passthrough_control_cmd_to_fsm_state(passthrough_control_cmd_);
        SUNRAY_INFO("[sunray_planning] received special planning command={}, forward once to uav_control",
                    sunray_planning::planning_cmd_to_string(plan_cmd));
        return;
    }

    passthrough_control_cmd_ = sunray_msgs::UAVControlCMD::UNDEFINE;

    if (!sunray_planning::is_planning_goal_cmd(plan_cmd)) {
        SUNRAY_WARN("[sunray_planning] unsupported planning command={}, ignore",
                    sunray_planning::planning_cmd_to_string(plan_cmd));
        task_active_ = false;
        fsm_state_ = PlanningFsmState::READY;
        return;
    }

    const ros::Time now = ros::Time::now();
    std::string reject_reason;
    if (!control_fsm_allows_planning_goal(now, reject_reason)) {
        SUNRAY_WARN("[sunray_planning] reject planning command={} because {}",
                    sunray_planning::planning_cmd_to_string(plan_cmd),
                    reject_reason);
        task_active_ = false;
        return;
    }

    const uint8_t planning_frame = sunray_planning::planning_cmd_to_frame(plan_cmd);

    if (msg->waypoints.empty()) {
        SUNRAY_WARN("[sunray_planning] received planning command={} with empty waypoints, ignore",
                    sunray_planning::planning_cmd_to_string(plan_cmd));
        task_active_ = false;
        fsm_state_ = PlanningFsmState::READY;
        return;
    }

    if (msg->waypoints.size() > 1) {
        SUNRAY_WARN(
            "[sunray_planning] multi-waypoint command received, only the first waypoint is forwarded in this first version");
    }
    const sunray_msgs::PlanningWaypoint& target_waypoint = msg->waypoints.front();

    active_target_.position = Eigen::Vector3d(target_waypoint.position.x,
                                              target_waypoint.position.y,
                                              target_waypoint.position.z);
    active_target_.yaw = target_waypoint.yaw;
    active_target_.planning_frame = planning_frame;
    active_target_.cmd_source = msg->plan_cmd_source;
    active_target_.waypoint_count = msg->waypoints.size();
    active_target_.waypoint_index = 0;

    if (!planner_ || !planner_->send_goal(active_target_)) {
        const PlannerType planner_type = planner_type_from_string(selected_planner_type_);
        SUNRAY_ERROR("[sunray_planning] failed to send goal to planner '{}'",
                     planner_type_to_string(planner_type));
        task_active_ = false;
        hover_hold_ = true;
        passthrough_control_cmd_ = sunray_msgs::UAVControlCMD::HOVER;
        fsm_state_ = PlanningFsmState::HOVER;
        return;
    }

    task_active_ = true;
    control_fsm_trajectory_ack_ = false;
    last_goal_accept_stamp_ = ros::Time::now();
    fsm_state_ = active_task_to_fsm_state();
}

void PlanningFSM::control_fsm_state_callback(const sunray_msgs::UAVControlState::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    last_control_fsm_state_ = *msg;
    last_control_fsm_state_stamp_ =
        msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_control_fsm_state_ = true;

    if (!follow_control_fsm_) {
        return;
    }

    const bool planner_generated_motion =
        task_active_ && msg->control_state == sunray_msgs::UAVControlState::MOVE &&
        msg->last_cmd.control_cmd == sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY;
    const bool keep_arrived_state =
        task_arrived_ && msg->control_state == sunray_msgs::UAVControlState::HOVER;
    const bool keep_hover_hold_state =
        hover_hold_ && msg->control_state == sunray_msgs::UAVControlState::HOVER;
    if (planner_generated_motion) {
        control_fsm_trajectory_ack_ = true;
        return;
    }
    if (keep_arrived_state || keep_hover_hold_state) {
        return;
    }

    const ros::Time now = ros::Time::now();
    const bool in_goal_handshake_grace_window =
        task_active_ && !control_fsm_trajectory_ack_ &&
        (now - last_goal_accept_stamp_).toSec() <= kControlFsmOverrideGraceSec;
    if (in_goal_handshake_grace_window && msg->control_state == sunray_msgs::UAVControlState::HOVER) {
        return;
    }

    if (task_active_ || task_arrived_ || hover_hold_ ||
        passthrough_control_cmd_ != sunray_msgs::UAVControlCMD::UNDEFINE) {
        SUNRAY_WARN(
            "[sunray_planning] control FSM override detected: state={} control_cmd={}, clear local planning context",
            sunray_planning::control_fsm_state_to_string(msg->control_state),
            std::to_string(msg->last_cmd.control_cmd));
    }

    task_active_ = false;
    task_arrived_ = false;
    hover_hold_ = (msg->control_state == sunray_msgs::UAVControlState::HOVER);
    passthrough_control_cmd_ = sunray_msgs::UAVControlCMD::UNDEFINE;
}

void PlanningFSM::process_timer_callback(const ros::TimerEvent& event) {
    (void)event;
    process();
}

void PlanningFSM::planning_state_timer_callback(const ros::TimerEvent& event) {
    (void)event;
    pub_planning_state();
}

void PlanningFSM::process() {
    if (!is_ready()) {
        fsm_state_ = PlanningFsmState::INIT;
        return;
    }

    const PlannerSnapshot snapshot = planner_->get_planner_state();

    if (task_active_) {
        if (snapshot.planner_state == PlannerExecState::SUCCESS) {
            task_active_ = false;
            task_arrived_ = true;
            hover_hold_ = false;
            passthrough_control_cmd_ = sunray_msgs::UAVControlCMD::HOVER;
            fsm_state_ = PlanningFsmState::ARRIVED;
        } else if (snapshot.planner_state == PlannerExecState::FAIL ||
                   snapshot.planner_state == PlannerExecState::EMERGENCY_STOP) {
            task_active_ = false;
            task_arrived_ = false;
            hover_hold_ = true;
            passthrough_control_cmd_ = sunray_msgs::UAVControlCMD::HOVER;
            fsm_state_ = PlanningFsmState::HOVER;
        } else if (auto_hover_on_timeout_ && !snapshot.has_valid_output &&
                   !snapshot.last_output_stamp.isZero()) {
            task_active_ = false;
            task_arrived_ = false;
            hover_hold_ = true;
            passthrough_control_cmd_ = sunray_msgs::UAVControlCMD::HOVER;
            fsm_state_ = PlanningFsmState::HOVER;
        } else {
            fsm_state_ = active_task_to_fsm_state();
        }
    } else if (passthrough_control_cmd_ != sunray_msgs::UAVControlCMD::UNDEFINE) {
        fsm_state_ = passthrough_control_cmd_to_fsm_state(passthrough_control_cmd_);
    } else if (task_arrived_) {
        fsm_state_ = PlanningFsmState::ARRIVED;
    } else if (hover_hold_) {
        fsm_state_ = PlanningFsmState::HOVER;
    } else {
        fsm_state_ = PlanningFsmState::READY;
    }

    pub_control_cmd();
    printf_terminal();
}

void PlanningFSM::pub_control_cmd() {
    if (!planner_) {
        return;
    }

    const PlannerSnapshot snapshot = planner_->get_planner_state();

    if (task_active_) {
        PlannerPositionCommand planner_cmd;
        if (planner_->get_planner_positioncmd(planner_cmd)) {
            const uint8_t cmd_source = sunray_msgs::UAVControlCMD::PLANNING;
            control_pub_.publish(build_trajectory_control_cmd(planner_cmd, cmd_source));
        }
        return;
    }

    if (passthrough_control_cmd_ == sunray_msgs::UAVControlCMD::UNDEFINE) {
        return;
    }

    const uint8_t control_cmd = passthrough_control_cmd_;
    const uint8_t cmd_source = sunray_msgs::UAVControlCMD::PLANNING;
    control_pub_.publish(build_special_control_cmd(control_cmd, snapshot, cmd_source));
    passthrough_control_cmd_ = sunray_msgs::UAVControlCMD::UNDEFINE;
}

void PlanningFSM::pub_planning_state() {
    sunray_msgs::UAVPlanningState planning_state_msg;
    const ros::Time now = ros::Time::now();
    planning_state_msg.header.stamp = now;

    const PlannerSnapshot snapshot = planner_ ? planner_->get_planner_state() : PlannerSnapshot{};
    const PlannerType planner_type =
        planner_ ? snapshot.planner_type : planner_type_from_string(selected_planner_type_);
    const PlanningFsmState effective_state = effective_fsm_state(now);
    planning_state_msg.planner_type_string = planner_type_to_string(planner_type);

    switch (effective_state) {
    case PlanningFsmState::READY:
        planning_state_msg.sunray_planning_state = sunray_msgs::UAVPlanningState::INIT;
        break;
    case PlanningFsmState::TAKEOFF:
        planning_state_msg.sunray_planning_state = sunray_msgs::UAVPlanningState::TAKEOFF;
        break;
    case PlanningFsmState::LAND:
        planning_state_msg.sunray_planning_state = sunray_msgs::UAVPlanningState::LAND;
        break;
    case PlanningFsmState::PLANNING:
    case PlanningFsmState::MOVE:
    case PlanningFsmState::RETURN:
    case PlanningFsmState::HOVER:
        planning_state_msg.sunray_planning_state = sunray_msgs::UAVPlanningState::PLANNING;
        break;
    case PlanningFsmState::ARRIVED:
        planning_state_msg.sunray_planning_state = sunray_msgs::UAVPlanningState::ARRIVED;
        break;
    case PlanningFsmState::EMERGENCY_KILL:
        planning_state_msg.sunray_planning_state = sunray_msgs::UAVPlanningState::PLAN_FAILED;
        break;
    case PlanningFsmState::INIT:
    default:
        planning_state_msg.sunray_planning_state = sunray_msgs::UAVPlanningState::OFF;
        break;
    }

    if (snapshot.planner_state == PlannerExecState::FAIL ||
        snapshot.planner_state == PlannerExecState::EMERGENCY_STOP) {
        planning_state_msg.sunray_planning_state = sunray_msgs::UAVPlanningState::PLAN_FAILED;
    }

    if (has_control_fsm_state_) {
        planning_state_msg.home_point = last_control_fsm_state_.home_point;
    }
    if (has_last_planning_cmd_) {
        planning_state_msg.planning_cmd = last_planning_cmd_;
    }

    planning_state_pub_.publish(planning_state_msg);
}

sunray_msgs::UAVControlCMD PlanningFSM::build_trajectory_control_cmd(
    const PlannerPositionCommand& planner_cmd, const uint8_t cmd_source) const {
    sunray_msgs::UAVControlCMD control_cmd;
    control_cmd.header.stamp = planner_cmd.stamp.isZero() ? ros::Time::now() : planner_cmd.stamp;
    control_cmd.cmd_source = cmd_source;
    control_cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY;
    control_cmd.desired_pos.x = planner_cmd.position.x();
    control_cmd.desired_pos.y = planner_cmd.position.y();
    control_cmd.desired_pos.z = planner_cmd.position.z();
    control_cmd.desired_vel.x = planner_cmd.velocity.x();
    control_cmd.desired_vel.y = planner_cmd.velocity.y();
    control_cmd.desired_vel.z = planner_cmd.velocity.z();
    control_cmd.desired_acc.x = planner_cmd.acceleration.x();
    control_cmd.desired_acc.y = planner_cmd.acceleration.y();
    control_cmd.desired_acc.z = planner_cmd.acceleration.z();
    control_cmd.desired_jerk.x = planner_cmd.jerk.x();
    control_cmd.desired_jerk.y = planner_cmd.jerk.y();
    control_cmd.desired_jerk.z = planner_cmd.jerk.z();
    control_cmd.desired_yaw = planner_cmd.yaw;
    control_cmd.desired_yaw_rate = planner_cmd.yaw_rate;
    control_cmd.fixed_height = 0.0;
    // The current UAVControlCMD contract can only select one yaw mode at a time.
    // Prefer absolute yaw to keep trajectory heading deterministic across planners.
    control_cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
    return control_cmd;
}

sunray_msgs::UAVControlCMD PlanningFSM::build_special_control_cmd(
    const uint8_t control_cmd,
    const PlannerSnapshot& snapshot,
    const uint8_t cmd_source) const {
    sunray_msgs::UAVControlCMD control_cmd_msg;
    control_cmd_msg.header.stamp = ros::Time::now();
    control_cmd_msg.cmd_source = cmd_source;
    control_cmd_msg.control_cmd = control_cmd;
    control_cmd_msg.yaw_mode = sunray_msgs::UAVControlCMD::KEEP_YAW;

    if (has_last_planning_cmd_ && !last_planning_cmd_.waypoints.empty()) {
        const uint32_t waypoint_index = sunray_planning::clamp_waypoint_index(
            last_planning_cmd_, snapshot.current_waypoint_index);
        control_cmd_msg.desired_yaw = last_planning_cmd_.waypoints[waypoint_index].yaw;
    }

    return control_cmd_msg;
}

bool PlanningFSM::has_fresh_control_fsm_state(const ros::Time& now) const {
    if (!has_control_fsm_state_ || last_control_fsm_state_stamp_.isZero()) {
        return false;
    }
    return (now - last_control_fsm_state_stamp_).toSec() <= control_fsm_state_timeout_sec_;
}

bool PlanningFSM::control_fsm_allows_planning_goal(const ros::Time& now,
                                                   std::string& reason) const {
    if (!follow_control_fsm_) {
        return true;
    }
    if (!has_fresh_control_fsm_state(now)) {
        reason = "control FSM state is unavailable or timeout";
        return false;
    }

    switch (last_control_fsm_state_.control_state) {
    case sunray_msgs::UAVControlState::HOVER:
    case sunray_msgs::UAVControlState::MOVE:
        return true;
    case sunray_msgs::UAVControlState::OFF:
    case sunray_msgs::UAVControlState::INIT:
        reason = "uav_control is not ready";
        return false;
    case sunray_msgs::UAVControlState::TAKEOFF:
        reason = "uav_control is in TAKEOFF";
        return false;
    case sunray_msgs::UAVControlState::RETURN:
        reason = "uav_control is in RETURN";
        return false;
    case sunray_msgs::UAVControlState::LAND:
        reason = "uav_control is in LAND";
        return false;
    case sunray_msgs::UAVControlState::EMERGENCY_KILL:
        reason = "uav_control is in EMERGENCY_KILL";
        return false;
    default:
        reason = "uav_control is in unknown state";
        return false;
    }
}

PlanningFsmState PlanningFSM::effective_fsm_state(const ros::Time& now) const {
    if (!follow_control_fsm_ || !has_fresh_control_fsm_state(now)) {
        return fsm_state_;
    }

    switch (last_control_fsm_state_.control_state) {
    case sunray_msgs::UAVControlState::OFF:
    case sunray_msgs::UAVControlState::INIT:
        return PlanningFsmState::INIT;
    case sunray_msgs::UAVControlState::TAKEOFF:
        return PlanningFsmState::TAKEOFF;
    case sunray_msgs::UAVControlState::HOVER:
        return task_arrived_ ? PlanningFsmState::ARRIVED : PlanningFsmState::HOVER;
    case sunray_msgs::UAVControlState::RETURN:
        return PlanningFsmState::RETURN;
    case sunray_msgs::UAVControlState::LAND:
        return PlanningFsmState::LAND;
    case sunray_msgs::UAVControlState::MOVE:
        if (task_active_ &&
            has_last_planning_cmd_ &&
            sunray_planning::is_planning_goal_cmd(last_planning_cmd_.plan_cmd) &&
            last_control_fsm_state_.last_cmd.control_cmd ==
                sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY) {
            return active_task_to_fsm_state();
        }
        return PlanningFsmState::MOVE;
    case sunray_msgs::UAVControlState::EMERGENCY_KILL:
        return PlanningFsmState::EMERGENCY_KILL;
    default:
        return fsm_state_;
    }
}
