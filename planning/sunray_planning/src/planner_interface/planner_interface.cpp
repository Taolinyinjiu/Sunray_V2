#include "planner_interface/planner_interface.hpp"

void PlannerInterface::init(ros::NodeHandle& nh, const PlannerRuntimeConfig& config) {
    config_ = config;
    snapshot_ = PlannerSnapshot{};
    snapshot_.planner_type = planner_type_from_string(config_.planner_type);
    snapshot_.planner_state = PlannerExecState::WAIT_TARGET;
    snapshot_.planner_state_string = planner_exec_state_to_string(snapshot_.planner_state);
    snapshot_.current_waypoint_index = 0;
    snapshot_.ready = false;
    latest_cmd_ = sunray_msgs::UAVControlCMD{};
    latest_cmd_.control_cmd = sunray_msgs::UAVControlCMD::UNDEFINE;

    bind_topics(nh);

    if (!config_.planner_state_topic.empty()) {
        unified_state_sub_ = nh.subscribe(config_.planner_state_topic,
                                          10,
                                          &PlannerInterface::unified_state_callback,
                                          this);
    }

    snapshot_.ready = true;
    snapshot_.last_state_stamp = ros::Time::now();
}

bool PlannerInterface::is_ready() const { return snapshot_.ready; }

bool PlannerInterface::fetch_latest_control_cmd(sunray_msgs::UAVControlCMD& cmd,
                                                const ros::Time& now) const {
    if (!snapshot_.ready || !snapshot_.has_valid_output || snapshot_.last_output_stamp.isZero()) {
        return false;
    }
    if ((now - snapshot_.last_output_stamp).toSec() > config_.cmd_timeout_sec) {
        return false;
    }
    if (latest_cmd_.control_cmd == sunray_msgs::UAVControlCMD::UNDEFINE) {
        return false;
    }
    cmd = latest_cmd_;
    return true;
}

PlannerSnapshot PlannerInterface::get_state() const { return snapshot_; }

const PlannerRuntimeConfig& PlannerInterface::get_config() const { return config_; }

void PlannerInterface::set_latest_control_cmd(const sunray_msgs::UAVControlCMD& cmd) {
    latest_cmd_ = cmd;
    if (latest_cmd_.header.stamp.isZero()) {
        latest_cmd_.header.stamp = ros::Time::now();
    }
    snapshot_.ready = true;
    snapshot_.has_valid_output = true;
    snapshot_.last_output_stamp = latest_cmd_.header.stamp;
}

void PlannerInterface::set_planner_state(const PlannerExecState planner_state, const ros::Time& stamp) {
    snapshot_.planner_state = planner_state;
    snapshot_.planner_state_string = planner_exec_state_to_string(planner_state);
    snapshot_.last_state_stamp = stamp;

    switch (planner_state) {
    case PlannerExecState::GENERATE:
    case PlannerExecState::REPLAN:
    case PlannerExecState::EXEC:
    case PlannerExecState::PAUSE:
        snapshot_.goal_active = true;
        break;
    case PlannerExecState::SUCCESS:
    case PlannerExecState::FAIL:
    case PlannerExecState::EMERGENCY_STOP:
        snapshot_.goal_active = false;
        if (planner_state != PlannerExecState::SUCCESS) {
            snapshot_.has_valid_output = false;
        }
        break;
    case PlannerExecState::INIT:
    case PlannerExecState::WAIT_TARGET:
    case PlannerExecState::UNDEFINE:
    default:
        snapshot_.goal_active = false;
        break;
    }
}

void PlannerInterface::mark_goal_sent(const ros::Time& stamp) {
    snapshot_.ready = true;
    snapshot_.goal_active = true;
    snapshot_.current_waypoint_index = 0;
    snapshot_.last_goal_stamp = stamp;
    set_planner_state(PlannerExecState::GENERATE, stamp);
}

void PlannerInterface::unified_state_callback(const sunray_msgs::UAVPlanningState::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    const PlannerType expected_planner_type = planner_type_from_string(config_.planner_type);
    if (msg->planner_type != 0 &&
        msg->planner_type != static_cast<uint8_t>(expected_planner_type)) {
        return;
    }

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    snapshot_.ready = true;
    snapshot_.planner_type = expected_planner_type;
    snapshot_.planner_state = planner_exec_state_from_msg(msg->planner_state);
    snapshot_.planner_state_string =
        msg->planner_state_string.empty() ? planner_exec_state_to_string(snapshot_.planner_state)
                                          : msg->planner_state_string;
    snapshot_.current_waypoint_index = msg->current_waypoint_index;
    snapshot_.last_state_stamp = stamp;

    switch (snapshot_.planner_state) {
    case PlannerExecState::GENERATE:
    case PlannerExecState::REPLAN:
    case PlannerExecState::EXEC:
    case PlannerExecState::PAUSE:
        snapshot_.goal_active = true;
        break;
    case PlannerExecState::SUCCESS:
    case PlannerExecState::FAIL:
    case PlannerExecState::EMERGENCY_STOP:
    case PlannerExecState::INIT:
    case PlannerExecState::WAIT_TARGET:
    case PlannerExecState::UNDEFINE:
    default:
        snapshot_.goal_active = false;
        if (snapshot_.planner_state != PlannerExecState::SUCCESS) {
            snapshot_.has_valid_output = false;
        }
        break;
    }
}
