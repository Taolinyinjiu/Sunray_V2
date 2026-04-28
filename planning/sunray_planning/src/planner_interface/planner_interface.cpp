#include "planner_interface/planner_interface.hpp"

void PlannerInterface::init(ros::NodeHandle& nh, const std::string& uav_ns) {
    uav_ns_ = uav_ns;
    goal_topic_ = default_goal_topic(uav_ns_);
    goal_frame_id_ = default_goal_frame_id();
    position_cmd_topic_ = default_position_cmd_topic(uav_ns_);
    planner_state_topic_ = default_planner_state_topic(uav_ns_);
    cmd_timeout_sec_ = default_cmd_timeout_sec();
    state_timeout_sec_ = default_state_timeout_sec();
    snapshot_ = PlannerSnapshot{};
    snapshot_.planner_type = planner_type();
    snapshot_.planner_state = PlannerExecState::WAIT_TARGET;
    snapshot_.planner_state_string = planner_exec_state_to_string(snapshot_.planner_state);
    snapshot_.current_waypoint_index = 0;
    snapshot_.ready = false;
    latest_position_cmd_ = PlannerPositionCommand{};

    bind_topics(nh);

    if (!planner_state_topic_.empty()) {
        unified_state_sub_ = nh.subscribe(planner_state_topic_,
                                          10,
                                          &PlannerInterface::unified_state_callback,
                                          this);
    }

    snapshot_.ready = true;
    snapshot_.last_state_stamp = ros::Time::now();
}

bool PlannerInterface::is_ready() const { return snapshot_.ready; }

bool PlannerInterface::fetch_latest_position_cmd(PlannerPositionCommand& cmd,
                                                 const ros::Time& now) const {
    const PlannerSnapshot snapshot = get_state(now);
    if (!snapshot.ready || !snapshot.has_valid_output || snapshot.last_output_stamp.isZero()) {
        return false;
    }
    cmd = latest_position_cmd_;
    if (cmd.stamp.isZero()) {
        cmd.stamp = snapshot.last_output_stamp;
    }
    return true;
}

PlannerSnapshot PlannerInterface::get_state(const ros::Time& now) const {
    PlannerSnapshot snapshot = snapshot_;

    if (!now.isZero()) {
        if (!planner_state_topic_.empty() && state_timeout_sec_ > 0.0 &&
            !snapshot.last_state_stamp.isZero() &&
            (now - snapshot.last_state_stamp).toSec() > state_timeout_sec_) {
            snapshot.planner_state = PlannerExecState::UNDEFINE;
            snapshot.planner_state_string = "STALE";
        }

        if (cmd_timeout_sec_ > 0.0 && !snapshot.last_output_stamp.isZero() &&
            (now - snapshot.last_output_stamp).toSec() > cmd_timeout_sec_) {
            snapshot.has_valid_output = false;
        }
    }

    return snapshot;
}

const std::string& PlannerInterface::goal_topic() const { return goal_topic_; }

const std::string& PlannerInterface::goal_frame_id() const { return goal_frame_id_; }

const std::string& PlannerInterface::position_cmd_topic() const { return position_cmd_topic_; }

const std::string& PlannerInterface::planner_state_topic() const { return planner_state_topic_; }

std::string PlannerInterface::default_goal_frame_id() const { return "world"; }

std::string PlannerInterface::default_planner_state_topic(const std::string& uav_ns) const {
    (void)uav_ns;
    return "";
}

double PlannerInterface::default_cmd_timeout_sec() const { return 0.3; }

double PlannerInterface::default_state_timeout_sec() const { return 1.0; }

void PlannerInterface::set_latest_position_cmd(const PlannerPositionCommand& cmd) {
    latest_position_cmd_ = cmd;
    if (latest_position_cmd_.stamp.isZero()) {
        latest_position_cmd_.stamp = ros::Time::now();
    }
    snapshot_.ready = true;
    snapshot_.has_valid_output = true;
    snapshot_.last_output_stamp = latest_position_cmd_.stamp;
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
    snapshot_.has_valid_output = false;
    snapshot_.current_waypoint_index = 0;
    snapshot_.last_goal_stamp = stamp;
    snapshot_.last_output_stamp = ros::Time();
    latest_position_cmd_ = PlannerPositionCommand{};
    set_planner_state(PlannerExecState::GENERATE, stamp);
}

void PlannerInterface::unified_state_callback(const sunray_msgs::UAVPlanningState::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    const PlannerType expected_planner_type = planner_type();
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
