#include "planner_interface/ego_planner.hpp"

#include <cmath>
#include <stdexcept>

#include "string_uav_namespace_utils.hpp"

namespace {
geometry_msgs::PoseStamped build_pose_goal(const PlanningTarget& target,
                                           const std::string& frame_id) {
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = frame_id;
    goal_msg.pose.position.x = target.position.x();
    goal_msg.pose.position.y = target.position.y();
    goal_msg.pose.position.z = target.position.z();

    const double half_yaw = 0.5 * target.yaw;
    goal_msg.pose.orientation.x = 0.0;
    goal_msg.pose.orientation.y = 0.0;
    goal_msg.pose.orientation.z = std::sin(half_yaw);
    goal_msg.pose.orientation.w = std::cos(half_yaw);
    return goal_msg;
}

bool is_invalid_flag(const uint8_t trajectory_flag) {
    return trajectory_flag == sunray_planner_msgs::EgoPositionCommand::TRAJECTORY_STATUS_EMPTY ||
           trajectory_flag == sunray_planner_msgs::EgoPositionCommand::TRAJECTROY_STATUS_ABORT ||
           trajectory_flag == sunray_planner_msgs::EgoPositionCommand::TRAJECTORY_STATUS_ILLEGAL_START ||
           trajectory_flag == sunray_planner_msgs::EgoPositionCommand::TRAJECTORY_STATUS_ILLEGAL_FINAL ||
           trajectory_flag == sunray_planner_msgs::EgoPositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE;
}
}  // namespace

void EgoPlanner::init(ros::NodeHandle& private_nh) {
    std::string uav_name;
    int uav_id = 0;

    if (!private_nh.getParam("/uav_name", uav_name) || uav_name.empty()) {
        throw std::runtime_error("EgoPlanner: missing or empty /uav_name");
    }
    if (!private_nh.getParam("/uav_id", uav_id) || uav_id <= 0) {
        throw std::runtime_error("EgoPlanner: missing or invalid /uav_id");
    }

    uav_ns_ = sunray_common::normalize_uav_ns(uav_name + std::to_string(uav_id));

    const std::string goal_topic = uav_ns_ + "/sunray/planning/ego_planner/target_point";
    const std::string position_cmd_topic = uav_ns_ + "/sunray/planning/ego_planner/position_cmd";
    const std::string planner_state_topic = uav_ns_ + "/sunray/planning/ego_planner/state";

    goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);
    position_cmd_sub_ =
        private_nh.subscribe(position_cmd_topic, 10, &EgoPlanner::position_cmd_callback, this);
    unified_state_sub_ =
        private_nh.subscribe(planner_state_topic, 10, &EgoPlanner::unified_state_callback, this);

    snapshot_.planner_type = PlannerType::EGO;
    snapshot_.planner_state = PlannerExecState::WAIT_TARGET;
    snapshot_.planner_state_string = planner_exec_state_to_string(snapshot_.planner_state);
    snapshot_.ready = true;
    snapshot_.goal_active = false;
    snapshot_.has_valid_output = false;
    snapshot_.current_waypoint_index = 0;
    snapshot_.last_state_stamp = ros::Time::now();

    ready_ = true;
}

bool EgoPlanner::send_goal(const PlanningTarget& target) {
    goal_pub_.publish(build_pose_goal(target, "world"));

    const ros::Time stamp = ros::Time::now();
    snapshot_.goal_active = true;
    snapshot_.has_valid_output = false;
    snapshot_.current_waypoint_index = 0;
    snapshot_.last_goal_stamp = stamp;
    snapshot_.last_output_stamp = ros::Time();
    snapshot_.planner_state = PlannerExecState::GENERATE;
    snapshot_.planner_state_string = planner_exec_state_to_string(snapshot_.planner_state);
    snapshot_.last_state_stamp = stamp;
    latest_cmd_ = PlannerPositionCommand{};

    return true;
}

bool EgoPlanner::get_planner_positioncmd(PlannerPositionCommand& cmd) {
    const ros::Time now = ros::Time::now();
    if (!ready_ || !snapshot_.has_valid_output || snapshot_.last_output_stamp.isZero()) {
        return false;
    }
    if (cmd_timeout_sec_ > 0.0 && (now - snapshot_.last_output_stamp).toSec() > cmd_timeout_sec_) {
        return false;
    }
    cmd = latest_cmd_;
    if (cmd.stamp.isZero()) {
        cmd.stamp = snapshot_.last_output_stamp;
    }
    return true;
}

bool EgoPlanner::is_ready() const { return ready_; }

PlannerSnapshot EgoPlanner::get_planner_state() const {
    PlannerSnapshot snapshot = snapshot_;
    const ros::Time now = ros::Time::now();

    if (state_timeout_sec_ > 0.0 && !snapshot.last_state_stamp.isZero() &&
        (now - snapshot.last_state_stamp).toSec() > state_timeout_sec_) {
        snapshot.planner_state = PlannerExecState::UNDEFINE;
        snapshot.planner_state_string = "STALE";
    }

    if (cmd_timeout_sec_ > 0.0 && !snapshot.last_output_stamp.isZero() &&
        (now - snapshot.last_output_stamp).toSec() > cmd_timeout_sec_) {
        snapshot.has_valid_output = false;
    }

    return snapshot;
}

void EgoPlanner::position_cmd_callback(const sunray_planner_msgs::EgoPositionCommand::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

    if (is_invalid_flag(msg->trajectory_flag)) {
        snapshot_.planner_state = PlannerExecState::FAIL;
        snapshot_.planner_state_string = planner_exec_state_to_string(snapshot_.planner_state);
        snapshot_.last_state_stamp = stamp;
        snapshot_.goal_active = false;
        snapshot_.has_valid_output = false;
        return;
    }

    latest_cmd_.stamp = stamp;
    latest_cmd_.position = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    latest_cmd_.velocity = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    latest_cmd_.acceleration =
        Eigen::Vector3d(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
    latest_cmd_.jerk = Eigen::Vector3d::Zero();
    latest_cmd_.yaw = msg->yaw;
    latest_cmd_.yaw_rate = msg->yaw_dot;

    snapshot_.has_valid_output = true;
    snapshot_.last_output_stamp = stamp;

    const bool is_success =
        msg->trajectory_flag == sunray_planner_msgs::EgoPositionCommand::TRAJECTORY_STATUS_COMPLETED;
    snapshot_.planner_state = is_success ? PlannerExecState::SUCCESS : PlannerExecState::EXEC;
    snapshot_.planner_state_string = planner_exec_state_to_string(snapshot_.planner_state);
    snapshot_.last_state_stamp = stamp;
    snapshot_.goal_active = !is_success;
}

void EgoPlanner::unified_state_callback(const sunray_msgs::UAVPlanningState::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    if (msg->planner_type != 0 && msg->planner_type != static_cast<uint8_t>(PlannerType::EGO)) {
        return;
    }

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
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
