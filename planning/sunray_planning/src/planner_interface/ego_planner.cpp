#include "planner_interface/ego_planner.hpp"

#include <cmath>
#include <stdexcept>

#include "agent_key_helper.hpp"

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
    bool use_private_agent_key = false;
    private_nh.param("use_private_agent_key", use_private_agent_key, false);
    uav_ns_ = use_private_agent_key ? sunray_common::get_agent_key_from_private()
                                    : sunray_common::get_agent_key_from_global();

    const std::string goal_topic = uav_ns_ + "/sunray/planning/ego_planner/target_point";
    const std::string position_cmd_topic = uav_ns_ + "/sunray/planning/ego_planner/position_cmd";

    goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);
    position_cmd_sub_ =
        private_nh.subscribe(position_cmd_topic, 10, &EgoPlanner::position_cmd_callback, this);

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
