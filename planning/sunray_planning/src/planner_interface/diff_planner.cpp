#include "planner_interface/diff_planner.hpp"

#include <cmath>

namespace {
geometry_msgs::PoseStamped build_pose_goal(const PlanningTarget& target,
                                           const std::string& goal_frame_id) {
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = goal_frame_id;
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
    return trajectory_flag == sunray_planner_msgs::DiffPositionCommand::TRAJECTORY_STATUS_EMPTY ||
           trajectory_flag == sunray_planner_msgs::DiffPositionCommand::TRAJECTROY_STATUS_ABORT ||
           trajectory_flag == sunray_planner_msgs::DiffPositionCommand::TRAJECTORY_STATUS_ILLEGAL_START ||
           trajectory_flag == sunray_planner_msgs::DiffPositionCommand::TRAJECTORY_STATUS_ILLEGAL_FINAL ||
           trajectory_flag == sunray_planner_msgs::DiffPositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE;
}
}  // namespace

PlannerType DiffPlanner::planner_type() const { return PlannerType::DIFF; }

std::string DiffPlanner::default_goal_topic(const std::string& uav_ns) const {
    return uav_ns + "/sunray/planning/diff_planner/target_point";
}

std::string DiffPlanner::default_goal_frame_id() const { return "world"; }

std::string DiffPlanner::default_position_cmd_topic(const std::string& uav_ns) const {
    return uav_ns + "/sunray/planning/diff_planner/position_cmd";
}

void DiffPlanner::bind_topics(ros::NodeHandle& nh) {
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1);
    position_cmd_sub_ =
        nh.subscribe(position_cmd_topic_, 10, &DiffPlanner::position_cmd_callback, this);
}

bool DiffPlanner::send_goal(const PlanningTarget& target) {
    goal_pub_.publish(build_pose_goal(target, goal_frame_id_));
    mark_goal_sent(ros::Time::now());
    return true;
}

void DiffPlanner::position_cmd_callback(const sunray_planner_msgs::DiffPositionCommand::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    if (is_invalid_flag(msg->trajectory_flag)) {
        set_planner_state(PlannerExecState::FAIL, stamp);
        return;
    }

    PlannerPositionCommand planner_cmd;
    planner_cmd.stamp = stamp;
    planner_cmd.position = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    planner_cmd.velocity = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    planner_cmd.acceleration =
        Eigen::Vector3d(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
    planner_cmd.jerk = Eigen::Vector3d(msg->jerk.x, msg->jerk.y, msg->jerk.z);
    planner_cmd.yaw = msg->yaw;
    planner_cmd.yaw_rate = msg->yaw_dot;

    set_latest_position_cmd(planner_cmd);
    set_planner_state(msg->trajectory_flag == sunray_planner_msgs::DiffPositionCommand::TRAJECTORY_STATUS_COMPLETED
                          ? PlannerExecState::SUCCESS
                          : PlannerExecState::EXEC,
                      stamp);
}
