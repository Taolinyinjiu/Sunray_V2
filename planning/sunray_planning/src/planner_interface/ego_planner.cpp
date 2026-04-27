#include "planner_interface/ego_planner.hpp"

#include <cmath>

namespace {
geometry_msgs::PoseStamped build_pose_goal(const PlanningTarget& target,
                                           const PlannerRuntimeConfig& config) {
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = config.goal_frame_id;
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
    return trajectory_flag == ego_planner_msgs::PositionCommand::TRAJECTORY_STATUS_EMPTY ||
           trajectory_flag == ego_planner_msgs::PositionCommand::TRAJECTROY_STATUS_ABORT ||
           trajectory_flag == ego_planner_msgs::PositionCommand::TRAJECTORY_STATUS_ILLEGAL_START ||
           trajectory_flag == ego_planner_msgs::PositionCommand::TRAJECTORY_STATUS_ILLEGAL_FINAL ||
           trajectory_flag == ego_planner_msgs::PositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE;
}
}  // namespace

void EgoPlanner::bind_topics(ros::NodeHandle& nh) {
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(config_.goal_topic, 1);
    position_cmd_sub_ = nh.subscribe(
        config_.position_cmd_topic, 10, &EgoPlanner::position_cmd_callback, this);
}

bool EgoPlanner::send_goal(const PlanningTarget& target) {
    goal_pub_.publish(build_pose_goal(target, config_));
    mark_goal_sent(ros::Time::now());
    return true;
}

void EgoPlanner::position_cmd_callback(const ego_planner_msgs::PositionCommand::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    if (is_invalid_flag(msg->trajectory_flag)) {
        set_planner_state(PlannerExecState::FAIL, stamp);
        return;
    }

    sunray_msgs::UAVControlCMD control_cmd;
    control_cmd.header = msg->header;
    control_cmd.header.stamp = stamp;
    control_cmd.cmd_source = sunray_msgs::UAVControlCMD::CONTROL_CMD;
    control_cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY;
    control_cmd.desired_pos.x = msg->position.x;
    control_cmd.desired_pos.y = msg->position.y;
    control_cmd.desired_pos.z = msg->position.z;
    control_cmd.desired_vel = msg->velocity;
    control_cmd.desired_acc = msg->acceleration;
    control_cmd.desired_jerk.x = 0.0;
    control_cmd.desired_jerk.y = 0.0;
    control_cmd.desired_jerk.z = 0.0;
    control_cmd.desired_yaw = msg->yaw;
    control_cmd.desired_yaw_rate = 0.0;
    // MOVE_TRAJECTORY 当前在 uav_control 链路中无法同时无损表达 yaw 和 yaw_rate。
    // 这里优先保留绝对 yaw，让下游按规划轨迹给出的朝向跟踪。
    control_cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
    control_cmd.fixed_height = 0.0;

    set_latest_control_cmd(control_cmd);
    set_planner_state(msg->trajectory_flag == ego_planner_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED
                          ? PlannerExecState::SUCCESS
                          : PlannerExecState::EXEC,
                      stamp);
}
