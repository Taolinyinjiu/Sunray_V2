// 中文说明：目标点下发实现，发布 /orca/setup
#include "goal_dispatcher.h"
#include <tf/transform_datatypes.h>

namespace agent_swarm
{

void GoalDispatcher::init(ros::NodeHandle &nh, const std::string &agent_name, int agent_id)
{
    std::string topic = "/" + agent_name + std::to_string(agent_id) + "/orca/setup";
    pub_ = nh.advertise<sunray_msgs::OrcaSetup>(topic, 10);
}

void GoalDispatcher::publishGoal(const geometry_msgs::Pose &target_pose, bool run_mode)
{
    sunray_msgs::OrcaSetup msg;
    msg.header.stamp = ros::Time::now();
    msg.cmd = run_mode ? sunray_msgs::OrcaSetup::GOAL_RUN : sunray_msgs::OrcaSetup::GOAL;
    msg.desired_pos[0] = target_pose.position.x;
    msg.desired_pos[1] = target_pose.position.y;
    msg.desired_pos[2] = target_pose.position.z;
    msg.desired_yaw = tf::getYaw(target_pose.orientation);
    pub_.publish(msg);
}

} // namespace agent_swarm
