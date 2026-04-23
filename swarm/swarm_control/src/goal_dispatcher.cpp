/*
本程序功能：
    1、实现 GoalDispatcher 的初始化，创建 /{agent_name}{id}/orca/setup 发布器
    2、实现 publishGoal，将目标 Pose 打包为 OrcaSetup 消息（含位置和偏航角）
    3、根据 run_mode 选择发布 GOAL（仅设目标）或 GOAL_RUN（设目标并启动）
*/
#include "goal_dispatcher.h"
#include <tf/transform_datatypes.h>

namespace swarm_control
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

} // namespace swarm_control
