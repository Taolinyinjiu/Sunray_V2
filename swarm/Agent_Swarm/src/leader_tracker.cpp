// 中文说明：Leader 追踪实现，订阅并缓存 Leader 位姿
#include "leader_tracker.h"

namespace agent_swarm
{

void LeaderTracker::init(ros::NodeHandle &nh, int leader_id, const std::string &agent_name)
{
    std::string topic;
    if (leader_id > 100)
    {
        int ugv_id = leader_id - 100;
        topic = "/ugv" + std::to_string(ugv_id) + "/sunray/localization/local_odom";
    }
    else
    {
        topic = "/" + agent_name + std::to_string(leader_id) + "/sunray/localization/local_odom";
    }
    sub_ = nh.subscribe<nav_msgs::Odometry>(topic, 10, &LeaderTracker::odomCallback, this);
}

void LeaderTracker::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    leader_pose_ = msg->pose.pose;
    last_stamp_ = msg->header.stamp;
    has_pose_ = true;
}

bool LeaderTracker::getLeaderPose(geometry_msgs::Pose &pose_out) const
{
    if (!has_pose_)
    {
        return false;
    }
    pose_out = leader_pose_;
    return true;
}

bool LeaderTracker::isFresh(double timeout_sec) const
{
    if (!has_pose_)
    {
        return false;
    }
    return (ros::Time::now() - last_stamp_).toSec() <= timeout_sec;
}

} // namespace agent_swarm
