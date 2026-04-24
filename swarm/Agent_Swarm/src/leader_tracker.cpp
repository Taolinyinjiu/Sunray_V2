/*
本程序功能：
    1、实现 LeaderTracker 的初始化，根据 leader_id 自动选择 UAV/UGV 话题前缀并订阅
    2、odomCallback 收到 Leader 里程计时加锁更新缓存位姿和时间戳
    3、getLeaderPose 加锁读取 Leader 最新位姿
    4、isFresh 加锁判断 Leader 数据是否在超时范围内
*/
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
    std::lock_guard<std::mutex> lock(mutex_);
    leader_pose_ = msg->pose.pose;
    last_stamp_ = msg->header.stamp;
    has_pose_ = true;
}

bool LeaderTracker::getLeaderPose(geometry_msgs::Pose &pose_out) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_pose_)
    {
        return false;
    }
    pose_out = leader_pose_;
    return true;
}

bool LeaderTracker::isFresh(double timeout_sec) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_pose_)
    {
        return false;
    }
    return (ros::Time::now() - last_stamp_).toSec() <= timeout_sec;
}

} // namespace agent_swarm
