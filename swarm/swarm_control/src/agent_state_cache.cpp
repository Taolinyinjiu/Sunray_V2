/*
本程序功能：
    1、实现 AgentStateCache 的初始化，根据 agent_num 批量订阅所有 agent 的 local_odom 话题
    2、odomCallback 收到里程计时加锁写入缓存，保证线程安全
*/
#include "agent_state_cache.h"
#include <boost/bind.hpp>

namespace swarm_control
{

void AgentStateCache::init(ros::NodeHandle &nh, int agent_num, int agent_type, const std::string &agent_name)
{
    agent_num_ = agent_num;
    agent_type_ = agent_type;
    agent_name_ = agent_name;

    for (int i = 0; i < agent_num_; ++i)
    {
        int id = i + 1;
        std::string topic = "/" + agent_name_ + std::to_string(id) + "/sunray/localization/local_odom";
        subs_[id] = nh.subscribe<nav_msgs::Odometry>(topic, 10, boost::bind(&AgentStateCache::odomCallback, this, _1, id));
    }
}

void AgentStateCache::odomCallback(const nav_msgs::Odometry::ConstPtr &msg, int idx)
{
    std::lock_guard<std::mutex> lock(mutex_);
    agent_state_[idx] = *msg;
}

} // namespace swarm_control
