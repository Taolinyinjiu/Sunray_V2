// 中文说明：状态缓存实现，订阅并缓存全部 agent odom
#include "agent_state_cache.h"
#include <boost/bind.hpp>

namespace agent_swarm
{

void AgentStateCache::init(ros::NodeHandle &nh, int agent_num, int agent_type, const std::string &agent_name,
                           int leader_id)
{
    agent_num_ = agent_num;
    agent_type_ = agent_type;
    agent_name_ = agent_name;
    leader_id_ = leader_id;

    int leader_index = (leader_id > 100) ? (leader_id - 100) : leader_id;
    bool leader_is_ugv = (leader_id > 100);

    for (int i = 0; i < agent_num_; ++i)
    {
        int id = i + 1;
        if (leader_is_ugv && id == leader_index)
        {
            std::string topic = "/ugv" + std::to_string(id) + "/sunray/localization/local_odom";
            subs_[id] = nh.subscribe<nav_msgs::Odometry>(topic, 10, boost::bind(&AgentStateCache::odomCallback, this, _1, id));
        }
        else
        {
            std::string topic = "/" + agent_name_ + std::to_string(id) + "/sunray/localization/local_odom";
            subs_[id] = nh.subscribe<nav_msgs::Odometry>(topic, 10, boost::bind(&AgentStateCache::odomCallback, this, _1, id));
        }
    }
}

void AgentStateCache::odomCallback(const nav_msgs::Odometry::ConstPtr &msg, int idx)
{
    agent_state_[idx] = *msg;
}

} // namespace agent_swarm
