// 中文说明：状态缓存接口，订阅并标准化多机状态
#pragma once

#include <map>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace agent_swarm
{

// 代理状态缓存
class AgentStateCache
{
  public:
    // 初始化订阅（leader_id 用于混合编队：leader_id>100 表示 UGV Leader）
    void init(ros::NodeHandle &nh, int agent_num, int agent_type, const std::string &agent_name, int leader_id = 1);
    // 获取当前缓存
    const std::map<int, nav_msgs::Odometry> &states() const
    {
        return agent_state_;
    }

  private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg, int idx);

    int agent_num_{1};
    int agent_type_{0};
    int leader_id_{1};
    std::string agent_name_{"uav"};
    std::map<int, nav_msgs::Odometry> agent_state_{};
    std::map<int, ros::Subscriber> subs_{};
};

} // namespace agent_swarm
