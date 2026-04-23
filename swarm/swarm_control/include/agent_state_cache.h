/*
本程序功能：
    1、定义 AgentStateCache 类，用于缓存集群中所有 agent 的里程计状态
    2、根据 agent_num 自动订阅所有 agent 的 local_odom 话题
    3、支持异构编队：leader_id > 100 时自动订阅 UGV 话题前缀
    4、提供线程安全的 states() 接口，返回当前全体 agent 状态快照
    5、供 ORCA 引擎镜像仿真和 home 位姿缓存使用
*/
#pragma once

#include <map>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace swarm_control
{

// 代理状态缓存
class AgentStateCache
{
  public:
    // 初始化订阅（leader_id 用于混合编队：leader_id>100 表示 UGV Leader）
    void init(ros::NodeHandle &nh, int agent_num, int agent_type, const std::string &agent_name, int leader_id = 1);
    // 获取当前缓存的快照（线程安全）
    std::map<int, nav_msgs::Odometry> states() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return agent_state_;
    }

  private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg, int idx);

    int agent_num_{1};
    int agent_type_{0};
    int leader_id_{1};
    std::string agent_name_{"uav"};
    mutable std::mutex mutex_{};
    std::map<int, nav_msgs::Odometry> agent_state_{};
    std::map<int, ros::Subscriber> subs_{};
};

} // namespace swarm_control
