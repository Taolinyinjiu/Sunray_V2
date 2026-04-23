/*
本程序功能：
    1、定义 LeaderTracker 类，订阅 Leader 的里程计话题并缓存其最新位姿
    2、支持异构编队：leader_id > 100 时自动订阅 UGV 话题前缀（/ugv{id}/...）
    3、提供线程安全的 getLeaderPose() 获取 Leader 位姿
    4、提供 isFresh() 判断 Leader 数据是否在指定超时范围内，用于状态机安全兜底
*/
#pragma once

#include <geometry_msgs/Pose.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace swarm_control
{

// Leader 追踪器
class LeaderTracker
{
  public:
    // 初始化订阅
    void init(ros::NodeHandle &nh, int leader_id, const std::string &agent_name);
    // 获取 Leader 位姿（线程安全）
    bool getLeaderPose(geometry_msgs::Pose &pose_out) const;
    // 判断数据是否超时（线程安全）
    bool isFresh(double timeout_sec) const;

  private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    mutable std::mutex mutex_{};
    geometry_msgs::Pose leader_pose_{};
    ros::Time last_stamp_{};
    bool has_pose_{false};
    ros::Subscriber sub_{};
};

} // namespace swarm_control
