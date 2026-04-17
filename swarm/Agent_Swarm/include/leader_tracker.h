// 中文说明：Leader 追踪接口，统一输出 Leader 位姿
#pragma once

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace agent_swarm
{

// Leader 追踪器
class LeaderTracker
{
  public:
    // 初始化订阅
    void init(ros::NodeHandle &nh, int leader_id, const std::string &agent_name);
    // 获取 Leader 位姿
    bool getLeaderPose(geometry_msgs::Pose &pose_out) const;
    // 判断数据是否超时
    bool isFresh(double timeout_sec) const;

  private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    geometry_msgs::Pose leader_pose_{};
    ros::Time last_stamp_{};
    bool has_pose_{false};
    ros::Subscriber sub_{};
};

} // namespace agent_swarm
