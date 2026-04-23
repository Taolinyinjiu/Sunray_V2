/*
本程序功能：
    1、定义 GoalDispatcher 类，封装 ORCA 目标点的下发逻辑
    2、将编队策略计算出的目标 Pose 打包为 sunray_msgs::OrcaSetup 消息
    3、发布到 /{agent_name}{id}/orca/setup 话题，供本地和其他 agent 的 ORCA 引擎使用
*/
#pragma once

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sunray_msgs/OrcaSetup.h>

namespace agent_swarm
{

// 目标点下发器
class GoalDispatcher
{
  public:
    // 初始化发布器
    void init(ros::NodeHandle &nh, const std::string &agent_name, int agent_id);
    // 发布目标点
    void publishGoal(const geometry_msgs::Pose &target_pose, bool run_mode);

  private:
    ros::Publisher pub_{};
};

} // namespace agent_swarm
