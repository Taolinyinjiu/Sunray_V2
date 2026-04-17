// 中文说明：目标点下发接口，封装 OrcaSetup
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
