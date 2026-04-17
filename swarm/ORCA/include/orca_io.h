// 中文说明：ORCA 通信接口，ROS 话题适配
#pragma once

#include "orca_engine.h"
#include <map>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sunray_msgs/OrcaCmd.h>
#include <sunray_msgs/OrcaSetup.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace orca_swarm
{

// ORCA 通信适配层
class OrcaIO
{
  public:
    // 初始化订阅与发布
    void init(ros::NodeHandle &nh, int agent_id, int agent_num, const std::string &agent_name, OrcaEngine *engine);
    // 发布 ORCA 指令
    void publishCmd(const sunray_msgs::OrcaCmd &cmd);
    // 发布围栏可视化
    void publishFenceMarkers(const visualization_msgs::MarkerArray &markers);

  private:
    // agent_state 回调
    void agentStateCb(const nav_msgs::Odometry::ConstPtr &msg, int idx);
    // orca/setup 回调
    void setupCb(const sunray_msgs::OrcaSetup::ConstPtr &msg, int idx);

    int agent_id_{1};
    int agent_num_{1};
    std::string agent_name_{"uav"};
    OrcaEngine *engine_{nullptr};

    ros::Publisher cmd_pub_{};
    ros::Publisher goal_pub_{};
    ros::Publisher fence_pub_{};
    std::map<int, ros::Subscriber> state_subs_{};
    std::map<int, ros::Subscriber> setup_subs_{};
};

} // namespace orca_swarm
