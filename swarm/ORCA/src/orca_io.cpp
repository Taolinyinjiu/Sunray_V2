// 中文说明：ORCA 通信实现，收发消息与可视化
#include "orca_io.h"
#include <boost/bind.hpp>
#include <tf/transform_datatypes.h>

namespace orca_swarm
{

void OrcaIO::init(ros::NodeHandle &nh, int agent_id, int agent_num, const std::string &agent_name, OrcaEngine *engine)
{
    agent_id_ = agent_id;
    agent_num_ = agent_num;
    agent_name_ = agent_name;
    engine_ = engine;

    std::string prefix = "/" + agent_name_ + std::to_string(agent_id_);
    cmd_pub_ = nh.advertise<sunray_msgs::OrcaCmd>(prefix + "/orca_cmd", 10);
    goal_pub_ = nh.advertise<visualization_msgs::Marker>(prefix + "/orca/goal", 10);
    fence_pub_ = nh.advertise<visualization_msgs::MarkerArray>(prefix + "/orca/geo_fence", 10);

    for (int i = 0; i < agent_num_; ++i)
    {
        int id = i + 1;
        std::string agent_prefix = "/" + agent_name_ + std::to_string(id);
        state_subs_[id] = nh.subscribe<nav_msgs::Odometry>(agent_prefix + "/orca/agent_state", 10,
                                                           boost::bind(&OrcaIO::agentStateCb, this, _1, i));
        setup_subs_[id] = nh.subscribe<sunray_msgs::OrcaSetup>(agent_prefix + "/orca/setup", 10,
                                                               boost::bind(&OrcaIO::setupCb, this, _1, i));
    }
}

void OrcaIO::agentStateCb(const nav_msgs::Odometry::ConstPtr &msg, int idx)
{
    if (engine_)
    {
        engine_->updateAgentState(idx, *msg);
    }
}

void OrcaIO::setupCb(const sunray_msgs::OrcaSetup::ConstPtr &msg, int idx)
{
    if (engine_)
    {
        engine_->handleSetup(idx, *msg);
    }
    // 仅对本机目标可视化
    if (idx == agent_id_ - 1 &&
        (msg->cmd == sunray_msgs::OrcaSetup::GOAL || msg->cmd == sunray_msgs::OrcaSetup::GOAL_RUN))
    {
        visualization_msgs::Marker goal_marker;
        goal_marker.header.frame_id = "world";
        goal_marker.header.stamp = ros::Time::now();
        goal_marker.ns = "goal";
        goal_marker.id = agent_id_;
        goal_marker.type = visualization_msgs::Marker::SPHERE;
        goal_marker.action = visualization_msgs::Marker::ADD;
        goal_marker.pose.position.x = msg->desired_pos[0];
        goal_marker.pose.position.y = msg->desired_pos[1];
        goal_marker.pose.position.z = msg->desired_pos[2];
        goal_marker.pose.orientation = tf::createQuaternionMsgFromYaw(msg->desired_yaw);
        goal_marker.scale.x = 0.2;
        goal_marker.scale.y = 0.2;
        goal_marker.scale.z = 0.2;
        goal_marker.color.a = 1.0;
        goal_marker.color.r = static_cast<float>(((agent_id_)*123) % 256) / 255.0f;
        goal_marker.color.g = static_cast<float>(((agent_id_)*456) % 256) / 255.0f;
        goal_marker.color.b = static_cast<float>(((agent_id_)*789) % 256) / 255.0f;
        goal_pub_.publish(goal_marker);
    }
}

void OrcaIO::publishCmd(const sunray_msgs::OrcaCmd &cmd)
{
    cmd_pub_.publish(cmd);
}

void OrcaIO::publishFenceMarkers(const visualization_msgs::MarkerArray &markers)
{
    fence_pub_.publish(markers);
}

} // namespace orca_swarm
