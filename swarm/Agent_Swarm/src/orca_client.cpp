// 中文说明：ORCA 客户端实现，缓存 ORCA 输出
#include "orca_client.h"

namespace agent_swarm
{

void OrcaClient::init(ros::NodeHandle &nh, const std::string &agent_name, int agent_id)
{
    std::string topic = "/" + agent_name + std::to_string(agent_id) + "/orca_cmd";
    sub_ = nh.subscribe<sunray_msgs::OrcaCmd>(topic, 10, &OrcaClient::cmdCallback, this);
}

void OrcaClient::cmdCallback(const sunray_msgs::OrcaCmd::ConstPtr &msg)
{
    last_cmd_ = *msg;
    last_stamp_ = msg->header.stamp;
    has_cmd_ = true;
}

bool OrcaClient::isFresh(double timeout_sec) const
{
    if (!has_cmd_)
    {
        return false;
    }
    return (ros::Time::now() - last_stamp_).toSec() <= timeout_sec;
}

bool OrcaClient::getLastCmd(sunray_msgs::OrcaCmd &out) const
{
    if (!has_cmd_)
    {
        return false;
    }
    out = last_cmd_;
    return true;
}

} // namespace agent_swarm
