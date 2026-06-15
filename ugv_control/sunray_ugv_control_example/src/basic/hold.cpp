/*
本程序功能：
    1、演示如何向 sunray_ugv_control 发布一次 HOLD 停车命令
    2、适合第一次检查 UGV 控制命令话题是否连通

命令发布规则：
    HOLD 是事件型命令，发布一次即可。
    sunray_ugv_control 收到 HOLD 后，会持续向 /cmd_vel 发布零速度。
*/

#include <ros/ros.h>
#include <sunray_msgs/UGVControlCMD.h>

#include <string>

namespace
{
std::string makeAgentPrefix(const std::string &agent_name, const int agent_id)
{
    return "/" + agent_name + std::to_string(agent_id);
}
}  // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_hold_example_node");
    ros::NodeHandle nh("~");

    std::string agent_name = "ugv";
    int agent_id = 1;
    nh.param("agent_name", agent_name, std::string("ugv"));
    nh.param("agent_id", agent_id, 1);
    if (agent_id < 1)
    {
        ROS_WARN("agent_id=%d is invalid, fallback to 1.", agent_id);
        agent_id = 1;
    }

    const std::string agent_prefix = makeAgentPrefix(agent_name, agent_id);
    const std::string cmd_topic = agent_prefix + "/sunray/ugv_control/control_cmd";

    ros::Publisher cmd_pub = nh.advertise<sunray_msgs::UGVControlCMD>(cmd_topic, 10);
    ros::Duration(0.5).sleep();

    sunray_msgs::UGVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd_source = sunray_msgs::UGVControlCMD::EXAMPLE_DEMO;
    cmd.control_cmd = sunray_msgs::UGVControlCMD::HOLD;

    cmd_pub.publish(cmd);
    ros::spinOnce();

    ROS_INFO("UGV HOLD example published one HOLD command to %s", cmd_topic.c_str());
    return 0;
}
