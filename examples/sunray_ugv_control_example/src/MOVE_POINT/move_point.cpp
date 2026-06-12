/*
本程序功能：
    1、演示如何向 sunray_ugv_control 发布一次 MOVE_POINT 点位控制命令
    2、适合学习 UGVControlCMD 中 desired_pos 和 desired_yaw 的用法

命令发布规则：
    MOVE_POINT 是事件型命令，发布一次即可。
    sunray_ugv_control 会根据 local_odom 持续计算 cmd_vel，直到满足到点阈值后切换 HOLD。

坐标系约定：
    desired_pos.x/y 是本地世界坐标系目标点，单位 m。
    desired_pos.z 对 UGV 通常填 0。
    desired_yaw 是本地世界坐标系 yaw，单位 rad；launch 中用 deg 输入，代码里转成 rad。
*/

#include <ros/ros.h>
#include <sunray_msgs/UGVControlCMD.h>

#include <cmath>
#include <string>

namespace
{
double degToRad(const double deg)
{
    return deg * M_PI / 180.0;
}

std::string makeAgentPrefix(const std::string &agent_name, const int agent_id)
{
    return "/" + agent_name + std::to_string(agent_id);
}
}  // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_move_point_example_node");
    ros::NodeHandle nh("~");

    std::string agent_name = "ugv";
    int agent_id = 1;
    double target_x = 1.0;
    double target_y = 0.0;
    double target_yaw_deg = 0.0;

    nh.param("agent_name", agent_name, std::string("ugv"));
    nh.param("agent_id", agent_id, 1);
    nh.param("target_x", target_x, 1.0);
    nh.param("target_y", target_y, 0.0);
    nh.param("target_yaw_deg", target_yaw_deg, 0.0);
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
    cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_POINT;
    cmd.desired_pos.x = target_x;
    cmd.desired_pos.y = target_y;
    cmd.desired_pos.z = 0.0;
    cmd.desired_yaw = degToRad(target_yaw_deg);

    cmd_pub.publish(cmd);
    ros::spinOnce();

    ROS_INFO("UGV MOVE_POINT example published target to %s: x=%.2f y=%.2f yaw=%.2f deg",
             cmd_topic.c_str(),
             target_x,
             target_y,
             target_yaw_deg);
    return 0;
}
