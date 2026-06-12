/*
本程序功能：
    1、演示如何向 sunray_ugv_control 持续发布 MOVE_VELOCITY_BODY 车体系速度命令
    2、演示速度类命令必须持续发布，并且每帧都要填写 header.stamp
    3、命令结束后自动发布 HOLD，避免车辆继续保留旧速度

命令发布规则：
    MOVE_VELOCITY_BODY 是连续型命令，需要按照固定频率持续发布。
    如果发布中断，sunray_ugv_control 会在 wait_velcmd_time 后切换 HOLD。

底盘差异：
    麦克纳姆轮可以使用 cmd_vel.linear.y 横向速度。
    差速轮只使用 cmd_vel.linear.x 和 cmd_vel.angular.z，linear.y 会被控制器忽略并在诊断信息中告警。
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

sunray_msgs::UGVControlCMD makeBaseCmd()
{
    sunray_msgs::UGVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd_source = sunray_msgs::UGVControlCMD::EXAMPLE_DEMO;
    return cmd;
}
}  // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_move_velocity_body_example_node");
    ros::NodeHandle nh("~");

    std::string agent_name = "ugv";
    int agent_id = 1;
    double vx = 0.2;
    double vy = 0.0;
    double wz_deg_s = 0.0;
    double duration = 3.0;
    double publish_hz = 20.0;

    nh.param("agent_name", agent_name, std::string("ugv"));
    nh.param("agent_id", agent_id, 1);
    nh.param("vx", vx, 0.2);
    nh.param("vy", vy, 0.0);
    nh.param("wz_deg_s", wz_deg_s, 0.0);
    nh.param("duration", duration, 3.0);
    nh.param("publish_hz", publish_hz, 20.0);
    if (agent_id < 1)
    {
        ROS_WARN("agent_id=%d is invalid, fallback to 1.", agent_id);
        agent_id = 1;
    }
    if (duration <= 0.0)
    {
        ROS_WARN("duration=%.2f is invalid, fallback to 3.0s.", duration);
        duration = 3.0;
    }
    if (publish_hz <= 0.0)
    {
        ROS_WARN("publish_hz=%.2f is invalid, fallback to 20Hz.", publish_hz);
        publish_hz = 20.0;
    }

    const std::string agent_prefix = makeAgentPrefix(agent_name, agent_id);
    const std::string cmd_topic = agent_prefix + "/sunray/ugv_control/control_cmd";

    ros::Publisher cmd_pub = nh.advertise<sunray_msgs::UGVControlCMD>(cmd_topic, 10);
    ros::Duration(0.5).sleep();

    ROS_INFO("UGV MOVE_VELOCITY_BODY example starts: topic=%s vx=%.2f vy=%.2f wz=%.2f deg/s duration=%.2f publish_hz=%.1f",
             cmd_topic.c_str(),
             vx,
             vy,
             wz_deg_s,
             duration,
             publish_hz);

    ros::Rate rate(publish_hz);
    const ros::Time end_time = ros::Time::now() + ros::Duration(duration);
    while (ros::ok() && ros::Time::now() < end_time)
    {
        sunray_msgs::UGVControlCMD cmd = makeBaseCmd();
        cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY;
        cmd.cmd_vel.linear.x = vx;
        cmd.cmd_vel.linear.y = vy;
        cmd.cmd_vel.linear.z = 0.0;
        cmd.cmd_vel.angular.x = 0.0;
        cmd.cmd_vel.angular.y = 0.0;
        cmd.cmd_vel.angular.z = degToRad(wz_deg_s);
        cmd_pub.publish(cmd);

        ros::spinOnce();
        rate.sleep();
    }

    sunray_msgs::UGVControlCMD hold_cmd = makeBaseCmd();
    hold_cmd.control_cmd = sunray_msgs::UGVControlCMD::HOLD;
    for (int i = 0; i < 5 && ros::ok(); ++i)
    {
        hold_cmd.header.stamp = ros::Time::now();
        cmd_pub.publish(hold_cmd);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("UGV MOVE_VELOCITY_BODY example finished, HOLD command sent.");
    return 0;
}
