/*
本程序功能：
    1、实现 ControlCommandMapper 的初始化，创建 uav_control_cmd 发布器
    2、实现 publishHover：发布悬停指令（HOVER + KEEP_YAW）
    3、实现 publishTakeoff/publishLand：发布起飞/降落指令（仅 UAV 有效）
    4、实现 publishPosTarget：发布位置控制指令（MOVE_POINT）
    5、实现 publishFromOrca：根据 ORCA 输出状态选择速度/位置控制，含 vz P 控制器
*/
#include "control_command_mapper.h"
#include <algorithm>
#include <tf/transform_datatypes.h>

namespace agent_swarm
{
namespace
{

double clampAbs(double value, double limit)
{
    return std::max(-limit, std::min(limit, value));
}

} // namespace

void ControlCommandMapper::init(ros::NodeHandle &nh, const std::string &agent_name, int agent_id, int agent_type)
{
    agent_type_ = agent_type;
    std::string prefix = "/" + agent_name + std::to_string(agent_id);
    uav_pub_ = nh.advertise<sunray_msgs::UAVControlCMD>(prefix + "/sunray/uav_control_cmd", 10);
}

sunray_msgs::UAVControlCMD ControlCommandMapper::makeBaseCmd() const
{
    sunray_msgs::UAVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd_source = sunray_msgs::UAVControlCMD::CONTROL_CMD;
    cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
    cmd.fixed_height = false;
    return cmd;
}

void ControlCommandMapper::publishHover()
{
    if (agent_type_ != 0)
    {
        ROS_WARN_THROTTLE(2.0, "UGV output is not wired to a stable control interface yet.");
        return;
    }
    auto cmd = makeBaseCmd();
    cmd.control_cmd = sunray_msgs::UAVControlCMD::HOVER;
    cmd.yaw_mode = sunray_msgs::UAVControlCMD::KEEP_YAW;
    uav_pub_.publish(cmd);
}

void ControlCommandMapper::publishTakeoff(double altitude)
{
    if (agent_type_ != 0)
    {
        ROS_WARN_THROTTLE(2.0, "UGV takeoff command ignored.");
        return;
    }
    auto cmd = makeBaseCmd();
    cmd.control_cmd = sunray_msgs::UAVControlCMD::TAKEOFF;
    cmd.desired_pos.z = altitude;
    uav_pub_.publish(cmd);
}

void ControlCommandMapper::publishLand()
{
    if (agent_type_ != 0)
    {
        ROS_WARN_THROTTLE(2.0, "UGV land command ignored.");
        return;
    }
    auto cmd = makeBaseCmd();
    cmd.control_cmd = sunray_msgs::UAVControlCMD::LAND;
    cmd.yaw_mode = sunray_msgs::UAVControlCMD::KEEP_YAW;
    uav_pub_.publish(cmd);
}

void ControlCommandMapper::publishPosTarget(const geometry_msgs::Pose &target_pose)
{
    if (agent_type_ != 0)
    {
        ROS_WARN_THROTTLE(2.0, "UGV position command ignored.");
        return;
    }
    auto cmd = makeBaseCmd();
    cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_POINT;
    cmd.desired_pos.x = target_pose.position.x;
    cmd.desired_pos.y = target_pose.position.y;
    cmd.desired_pos.z = target_pose.position.z;
    cmd.desired_yaw = tf::getYaw(target_pose.orientation);
    uav_pub_.publish(cmd);
}

void ControlCommandMapper::publishFromOrca(const sunray_msgs::OrcaCmd &orca, double current_z)
{
    if (agent_type_ != 0)
    {
        ROS_WARN_THROTTLE(2.0, "UGV ORCA output is not wired to a stable control interface yet.");
        return;
    }

    if (orca.state == sunray_msgs::OrcaCmd::STOP)
    {
        publishHover();
        return;
    }

    if (orca.state == sunray_msgs::OrcaCmd::ARRIVED)
    {
        const double xy_speed =
            std::hypot(static_cast<double>(orca.linear[0]), static_cast<double>(orca.linear[1]));
        if (xy_speed > 0.05)
        {
            auto cmd = makeBaseCmd();
            cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_VELOCITY;
            cmd.desired_vel.x = orca.linear[0];
            cmd.desired_vel.y = orca.linear[1];
            cmd.desired_vel.z = clampAbs(1.5 * (static_cast<double>(orca.goal_pos[2]) - current_z), 0.5);
            cmd.desired_yaw = orca.goal_yaw;
            uav_pub_.publish(cmd);
            return;
        }

        geometry_msgs::Pose target_pose;
        target_pose.position.x = orca.goal_pos[0];
        target_pose.position.y = orca.goal_pos[1];
        target_pose.position.z = orca.goal_pos[2];
        target_pose.orientation = tf::createQuaternionMsgFromYaw(orca.goal_yaw);
        publishPosTarget(target_pose);
        return;
    }

    auto cmd = makeBaseCmd();
    cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_VELOCITY;
    cmd.desired_vel.x = orca.linear[0];
    cmd.desired_vel.y = orca.linear[1];
    const double z_vel = clampAbs(1.5 * (static_cast<double>(orca.goal_pos[2]) - current_z), 0.5);
    cmd.desired_vel.z = z_vel;
    cmd.desired_yaw = orca.goal_yaw;
    uav_pub_.publish(cmd);
}

} // namespace agent_swarm
