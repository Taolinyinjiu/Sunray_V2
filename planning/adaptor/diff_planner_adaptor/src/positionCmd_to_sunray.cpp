/**
 * @file positionCmd_to_sunray.cpp
 * @brief 将Diff_Planner输出的Position命令,从 diff_planner_msgs::PositionCommand 转换为
 * sunray_msgs::UAVControlCMD并发布到Sunray_FSM订阅的话题
 */

#include <ros/ros.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <diff_planner_msgs/PositionCommand.h>
#include <cmath>

// 全局的发布者
ros::Publisher control_pub;

// PositionCommand话题回调函数
void positionCmdCallback(const diff_planner_msgs::PositionCommand::ConstPtr& msg) {

    sunray_msgs::UAVControlCMD control_cmd;

    control_cmd.header = msg->header;

    control_cmd.cmd_source = sunray_msgs::UAVControlCMD::CONTROL_CMD;
    control_cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY;

    control_cmd.desired_pos.x = msg->position.x;
    control_cmd.desired_pos.y = msg->position.y;
    control_cmd.desired_pos.z = msg->position.z;
    control_cmd.desired_vel = msg->velocity;
    control_cmd.desired_acc = msg->acceleration;
    control_cmd.desired_jerk = msg->jerk;

    control_cmd.desired_yaw = msg->yaw;
    control_cmd.desired_yaw_rate = msg->yaw_dot;

    if (std::abs(msg->yaw_dot) > 1e-6) {
        control_cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAWRATE;
    } else {
        control_cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
    }

    control_cmd.wgs84_pos.latitude = 0.0;
    control_cmd.wgs84_pos.longitude = 0.0;
    control_cmd.wgs84_pos.altitude = 0.0;

    control_cmd.fixed_height = false;

    if (msg->trajectory_flag == diff_planner_msgs::PositionCommand::TRAJECTORY_STATUS_EMPTY ||
        msg->trajectory_flag == diff_planner_msgs::PositionCommand::TRAJECTROY_STATUS_ABORT ||
        msg->trajectory_flag ==
            diff_planner_msgs::PositionCommand::TRAJECTORY_STATUS_ILLEGAL_START ||
        msg->trajectory_flag ==
            diff_planner_msgs::PositionCommand::TRAJECTORY_STATUS_ILLEGAL_FINAL ||
        msg->trajectory_flag == diff_planner_msgs::PositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE) {
        ROS_WARN("Invalid trajectory status: %d", msg->trajectory_flag);
        return;
    }

    control_pub.publish(control_cmd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "positioncmd_to_sunray_node");
    // 使用私有命名空间
    ros::NodeHandle nh("~");

    std::string cmd_sub_topic, control_pub_topic;
    // 从launch文件中获取参数，如果没有则使用默认值
    if (!nh.getParam("cmd_sub_topic", cmd_sub_topic)) {
        ROS_WARN("Parameter 'cmd_sub_topic' not found, using default: /position_cmd");
        cmd_sub_topic = "/position_cmd";
    }

    if (!nh.getParam("control_pub_topic", control_pub_topic)) {
        ROS_WARN(
            "Parameter 'control_pub_topic' not found, using default: /uav1/sunray/uav_control_cmd");
        control_pub_topic = "/uav1/sunray/uav_control_cmd";
    }

    ROS_INFO("positionCmd_to_sunray node starting...");
    ROS_INFO("Subscribing to: %s", cmd_sub_topic.c_str());
    ROS_INFO("Publishing to: %s", control_pub_topic.c_str());

    ros::Subscriber cmd_sub = nh.subscribe(cmd_sub_topic, 10, positionCmdCallback);
    control_pub = nh.advertise<sunray_msgs::UAVControlCMD>(control_pub_topic, 10);

    ROS_INFO("positionCmd_to_sunray node initialized successfully");

    ros::spin();

    return 0;
}
