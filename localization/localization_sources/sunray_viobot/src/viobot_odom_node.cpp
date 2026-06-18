// TODO: 1. 添加VIOBOT算法启动命令？ 
// 2. 添加参数用于快速切换高频与不同算法？
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <signal.h>
#include "sunray_log.hpp"

ros::Publisher viobot_odom_pub;

// 中断信号
void MySigintHandler(int sig) {

    ros::shutdown();
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    nav_msgs::Odometry odom_msg = *msg;

    // viobot默认里程计话题输出位置信息正确，姿态四元数与无人机不一致，需要转换，速度与角速度都需要变换
    tf2::Quaternion q;
    q.setW(msg->pose.pose.orientation.w);
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);

    // 绕 Z 轴旋转 90°
    tf2::Quaternion q_z;
    q_z.setRPY(0, 0, M_PI / 2);

    // 绕 Y 轴旋转 -90°
    tf2::Quaternion q_y;
    q_y.setRPY(0, -M_PI / 2, 0);

    // 组合旋转（顺序：先 q_z，再 q_y）
    q = q * q_z * q_y;

    // 发布里程计消息
    // 位置一致，直接填充
    odom_msg.pose.pose.position.x = msg->pose.pose.position.x;
    odom_msg.pose.pose.position.y = msg->pose.pose.position.y;
    odom_msg.pose.pose.position.z = msg->pose.pose.position.z;
    // 姿态不一致，填充变换后的姿态
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    // 速度方向不一致，转换速度方向
    odom_msg.twist.twist.linear.x = -msg->twist.twist.linear.x;
    odom_msg.twist.twist.linear.y =  msg->twist.twist.linear.y;
    odom_msg.twist.twist.linear.z = -msg->twist.twist.linear.z;
    // 角速度方向不一致，转换角速度
    odom_msg.twist.twist.angular.x = -msg->twist.twist.angular.x;
    odom_msg.twist.twist.angular.y =  msg->twist.twist.angular.y;
    odom_msg.twist.twist.angular.z = -msg->twist.twist.angular.z;
    
    // 发布里程计数据
    viobot_odom_pub.publish(odom_msg);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "viobot_odom_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 中断信号注册
    signal(SIGINT, MySigintHandler);

    std::string odom_pub_topic;
    std::string odom_sub_topic;
    private_nh.param<std::string>("odom_pub_topic", odom_pub_topic, "sunray/odometry");
    private_nh.param<std::string>("odom_sub_topic", odom_sub_topic, "/baton/stereo3/odometry");

    viobot_odom_pub = nh.advertise<nav_msgs::Odometry>(odom_pub_topic, 10);
    ros::Subscriber viobot_odom_sub = nh.subscribe(odom_sub_topic, 10, OdomCallback);

    SUNRAY_INFO("Subscribe odometry topic: {}", odom_sub_topic);
    SUNRAY_INFO("Publish odometry topic: {}", odom_pub_topic);

    ros::spin();
    return 0;
}
