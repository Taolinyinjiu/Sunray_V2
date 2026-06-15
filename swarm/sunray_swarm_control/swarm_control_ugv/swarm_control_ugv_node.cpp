/*
本文件功能：
    1、作为 Swarm_Control_UGV 的 ROS 入口节点
    2、负责初始化 ROS 并创建类实例
*/
#include "swarm_control_ugv.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_control_ugv_node");

    ros::NodeHandle nh("~");

    swarm_control::Swarm_Control_UGV swarm_control_ugv_node(nh);
    ros::spin();
    return 0;
}
