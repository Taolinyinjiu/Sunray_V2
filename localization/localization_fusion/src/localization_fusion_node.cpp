#include <ros/ros.h>

#include <csignal>

#include "localization_fusion.hpp"

// 捕获中断信号
void mySigintHandler(int /*sig*/) {
    ros::shutdown();
}
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "localization_fusion_node");
    ros::NodeHandle nh;

    // 设置中断信号处理函数
    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    // 声明节点
    LocalizationFusion localization_fusion_node(nh);
    // localization_fusion_node初始化
    if (!localization_fusion_node.Init()) {
        ROS_ERROR("[localization_fusion] Node initialization failed.");
        return 1;
    }

    ros::spin();
    return 0;
}
