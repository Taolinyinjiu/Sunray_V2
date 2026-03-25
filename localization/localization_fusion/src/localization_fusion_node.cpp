#include <ros/ros.h>

#include "localization_fusion.hpp"

// TODO：添加打印日志相关，LocalizaitonFusion类中添加show之类的函数，引入sunray_log进行
int main(int argc, char **argv) {
  ros::init(argc, argv, "localization_fusion_node");
  ros::NodeHandle nh;

  LocalizationFusion node(nh);
  if (!node.Init()) {
    ROS_ERROR("[localization_fusion] Node initialization failed.");
    return 1;
  }

  node.Spin();
  return 0;
}
