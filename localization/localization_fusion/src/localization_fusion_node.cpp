#include <ros/ros.h>

#include "localization_fusion.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "localization_fusion_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  localization_fusion::LocalizationFusion node(nh, private_nh);
  if (!node.Init()) {
    ROS_ERROR("[localization_fusion] Node initialization failed.");
    return 1;
  }

  node.Spin();
  return 0;
}
