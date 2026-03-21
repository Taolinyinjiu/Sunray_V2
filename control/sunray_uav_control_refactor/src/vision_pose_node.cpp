#include "vision_pose/vision_pose.hpp"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "vision_pose");
  ros::NodeHandle nh;

  VisionPose node(nh);
  if (!node.Init()) {
    ROS_ERROR("[vision_pose] init failed.");
    return 1;
  }

  node.Spin();
  return 0;
}

