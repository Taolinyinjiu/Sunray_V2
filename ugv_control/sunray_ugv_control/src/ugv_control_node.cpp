#include <ros/ros.h>
#include "ugv_control_fsm.h"

int main(int argc, char** argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "ugv_control_node");
  ros::NodeHandle nh;

  // 创建UGV控制状态机
  sunray_ugv_control::UGVControlFSM ugv_control_fsm(nh);

  // 运行ROS循环
  ros::spin();

  return 0;
}
