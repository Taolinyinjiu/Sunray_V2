#include "sunray_helper/sunray_helper.h"

#include <ros/ros.h>

#include <Eigen/Dense>

int main(int argc, char **argv) {
  ros::init(argc, argv, "sunray_helper_demo_node");
  ros::NodeHandle nh;

  Sunray_Helper helper(nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  double takeoff_wait_s = 5.0;
  double move_wait_s = 5.0;
  double land_wait_s = 5.0;
  nh.param("takeoff_wait_s", takeoff_wait_s, takeoff_wait_s);
  nh.param("move_wait_s", move_wait_s, move_wait_s);
  nh.param("land_wait_s", land_wait_s, land_wait_s);

  ROS_INFO("[SunrayHelperDemo] takeoff");
  if (!helper.takeoff_block()) {
    ROS_WARN("[SunrayHelperDemo] takeoff request rejected or service unavailable");
  }
  ros::Duration(takeoff_wait_s).sleep();

  Eigen::Vector3d start_pos = helper.get_uav_position();
  Eigen::Vector3d forward_pos = start_pos + Eigen::Vector3d(1.0, 0.0, 0.0);
  ROS_INFO("[SunrayHelperDemo] move forward 1m");
  helper.set_position_async(forward_pos);
  ros::Duration(move_wait_s).sleep();

  Eigen::Vector3d left_pos = forward_pos + Eigen::Vector3d(0.0, 1.0, 0.0);
  ROS_INFO("[SunrayHelperDemo] move left 1m");
  helper.set_position_async(left_pos);
  ros::Duration(move_wait_s).sleep();

  ROS_INFO("[SunrayHelperDemo] land");
  if (!helper.land_block()) {
    ROS_WARN("[SunrayHelperDemo] land request rejected or service unavailable");
  }
  ros::Duration(land_wait_s).sleep();

  ROS_INFO("[SunrayHelperDemo] done");
  ros::shutdown();
  return 0;
}
