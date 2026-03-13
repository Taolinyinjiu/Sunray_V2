#include "sunray_statemachine/sunray_statemachine.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "sunray_fsm_node");
  ros::NodeHandle nh;

  sunray_fsm::Sunray_StateMachine fsm(nh);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
