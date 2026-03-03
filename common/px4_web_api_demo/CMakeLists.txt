cmake_minimum_required(VERSION 3.0.2)
project(px4_web_api_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  mavros_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy mavros_msgs
)

catkin_install_python(PROGRAMS
  scripts/px4_web_api_demo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

