cmake_minimum_required(VERSION 3.0.2)
project(px4_web_ui_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  CATKIN_DEPENDS rospy
)

catkin_install_python(PROGRAMS
  scripts/px4_web_ui_demo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  web/index.html
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/web
)

