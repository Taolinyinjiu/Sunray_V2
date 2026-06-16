cmake_minimum_required(VERSION 3.0.2)
project(joystick_ros_bridge)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS mavros_msgs roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joystick_ros_bridge_node
  src/joystick_ros_bridge_node.cpp
)

target_link_libraries(joystick_ros_bridge_node
  ${catkin_LIBRARIES}
)

install(TARGETS joystick_ros_bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)
