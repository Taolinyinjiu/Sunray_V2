cmake_minimum_required(VERSION 3.0.2)
project(sunray_common)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  sunray_msgs
  tf
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    mavros_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    sunray_msgs
    tf
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(
    DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
endif()
