cmake_minimum_required(VERSION 3.0.2)
project(sunray_uav_control)

add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  sunray_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    mavros_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    sunray_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vision_pose_node
  src/vision_pose_node.cpp
  src/vision_pose.cpp
)

target_link_libraries(vision_pose_node
  ${catkin_LIBRARIES}
)

add_dependencies(vision_pose_node
  ${catkin_EXPORTED_TARGETS}
)
