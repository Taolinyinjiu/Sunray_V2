cmake_minimum_required(VERSION 3.0.2)
project(planner_tools)

add_compile_options(-std=c++14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  sunray_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    pcl_conversions
    pcl_ros
    roscpp
    roslib
    rospy
    sensor_msgs
    std_msgs
    sunray_msgs
    tf
    tf2
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/sunray_common/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/sunray_log/
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/sunray_log/spdlog/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(rviz_goal_bridge
  src/rviz_goal_bridge.cpp
)

add_executable(point_cloud_transform
  src/point_cloud_transform.cpp
)

target_link_libraries(rviz_goal_bridge
  ${catkin_LIBRARIES}
)

target_link_libraries(point_cloud_transform
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
