cmake_minimum_required(VERSION 3.0.2)
project(external_fusion)

add_compile_options(-std=c++14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# NOTE:
# 1) 当前先提供最小可识别的 ROS 包骨架；
# 2) 后续你在 src/ 中添加节点源码后，可在这里 add_executable + target_link_libraries。

