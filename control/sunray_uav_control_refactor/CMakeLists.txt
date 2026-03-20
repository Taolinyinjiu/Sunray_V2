cmake_minimum_required(VERSION 3.0.2)
project(sunray_uav_control)

add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  px4_bridge
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
    px4_bridge
    roscpp
    sensor_msgs
    std_msgs
    sunray_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Refactor package build targets will be added here, for example:
# add_library(${PROJECT_NAME}_core
#   src/xxx.cpp
# )
#
# add_executable(${PROJECT_NAME}_node
#   src/xxx_node.cpp
# )
#
# target_link_libraries(${PROJECT_NAME}_node
#   ${PROJECT_NAME}_core
#   ${catkin_LIBRARIES}
# )
