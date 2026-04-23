cmake_minimum_required(VERSION 3.0.2)
project(sunray_planning)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sunray_common
  sunray_msgs
)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sunray_planning
  CATKIN_DEPENDS
    roscpp
    sunray_common
    sunray_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(sunray_planning
  src/planner_interface/ego_planner.cpp
)

target_link_libraries(sunray_planning
  ${catkin_LIBRARIES}
  yaml-cpp
)
