cmake_minimum_required(VERSION 3.0.2)
project(sunray_planning)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  sunray_planner_msgs
  geometry_msgs
  roscpp
  roslib
  sunray_common
  sunray_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sunray_planning
  CATKIN_DEPENDS
    sunray_planner_msgs
    geometry_msgs
    roscpp
    roslib
    sunray_common
    sunray_msgs
)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/sunray_log/
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/sunray_log/spdlog/include
  ${catkin_INCLUDE_DIRS}
)

add_library(sunray_planning
  src/sunray_planning_fsm.cpp
  src/sunray_planning_logs.cpp
  src/planner_interface/diff_planner.cpp
  src/planner_interface/ego_planner.cpp
)

target_link_libraries(sunray_planning
  ${catkin_LIBRARIES}
)

add_executable(sunray_planning_node
  src/sunray_planning_node.cpp
)

target_link_libraries(sunray_planning_node
  sunray_planning
  ${catkin_LIBRARIES}
)
