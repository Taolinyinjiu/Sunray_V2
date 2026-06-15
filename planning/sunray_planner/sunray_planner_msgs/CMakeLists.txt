cmake_minimum_required(VERSION 2.8.3)
project(sunray_planner_msgs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  DiffGoalSet.msg
  DiffPositionCommand.msg
  EgoPositionCommand.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs
)
