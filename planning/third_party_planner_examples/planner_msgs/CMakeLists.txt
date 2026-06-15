cmake_minimum_required(VERSION 3.0.2)
project(planner_msgs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  DiffGoalSet.msg
  DiffPositionCommand.msg
  EgoPositionCommand.msg
  FuelPositionCommand.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_msgs
)
