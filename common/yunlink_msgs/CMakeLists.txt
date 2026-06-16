cmake_minimum_required(VERSION 3.0.2)
project(yunlink_msgs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  CommandMeta.msg
  TakeoffCommand.msg
  LandCommand.msg
  ReturnCommand.msg
  GotoCommand.msg
  VelocitySetpointCommand.msg
  CommandExecutionStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
