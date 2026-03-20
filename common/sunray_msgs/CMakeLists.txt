cmake_minimum_required(VERSION 3.0.2)
project(sunray_msgs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  sensor_msgs
  std_msgs
)

## Add message definitions after creating files under msg/
add_message_files(
  FILES
  OdomStatus.msg
  # Px4StateCore.msg
  # Px4StateHealth.msg
  Px4State.msg
)

## Add service definitions after creating files under srv/
# add_service_files(
#   FILES
#   Example.srv
# )

## Enable after adding at least one msg/srv file
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    message_runtime
    nav_msgs
    sensor_msgs
    std_msgs
)
