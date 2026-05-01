cmake_minimum_required(VERSION 3.0.2)
project(sunray_msgs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geographic_msgs
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
  OrcaCmd.msg
  OrcaSetup.msg
  Px4State.msg
  PlanningWaypoint.msg
  UAVControlCMD.msg
  UAVControlFSMState.msg
  UAVControllerState.msg
  UAVPlanningCMD.msg
  UAVPlanningState.msg
  UAVSwarmCMD.msg
  UAVSwarmState.msg
  UGVControlCMD.msg
  UGVControlFSMState.msg
  UGVSwarmCMD.msg
  UGVSwarmState.msg
  Vector2.msg
  Formation.msg
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
  geographic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geographic_msgs
    geometry_msgs
    message_runtime
    nav_msgs
    sensor_msgs
    std_msgs
)
