cmake_minimum_required(VERSION 3.0.2)
project(ugv_control_tools)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sunray_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sunray_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ugv_terminal_control_node
  src/terminal_control/ugv_terminal_control.cpp
)

target_link_libraries(ugv_terminal_control_node
  ${catkin_LIBRARIES}
)

add_dependencies(ugv_terminal_control_node ${catkin_EXPORTED_TARGETS})

install(TARGETS
  ugv_terminal_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
