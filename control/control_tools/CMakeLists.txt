cmake_minimum_required(VERSION 3.0.2)
project(control_tools)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sunray_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sunray_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(uav_terminal_control_node
  src/terminal_control/uav_terminal_control.cpp
)

target_link_libraries(uav_terminal_control_node
  ${catkin_LIBRARIES}
)

add_dependencies(uav_terminal_control_node ${catkin_EXPORTED_TARGETS})

add_executable(ugv_terminal_control_node
  src/terminal_control/ugv_terminal_control.cpp
)

target_link_libraries(ugv_terminal_control_node
  ${catkin_LIBRARIES}
)

add_dependencies(ugv_terminal_control_node ${catkin_EXPORTED_TARGETS})

add_executable(terminal_control_ugv_node
  src/terminal_control/terminal_control_ugv.cpp
)

target_link_libraries(terminal_control_ugv_node
  ${catkin_LIBRARIES}
)

add_dependencies(terminal_control_ugv_node ${catkin_EXPORTED_TARGETS})

add_executable(triangle_velocity_test
  src/velocity_test/triangle_velocity_test.cpp
)

target_link_libraries(triangle_velocity_test
  ${catkin_LIBRARIES}
)

add_dependencies(triangle_velocity_test ${catkin_EXPORTED_TARGETS})
