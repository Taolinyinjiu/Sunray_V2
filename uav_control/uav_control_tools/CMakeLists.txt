cmake_minimum_required(VERSION 3.0.2)
project(uav_control_tools)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  visualization_msgs
  rviz
  sunray_msgs
)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

qt5_add_resources(UAV_CONTROL_TOOLS_QRC
  resources/uav_control_tools.qrc
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs visualization_msgs rviz sunray_msgs
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

add_executable(uav_control_panel_node
  src/uav_control_panel_node.cpp
  ${UAV_CONTROL_TOOLS_QRC}
)

target_link_libraries(uav_control_panel_node
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

add_dependencies(uav_control_panel_node ${catkin_EXPORTED_TARGETS})

add_executable(triangle_velocity_test
  src/velocity_test/triangle_velocity_test.cpp
)

target_link_libraries(triangle_velocity_test
  ${catkin_LIBRARIES}
)

add_dependencies(triangle_velocity_test ${catkin_EXPORTED_TARGETS})

install(TARGETS
  uav_terminal_control_node
  uav_control_panel_node
  triangle_velocity_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
