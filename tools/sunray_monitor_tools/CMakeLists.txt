cmake_minimum_required(VERSION 3.0.2)
project(sunray_monitor_tools)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  topic_tools
  rviz
)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sunray_monitor_panel_node
  src/sunray_monitor_panel_node.cpp
)
add_dependencies(sunray_monitor_panel_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(sunray_monitor_panel_node
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

install(TARGETS sunray_monitor_panel_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)
