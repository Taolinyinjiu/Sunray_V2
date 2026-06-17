cmake_minimum_required(VERSION 3.0.2)
project(sunray_param_manager)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(module_a_localization_node src/module_a_localization_node.cpp)
target_link_libraries(module_a_localization_node ${catkin_LIBRARIES})

add_executable(module_b_planning_node src/module_b_planning_node.cpp)
target_link_libraries(module_b_planning_node ${catkin_LIBRARIES})

add_executable(module_c_control_node src/module_c_control_node.cpp)
target_link_libraries(module_c_control_node ${catkin_LIBRARIES})

install(TARGETS
  module_a_localization_node
  module_b_planning_node
  module_c_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
