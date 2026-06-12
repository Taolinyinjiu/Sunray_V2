cmake_minimum_required(VERSION 3.0.2)
project(sunray_ugv_control_example)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sunray_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ugv_hold_node
  src/basic/hold.cpp
)
add_dependencies(ugv_hold_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ugv_hold_node
  ${catkin_LIBRARIES}
)

add_executable(ugv_move_point_node
  src/MOVE_POINT/move_point.cpp
)
add_dependencies(ugv_move_point_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ugv_move_point_node
  ${catkin_LIBRARIES}
)

add_executable(ugv_move_velocity_body_node
  src/MOVE_VELOCITY_BODY/move_velocity_body.cpp
)
add_dependencies(ugv_move_velocity_body_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ugv_move_velocity_body_node
  ${catkin_LIBRARIES}
)

install(TARGETS
  ugv_hold_node
  ugv_move_point_node
  ugv_move_velocity_body_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
