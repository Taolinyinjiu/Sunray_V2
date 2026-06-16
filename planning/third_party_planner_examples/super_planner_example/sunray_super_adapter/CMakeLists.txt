cmake_minimum_required(VERSION 3.0.2)
project(sunray_super_adapter)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  planner_msgs
  sunray_msgs
)

catkin_package(
  CATKIN_DEPENDS
    planner_msgs
    roscpp
    sunray_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(Super2Sunray_node src/Super2Sunray.cpp)

add_dependencies(Super2Sunray_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(Super2Sunray_node
  ${catkin_LIBRARIES}
)
