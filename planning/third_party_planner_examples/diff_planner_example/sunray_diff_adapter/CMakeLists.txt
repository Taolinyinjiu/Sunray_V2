cmake_minimum_required(VERSION 3.0.2)
project(sunray_diff_adapter)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  planner_msgs
  roscpp
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

add_executable(Diff2Sunray_node src/Diff2Sunray.cpp)

add_dependencies(Diff2Sunray_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(Diff2Sunray_node
  ${catkin_LIBRARIES}
)
