cmake_minimum_required(VERSION 3.0.2)
project(sunray_fuel_adapter)

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

add_executable(Fuel2Sunray_node src/Fuel2Sunray.cpp)

add_dependencies(Fuel2Sunray_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(Fuel2Sunray_node
  ${catkin_LIBRARIES}
)
