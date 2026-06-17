cmake_minimum_required(VERSION 3.0.2)
project(sunray_logger)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roslib
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sunray_logger
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roslib
    roscpp
    sensor_msgs
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(sunray_logger
  src/ulog_writer.cpp
)

target_link_libraries(sunray_logger
  ${catkin_LIBRARIES}
)

add_dependencies(sunray_logger
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(simple_ulog_logger_example
  examples/simple_ulog_logger_example.cpp
)

target_link_libraries(simple_ulog_logger_example
  sunray_logger
  ${catkin_LIBRARIES}
)

add_dependencies(simple_ulog_logger_example
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS sunray_logger simple_ulog_logger_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
