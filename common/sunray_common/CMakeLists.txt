cmake_minimum_required(VERSION 3.0.2)
project(sunray_common)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(
    DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
endif()
