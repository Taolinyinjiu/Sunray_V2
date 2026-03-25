cmake_minimum_required(VERSION 3.0.2)
project(localization_fusion)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  sunray_common
  sunray_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    sunray_common
    sunray_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_filters
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Core library: class implementation (no main function)
add_library(localization_fusion_core
  src/localization_fusion.cpp
  src/localization_fusion_utils.cpp
)
target_link_libraries(localization_fusion_core
  ${catkin_LIBRARIES}
  yaml-cpp
)
add_dependencies(localization_fusion_core
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Node executable: process entry
add_executable(localization_fusion_node
  src/localization_fusion_node.cpp
)
target_link_libraries(localization_fusion_node
  localization_fusion_core
  ${catkin_LIBRARIES}
)
add_dependencies(localization_fusion_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS localization_fusion_core localization_fusion_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )
endif()
