cmake_minimum_required(VERSION 3.0.2)
project(ekf_odometry)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(Sophus QUIET)

set(USE_BUNDLED_SOPHUS FALSE)
if(NOT Sophus_FOUND)
  set(USE_BUNDLED_SOPHUS TRUE)
elseif(Sophus_LIBRARIES)
  foreach(SOPHUS_LIBRARY ${Sophus_LIBRARIES})
    if(IS_ABSOLUTE ${SOPHUS_LIBRARY} AND NOT EXISTS ${SOPHUS_LIBRARY})
      message(WARNING "Ignoring broken Sophus package because ${SOPHUS_LIBRARY} does not exist")
      set(USE_BUNDLED_SOPHUS TRUE)
    endif()
  endforeach()
endif()

if(USE_BUNDLED_SOPHUS)
  set(SOPHUS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/Sophus)
  if(EXISTS ${SOPHUS_SOURCE_DIR}/sophus/so3.h)
    message(STATUS "Sophus package not found; using bundled Sophus from ${SOPHUS_SOURCE_DIR}")
    set(Sophus_INCLUDE_DIRS ${SOPHUS_SOURCE_DIR})
    add_library(sophus_vendor
      ${SOPHUS_SOURCE_DIR}/sophus/so2.cpp
      ${SOPHUS_SOURCE_DIR}/sophus/se2.cpp
      ${SOPHUS_SOURCE_DIR}/sophus/se3.cpp
      ${SOPHUS_SOURCE_DIR}/sophus/so3.cpp
      ${SOPHUS_SOURCE_DIR}/sophus/scso3.cpp
      ${SOPHUS_SOURCE_DIR}/sophus/sim3.cpp
    )
    target_include_directories(sophus_vendor PUBLIC
      ${Sophus_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIR}
    )
    set(Sophus_LIBRARIES sophus_vendor)
  else()
    message(FATAL_ERROR
      "Could not find Sophus and bundled Sophus is missing at ${SOPHUS_SOURCE_DIR}. "
      "Install Sophus or set Sophus_DIR to a directory containing SophusConfig.cmake."
    )
  endif()
endif()


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES imu_process ekf_filter
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    sensor_msgs 
    nav_msgs 
    geometry_msgs
    message_filters
    tf2
    tf2_ros
    tf2_geometry_msgs
    eigen_conversions
  DEPENDS EIGEN3 Sophus
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Sophus_INCLUDE_DIRS}
)

# IMU Process library
add_library(imu_process
  src/imu_process.cpp
)
target_link_libraries(imu_process
  ${catkin_LIBRARIES}
)

# EKF library
add_library(ekf_filter
  src/ekf_filter.cpp
)
target_link_libraries(ekf_filter
  ${catkin_LIBRARIES}
  ${Sophus_LIBRARIES}
)

# Main node executable
add_executable(ekf_odometry_node
  src/ekf_odometry_node.cpp
)
target_link_libraries(ekf_odometry_node
  imu_process
  ekf_filter
  ${catkin_LIBRARIES}
)

# Mocap pose/twist to odometry node
add_executable(mocap_odom_node
  src/mocap_odom_node.cpp
)
target_link_libraries(mocap_odom_node
  ${catkin_LIBRARIES}
)
