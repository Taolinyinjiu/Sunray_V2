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
  tf2
  tf2_ros
  tf2_geometry_msgs
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    sensor_msgs 
    nav_msgs 
    geometry_msgs
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
