cmake_minimum_required(VERSION 2.8.3)
project(sunray_sim)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    sunray_sim_sensing
    sunray_sim_uav
    sunray_sim_ugv
    sunray_sim_core
  CATKIN_DEPENDS
    geometry_msgs
    mavros_msgs
    nav_msgs
    pcl_conversions
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    std_msgs
    tf2_ros
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(sunray_sim_sensing
  src/map_sensing/global_map_server.cpp
  src/map_sensing/local_mid360_simulator.cpp
  src/imu/imu_model.cpp
)
target_link_libraries(
  sunray_sim_sensing
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(sunray_sim_uav
  src/uav/uav_plant.cpp
  src/uav/quadrotor_dynamics.cpp
  src/uav/px4_mavros_sim.cpp
)
target_link_libraries(
  sunray_sim_uav
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  sunray_sim_sensing
)

add_library(sunray_sim_ugv
  src/ugv/ugv_plant.cpp
  src/ugv/ugv_dynamics.cpp
)
target_link_libraries(
  sunray_sim_ugv
  ${catkin_LIBRARIES}
  sunray_sim_sensing
)

add_library(sunray_sim_core
  src/core/single_uav_simulator.cpp
  src/core/single_ugv_simulator.cpp
  src/core/sim_visualizer.cpp
)
target_link_libraries(
  sunray_sim_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  sunray_sim_sensing
  sunray_sim_uav
  sunray_sim_ugv
)

add_executable(sunray_sim_node src/sunray_sim_node.cpp)
add_dependencies(sunray_sim_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  sunray_sim_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  sunray_sim_sensing
  sunray_sim_uav
  sunray_sim_ugv
  sunray_sim_core
)

install(TARGETS
  sunray_sim_sensing
  sunray_sim_uav
  sunray_sim_ugv
  sunray_sim_core
  sunray_sim_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ config/ resource/ rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  test/minimal_closed_loop_check.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
