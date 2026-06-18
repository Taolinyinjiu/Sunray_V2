cmake_minimum_required(VERSION 2.8.3)
project(sunray_mavros_sim)

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
    sunray_quadrotor_dynamics
    sunray_imu_model
    sunray_px4_control_sim
    sunray_fake_mavros_bridge
    sunray_global_map_server
    sunray_local_mid360_simulator
    sunray_quadrotor_simulator
    sunray_ugv_dynamics
    sunray_ugv_simulator
    sunray_sim_visualizer
    sunray_single_uav_simulator
    sunray_single_ugv_simulator
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

add_library(sunray_quadrotor_dynamics src/quadrotor_dynamics.cpp)
target_link_libraries(sunray_quadrotor_dynamics ${catkin_LIBRARIES})

add_library(sunray_ugv_dynamics src/ugv_dynamics.cpp)
target_link_libraries(sunray_ugv_dynamics ${catkin_LIBRARIES})

add_library(sunray_imu_model src/imu_model.cpp)
target_link_libraries(sunray_imu_model ${catkin_LIBRARIES})

add_library(sunray_px4_control_sim src/px4_control_sim.cpp)
target_link_libraries(sunray_px4_control_sim ${catkin_LIBRARIES})

add_library(sunray_fake_mavros_bridge src/fake_mavros_bridge.cpp)
target_link_libraries(sunray_fake_mavros_bridge ${catkin_LIBRARIES})

add_library(sunray_global_map_server src/global_map_server.cpp)
target_link_libraries(
  sunray_global_map_server
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(sunray_quadrotor_simulator src/quadrotor_simulator.cpp)
target_link_libraries(
  sunray_quadrotor_simulator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  sunray_quadrotor_dynamics
  sunray_imu_model
)

add_library(sunray_ugv_simulator src/ugv_simulator.cpp)
target_link_libraries(
  sunray_ugv_simulator
  ${catkin_LIBRARIES}
  sunray_ugv_dynamics
  sunray_imu_model
)

add_library(sunray_local_mid360_simulator src/local_mid360_simulator.cpp)
target_link_libraries(
  sunray_local_mid360_simulator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(sunray_single_uav_simulator src/single_uav_simulator.cpp)
target_link_libraries(
  sunray_single_uav_simulator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  sunray_local_mid360_simulator
  sunray_quadrotor_simulator
  sunray_px4_control_sim
  sunray_fake_mavros_bridge
)

add_library(sunray_single_ugv_simulator src/single_ugv_simulator.cpp)
target_link_libraries(
  sunray_single_ugv_simulator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  sunray_local_mid360_simulator
  sunray_ugv_simulator
)

add_library(sunray_sim_visualizer src/sim_visualizer.cpp)
target_link_libraries(
  sunray_sim_visualizer
  ${catkin_LIBRARIES}
)

add_executable(sunray_mavros_sim_node src/sunray_mavros_sim_node.cpp)
add_dependencies(sunray_mavros_sim_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  sunray_mavros_sim_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  sunray_global_map_server
  sunray_single_uav_simulator
  sunray_single_ugv_simulator
  sunray_sim_visualizer
)

install(TARGETS
  sunray_quadrotor_dynamics
  sunray_imu_model
  sunray_px4_control_sim
  sunray_fake_mavros_bridge
  sunray_global_map_server
  sunray_local_mid360_simulator
  sunray_quadrotor_simulator
  sunray_ugv_dynamics
  sunray_ugv_simulator
  sunray_sim_visualizer
  sunray_single_uav_simulator
  sunray_single_ugv_simulator
  sunray_mavros_sim_node
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
