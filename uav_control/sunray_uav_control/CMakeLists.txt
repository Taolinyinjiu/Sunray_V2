cmake_minimum_required(VERSION 3.0.2)
project(sunray_uav_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
set(Eigen3_LIBRARIES Eigen3::Eigen)
set(yaml-cpp_INCLUDE_DIRS /usr/include)
set(yaml-cpp_LIBRARIES yaml-cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  geographic_msgs
  mavros_msgs
  sunray_common
  sunray_logger
  sunray_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sunray_uav_control_lib
  CATKIN_DEPENDS
    roscpp
    roslib
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    geographic_msgs
    mavros_msgs
    sunray_common
    sunray_logger
    sunray_msgs
  DEPENDS Eigen3 yaml-cpp
)

# -----------------------------
# Core library
# -----------------------------
add_library(sunray_uav_control_lib
  src/mavros_helper/mavros_helper.cpp
  src/controller/px4_origin_controller/px4_origin_controller.cpp
  src/controller/px4_origin_controller/px4_origin_loadparam.cpp
  src/controller/core_algorithm/hoverthrust_estimator.cpp
  src/controller/geometric_controller/geometric_controller.cpp
  src/controller/geometric_controller/geometric_loadparam.cpp
  src/controller/core_algorithm/geometric_attitude_control.cpp
  src/odom_filter/odom_kalman_filter.cpp
  src/utils/control_config_loader.cpp
  src/statemachine/sunray_fsm_loadparam.cpp
  src/statemachine/sunray_fsm.cpp
  src/statemachine/sunray_fsm_transition.cpp
  src/statemachine/sunray_fsm_control.cpp
  src/statemachine/sunray_fsm_checks.cpp
)
add_dependencies(sunray_uav_control_lib ${catkin_EXPORTED_TARGETS})

target_include_directories(sunray_uav_control_lib
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(sunray_uav_control_lib
  PUBLIC
    ${catkin_LIBRARIES}
    yaml-cpp
)

# Enable detailed controller / estimator debug logs guarded by #ifdef DEBUG.
target_compile_definitions(sunray_uav_control_lib
  PRIVATE
    DEBUG
)

# -----------------------------
# Main FSM node
# -----------------------------
add_executable(uav_control_node
  src/uav_control_node.cpp
)
add_dependencies(uav_control_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(uav_control_node
  PRIVATE
    sunray_uav_control_lib
)

add_executable(uav_control_monitor_node
  src/tools/uav_control_monitor_node.cpp
)
add_dependencies(uav_control_monitor_node ${catkin_EXPORTED_TARGETS})
target_include_directories(uav_control_monitor_node
  PRIVATE
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(uav_control_monitor_node
  PRIVATE
    ${catkin_LIBRARIES}
)

add_executable(rviz_visualization_uav_control_node
  src/tools/rviz_visualization_uav_control_node.cpp
)
add_dependencies(rviz_visualization_uav_control_node ${catkin_EXPORTED_TARGETS})
target_include_directories(rviz_visualization_uav_control_node
  PRIVATE
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(rviz_visualization_uav_control_node
  PRIVATE
    ${catkin_LIBRARIES}
)

# -----------------------------
# Runtime test/demo executables
# -----------------------------
add_executable(origin_controller_takeoff_test
  test/px4_origin_controller_takeoff/origin_controller_test.cpp
)
add_dependencies(origin_controller_takeoff_test ${catkin_EXPORTED_TARGETS})

target_link_libraries(origin_controller_takeoff_test
  PRIVATE
    sunray_uav_control_lib
)

add_executable(origin_controller_square_test
  test/px4_origin_controller_takeoff/origin_controller_square.cpp
)
add_dependencies(origin_controller_square_test ${catkin_EXPORTED_TARGETS})

target_link_libraries(origin_controller_square_test
  PRIVATE
    sunray_uav_control_lib
)

add_executable(origin_controller_square_test_velocity
  test/px4_origin_controller_takeoff/origin_controller_square_velocity.cpp
)
add_dependencies(origin_controller_square_test_velocity ${catkin_EXPORTED_TARGETS})

target_link_libraries(origin_controller_square_test_velocity
  PRIVATE
    sunray_uav_control_lib
)

add_executable(hoverthrust_estimator_ros_test_node
  test/hoverthrust_estimator/hoverthrust_estimator_ros_test_node.cpp
)
add_dependencies(hoverthrust_estimator_ros_test_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(hoverthrust_estimator_ros_test_node
  PRIVATE
    sunray_uav_control_lib
)

# -----------------------------
# Unit tests
# -----------------------------
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(base_controller_compile_test
    test/base_controller_compile_test.cpp
  )
  if(TARGET base_controller_compile_test)
    target_include_directories(base_controller_compile_test
      PRIVATE
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(base_controller_compile_test
        sunray_uav_control_lib
    )
  endif()

  catkin_add_gtest(controller_output_types_compile_test
    test/controller_output_types_compile_test.cpp
  )
  if(TARGET controller_output_types_compile_test)
    target_include_directories(controller_output_types_compile_test
      PRIVATE
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(controller_output_types_compile_test
        sunray_uav_control_lib
    )
  endif()

  catkin_add_gtest(body_frame_reference_helper_test
    test/body_frame_reference_helper_test.cpp
  )
  if(TARGET body_frame_reference_helper_test)
    target_include_directories(body_frame_reference_helper_test
      PRIVATE
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(body_frame_reference_helper_test
        sunray_uav_control_lib
    )
  endif()

  catkin_add_gtest(odom_kalman_filter_test
    test/odom_kalman_filter_test.cpp
  )
  if(TARGET odom_kalman_filter_test)
    target_include_directories(odom_kalman_filter_test
      PRIVATE
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(odom_kalman_filter_test
        sunray_uav_control_lib
    )
  endif()
endif()

# -----------------------------
# Install
# -----------------------------
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(TARGETS
  sunray_uav_control_lib
  uav_control_node
  uav_control_monitor_node
  origin_controller_takeoff_test
  origin_controller_square_test
  origin_controller_square_test_velocity
  hoverthrust_estimator_ros_test_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
