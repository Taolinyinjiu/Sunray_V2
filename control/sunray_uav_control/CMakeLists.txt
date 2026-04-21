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
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  geographic_msgs
  mavros_msgs
  px4_bridge
  sunray_common
  sunray_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sunray_uav_control_lib
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    geographic_msgs
    mavros_msgs
    px4_bridge
    sunray_common
    sunray_msgs
  DEPENDS Eigen3 yaml-cpp
)

# -----------------------------
# Core library
# -----------------------------
add_library(sunray_uav_control_lib
  src/mavros_helper/mavros_helper.cpp
  src/mavros_helper/mavros_helper_data_types.cpp
  src/controller/px4_origin_controller/px4_origin_controller.cpp
  src/controller/px4_origin_controller/px4_origin_controller_logs.cpp
  src/controller/core_algorithm/hoverthrust_estimator.cpp
  src/controller/geometric_controller/geometric_controller.cpp
  src/controller/geometric_controller/geometric_loadparam.cpp
  src/controller/geometric_controller/geometric_logs.cpp
  src/controller/raptor_controller/raptor_controller.cpp
  src/controller/raptor_controller/raptor_loadparam.cpp
  src/controller/raptor_controller/raptor_logs.cpp
  src/controller/core_algorithm/geometric_attitude_control.cpp
  src/controller/core_algorithm/attitude_slover.cpp
  src/statemachine/sunray_fsm_loadparam.cpp
  src/statemachine/sunray_fsm.cpp
  src/statemachine/sunray_fsm_transition.cpp
  src/statemachine/sunray_fsm_control.cpp
  src/statemachine/sunray_fsm_checks.cpp
  src/statemachine/sunray_fsm_logs.cpp
)

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

target_link_libraries(uav_control_node
  PRIVATE
    sunray_uav_control_lib
)

# -----------------------------
# Runtime test/demo executables
# -----------------------------
add_executable(origin_controller_takeoff_test
  test/px4_origin_controller_takeoff/origin_controller_test.cpp
)

target_link_libraries(origin_controller_takeoff_test
  PRIVATE
    sunray_uav_control_lib
)

add_executable(origin_controller_square_test
  test/px4_origin_controller_takeoff/origin_controller_square.cpp
)

target_link_libraries(origin_controller_square_test
  PRIVATE
    sunray_uav_control_lib
)

add_executable(origin_controller_square_test_velocity
  test/px4_origin_controller_takeoff/origin_controller_square_velocity.cpp
)

target_link_libraries(origin_controller_square_test_velocity
  PRIVATE
    sunray_uav_control_lib
)

add_executable(hoverthrust_estimator_ros_test_node
  test/hoverthrust_estimator/hoverthrust_estimator_ros_test_node.cpp
)

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
    target_link_libraries(base_controller_compile_test
        sunray_uav_control_lib
    )
  endif()

  catkin_add_gtest(controller_output_types_compile_test
    test/controller_output_types_compile_test.cpp
  )
  if(TARGET controller_output_types_compile_test)
    target_link_libraries(controller_output_types_compile_test
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
  origin_controller_takeoff_test
  origin_controller_square_test
  origin_controller_square_test_velocity
  hoverthrust_estimator_ros_test_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
