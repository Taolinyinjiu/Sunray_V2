cmake_minimum_required(VERSION 3.0.2)
project(sunray_uav_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  px4_bridge
  roscpp
  sensor_msgs
  sunray_common
  sunray_msgs
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sunray_uav_control_lib
  CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs px4_bridge roscpp sensor_msgs sunray_common sunray_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(sunray_uav_control_lib
  src/mavros_helper/mavros_helper.cpp
  src/controller/utils.cpp
)

target_link_libraries(sunray_uav_control_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  yaml-cpp
)

add_executable(origin_controller_takeoff_test
  test/px4_origin_controller_takeoff/origin_controller_test.cpp
  src/controller/px4_origin_controller.cpp
)

target_link_libraries(origin_controller_takeoff_test
  sunray_uav_control_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  yaml-cpp
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(base_controller_compile_test
    test/base_controller_compile_test.cpp
  )
  if(TARGET base_controller_compile_test)
    target_link_libraries(base_controller_compile_test
      ${catkin_LIBRARIES}
      ${EIGEN3_LIBRARIES}
    )
  endif()

  catkin_add_gtest(controller_output_types_compile_test
    test/controller_output_types_compile_test.cpp
  )
  if(TARGET controller_output_types_compile_test)
    target_link_libraries(controller_output_types_compile_test
      ${catkin_LIBRARIES}
      ${EIGEN3_LIBRARIES}
    )
  endif()
endif()

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
  origin_controller_takeoff_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
