cmake_minimum_required(VERSION 3.0.2)
project(sunray_ugv_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  std_msgs
  geographic_msgs
  sunray_common
  sunray_msgs
  geometry_msgs
  nav_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sunray_ugv_control_lib
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    std_msgs
    geographic_msgs
    sunray_common
    sunray_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(sunray_ugv_control_lib
  src/ugv_controller.cpp
  src/ugv_control_fsm.cpp
)

add_dependencies(sunray_ugv_control_lib ${catkin_EXPORTED_TARGETS})

target_link_libraries(sunray_ugv_control_lib
  PUBLIC
    ${catkin_LIBRARIES}
    yaml-cpp
)

add_executable(ugv_control_node
  src/ugv_control_node.cpp
)

add_dependencies(ugv_control_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(ugv_control_node
  PRIVATE
    sunray_ugv_control_lib
)

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
  sunray_ugv_control_lib
  ugv_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(ugv_controller_test
    test/ugv_controller.test
    test/ugv_controller_test.cpp
  )
  if(TARGET ugv_controller_test)
    target_include_directories(ugv_controller_test
      PRIVATE
        include
        ${catkin_INCLUDE_DIRS}
    )
    target_link_libraries(ugv_controller_test
      sunray_ugv_control_lib
    )
  endif()

  add_rostest_gtest(ugv_control_fsm_test
    test/ugv_control_fsm.test
    test/ugv_control_fsm_test.cpp
  )
  if(TARGET ugv_control_fsm_test)
    target_include_directories(ugv_control_fsm_test
      PRIVATE
        include
        ${catkin_INCLUDE_DIRS}
    )
    target_link_libraries(ugv_control_fsm_test
      sunray_ugv_control_lib
    )
  endif()
endif()
