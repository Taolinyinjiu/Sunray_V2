cmake_minimum_required(VERSION 3.0.2)
project(yunlink_ros_bridge)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  std_msgs
  sunray_msgs
)

get_filename_component(SUNRAY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(YUNLINK_ROOT "${SUNRAY_ROOT}/thirdparty/yunlink" CACHE PATH "Path to the local yunlink repository")
set(YUNLINK_BINARY_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/thirdparty/yunlink"
    CACHE PATH
    "Binary directory used to build yunlink as a subproject"
)

find_package(Threads REQUIRED)

if(NOT EXISTS "${YUNLINK_ROOT}/CMakeLists.txt")
  message(FATAL_ERROR
    "yunlink source tree not found: ${YUNLINK_ROOT}. "
    "Expected git submodule at thirdparty/yunlink. "
    "Run: git submodule update --init --recursive thirdparty/yunlink"
  )
endif()

if(NOT TARGET yunlink)
  set(YUNLINK_BUILD_EXAMPLES OFF CACHE BOOL "Build yunlink examples" FORCE)
  set(YUNLINK_BUILD_TESTS OFF CACHE BOOL "Build yunlink tests" FORCE)
  add_subdirectory("${YUNLINK_ROOT}" "${YUNLINK_BINARY_DIR}" EXCLUDE_FROM_ALL)
endif()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(yunlink_ros_bridge_node src/yunlink_ros_bridge_node.cpp)
add_dependencies(yunlink_ros_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(yunlink_ros_bridge_node
  ${catkin_LIBRARIES}
  yunlink
  Threads::Threads
)
