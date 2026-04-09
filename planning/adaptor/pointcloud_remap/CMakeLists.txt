cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_remap)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf
  tf2
  tf2_sensor_msgs
  nav_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_conversions pcl_ros tf tf2 tf2_sensor_msgs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###############################
##      声明可执行cpp文件     ##
###############################

add_executable(pointcloud_remap_using_tf_node src/pointcloud_remap_using_tf.cpp)
add_dependencies(pointcloud_remap_using_tf_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_remap_using_tf_node ${catkin_LIBRARIES})

add_executable(pointcloud_remap_directly_node src/pointcloud_remap_directly.cpp)
add_dependencies(pointcloud_remap_directly_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_remap_directly_node ${catkin_LIBRARIES})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(TARGETS pointcloud_remap_using_tf_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
