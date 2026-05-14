cmake_minimum_required(VERSION 3.0.2)
project(sunray_launcher_panel)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 通用 Sunray 启动器面板：
# - 从 config/sunray_launcher.yaml 读取可启动的 ROS launch 列表。
# - 每个 launch 在独立 terminal 中运行，便于查看原始输出。
# - 支持通过面板停止对应 roslaunch 进程组。
add_executable(sunray_launcher_node
  src/tools/sunray_launcher_node.cpp
)
add_dependencies(sunray_launcher_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(sunray_launcher_node
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

install(TARGETS sunray_launcher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY logo/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/logo
  FILES_MATCHING
    PATTERN "*.png"
    PATTERN "*.jpg"
    PATTERN "*.jpeg"
)
