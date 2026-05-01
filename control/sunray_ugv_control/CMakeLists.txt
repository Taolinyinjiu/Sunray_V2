cmake_minimum_required(VERSION 3.0.2)
project(sunray_ugv_control)

add_compile_options(-std=c++11)

# ------------------------------------------------------------------------------
# 依赖
# - roscpp: ROS C++ 节点
# - sunray_msgs: UGVControlCMD / UGVControlFSMState 等消息
# - geometry_msgs / nav_msgs / std_msgs: 控制指令、里程计和基础消息
# - visualization_msgs: RViz marker 可视化
# - Eigen3: 控制器内部向量计算
# ------------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  sunray_msgs
)
find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

catkin_package(
  INCLUDE_DIRS include src/controller
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs visualization_msgs sunray_msgs
  DEPENDS EIGEN3
)

# ------------------------------------------------------------------------------
# 头文件路径
# - include: ugv_control_fsm.h
# - src/controller: UGVController / DifferentialController / MecanumController
# ------------------------------------------------------------------------------
include_directories(
  include
  src/controller
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------
# sunray_ugv_control 库
# 编译内容：
# - src/controller/ugv_controller.cpp: 控制器基类，负责参数读取、状态缓存、状态发布
# - src/controller/differential_controller.cpp: 差速底盘控制器
# - src/controller/mecanum_controller.cpp: 麦克纳姆底盘控制器
# - src/ugv_control_fsm.cpp: UGV 控制状态机，负责话题订阅发布和控制器调度
# ------------------------------------------------------------------------------
add_library(${PROJECT_NAME}
  src/controller/ugv_controller.cpp
  src/controller/differential_controller.cpp
  src/controller/mecanum_controller.cpp
  src/ugv_control_fsm.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# ------------------------------------------------------------------------------
# node
# ROS 入口节点：
# - 源码: src/ugv_control_node.cpp
# - 运行文件: devel/lib/sunray_ugv_control/node
# - roslaunch: sunray_ugv_control/launch/ugv_control.launch
# ------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node
  src/ugv_control_node.cpp
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# ------------------------------------------------------------------------------
# 工具节点
# - ugv_control_monitor_node: 订阅 UGVControlFSMState 并在终端打印状态面板
# - rviz_visualization_ugv_control_node: 订阅 UGVControlFSMState 并发布 RViz marker
# - ugv_control_panel_node: Qt 控制面板，发布 UGVControlCMD 并显示 UGVControlFSMState
# ------------------------------------------------------------------------------
add_executable(ugv_control_monitor_node
  src/tools/ugv_control_monitor_node.cpp
)
add_dependencies(ugv_control_monitor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ugv_control_monitor_node
  ${catkin_LIBRARIES}
)

add_executable(rviz_visualization_ugv_control_node
  src/tools/rviz_visualization_ugv_control_node.cpp
)
add_dependencies(rviz_visualization_ugv_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(rviz_visualization_ugv_control_node
  ${catkin_LIBRARIES}
)

add_executable(ugv_control_panel_node
  src/tools/ugv_control_panel_node.cpp
)
add_dependencies(ugv_control_panel_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ugv_control_panel_node
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

# ------------------------------------------------------------------------------
# 安装
# - node: ROS 可执行节点
# - library: 控制库
# - headers: 当前包需要导出的头文件
# - launch/config: 启动文件和参数文件
# ------------------------------------------------------------------------------
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  ugv_control_monitor_node
  rviz_visualization_ugv_control_node
  ugv_control_panel_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  include/ugv_control_fsm.h
  src/controller/ugv_controller.h
  src/controller/differential_controller.h
  src/controller/mecanum_controller.h
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/ config/ rviz/ meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch" PATTERN "*.yaml" PATTERN "*.rviz" PATTERN "*.dae"
)
