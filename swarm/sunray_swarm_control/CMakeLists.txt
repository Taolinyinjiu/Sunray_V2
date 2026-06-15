cmake_minimum_required(VERSION 3.0.2)
project(sunray_swarm_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf
  tf2
  tf2_geometry_msgs
  sunray_msgs
)
find_package(Curses REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

catkin_package()

# ------------------------------------------------------------------------------
# 全局头文件路径
# - catkin_INCLUDE_DIRS: ROS / sunray_msgs 生成的消息头文件
# - common/sunray_log: Sunray 统一日志工具
# - formation、ORCA: swarm 内部纯算法模块头文件
# ------------------------------------------------------------------------------
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/../../common/sunray_log/
  ${PROJECT_SOURCE_DIR}/../../common/sunray_log/spdlog/include
  ${PROJECT_SOURCE_DIR}/swarm_control_uav
  ${PROJECT_SOURCE_DIR}/swarm_control_ugv
  ${PROJECT_SOURCE_DIR}/formation
  ${PROJECT_SOURCE_DIR}/ORCA
  ${PROJECT_SOURCE_DIR}/ORCA/orca_lib
)

# ------------------------------------------------------------------------------
# ORCA 原生 RVO2 内核源码
# 说明：
# - ORCA/orca_lib 是原生算法实现，不在业务代码中直接使用；
# - ORCA/ORCA.cpp 是我们封装后的统一 API 入口。
# ------------------------------------------------------------------------------
set(ORCA_CORE_SRCS
  ${PROJECT_SOURCE_DIR}/ORCA/orca_lib/Agent.cpp
  ${PROJECT_SOURCE_DIR}/ORCA/orca_lib/KdTree.cpp
  ${PROJECT_SOURCE_DIR}/ORCA/orca_lib/Obstacle.cpp
  ${PROJECT_SOURCE_DIR}/ORCA/orca_lib/RVOSimulator.cpp
)

# ------------------------------------------------------------------------------
# formation_lib
# 纯 C++ 阵型目标点生成库，不订阅/发布 ROS 话题。
# 输入 sunray_msgs/Formation，输出指定 agent 的目标 x/y/z/yaw。
# ------------------------------------------------------------------------------
add_library(formation_lib
  ${PROJECT_SOURCE_DIR}/formation/formation.cpp
)
add_dependencies(formation_lib ${catkin_EXPORTED_TARGETS})
target_include_directories(formation_lib PUBLIC
  ${PROJECT_SOURCE_DIR}/formation
)

# ------------------------------------------------------------------------------
# orca_api_lib
# ORCA 避障算法封装库，不订阅/发布 ROS 话题。
# 上层只通过 ORCA.h 设置 agent 状态/目标点并获取速度指令。
# ------------------------------------------------------------------------------
add_library(orca_api_lib
  ${PROJECT_SOURCE_DIR}/ORCA/ORCA.cpp
  ${ORCA_CORE_SRCS}
)
add_dependencies(orca_api_lib ${catkin_EXPORTED_TARGETS})
target_include_directories(orca_api_lib PUBLIC
  ${PROJECT_SOURCE_DIR}/ORCA
  ${PROJECT_SOURCE_DIR}/ORCA/orca_lib
)

# ------------------------------------------------------------------------------
# orca_test
# ORCA + formation 压力测试节点：
# - 使用 formation_lib 生成阵型目标点；
# - 使用 orca_api_lib 解算速度；
# - 不发布/订阅业务话题，仅输出测试报告。
# ------------------------------------------------------------------------------
add_executable(orca_test
  ${PROJECT_SOURCE_DIR}/ORCA/test/orca_test.cpp
)
add_dependencies(orca_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(orca_test
  orca_api_lib
  formation_lib
  ${catkin_LIBRARIES}
)

# ------------------------------------------------------------------------------
# swarm_control_uav_lib / swarm_control_uav_node
# UAV 集群控制核心：
# - swarm_control_uav_lib 封装状态机、话题订阅发布、formation/ORCA 调用；
# - swarm_control_uav_node 只是 ROS main 入口。
# ------------------------------------------------------------------------------
add_library(swarm_control_support INTERFACE)
target_link_libraries(swarm_control_support INTERFACE ${catkin_LIBRARIES})

add_library(swarm_control_uav_lib
  ${PROJECT_SOURCE_DIR}/swarm_control_uav/swarm_control_uav.cpp
)
add_dependencies(swarm_control_uav_lib ${catkin_EXPORTED_TARGETS})
target_include_directories(swarm_control_uav_lib PUBLIC
  ${PROJECT_SOURCE_DIR}/swarm_control_uav
  ${PROJECT_SOURCE_DIR}/formation
  ${PROJECT_SOURCE_DIR}/ORCA
)
target_link_libraries(swarm_control_uav_lib
  formation_lib
  orca_api_lib
  ${catkin_LIBRARIES}
)

add_executable(swarm_control_uav_node
  ${PROJECT_SOURCE_DIR}/swarm_control_uav/swarm_control_uav_node.cpp
)
add_dependencies(swarm_control_uav_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_control_uav_node swarm_control_uav_lib ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------
# swarm_control_ugv_lib / swarm_control_ugv_node
# UGV 集群控制核心：
# - swarm_control_ugv_lib 封装状态机、话题订阅发布、formation/ORCA 调用；
# - swarm_control_ugv_node 只是 ROS main 入口。
# ------------------------------------------------------------------------------
add_library(swarm_control_ugv_lib
  ${PROJECT_SOURCE_DIR}/swarm_control_ugv/swarm_control_ugv.cpp
)
add_dependencies(swarm_control_ugv_lib ${catkin_EXPORTED_TARGETS})
target_include_directories(swarm_control_ugv_lib PUBLIC
  ${PROJECT_SOURCE_DIR}/swarm_control_ugv
  ${PROJECT_SOURCE_DIR}/formation
  ${PROJECT_SOURCE_DIR}/ORCA
)
target_link_libraries(swarm_control_ugv_lib
  formation_lib
  orca_api_lib
  ${catkin_LIBRARIES}
)

add_executable(swarm_control_ugv_node
  ${PROJECT_SOURCE_DIR}/swarm_control_ugv/swarm_control_ugv_node.cpp
)
add_dependencies(swarm_control_ugv_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_control_ugv_node swarm_control_ugv_lib ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------
# 工具节点：终端/可视化/监控
# - uav_swarm_cmd_pub_terminal: 行式终端交互，发布 UAVSwarmCMD；
# - ugv_swarm_cmd_pub_terminal: 行式终端交互，发布 UGVSwarmCMD；
# - formation_tui: ncurses 图形终端，编辑/发布阵型指令；
# - rviz_visualization_uav_node: 将 UAVSwarmState 转为 RViz marker；
# - rviz_visualization_ugv_node: 将 UGVSwarmState 转为 RViz marker；
# - swarm_control_uav_monitor_node: 汇总打印多个 UAVSwarmState。
# - swarm_control_ugv_monitor_node: 汇总打印多个 UGVSwarmState。
# ------------------------------------------------------------------------------
add_executable(uav_swarm_cmd_pub_terminal
  ${PROJECT_SOURCE_DIR}/utils/uav_swarm_cmd_pub_terminal.cpp
)
add_dependencies(uav_swarm_cmd_pub_terminal ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav_swarm_cmd_pub_terminal ${catkin_LIBRARIES})

add_executable(ugv_swarm_cmd_pub_terminal
  ${PROJECT_SOURCE_DIR}/utils/ugv_swarm_cmd_pub_terminal.cpp
)
add_dependencies(ugv_swarm_cmd_pub_terminal ${catkin_EXPORTED_TARGETS})
target_link_libraries(ugv_swarm_cmd_pub_terminal ${catkin_LIBRARIES})

add_executable(formation_tui
  ${PROJECT_SOURCE_DIR}/utils/formation_tui.cpp
)
add_dependencies(formation_tui ${catkin_EXPORTED_TARGETS})
target_link_libraries(formation_tui
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)

add_executable(rviz_visualization_uav_node
  ${PROJECT_SOURCE_DIR}/utils/rviz_visualization_uav_node.cpp
)
add_dependencies(rviz_visualization_uav_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(rviz_visualization_uav_node ${catkin_LIBRARIES})

add_executable(rviz_visualization_ugv_node
  ${PROJECT_SOURCE_DIR}/utils/rviz_visualization_ugv_node.cpp
)
add_dependencies(rviz_visualization_ugv_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(rviz_visualization_ugv_node ${catkin_LIBRARIES})

add_executable(swarm_control_uav_monitor_node
  ${PROJECT_SOURCE_DIR}/utils/swarm_control_uav_monitor_node.cpp
)
add_dependencies(swarm_control_uav_monitor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_control_uav_monitor_node ${catkin_LIBRARIES})

add_executable(swarm_control_ugv_monitor_node
  ${PROJECT_SOURCE_DIR}/utils/swarm_control_ugv_monitor_node.cpp
)
add_dependencies(swarm_control_ugv_monitor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_control_ugv_monitor_node ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------
# Qt 地面站初版
# - swarm_control_panel_node: ROS + Qt Widgets 控制面板；
# - 发布 UAVSwarmCMD / UGVSwarmCMD；
# - 订阅 UAVSwarmState / UGVSwarmState 并在表格中显示关键状态。
# ------------------------------------------------------------------------------
add_executable(swarm_control_panel_node
  ${PROJECT_SOURCE_DIR}/utils/swarm_control_panel_node.cpp
)
add_dependencies(swarm_control_panel_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_control_panel_node
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

# ------------------------------------------------------------------------------
# 旧版 orca_engine 兼容入口
# 如果旧文件仍存在，则继续编译旧 swarm_uav/ugv 节点；
# 如果旧文件已被删除，则跳过，避免影响新版 ORCA/formation 架构。
# ------------------------------------------------------------------------------
set(LEGACY_ORCA_ENGINE_CPP ${PROJECT_SOURCE_DIR}/ORCA/orca_engine.cpp)
set(LEGACY_ORCA_ENGINE_H ${PROJECT_SOURCE_DIR}/ORCA/orca_engine.h)

if(EXISTS ${LEGACY_ORCA_ENGINE_CPP} AND EXISTS ${LEGACY_ORCA_ENGINE_H})
  add_library(orca_engine_lib
    ${LEGACY_ORCA_ENGINE_CPP}
    ${ORCA_CORE_SRCS}
  )
  add_dependencies(orca_engine_lib ${catkin_EXPORTED_TARGETS})
  target_include_directories(orca_engine_lib PUBLIC
    ${PROJECT_SOURCE_DIR}/ORCA
    ${PROJECT_SOURCE_DIR}/ORCA/orca_lib
  )

  add_executable(uav_swarm_node ${PROJECT_SOURCE_DIR}/swarm_control/src/swarm_uav_control_fsm.cpp)
  add_dependencies(uav_swarm_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(uav_swarm_node swarm_control_support orca_engine_lib ${catkin_LIBRARIES})

  add_executable(ugv_swarm_node ${PROJECT_SOURCE_DIR}/swarm_control/src/swarm_ugv_control_fsm.cpp)
  add_dependencies(ugv_swarm_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(ugv_swarm_node swarm_control_support orca_engine_lib ${catkin_LIBRARIES})
else()
  message(STATUS "Legacy orca_engine not found in ORCA; skip swarm_uav_control_fsm and swarm_ugv_control_fsm until the new ORCA API is integrated.")
endif()
