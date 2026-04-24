cmake_minimum_required(VERSION 3.0.2)
project(sunray_swarm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  tf2
  tf2_geometry_msgs
  sunray_msgs
)
find_package(Curses REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/include
  ${PROJECT_SOURCE_DIR}/ORCA/include
)

set(AGENT_SWARM_SUPPORT_SRCS
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/formation_state_machine.cpp
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/leader_tracker.cpp
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/formation_policies.cpp
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/formation_policy_factory.cpp
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/agent_state_cache.cpp
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/goal_dispatcher.cpp
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/control_command_mapper.cpp
)

set(ORCA_RVO_SRCS
  ${PROJECT_SOURCE_DIR}/ORCA/src/Agent.cpp
  ${PROJECT_SOURCE_DIR}/ORCA/src/KdTree.cpp
  ${PROJECT_SOURCE_DIR}/ORCA/src/Obstacle.cpp
  ${PROJECT_SOURCE_DIR}/ORCA/src/RVOSimulator.cpp
)

set(ORCA_LIB_SRCS
  ${PROJECT_SOURCE_DIR}/ORCA/src/orca_engine.cpp
  ${PROJECT_SOURCE_DIR}/ORCA/src/obstacle_builder.cpp
  ${ORCA_RVO_SRCS}
)

add_library(orca_engine_lib ${ORCA_LIB_SRCS})
add_dependencies(orca_engine_lib ${catkin_EXPORTED_TARGETS})
target_include_directories(orca_engine_lib PUBLIC ${PROJECT_SOURCE_DIR}/ORCA/include)

add_library(agent_swarm_support ${AGENT_SWARM_SUPPORT_SRCS})
add_dependencies(agent_swarm_support ${catkin_EXPORTED_TARGETS})
target_link_libraries(agent_swarm_support orca_engine_lib ${catkin_LIBRARIES})

add_executable(uav_swarm_node ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/swarm_uav_control_fsm.cpp)
add_dependencies(uav_swarm_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav_swarm_node agent_swarm_support orca_engine_lib ${catkin_LIBRARIES})

add_executable(ugv_swarm_node ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/swarm_ugv_control_fsm.cpp)
add_dependencies(ugv_swarm_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ugv_swarm_node agent_swarm_support orca_engine_lib ${catkin_LIBRARIES})

add_executable(uav_command_pub ${PROJECT_SOURCE_DIR}/Agent_Swarm/utils/formation_switch.cpp)
add_dependencies(uav_command_pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav_command_pub ${catkin_LIBRARIES})

add_executable(formation_tui ${PROJECT_SOURCE_DIR}/Agent_Swarm/utils/formation_tui.cpp)
add_dependencies(formation_tui ${catkin_EXPORTED_TARGETS})
target_link_libraries(formation_tui ${catkin_LIBRARIES} ${CURSES_LIBRARIES})
