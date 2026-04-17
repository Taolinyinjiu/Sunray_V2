cmake_minimum_required(VERSION 3.0.2)
project(sunray_swarm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
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

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/include
  ${PROJECT_SOURCE_DIR}/ORCA/include
)

set(AGENT_SWARM_SRCS
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/agent_swarm_node.cpp
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/formation_state_machine.cpp
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/leader_tracker.cpp
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/formation_policies.cpp
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/formation_policy_factory.cpp
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/agent_state_cache.cpp
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/goal_dispatcher.cpp
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/orca_client.cpp
  ${PROJECT_SOURCE_DIR}/Agent_Swarm/src/control_command_mapper.cpp
)

add_executable(agent_swarm_node ${AGENT_SWARM_SRCS})
add_dependencies(agent_swarm_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(agent_swarm_node ${catkin_LIBRARIES})

add_executable(uav_command_pub ${PROJECT_SOURCE_DIR}/Agent_Swarm/utils/formation_switch.cpp)
add_dependencies(uav_command_pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav_command_pub ${catkin_LIBRARIES})

add_executable(formation_tui ${PROJECT_SOURCE_DIR}/Agent_Swarm/utils/formation_tui.cpp)
add_dependencies(formation_tui ${catkin_EXPORTED_TARGETS})
target_link_libraries(formation_tui ${catkin_LIBRARIES} ${CURSES_LIBRARIES})

set(ORCA_RVO_SRCS
  ${PROJECT_SOURCE_DIR}/ORCA/src/Agent.cpp
  ${PROJECT_SOURCE_DIR}/ORCA/src/KdTree.cpp
  ${PROJECT_SOURCE_DIR}/ORCA/src/Obstacle.cpp
  ${PROJECT_SOURCE_DIR}/ORCA/src/RVOSimulator.cpp
)

set(ORCA_SRCS
  ${PROJECT_SOURCE_DIR}/ORCA/src/orca_node.cpp
  ${PROJECT_SOURCE_DIR}/ORCA/src/orca_engine.cpp
  ${PROJECT_SOURCE_DIR}/ORCA/src/orca_io.cpp
  ${PROJECT_SOURCE_DIR}/ORCA/src/obstacle_builder.cpp
  ${ORCA_RVO_SRCS}
)

add_executable(orca_node ${ORCA_SRCS})
add_dependencies(orca_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(orca_node ${catkin_LIBRARIES})
