cmake_minimum_required(VERSION 3.10)
project(px4_param_tui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp mavros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../common/px4_bridge/include
)

set(FTXUI_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/build_scripts/tui/third_party/ftxui")
if(NOT EXISTS "${FTXUI_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "Bundled FTXUI not found: ${FTXUI_SOURCE_DIR}")
endif()

if(NOT TARGET ftxui::screen)
  add_subdirectory("${FTXUI_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/ftxui")
endif()

add_executable(px4_param_tui_node
  src/px4_param_tui_node.cpp
  ../../common/px4_bridge/src/px4_param_manager.cpp
)

target_link_libraries(px4_param_tui_node
  ${catkin_LIBRARIES}
  ftxui::screen
  ftxui::dom
  ftxui::component
)
