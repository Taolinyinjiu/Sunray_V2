cmake_minimum_required(VERSION 3.12)
project(scripts_manage)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sunray_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sunray_msgs
)

set(FTXUI_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(FTXUI_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(FTXUI_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FTXUI_BUILD_TESTS_FUZZER OFF CACHE BOOL "" FORCE)
set(FTXUI_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)

set(SUNRAY_FTXUI_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../build_scripts/tui/third_party/ftxui"
)
if(NOT EXISTS "${SUNRAY_FTXUI_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "FTXUI dependency is missing: ${SUNRAY_FTXUI_DIR}. "
    "Run tools/build_scripts/tui/check_dependencies.sh first."
  )
endif()

if(NOT TARGET ftxui::component)
  add_subdirectory(
    "${SUNRAY_FTXUI_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/ftxui"
    EXCLUDE_FROM_ALL
  )
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(scripts_manage_tui
  src/scripts_manage_tui.cpp
)

add_dependencies(scripts_manage_tui
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(scripts_manage_tui
  ${catkin_LIBRARIES}
  ftxui::screen
  ftxui::dom
  ftxui::component
)

install(TARGETS scripts_manage_tui
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
