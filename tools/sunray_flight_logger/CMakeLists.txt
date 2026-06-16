cmake_minimum_required(VERSION 3.12)
project(sunray_flight_logger)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  mavros_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    roslib
    mavros_msgs
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

add_executable(sunray_flight_logger_tui
  src/sunray_flight_logger_tui.cpp
)

add_dependencies(sunray_flight_logger_tui
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(sunray_flight_logger_tui
  ${catkin_LIBRARIES}
  ftxui::screen
  ftxui::dom
  ftxui::component
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
  target_link_libraries(sunray_flight_logger_tui stdc++fs)
endif()

install(TARGETS sunray_flight_logger_tui
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
