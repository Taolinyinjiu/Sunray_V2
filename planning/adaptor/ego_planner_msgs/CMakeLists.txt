cmake_minimum_required(VERSION 2.8.3)
project(ego_planner_msgs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  message_generation
)

# 查找Eigen包
find_package(Eigen3 REQUIRED)

# 包含路径
include_directories(
    ${PROJECT_SOURCE_DIR}/include  # 使用绝对路径指向本包的 include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# 添加要生成的msg消息
add_message_files(
  FILES
  AuxCommand.msg
  Corrections.msg
  Gains.msg
  OutputData.msg
  PositionCommand.msg
  PPROutputData.msg
  Serial.msg
  SO3Command.msg
  StatusData.msg
  TRPYCommand.msg
  Odometry.msg
  PolynomialTrajectory.msg
  LQRTrajectory.msg
)

# 生成消息
generate_messages(
    DEPENDENCIES
    geometry_msgs
    nav_msgs
)

# catkin导出配置
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES encode_msgs decode_msgs
  CATKIN_DEPENDS message_runtime geometry_msgs nav_msgs
)

message(STATUS "Current include dirs: ${catkin_INCLUDE_DIRS}")

# 生成库
add_library(decode_msgs src/decode_msgs.cpp)
add_library(encode_msgs src/encode_msgs.cpp)

# 建立依赖关系
add_dependencies(decode_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(encode_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 链接依赖
target_link_libraries(decode_msgs ${catkin_LIBRARIES})
target_link_libraries(encode_msgs ${catkin_LIBRARIES})
