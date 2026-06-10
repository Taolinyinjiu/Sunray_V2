cmake_minimum_required(VERSION 2.8.3)
project(open3d_loc)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
SET(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  message_generation
  eigen_conversions
)

set(Open3D_ROOT "" CACHE PATH "Open3D install prefix, for example /home/amov/open3d_install")
set(LIBCXX_ROOT "" CACHE PATH "Optional libc++ prefix for Open3D prebuilt packages")
if(NOT Open3D_DIR AND DEFINED ENV{Open3D_DIR})
  set(Open3D_DIR "$ENV{Open3D_DIR}" CACHE PATH "Open3D CMake package directory")
endif()
if(Open3D_ROOT)
  list(APPEND CMAKE_PREFIX_PATH
    "${Open3D_ROOT}"
    "${Open3D_ROOT}/lib/cmake"
    "${Open3D_ROOT}/lib/cmake/Open3D")
elseif(DEFINED ENV{Open3D_ROOT})
  list(APPEND CMAKE_PREFIX_PATH
    "$ENV{Open3D_ROOT}"
    "$ENV{Open3D_ROOT}/lib/cmake"
    "$ENV{Open3D_ROOT}/lib/cmake/Open3D")
endif()

find_package(Open3D REQUIRED)
if(TARGET Open3D::Open3D)
  set(OPEN3D_LINK_TARGET Open3D::Open3D)
else()
  set(OPEN3D_LINK_TARGET ${Open3D_LIBRARIES})
endif()
find_library(LIBCXX_LIBRARY c++)
find_library(LIBCXXABI_LIBRARY c++abi)
if(LIBCXX_ROOT AND NOT LIBCXX_LIBRARY)
  find_library(LIBCXX_LIBRARY c++ PATHS
    "${LIBCXX_ROOT}/usr/lib/x86_64-linux-gnu"
    "${LIBCXX_ROOT}/usr/lib/llvm-10/lib"
    NO_DEFAULT_PATH)
endif()
if(LIBCXX_ROOT AND NOT LIBCXXABI_LIBRARY)
  find_library(LIBCXXABI_LIBRARY c++abi PATHS
    "${LIBCXX_ROOT}/usr/lib/x86_64-linux-gnu"
    "${LIBCXX_ROOT}/usr/lib/llvm-10/lib"
    NO_DEFAULT_PATH)
endif()
if(LIBCXX_LIBRARY)
  list(APPEND OPEN3D_LINK_TARGET ${LIBCXX_LIBRARY})
endif()
if(LIBCXXABI_LIBRARY)
  list(APPEND OPEN3D_LINK_TARGET ${LIBCXXABI_LIBRARY})
endif()
if(LIBCXX_ROOT)
  link_directories(
    "${LIBCXX_ROOT}/usr/lib/x86_64-linux-gnu"
    "${LIBCXX_ROOT}/usr/lib/llvm-10/lib")
endif()

include_directories(
        ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}
  ${Open3D_INCLUDE_DIRS}
  include
)


catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS
  INCLUDE_DIRS
)

add_library(open3d_registration src/open3d_registration/open3d_registration.cpp)
target_link_libraries(open3d_registration ${OPEN3D_LINK_TARGET})
add_library(open3d_conversions src/open3d_conversions/open3d_conversions.cpp)
target_link_libraries(open3d_conversions ${OPEN3D_LINK_TARGET})


add_executable(global_localization_node src/global_localization.cpp)
target_link_libraries(global_localization_node ${OPEN3D_LINK_TARGET})
target_link_libraries(global_localization_node open3d_registration)
target_link_libraries(global_localization_node open3d_conversions)
target_link_libraries(global_localization_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# 安装可执行文件
install(TARGETS open3d_registration
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS open3d_conversions
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS global_localization_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY rviz_cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
