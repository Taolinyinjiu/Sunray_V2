cmake_minimum_required(VERSION 3.0.2)
project(sunray_perception)

add_compile_options(-std=c++14)

option(WITH_RKNN "Build RKNN backend" OFF)
option(WITH_TENSORRT "Build TensorRT backend" OFF)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roslib
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)
find_package(yaml-cpp REQUIRED)

if(WITH_RKNN)
  find_path(RKNN_INCLUDE_DIR rknn_api.h
    PATHS
      /usr/include
      /usr/local/include
      /usr/local/rknn/include
      /usr/local/RKNN/include
  )
  find_library(RKNN_RUNTIME_LIBRARY rknnrt
    PATHS
      /usr/lib
      /usr/local/lib
      /usr/local/rknn/lib
      /usr/local/RKNN/lib
  )
  if(NOT RKNN_INCLUDE_DIR OR NOT RKNN_RUNTIME_LIBRARY)
    message(FATAL_ERROR "WITH_RKNN=ON requires rknn_api.h and librknnrt")
  endif()
endif()

if(WITH_TENSORRT)
  file(GLOB CUDA_ROOT_CANDIDATES
    /usr/local/cuda
    /usr/local/cuda-*
  )
  find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    PATHS
      /usr/include
      /usr/include/aarch64-linux-gnu
      /usr/local/include
      /usr/local/cuda/include
  )
  find_path(CUDA_RUNTIME_INCLUDE_DIR cuda_runtime_api.h
    HINTS
      ${CUDA_ROOT_CANDIDATES}
    PATHS
      /usr/include
      /usr/include/aarch64-linux-gnu
    PATH_SUFFIXES
      include
      targets/aarch64-linux/include
      targets/x86_64-linux/include
  )
  find_library(TENSORRT_LIBRARY nvinfer
    PATHS
      /usr/lib
      /usr/lib/aarch64-linux-gnu
      /usr/local/lib
      /usr/local/cuda/lib64
  )
  find_library(CUDART_LIBRARY cudart
    HINTS
      ${CUDA_ROOT_CANDIDATES}
    PATHS
      /usr/lib
      /usr/lib/aarch64-linux-gnu
      /usr/local/cuda/lib64
    PATH_SUFFIXES
      lib64
      lib
      targets/aarch64-linux/lib
      targets/x86_64-linux/lib
  )
  if(NOT TENSORRT_INCLUDE_DIR OR NOT CUDA_RUNTIME_INCLUDE_DIR OR NOT TENSORRT_LIBRARY OR NOT CUDART_LIBRARY)
    message(FATAL_ERROR "WITH_TENSORRT=ON requires NvInfer.h, cuda_runtime_api.h, libnvinfer, and libcudart")
  endif()
endif()

add_message_files(
  FILES
  Detection.msg
  DetectionArray.msg
  Tracking.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    cv_bridge
    image_transport
    message_runtime
    roslib
    roscpp
    sensor_msgs
    std_msgs
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_core
  src/common/config_utils.cpp
  src/detection/detectors.cpp
  src/detection/npu_backend.cpp
  src/detection/npu_backend_rknn.cpp
  src/detection/npu_backend_tensorrt.cpp
  src/tracking/trackers.cpp
)
add_dependencies(${PROJECT_NAME}_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
if(WITH_RKNN)
  target_compile_definitions(${PROJECT_NAME}_core PRIVATE SUNRAY_WITH_RKNN)
  target_include_directories(${PROJECT_NAME}_core PRIVATE ${RKNN_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME}_core ${RKNN_RUNTIME_LIBRARY})
endif()
if(WITH_TENSORRT)
  target_compile_definitions(${PROJECT_NAME}_core PRIVATE SUNRAY_WITH_TENSORRT)
  target_include_directories(${PROJECT_NAME}_core PRIVATE ${TENSORRT_INCLUDE_DIR} ${CUDA_RUNTIME_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME}_core ${TENSORRT_LIBRARY} ${CUDART_LIBRARY})
endif()
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

add_executable(detection_node src/detection_node.cpp)
add_dependencies(detection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(detection_node
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

add_executable(tracking_node src/tracking_node.cpp)
add_dependencies(tracking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracking_node
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

install(TARGETS ${PROJECT_NAME}_core detection_node tracking_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config models aruco
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
