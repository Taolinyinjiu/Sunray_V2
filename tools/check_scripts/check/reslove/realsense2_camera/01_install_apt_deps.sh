#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[realsense2_camera] 安装 ROS/OpenCV 依赖"

sunray_apt_install \
    libopencv-dev \
    ros-noetic-ddynamic-reconfigure
