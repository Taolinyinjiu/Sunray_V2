#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[gazebo_plugin] 安装 Gazebo 依赖"

sunray_apt_install \
    gazebo11 \
    libgazebo11-dev \
    ros-noetic-gazebo-dev
