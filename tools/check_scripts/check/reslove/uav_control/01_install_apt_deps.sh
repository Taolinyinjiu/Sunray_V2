#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[uav_control] 安装控制模块依赖"

sunray_apt_install \
    ros-noetic-mavros \
    ros-noetic-mavros-msgs \
    ros-noetic-geographic-msgs \
    libyaml-cpp-dev \
    libeigen3-dev
