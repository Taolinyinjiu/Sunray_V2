#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[sunray_msgs] 安装 ROS 消息依赖"

sunray_apt_install \
    ros-noetic-mavros \
    ros-noetic-mavros-msgs \
    ros-noetic-geographic-msgs
