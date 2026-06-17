#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[yunlink_ros_bridge] 安装 ROS 消息依赖"

sunray_apt_install \
    ros-noetic-geographic-msgs
