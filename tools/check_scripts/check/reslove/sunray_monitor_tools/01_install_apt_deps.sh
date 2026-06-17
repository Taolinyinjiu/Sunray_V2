#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[sunray_monitor_tools] 安装 MAVROS/Qt5 依赖"

sunray_apt_install \
    ros-noetic-mavros \
    ros-noetic-mavros-msgs \
    qtbase5-dev
