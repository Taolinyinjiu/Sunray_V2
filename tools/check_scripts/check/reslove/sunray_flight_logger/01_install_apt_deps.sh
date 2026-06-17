#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[sunray_flight_logger] 安装 MAVROS 依赖"

sunray_apt_install \
    ros-noetic-mavros \
    ros-noetic-mavros-msgs
