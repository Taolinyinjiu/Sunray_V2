#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[vrpn_client_ros] 安装 VRPN 依赖"

sunray_apt_install \
    ros-noetic-vrpn
