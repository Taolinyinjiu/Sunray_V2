#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[turn_on_wheeltec_robot] 安装 serial 依赖"

sunray_apt_install \
    ros-noetic-serial
