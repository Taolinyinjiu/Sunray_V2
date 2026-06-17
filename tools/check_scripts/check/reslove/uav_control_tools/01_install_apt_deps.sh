#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[uav_control_tools] 安装 Qt5 依赖"

sunray_apt_install \
    qtbase5-dev
