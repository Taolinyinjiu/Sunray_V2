#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[ugv_control] 安装 Eigen3/Qt5 依赖"

sunray_apt_install \
    libeigen3-dev \
    qtbase5-dev
