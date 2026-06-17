#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[ekf_odometry] 安装 Eigen3 依赖"

sunray_apt_install \
    libeigen3-dev
