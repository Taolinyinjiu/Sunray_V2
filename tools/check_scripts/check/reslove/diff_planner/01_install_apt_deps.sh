#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[diff_planner] 安装 apt 依赖"

sunray_apt_install \
    build-essential \
    cmake \
    libopencv-dev \
    libpcl-dev \
    libeigen3-dev \
    libboost-all-dev
