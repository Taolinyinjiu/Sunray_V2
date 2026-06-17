#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[simulator_utils] 安装 apt 依赖"

sunray_apt_install \
    build-essential \
    cmake \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev \
    libboost-all-dev \
    libarmadillo-dev
