#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[vins_fusion_GPU] 安装 apt 依赖"

sunray_apt_install \
    build-essential \
    cmake \
    libopencv-dev \
    libceres-dev \
    libeigen3-dev \
    libboost-all-dev
