#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[livox_ros_driver2] 安装 apt 依赖"

sunray_apt_install \
    build-essential \
    cmake \
    libapr1-dev \
    libboost-all-dev \
    libpcl-dev
