#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[super_planner] 安装 apt 依赖"

sunray_apt_install \
    build-essential \
    cmake \
    libyaml-cpp-dev \
    libboost-all-dev \
    libeigen3-dev \
    libpcl-dev \
    libncurses-dev \
    libdw-dev
