#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[rog_map] 安装 apt 依赖"

sunray_apt_install \
    build-essential \
    cmake \
    libyaml-cpp-dev \
    libeigen3-dev \
    libpcl-dev \
    libdw-dev
