#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[fast_lio] 安装 apt 依赖"

sunray_apt_install \
    build-essential \
    cmake \
    libeigen3-dev \
    libpcl-dev \
    python3-dev
