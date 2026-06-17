#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[sunray_simulator] 安装 Boost 依赖"

sunray_apt_install \
    libboost-all-dev
