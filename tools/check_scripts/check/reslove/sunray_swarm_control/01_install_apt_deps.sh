#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[sunray_swarm_control] 安装 ncurses/Qt5 依赖"

sunray_apt_install \
    libncurses-dev \
    qtbase5-dev
