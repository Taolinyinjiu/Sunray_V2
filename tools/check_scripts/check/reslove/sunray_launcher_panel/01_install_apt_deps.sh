#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[sunray_launcher_panel] 安装 Qt5 依赖"

sunray_apt_install \
    qtbase5-dev
