#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[planner_tools] 安装 PCL 依赖"

sunray_apt_install \
    libpcl-dev
