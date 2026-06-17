#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[localization_fusion] 安装 yaml-cpp 依赖"

sunray_apt_install \
    libyaml-cpp-dev
