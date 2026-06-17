#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[open3d_loc] 安装可通过 apt 获取的依赖"

sunray_apt_install \
    libc++-dev \
    libc++abi-dev

echo "Open3D C++ SDK 不在 Ubuntu 20.04 默认 apt 源中稳定提供。"
echo "请手动安装 Open3D C++，并设置 Open3D_DIR 或 Open3D_ROOT。"
