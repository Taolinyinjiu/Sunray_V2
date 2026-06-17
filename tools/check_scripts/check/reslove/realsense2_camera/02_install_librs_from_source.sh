#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
sdk_dir="$repo_root/drivers/librealsense"
build_dir="$sdk_dir/build"

echo "[realsense2_camera] 构建并安装仓库内 librealsense"

if [[ ! -d "$sdk_dir" ]]; then
    echo "找不到 librealsense 源码: $sdk_dir" >&2
    exit 1
fi

sunray_apt_install \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    at

sunray_run cmake -S "$sdk_dir" -B "$build_dir" \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_WITH_CUDA=OFF
sunray_run cmake --build "$build_dir" -j"$(nproc)"
sunray_run sudo cmake --install "$build_dir"
sunray_run sudo ldconfig
