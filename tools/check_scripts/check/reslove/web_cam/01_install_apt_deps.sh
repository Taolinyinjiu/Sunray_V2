#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

echo "[web_cam] 安装 apt 依赖"

sunray_apt_install \
    build-essential \
    cmake \
    libopencv-dev \
    libavcodec-dev \
    libavutil-dev \
    libswscale-dev \
    libv4l-dev
