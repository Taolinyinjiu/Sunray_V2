#!/bin/bash
set -e

echo "[livox_ros_driver2] 安装 apt 依赖"

if ! command -v apt-get >/dev/null 2>&1; then
    echo "当前系统没有 apt-get，请手动安装 Boost、PCL、apr-1 开发包。"
    exit 1
fi

sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    libapr1-dev \
    libboost-all-dev \
    libpcl-dev
