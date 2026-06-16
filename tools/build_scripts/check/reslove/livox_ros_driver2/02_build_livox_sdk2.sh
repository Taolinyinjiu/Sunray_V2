#!/bin/bash
set -e

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
sdk_dir="$repo_root/drivers/Livox_SDK2"
build_dir="$sdk_dir/build"

echo "[livox_ros_driver2] 构建并安装 Livox SDK2"

if [[ ! -d "$sdk_dir" ]]; then
    echo "找不到 Livox_SDK2: $sdk_dir"
    exit 1
fi

cmake -S "$sdk_dir" -B "$build_dir"
cmake --build "$build_dir" -j"$(nproc)"
sudo cmake --install "$build_dir"
sudo ldconfig
