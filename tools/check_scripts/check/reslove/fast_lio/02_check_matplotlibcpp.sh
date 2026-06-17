#!/bin/bash
set -e

source "$(dirname "${BASH_SOURCE[0]}")/../common.sh"

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
header_path="$repo_root/localization/third_party_localization/fast_lio/include/matplotlibcpp.h"

echo "[fast_lio] 检查 matplotlibcpp.h"

if [[ -f "$header_path" ]]; then
    echo "已找到: $header_path"
    exit 0
fi

echo "找不到 matplotlibcpp.h: $header_path" >&2
echo "请将 matplotlibcpp.h 放入 fast_lio/include，或安装到编译器 include 路径。" >&2
exit 1
