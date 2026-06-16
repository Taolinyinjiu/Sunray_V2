#!/bin/bash
# Sunray dependency check entrypoint

set -e

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly CHECK_MAIN="$SCRIPT_DIR/tools/build_scripts/check/main.sh"
export SUNRAY_CHECK_SCRIPT_NAME="$(basename "$0")"

if [[ ! -x "$CHECK_MAIN" ]]; then
    echo "依赖检查系统未找到或不可执行: $CHECK_MAIN"
    echo "请确认 tools/build_scripts/check/main.sh 存在"
    exit 1
fi

exec "$CHECK_MAIN" "$@"
