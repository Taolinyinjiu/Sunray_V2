#!/bin/bash
# Common helpers for Sunray dependency resolver scripts.

set -e

sunray_resolve_is_dry_run() {
    [[ "${SUNRAY_DRY_RUN:-false}" == "true" || "${SUNRAY_DRY_RUN:-0}" == "1" ]]
}

sunray_resolve_yes_flag() {
    if [[ "${SUNRAY_ASSUME_YES:-false}" == "true" || "${SUNRAY_ASSUME_YES:-0}" == "1" ]]; then
        echo "-y"
    else
        echo "-y"
    fi
}

sunray_run() {
    echo "+ $*"
    if sunray_resolve_is_dry_run; then
        return 0
    fi
    "$@"
}

sunray_require_command() {
    local command_name="$1"
    if ! command -v "$command_name" >/dev/null 2>&1; then
        echo "缺少命令: $command_name" >&2
        return 1
    fi
}

sunray_truthy() {
    local value="${1:-}"
    [[ "$value" == "true" || "$value" == "1" || "$value" == "yes" || "$value" == "on" ]]
}

sunray_apt_package_installed() {
    local package_name="$1"
    dpkg-query -W -f='${Status}' "$package_name" 2>/dev/null | grep -q "install ok installed"
}

sunray_apt_update_once() {
    local stamp_file="${SUNRAY_APT_UPDATE_STAMP:-${TMPDIR:-/tmp}/sunray_dependency_apt_update.${PPID}}"

    if [[ -f "$stamp_file" ]]; then
        return 0
    fi

    mkdir -p "$(dirname "$stamp_file")"
    if sunray_truthy "${SUNRAY_SKIP_APT_UPDATE:-false}"; then
        echo "[状态] 跳过 sudo apt-get update: SUNRAY_SKIP_APT_UPDATE=true"
        : > "$stamp_file"
        return 0
    fi

    echo "+ sudo apt-get update"
    if sunray_resolve_is_dry_run; then
        : > "$stamp_file"
        return 0
    fi

    local update_status=0
    if sudo apt-get update; then
        : > "$stamp_file"
        return 0
    else
        update_status=$?
    fi

    if sunray_truthy "${SUNRAY_STRICT_APT_UPDATE:-false}"; then
        return "$update_status"
    fi

    echo "[警告] sudo apt-get update 失败，继续尝试 apt-get install。" >&2
    echo "[警告] 如果后续安装失败，请先修复系统 apt 源后重试。" >&2
    : > "$stamp_file"
    return 0
}

sunray_apt_install() {
    sunray_require_command apt-get
    sunray_require_command dpkg-query

    local missing_packages=()
    local package_name

    for package_name in "$@"; do
        if ! sunray_apt_package_installed "$package_name"; then
            missing_packages+=("$package_name")
        fi
    done

    if [[ ${#missing_packages[@]} -eq 0 ]]; then
        echo "[状态] apt 依赖已安装，跳过: $*"
        return 0
    fi

    echo "[状态] 需要安装 apt 包: ${missing_packages[*]}"
    sunray_apt_update_once
    sunray_run sudo apt-get install -y "${missing_packages[@]}"
}

sunray_repo_root_from_resolver() {
    cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd
}
