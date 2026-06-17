#!/bin/bash
# Sunray dependency check system

set -e

readonly CHECK_ENTRY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly CHECKSCRIPTS_DIR="$(cd "${CHECK_ENTRY_DIR}/.." && pwd)"
readonly SCRIPT_DIR="$(cd "${CHECKSCRIPTS_DIR}/../.." && pwd)"
readonly WORKSPACE_ROOT="$SCRIPT_DIR"

cd "$WORKSPACE_ROOT"

source "$CHECKSCRIPTS_DIR/common/utils.sh"
source "$CHECKSCRIPTS_DIR/common/config.sh"
source "$CHECKSCRIPTS_DIR/check/config.sh"
source "$CHECKSCRIPTS_DIR/check/runner.sh"
source "$CHECKSCRIPTS_DIR/check/resolver.sh"
source "$CHECKSCRIPTS_DIR/check/cli.sh"

check_tui_binary_matches_host_arch() {
    local tui_binary="$CHECKSCRIPTS_DIR/bin/sunray_check_tui"
    local binary_info host_arch
    [[ -f "$tui_binary" ]] || return 1
    binary_info="$(file -b "$tui_binary" 2>/dev/null || true)"
    host_arch="$(uname -m)"

    case "$host_arch" in
        x86_64|amd64)
            [[ "$binary_info" == *"x86-64"* || "$binary_info" == *"x86_64"* ]]
            ;;
        aarch64|arm64)
            [[ "$binary_info" == *"aarch64"* || "$binary_info" == *"ARM64"* ]]
            ;;
        armv7l|armv7*|armhf)
            [[ "$binary_info" == *"ARM"* && "$binary_info" != *"aarch64"* ]]
            ;;
        *)
            return 0
            ;;
    esac
}

build_check_tui_if_needed() {
    local tui_binary="$CHECKSCRIPTS_DIR/bin/sunray_check_tui"
    local tui_src_dir="$CHECKSCRIPTS_DIR/tui"
    local build_dir="$tui_src_dir/build"
    local build_stamp="$build_dir/.sunray_check_tui.stamp"
    local need_build=false

    if [[ ! -f "$tui_binary" ]]; then
        print_status "TUI程序不存在，开始编译..."
        need_build=true
    elif ! check_tui_binary_matches_host_arch; then
        print_warning "检测到TUI程序架构与当前系统不匹配，重新编译..."
        need_build=true
    elif [[ ! -f "$build_stamp" ]]; then
        print_status "TUI构建记录不存在，重新编译..."
        need_build=true
    else
        local newest_src
        newest_src="$(find "$tui_src_dir" -path "*/third_party" -prune -o \( -name "*.cpp" -o -name "*.hpp" -o -name "CMakeLists.txt" \) -newer "$build_stamp" -print | head -n 1)"
        if [[ -n "$newest_src" ]]; then
            print_status "检测到TUI源码更新，重新编译..."
            need_build=true
        fi
    fi

    if [[ "$need_build" == true ]]; then
        mkdir -p "$build_dir" || { print_error "无法创建构建目录: $build_dir"; exit 1; }
        cmake -S "$tui_src_dir" -B "$build_dir" >/dev/null
        cmake --build "$build_dir" --target sunray_check_tui -j"$(nproc)" >/dev/null
        touch "$build_stamp"
        print_status "TUI程序编译完成"
    fi

    [[ -x "$tui_binary" ]] || { print_error "TUI程序不可执行: $tui_binary"; exit 1; }
}

start_check_tui() {
    build_check_tui_if_needed
    exec "$CHECKSCRIPTS_DIR/bin/sunray_check_tui"
}

main() {
    case "${1:-}" in
        --version|-V)
            show_check_version
            exit 0
            ;;
        --help|-h)
            show_check_help
            exit 0
            ;;
        ""|--tui)
            start_check_tui
            ;;
        --from-tui)
            shift
            ;;
        --resolve-from-tui)
            CHECK_RESOLVE_MODE=true
            shift
            ;;
    esac

    init_config || exit 1

    parse_check_arguments "$@"
    if [[ "${CHECK_RESOLVE_MODE:-false}" == true ]]; then
        run_check_resolve_flow
    else
        run_check_flow
    fi
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
