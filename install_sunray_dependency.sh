#!/bin/bash
# Sunray dependency installer entrypoint.

set -e

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly CHECK_SCRIPT="$SCRIPT_DIR/check.sh"
readonly MENU_FILE="$SCRIPT_DIR/tools/check_scripts/install_menu.yaml"

DRY_RUN=false
ASSUME_YES=false
VERIFY_AFTER=false
SKIP_APT_UPDATE=false
STRICT_APT_UPDATE=false
LIST_MENU=false
LIST_MODULES=false
SELECTED_TOKENS=()
DIRECT_TARGETS=()
SUDO_KEEPALIVE_PID=""

MENU_IDS=()
MENU_TITLES=()
MENU_TARGET_TYPES=()
MENU_TARGETS=()
MENU_DESCRIPTIONS=()

print_usage() {
    cat << EOF
Sunray dependency installer

Usage:
  ./install_sunray_dependency.sh
  ./install_sunray_dependency.sh [options] [menu-id...]

Options:
  -h, --help             Show this help
  -l, --list             List installer menu entries
  --list-modules         Run ./check.sh --list
  --dry-run              Show the commands without installing dependencies
  -y, --yes              Skip confirmation prompts
  --verify               Run ./check.sh for selected targets after resolve
  --skip-apt-update      Do not run apt-get update before installing packages
  --strict-apt-update    Treat apt-get update failures as fatal
  --module <name>        Resolve dependencies for one module
  --group <name>         Resolve dependencies for one module group

Examples:
  ./install_sunray_dependency.sh
  ./install_sunray_dependency.sh --dry-run 4
  ./install_sunray_dependency.sh --skip-apt-update 1
  ./install_sunray_dependency.sh --yes 2 4
  ./install_sunray_dependency.sh --module livox_ros_driver2
  ./install_sunray_dependency.sh --group S_150_Lidar

EOF
}

die() {
    echo "[错误] $*" >&2
    exit 1
}

status() {
    echo "[状态] $*"
}

warning() {
    echo "[警告] $*" >&2
}

quote_cmd() {
    local arg
    for arg in "$@"; do
        printf ' %q' "$arg"
    done
    printf '\n'
}

cleanup_sudo_keepalive() {
    if [[ -n "$SUDO_KEEPALIVE_PID" ]]; then
        kill "$SUDO_KEEPALIVE_PID" >/dev/null 2>&1 || true
    fi
}

preheat_sudo() {
    [[ "$DRY_RUN" == true ]] && return 0
    [[ ${#DIRECT_TARGETS[@]} -eq 0 ]] && return 0

    if [[ ${EUID:-$(id -u)} -eq 0 ]]; then
        warning "当前脚本以 root 身份运行。建议普通用户启动，让 resolver 脚本按需 sudo。"
        return 0
    fi

    command -v sudo >/dev/null 2>&1 || die "当前系统没有 sudo，无法执行需要提权的依赖安装。"

    status "依赖安装可能需要 sudo 权限，请输入密码以预热 sudo 会话。"
    sudo -v || die "sudo 认证失败，已取消依赖安装。"

    while true; do
        sudo -n true >/dev/null 2>&1 || exit
        sleep 60
        kill -0 "$$" >/dev/null 2>&1 || exit
    done 2>/dev/null &
    SUDO_KEEPALIVE_PID=$!
    trap cleanup_sudo_keepalive EXIT
}

clean_yaml_value() {
    local value="$1"
    value="${value%%#*}"
    value="${value#"${value%%[![:space:]]*}"}"
    value="${value%"${value##*[![:space:]]}"}"
    value="${value%\"}"
    value="${value#\"}"
    value="${value%\'}"
    value="${value#\'}"
    printf '%s' "$value"
}

load_menu() {
    [[ -f "$MENU_FILE" ]] || die "菜单配置不存在: $MENU_FILE"

    local line id title target_type target description
    id="" title="" target_type="" target="" description=""

    flush_menu_item() {
        [[ -n "$id" ]] || return 0
        MENU_IDS+=("$id")
        MENU_TITLES+=("$title")
        MENU_TARGET_TYPES+=("$target_type")
        MENU_TARGETS+=("$target")
        MENU_DESCRIPTIONS+=("$description")
    }

    while IFS= read -r line || [[ -n "$line" ]]; do
        case "$line" in
            "  - id:"*)
                flush_menu_item
                id="$(clean_yaml_value "${line#  - id:}")"
                title=""
                target_type=""
                target=""
                description=""
                ;;
            "    title:"*)
                title="$(clean_yaml_value "${line#    title:}")"
                ;;
            "    target_type:"*)
                target_type="$(clean_yaml_value "${line#    target_type:}")"
                ;;
            "    target:"*)
                target="$(clean_yaml_value "${line#    target:}")"
                ;;
            "    description:"*)
                description="$(clean_yaml_value "${line#    description:}")"
                ;;
        esac
    done < "$MENU_FILE"
    flush_menu_item

    [[ ${#MENU_IDS[@]} -gt 0 ]] || die "菜单配置为空: $MENU_FILE"
}

show_menu() {
    local i
    echo
    echo "Sunray 依赖安装工具"
    echo
    echo "该工具只解决 Sunray 模块额外依赖；基础 ROS Noetic 环境仍需提前安装。"
    echo "部分操作会使用 sudo 安装 apt 包或写入 /usr/local。"
    echo
    for i in "${!MENU_IDS[@]}"; do
        printf "%2s. %-34s %s\n" "${MENU_IDS[$i]}" "${MENU_TITLES[$i]}" "${MENU_DESCRIPTIONS[$i]}"
    done
    echo " 0. 退出"
    echo
}

list_menu() {
    show_menu
}

find_menu_index_by_id() {
    local wanted="$1"
    local i
    for i in "${!MENU_IDS[@]}"; do
        if [[ "${MENU_IDS[$i]}" == "$wanted" ]]; then
            printf '%s' "$i"
            return 0
        fi
    done
    return 1
}

append_direct_target() {
    local target="$1"
    local existing
    [[ -n "$target" ]] || return 0
    for existing in "${DIRECT_TARGETS[@]}"; do
        [[ "$existing" == "$target" ]] && return 0
    done
    DIRECT_TARGETS+=("$target")
}

handle_menu_token() {
    local token="$1"
    local index target_type target

    case "$token" in
        "" )
            return 0
            ;;
        0|q|Q|quit|exit)
            status "已取消依赖安装。"
            exit 0
            ;;
    esac

    index="$(find_menu_index_by_id "$token")" || die "未知菜单编号: $token"
    target_type="${MENU_TARGET_TYPES[$index]}"
    target="${MENU_TARGETS[$index]}"

    case "$target_type" in
        group|module)
            append_direct_target "$target"
            ;;
        action)
            case "$target" in
                list_modules)
                    LIST_MODULES=true
                    ;;
                *)
                    die "未知菜单动作: $target"
                    ;;
            esac
            ;;
        *)
            die "菜单项 ${MENU_IDS[$index]} 的 target_type 无效: $target_type"
            ;;
    esac
}

collect_interactive_selection() {
    local answer token
    while true; do
        show_menu
        read -r -p "请输入编号，可用空格分隔: " answer || exit 1
        answer="${answer//,/ }"
        [[ -n "${answer//[[:space:]]/}" ]] || continue
        read -ra SELECTED_TOKENS <<< "$answer"
        for token in "${SELECTED_TOKENS[@]}"; do
            handle_menu_token "$token"
        done
        break
    done
}

parse_arguments() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h|--help)
                print_usage
                exit 0
                ;;
            -l|--list)
                LIST_MENU=true
                shift
                ;;
            --list-modules)
                LIST_MODULES=true
                shift
                ;;
            --dry-run)
                DRY_RUN=true
                shift
                ;;
            -y|--yes)
                ASSUME_YES=true
                shift
                ;;
            --verify)
                VERIFY_AFTER=true
                shift
                ;;
            --skip-apt-update)
                SKIP_APT_UPDATE=true
                shift
                ;;
            --strict-apt-update)
                STRICT_APT_UPDATE=true
                shift
                ;;
            --module|--group)
                [[ $# -ge 2 ]] || die "$1 需要一个名称"
                append_direct_target "$2"
                shift 2
                ;;
            --)
                shift
                while [[ $# -gt 0 ]]; do
                    SELECTED_TOKENS+=("$1")
                    shift
                done
                ;;
            -*)
                die "未知选项: $1"
                ;;
            *)
                SELECTED_TOKENS+=("$1")
                shift
                ;;
        esac
    done
}

run_check_list_modules() {
    local cmd=("$CHECK_SCRIPT" "--list")
    echo
    echo "将执行:"
    quote_cmd "${cmd[@]}"
    "${cmd[@]}"
}

confirm_targets() {
    [[ "$ASSUME_YES" == true || "$DRY_RUN" == true ]] && return 0
    [[ ${#DIRECT_TARGETS[@]} -gt 0 ]] || return 0

    local answer
    echo
    echo "将解决以下模块或模块组的依赖:"
    printf '  - %s\n' "${DIRECT_TARGETS[@]}"
    echo
    read -r -p "是否继续? [y/N] " answer || return 1
    case "$answer" in
        y|Y|yes|YES)
            return 0
            ;;
        *)
            status "已取消依赖安装。"
            exit 0
            ;;
    esac
}

run_resolve_targets() {
    [[ ${#DIRECT_TARGETS[@]} -gt 0 ]] || return 0

    local cmd=("$CHECK_SCRIPT" "--resolve")
    [[ "$DRY_RUN" == true ]] && cmd+=("--dry-run")
    [[ "$ASSUME_YES" == true ]] && cmd+=("--yes")
    [[ "$SKIP_APT_UPDATE" == true ]] && cmd+=("--skip-apt-update")
    [[ "$STRICT_APT_UPDATE" == true ]] && cmd+=("--strict-apt-update")
    cmd+=("${DIRECT_TARGETS[@]}")

    echo
    echo "将执行:"
    quote_cmd "${cmd[@]}"

    if [[ "$DRY_RUN" != true ]]; then
        preheat_sudo
    fi
    "${cmd[@]}"
}

run_verify_targets() {
    [[ "$VERIFY_AFTER" == true ]] || return 0
    [[ ${#DIRECT_TARGETS[@]} -gt 0 ]] || return 0

    local cmd=("$CHECK_SCRIPT")
    cmd+=("${DIRECT_TARGETS[@]}")

    echo
    echo "将复查:"
    quote_cmd "${cmd[@]}"

    [[ "$DRY_RUN" == true ]] && return 0
    "${cmd[@]}"
}

main() {
    [[ -x "$CHECK_SCRIPT" ]] || die "依赖检查入口不可执行: $CHECK_SCRIPT"
    load_menu
    parse_arguments "$@"

    if [[ "$LIST_MENU" == true ]]; then
        list_menu
        [[ ${#SELECTED_TOKENS[@]} -eq 0 && ${#DIRECT_TARGETS[@]} -eq 0 && "$LIST_MODULES" == false ]] && exit 0
    fi

    local token
    for token in "${SELECTED_TOKENS[@]}"; do
        handle_menu_token "$token"
    done

    if [[ ${#SELECTED_TOKENS[@]} -eq 0 && ${#DIRECT_TARGETS[@]} -eq 0 && "$LIST_MODULES" == false ]]; then
        if [[ -t 0 ]]; then
            collect_interactive_selection
        else
            show_menu
            exit 1
        fi
    fi

    if [[ "$LIST_MODULES" == true ]]; then
        run_check_list_modules
    fi

    confirm_targets
    run_resolve_targets
    run_verify_targets

    if [[ ${#DIRECT_TARGETS[@]} -gt 0 && "$DRY_RUN" != true ]]; then
        echo
        status "依赖解决流程结束。可使用 ./check.sh <模块或组> 复查依赖状态。"
    fi
}

main "$@"
