#!/bin/bash
# CLI layer for Sunray dependency checks

CHECK_SCRIPT_NAME="${SUNRAY_CHECK_SCRIPT_NAME:-$(basename "$0")}"
CHECK_DRY_RUN=false
CHECK_AUTO_YES=false
CHECK_FORCE_TUI=false
CHECK_SELECTED_MODULES=()
CHECK_RESOLVED_MODULES=()

CHECK_SCRIPTS_ROOT="${CHECKSCRIPTS_DIR:-tools/check_scripts}"
CHECK_LAST_SELECTION_FILE="${CHECK_SCRIPTS_ROOT}/tui/build/.sunray_last_check_selection"

get_check_modules_config_file() {
    if [[ -n "${CHECKSCRIPTS_DIR:-}" && -f "${CHECKSCRIPTS_DIR}/modules.yaml" ]]; then
        echo "${CHECKSCRIPTS_DIR}/modules.yaml"
    elif [[ -n "${SCRIPT_DIR:-}" && -f "${SCRIPT_DIR}/tools/check_scripts/modules.yaml" ]]; then
        echo "${SCRIPT_DIR}/tools/check_scripts/modules.yaml"
    elif [[ -f "tools/check_scripts/modules.yaml" ]]; then
        echo "tools/check_scripts/modules.yaml"
    elif [[ -f "modules.yaml" ]]; then
        echo "modules.yaml"
    fi
}

get_check_group_modules() {
    local group="$1"
    local parsed_modules
    parsed_modules="$(get_group_modules "$group" || true)"
    if [[ -n "$parsed_modules" ]]; then
        printf '%s\n' $parsed_modules
        return 0
    fi

    local config_file
    config_file="$(get_check_modules_config_file)"
    [[ -n "$config_file" && -f "$config_file" ]] || return 1

    awk -v target="$group" '
        function trim(s) { gsub(/^[[:space:]]+|[[:space:]]+$/, "", s); return s }
        function emit_from_line(s,   n, a, i, token) {
            gsub(/[\[\]"]/, "", s)
            n = split(s, a, ",")
            for (i = 1; i <= n; i++) {
                token = trim(a[i])
                if (token != "") print token
            }
        }
        /^[[:space:]]*#/ || /^[[:space:]]*$/ { next }
        /^groups:/ { in_groups = 1; next }
        in_groups && match($0, /^  ([A-Za-z_][A-Za-z0-9_]*):[[:space:]]*$/, m) {
            current = m[1]
            in_target = (current == target)
            in_modules = 0
            next
        }
        in_target && /^[[:space:]]{4}modules:[[:space:]]*\[/ {
            line = $0
            sub(/^[^[]*\[/, "[", line)
            emit_from_line(line)
            in_modules = (line !~ /\]/)
            next
        }
        in_target && /^[[:space:]]{4}modules:[[:space:]]*$/ {
            in_modules = 1
            next
        }
        in_target && in_modules {
            if ($0 ~ /^[[:space:]]{6}/) {
                emit_from_line($0)
                if ($0 ~ /\]/) in_modules = 0
                next
            }
            in_modules = 0
        }
    ' "$config_file"
}

load_check_last_selection() {
    if [[ ! -f "$CHECK_LAST_SELECTION_FILE" ]]; then
        print_status "未找到上次选择记录: $CHECK_LAST_SELECTION_FILE"
        return 1
    fi

    local loaded_modules=()
    local line module
    while IFS= read -r line; do
        [[ "$line" =~ ^[[:space:]]*# ]] && continue
        [[ -z "$(trim "$line")" ]] && continue
        read -ra modules_in_line <<< "$line"
        for module in "${modules_in_line[@]}"; do
            module="$(trim "$module")"
            [[ -n "$module" ]] && loaded_modules+=("$module")
        done
    done < "$CHECK_LAST_SELECTION_FILE"

    [[ ${#loaded_modules[@]} -gt 0 ]] || return 1
    CHECK_SELECTED_MODULES=("${loaded_modules[@]}")
    print_status "已加载上次选择的 ${#loaded_modules[@]} 个模块: ${loaded_modules[*]}"
}

show_check_short_help() {
    echo "使用 $CHECK_SCRIPT_NAME --help 查看完整帮助信息"
}

list_check_modules() {
    load_check_config || return 1

    print_subtitle "模块额外依赖列表"
    echo

    local module description config_mark
    while IFS= read -r module; do
        [[ -z "$module" ]] && continue
        description="$(get_module_description "$module")"
        if module_has_check_config "$module"; then
            config_mark="${BRIGHT_GREEN}有额外依赖${NC}"
        else
            config_mark="${DIM}无额外依赖${NC}"
        fi
        printf "  ${BOLD}${WHITE}%-30s${NC} %-12b ${DIM}%s${NC}\n" "$module" "$config_mark" "$description"
    done < <(get_all_modules)
    echo
}

list_check_groups() {
    print_subtitle "可检查模块组"
    echo

    local group description module
    while IFS= read -r group; do
        [[ -z "$group" ]] && continue
        description="$(get_group_description "$group")"
        print_group "$group" "$description"
        while IFS= read -r module; do
            [[ -n "$module" ]] && echo "    ${BRIGHT_WHITE}$module${NC}"
        done < <(get_check_group_modules "$group")
        echo
    done < <(get_all_groups)
}

show_check_selection_prompt() {
    echo -e "${CYAN}Sunray 依赖检查系统${NC}"
    echo
    echo -e "${YELLOW}请指定要检查的模块:${NC}"
    echo
    echo -e "${GREEN}示例:${NC}"
    echo "  $CHECK_SCRIPT_NAME all"
    echo "  $CHECK_SCRIPT_NAME S_150_Basic"
    echo "  $CHECK_SCRIPT_NAME realsense2_camera fast_lio"
    echo "  $CHECK_SCRIPT_NAME -s"
    echo "  $CHECK_SCRIPT_NAME --list"
    echo
}

run_check_interactive_selection() {
    if [[ ! -t 0 ]]; then
        show_check_selection_prompt
        return 1
    fi

    echo -e "${CYAN}Sunray 依赖检查系统${NC}"
    echo
    echo -e "${YELLOW}常用模块组:${NC}"
    local group description
    while IFS= read -r group; do
        [[ -z "$group" ]] && continue
        description="$(get_group_description "$group")"
        printf "  %-24s %s\n" "$group" "$description"
    done < <(get_all_groups)
    echo
    echo -e "${YELLOW}输入要检查的模块或组，可用空格分隔。输入 all 检查全部，输入 q 退出。${NC}"

    local answer
    read -r -p "检查目标: " answer || return 1
    answer="$(trim "$answer")"
    case "$answer" in
        ""|q|Q|quit|exit)
            print_status "已取消依赖检查"
            return 1
            ;;
    esac

    read -ra CHECK_SELECTED_MODULES <<< "$answer"
    [[ ${#CHECK_SELECTED_MODULES[@]} -gt 0 ]]
}

parse_check_arguments() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h|--help)
                show_check_help
                exit 0
                ;;
            -y|--yes)
                CHECK_AUTO_YES=true
                shift
                ;;
            --tui)
                CHECK_FORCE_TUI=true
                shift
                ;;
            -l|--list)
                list_check_modules
                exit 0
                ;;
            -g|--groups)
                list_check_groups
                exit 0
                ;;
            --dry-run)
                CHECK_DRY_RUN=true
                shift
                ;;
            --from-tui)
                shift
                ;;
            --resolve|--resolve-from-tui)
                CHECK_RESOLVE_MODE=true
                shift
                ;;
            -s|--same)
                if ! load_check_last_selection; then
                    print_error "无法加载上次选择，请手动指定模块或先进行一次构建/TUI选择"
                    exit 1
                fi
                shift
                ;;
            -*)
                print_error "未知选项: $1"
                show_check_short_help
                exit 1
                ;;
            *)
                CHECK_SELECTED_MODULES+=("$1")
                shift
                ;;
        esac
    done

    if [[ ${#CHECK_SELECTED_MODULES[@]} -eq 0 || "$CHECK_FORCE_TUI" == true ]]; then
        if ! run_check_interactive_selection; then
            exit 1
        fi
    fi
}

expand_check_selection() {
    local expanded_modules=()
    local module all_module group_module sorted_module

    for module in "${CHECK_SELECTED_MODULES[@]}"; do
        case "$module" in
            all)
                while IFS= read -r all_module; do
                    [[ -n "$all_module" ]] && expanded_modules+=("$all_module")
                done < <(get_all_modules)
                ;;
            *)
                local is_module=false
                local is_group=false
                module_exists "$module" && is_module=true
                [[ -n "$(get_check_group_modules "$module")" ]] && is_group=true

                if [[ "$is_module" == true && "$is_group" == true ]]; then
                    print_warning "发现命名冲突: '$module' 同时是模块和组，检查工具默认按组展开"
                    while IFS= read -r group_module; do
                        [[ -n "$group_module" ]] && expanded_modules+=("$group_module")
                    done < <(get_check_group_modules "$module")
                elif [[ "$is_module" == true ]]; then
                    expanded_modules+=("$module")
                elif [[ "$is_group" == true ]]; then
                    while IFS= read -r group_module; do
                        [[ -n "$group_module" ]] && expanded_modules+=("$group_module")
                    done < <(get_check_group_modules "$module")
                else
                    print_error "未知模块或模块组: $module"
                    return 1
                fi
                ;;
        esac
    done

    local unique_modules=()
    while IFS= read -r sorted_module; do
        [[ -n "$sorted_module" ]] && unique_modules+=("$sorted_module")
    done < <(printf '%s\n' "${expanded_modules[@]}" | sort -u)

    CHECK_SELECTED_MODULES=("${unique_modules[@]}")
}

resolve_check_modules() {
    local resolved_modules=()
    local module

    while IFS= read -r module; do
        [[ -n "$module" ]] && resolved_modules+=("$module")
    done < <(resolve_dependencies "${CHECK_SELECTED_MODULES[@]}")

    if [[ ${#resolved_modules[@]} -eq 0 ]]; then
        print_error "没有找到要检查的模块"
        return 1
    fi

    CHECK_RESOLVED_MODULES=("${resolved_modules[@]}")
}

show_check_plan() {
    local index=1
    local module description

    echo
    echo -e "${CYAN}=== 依赖检查计划 ===${NC}"
    echo
    echo -e "${YELLOW}检查模块顺序:${NC}"
    for module in "${CHECK_RESOLVED_MODULES[@]}"; do
        description="$(get_module_description "$module")"
        printf "%2d. %-30s %s\n" "$index" "$module" "$description"
        index=$((index + 1))
    done
    echo
    echo -e "${YELLOW}总计:${NC} ${#CHECK_RESOLVED_MODULES[@]} 个模块"
}

run_check_flow() {
    echo -e "${CYAN}Sunray 依赖检查系统${NC}"

    load_check_config || return 1
    expand_check_selection || return 1
    resolve_check_modules || return 1
    show_check_plan

    if [[ "$CHECK_DRY_RUN" == true ]]; then
        print_status "预览模式 - 不执行实际检查"
        return 0
    fi

    run_dependency_checks_for_modules "${CHECK_RESOLVED_MODULES[@]}"
}

show_check_version() {
    echo "Sunray 依赖检查系统 v0.1"
}

show_check_help() {
    cat << EOF
${CYAN}Sunray 依赖检查系统${NC}

${YELLOW}用法:${NC}
  $CHECK_SCRIPT_NAME                  启动TUI交互式选择
  $CHECK_SCRIPT_NAME --tui            同上
  $CHECK_SCRIPT_NAME [选项] [组/模块...] CLI模式检查模块额外依赖

${YELLOW}选项:${NC}
  -h, --help          显示帮助
  -l, --list          列出可检查模块
  -g, --groups        列出模块组
  -s, --same          使用上次 TUI/构建选择
  --tui               启动TUI交互式选择
  --resolve           执行解决依赖脚本
  --dry-run           只显示检查计划
  --version, -V       显示版本

${YELLOW}示例:${NC}
  $CHECK_SCRIPT_NAME all
  $CHECK_SCRIPT_NAME S_150_Basic
  $CHECK_SCRIPT_NAME realsense2_camera fast_lio
  $CHECK_SCRIPT_NAME -s
  $CHECK_SCRIPT_NAME --resolve livox_ros_driver2
  $CHECK_SCRIPT_NAME --resolve --dry-run livox_ros_driver2

${YELLOW}说明:${NC}
  • 默认进入TUI，TUI中可选择检查依赖或执行解决依赖脚本
  • 检查依赖不会安装软件，也不会编译模块
  • 解决依赖会执行 tools/check_scripts/check/reslove/<module>/*.sh
  • 已安装基础 ROS Noetic 环境后通常自带的 ROS 包，不需要在这里重复检查
  • modules.yaml 负责模块和模块之间的关系
  • module_config.yaml 只声明模块额外需要的 ROS 包、系统库、SDK 或头文件

EOF
}
