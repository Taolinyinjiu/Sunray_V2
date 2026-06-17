#!/bin/bash
# Dependency resolver execution layer

CHECK_RESLOVE_DIR="${CHECKSCRIPTS_DIR:-tools/check_scripts}/check/reslove"

get_module_resolve_scripts() {
    local module="$1"
    local module_dir="$CHECK_RESLOVE_DIR/$module"
    [[ -d "$module_dir" ]] || return 0
    find "$module_dir" -maxdepth 1 -type f -name "*.sh" | sort
}

module_has_resolve_scripts() {
    local module="$1"
    [[ -n "$(get_module_resolve_scripts "$module")" ]]
}

run_module_resolve_scripts() {
    local module="$1"
    local script script_count=0 failed=0

    echo
    echo -e "${BOLD}${BRIGHT_CYAN}[ 解决依赖: ${module} ]${NC}"

    if ! module_has_resolve_scripts "$module"; then
        print_status "未找到解决脚本: $CHECK_RESLOVE_DIR/$module/*.sh"
        return 0
    fi

    while IFS= read -r script; do
        [[ -z "$script" ]] && continue
        script_count=$((script_count + 1))
        if [[ "$CHECK_DRY_RUN" == true ]]; then
            print_status "将执行: $script"
            continue
        fi
        print_status "执行: $script"
        if bash "$script"; then
            print_success "完成: $(basename "$script")"
        else
            failed=$((failed + 1))
            print_error "失败: $(basename "$script")"
        fi
    done < <(get_module_resolve_scripts "$module")

    [[ $script_count -gt 0 ]] || return 0
    [[ $failed -eq 0 ]]
}

run_check_resolve_flow() {
    echo -e "${CYAN}Sunray 依赖解决系统${NC}"

    load_check_config || return 1
    expand_check_selection || return 1
    resolve_check_modules || return 1
    show_check_plan

    local module failed=0
    if [[ "$CHECK_DRY_RUN" == true ]]; then
        print_status "预览模式 - 不执行解决脚本"
    fi

    for module in "${CHECK_RESOLVED_MODULES[@]}"; do
        if ! run_module_resolve_scripts "$module"; then
            failed=$((failed + 1))
        fi
    done

    echo
    if [[ $failed -eq 0 ]]; then
        print_success "依赖解决脚本执行完成"
        return 0
    fi

    print_error "有 ${failed} 个模块的依赖解决脚本执行失败"
    return 1
}
