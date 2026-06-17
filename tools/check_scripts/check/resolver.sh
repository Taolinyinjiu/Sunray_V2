#!/bin/bash
# Dependency resolver execution layer

CHECK_RESOLVE_DIR_PRIMARY="${CHECKSCRIPTS_DIR:-tools/check_scripts}/check/resolve"
CHECK_RESOLVE_DIR_LEGACY="${CHECKSCRIPTS_DIR:-tools/check_scripts}/check/reslove"

get_check_resolve_dir() {
    if [[ -d "$CHECK_RESOLVE_DIR_PRIMARY" ]]; then
        echo "$CHECK_RESOLVE_DIR_PRIMARY"
        return 0
    fi
    echo "$CHECK_RESOLVE_DIR_LEGACY"
}

get_module_resolve_scripts() {
    local module="$1"
    local module_dir
    module_dir="$(get_check_resolve_dir)/$module"
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
    local resolve_dir
    resolve_dir="$(get_check_resolve_dir)"

    echo
    echo -e "${BOLD}${BRIGHT_CYAN}[ 解决依赖: ${module} ]${NC}"

    if ! module_has_resolve_scripts "$module"; then
        if module_has_check_config "$module"; then
            print_warning "已声明额外依赖，但未找到解决脚本: $resolve_dir/$module/*.sh"
        else
            print_status "无额外依赖声明，跳过解决脚本"
        fi
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
        if SUNRAY_DRY_RUN="${SUNRAY_DRY_RUN:-false}" \
           SUNRAY_ASSUME_YES="${SUNRAY_ASSUME_YES:-false}" \
           bash "$script"; then
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

    local module failed=0 resolve_tmp_root=""
    resolve_tmp_root="$(mktemp -d "${TMPDIR:-/tmp}/sunray_resolve.XXXXXX")" || return 1
    export SUNRAY_APT_UPDATE_STAMP="$resolve_tmp_root/apt_update.stamp"

    if [[ "$CHECK_DRY_RUN" == true ]]; then
        export SUNRAY_DRY_RUN=true
        print_status "预览模式 - 不执行解决脚本"
    else
        export SUNRAY_DRY_RUN=false
    fi
    if [[ "$CHECK_AUTO_YES" == true ]]; then
        export SUNRAY_ASSUME_YES=true
    else
        export SUNRAY_ASSUME_YES=false
    fi
    if [[ "$CHECK_SKIP_APT_UPDATE" == true ]]; then
        export SUNRAY_SKIP_APT_UPDATE=true
    else
        export SUNRAY_SKIP_APT_UPDATE="${SUNRAY_SKIP_APT_UPDATE:-false}"
    fi
    if [[ "$CHECK_STRICT_APT_UPDATE" == true ]]; then
        export SUNRAY_STRICT_APT_UPDATE=true
    else
        export SUNRAY_STRICT_APT_UPDATE="${SUNRAY_STRICT_APT_UPDATE:-false}"
    fi

    for module in "${CHECK_RESOLVED_MODULES[@]}"; do
        if ! run_module_resolve_scripts "$module"; then
            failed=$((failed + 1))
        fi
    done

    echo
    if [[ $failed -eq 0 ]]; then
        rm -rf "$resolve_tmp_root"
        print_success "依赖解决脚本执行完成"
        return 0
    fi

    rm -rf "$resolve_tmp_root"
    print_error "有 ${failed} 个模块的依赖解决脚本执行失败"
    return 1
}
