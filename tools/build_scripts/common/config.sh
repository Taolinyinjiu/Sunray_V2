#!/bin/bash
# 配置管理模块

# 全局配置缓存
CONFIG_LOADED=0
CONFIG_VALIDATED=0

# YAML解析器
parse_yaml() {
    local file="$1"
    local prefix="$2"
    
    [[ ! -f "$file" ]] && { print_error "配置文件未找到: $file"; return 1; }
    
    awk -v prefix="$prefix" '
    BEGIN { level = 0; path[0] = prefix }
    /^[[:space:]]*#/ || /^[[:space:]]*$/ { next }
    {
        indent = match($0, /[^ ]/) - 1
        level = int(indent / 2)
        gsub(/^[[:space:]]+|[[:space:]]+$/, "")
        
        colon_pos = index($0, ":")
        if (colon_pos > 0) {
            key = substr($0, 1, colon_pos - 1)
            value = substr($0, colon_pos + 1)
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", key)
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
            gsub(/^["\\047]|["\\047]$/, "", value)
            gsub(/^\\[|\\]$/, "", value)
            
            path[level+1] = key
            full_path = ""
            for (i = 0; i <= level+1; i++) {
                if (path[i] != "") {
                    if (full_path != "") full_path = full_path "_"
                    full_path = full_path path[i]
                }
            }
            
            if (value != "") print full_path "=\"" value "\""
        }
    }' "$file"
}

# 加载模块配置
load_modules_config() {
    [[ $CONFIG_LOADED -eq 1 ]] && return 0

    local config_file=""
    if [[ -n "${BUILDSCRIPTS_DIR:-}" && -f "${BUILDSCRIPTS_DIR}/modules.yaml" ]]; then
        config_file="${BUILDSCRIPTS_DIR}/modules.yaml"
    elif [[ -f "${SCRIPT_DIR}/tools/build_scripts/modules.yaml" ]]; then
        config_file="${SCRIPT_DIR}/tools/build_scripts/modules.yaml"
    elif [[ -f "${SCRIPT_DIR}/tools/buildscripts/modules.yaml" ]]; then
        config_file="${SCRIPT_DIR}/tools/buildscripts/modules.yaml"
    elif [[ -f "${SCRIPT_DIR}/buildscripts/modules.yaml" ]]; then
        config_file="${SCRIPT_DIR}/buildscripts/modules.yaml"
    elif [[ -f "${SCRIPT_DIR}/modules.yaml" ]]; then
        config_file="${SCRIPT_DIR}/modules.yaml"
    else
        print_error "配置文件未找到: modules.yaml"
        return 1
    fi

    print_debug "加载模块配置: $config_file"

    eval "$(parse_yaml "$config_file" "CONFIG")"
    CONFIG_LOADED=1
    
    print_debug "配置加载完成"
    return 0
}

# 获取模块配置值 - bash/zsh兼容版本
get_module_config() {
    local var_name="CONFIG_modules_${1}_${2}"
    eval "local value=\"\$${var_name}\""
    echo "$value"
}

# 获取用户配置值
get_user_config() {
    local config_key="$1"
    local var_name="USER_${config_key}"
    local default_value="$2"
    
    local value="${!var_name}"
    echo "${value:-$default_value}"
}

# 获取所有模块
get_all_modules() {
    compgen -v | grep "^CONFIG_modules_.*_description$" | \
        sed 's/^CONFIG_modules_//; s/_description$//' | sort -u
}

# 获取所有组
get_all_groups() {
    compgen -v | grep "^CONFIG_groups_.*_description$" | \
        sed 's/^CONFIG_groups_//; s/_description$//' | sort -u
}

# 获取组模块
get_group_modules() {
    local modules_var="CONFIG_groups_${1}_modules"
    local modules_str="${!modules_var}"
    [[ -n "$modules_str" ]] && echo "$modules_str" | tr ',' ' ' | sed 's/[][\"]//g'
}

# 检查模块冲突
check_module_conflicts() {
    local modules=("$@")
    local conflicts=()
    local processed_pairs=()
    
    for module in "${modules[@]}"; do
        local conflicts_var="CONFIG_modules_${module}_conflicts_with"
        eval "local conflicts_str=\"\$${conflicts_var}\""
        
        [[ -z "$conflicts_str" ]] && continue
        
        IFS=',' read -ra conflicting_modules <<< "${conflicts_str//[\[\]\"]/}"
        
        for conflict_module in "${conflicting_modules[@]}"; do
            conflict_module=$(echo "$conflict_module" | xargs)
            
            if array_contains "$conflict_module" "${modules[@]}"; then
                local pair_key="${module}_${conflict_module}"
                local reverse_key="${conflict_module}_${module}"
                
                if ! array_contains "$pair_key" "${processed_pairs[@]}" && \
                   ! array_contains "$reverse_key" "${processed_pairs[@]}"; then
                    conflicts+=("$module 与 $conflict_module 冲突（无法同时编译）")
                    processed_pairs+=("$pair_key")
                fi
            fi
        done
    done
    
    printf '%s\n' "${conflicts[@]}"
}

# 解析依赖关系
# 依赖解析缓存（在 resolve_dependencies 中初始化）
_RESOLVE_RESOLVED_MODULES=()
_RESOLVE_VISITING_MODULES=()

resolve_module_dependencies() {
    local module="$1"

    [[ -z "$module" ]] && return 0

    if [[ ! "$module" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]]; then
        print_error "模块名不合法或暂不支持（仅支持字母/数字/下划线）: $module"
        return 1
    fi

    if array_contains "$module" "${_RESOLVE_RESOLVED_MODULES[@]}"; then
        return 0
    fi

    if array_contains "$module" "${_RESOLVE_VISITING_MODULES[@]}"; then
        print_error "检测到循环依赖: $module"
        return 1
    fi

    if ! module_exists "$module"; then
        print_error "未找到依赖模块: $module"
        return 1
    fi

    _RESOLVE_VISITING_MODULES+=("$module")

    local deps_raw
    deps_raw="$(get_module_config "$module" "dependencies")"
    deps_raw="${deps_raw//[\[\]\"]}"
    if [[ -n "$deps_raw" ]]; then
        local dep=""
        IFS=',' read -ra dep_list <<< "$deps_raw"
        for dep in "${dep_list[@]}"; do
            dep="$(trim "$dep")"
            [[ -z "$dep" ]] && continue
            resolve_module_dependencies "$dep" || return 1
        done
    fi

    local next_visiting=()
    local item=""
    for item in "${_RESOLVE_VISITING_MODULES[@]}"; do
        if [[ "$item" != "$module" ]]; then
            next_visiting+=("$item")
        fi
    done
    _RESOLVE_VISITING_MODULES=("${next_visiting[@]}")

    if ! array_contains "$module" "${_RESOLVE_RESOLVED_MODULES[@]}"; then
        _RESOLVE_RESOLVED_MODULES+=("$module")
    fi
}

resolve_dependencies() {
    local requested_modules=("$@")
    local module=""

    _RESOLVE_RESOLVED_MODULES=()
    _RESOLVE_VISITING_MODULES=()

    for module in "${requested_modules[@]}"; do
        resolve_module_dependencies "$module" || return 1
    done

    printf '%s\n' "${_RESOLVE_RESOLVED_MODULES[@]}"
}

# 快速配置验证
validate_config() {
    [[ $CONFIG_VALIDATED -eq 1 ]] && return 0
    
    print_debug "验证配置..."
    
    local modules=($(get_all_modules))
    [[ ${#modules[@]} -eq 0 ]] && { print_error "未找到任何模块配置"; return 1; }
    
    local test_module="${modules[0]}"
    for field in description source_path build_path; do
        local var_name="CONFIG_modules_${test_module}_${field}"
        [[ -z "${!var_name}" ]] && { print_error "模块 '$test_module' 缺少字段: $field"; return 1; }
    done
    
    # 检查模块名和组名冲突
    check_naming_conflicts
    
    CONFIG_VALIDATED=1
    print_debug "配置验证通过"
    return 0
}

# 检查命名冲突
check_naming_conflicts() {
    local modules=($(get_all_modules))
    local groups=($(get_all_groups))
    local conflicts=()
    
    for module in "${modules[@]}"; do
        for group in "${groups[@]}"; do
            if [[ "$module" == "$group" ]]; then
                conflicts+=("$module")
            fi
        done
    done
    
    if [[ ${#conflicts[@]} -gt 0 ]]; then
        print_warning "发现命名冲突："
        for conflict in "${conflicts[@]}"; do
            print_warning "  - '$conflict' 既是模块又是构建组"
        done
        print_info "使用时系统将询问您的意图"
        echo
    fi
}

# 获取构建配置
get_build_config() {
    local key="$1"
    local default="$2"
    
    local value=$(get_user_config "build_config_$key" "$default")
    if [ -z "$value" ]; then
        local var_name="CONFIG_build_config_$key"
        value="${!var_name:-$default}"
    fi
    
    echo "$value"
}

# 估算构建时间
estimate_build_time() {
    local modules=("$@")
    local total_time=0
    
    for module in "${modules[@]}"; do
        local time_var="CONFIG_modules_${module}_build_time_estimate"
        local module_time="${!time_var:-60}"
        total_time=$((total_time + module_time))
    done
    
    echo "$total_time"
}


# 初始化配置系统
init_config() {
    print_debug "初始化配置系统..."
    
    if [[ -z "${SCRIPT_DIR:-}" ]]; then
        SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
    fi
    
    load_modules_config && validate_config && {
        print_debug "配置系统初始化完成"
        return 0
    }
    
    return 1
}

# 获取组描述
get_group_description() {
    local group="$1"
    
    local desc_var="CONFIG_groups_${group}_description"
    local description="${!desc_var}"
    
    if [[ -n "$description" ]]; then
        echo "$description"
    else
        echo "构建组: $group"
    fi
}

# 获取模块描述 - bash/zsh兼容版本
get_module_description() {
    local module="$1"
    local var_name="CONFIG_modules_${module}_description"
    # bash/zsh兼容的间接变量引用
    eval "local value=\"\$${var_name}\""
    echo "$value"
}

# 检查模块是否存在 - bash/zsh兼容版本
module_exists() {
    local module="$1"
    local var_name="CONFIG_modules_${module}_description"
    eval "local value=\"\$${var_name}\""
    [[ -n "$value" ]]
}

# 获取构建顺序
get_build_order() {
    local modules=("$@")
    printf '%s\n' "${modules[@]}"
}
