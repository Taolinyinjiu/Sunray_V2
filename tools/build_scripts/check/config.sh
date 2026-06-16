#!/bin/bash
# Dependency check configuration helpers

CHECK_CONFIG_LOADED=0
CHECK_CONFIG_FILE=""
CHECK_MODULES=()
CHECK_MODULE_ITEMS_PREFIX="CHECK_MODULE_ITEMS_"

check_config_var_name() {
    local prefix="$1"
    local key="$2"
    printf '%s%s' "$prefix" "$key"
}

check_config_append_unique() {
    local list_name="$1"
    local value="$2"
    local item

    [[ -z "$value" ]] && return 0
    eval "local current=(\"\${${list_name}[@]}\")"
    for item in "${current[@]}"; do
        [[ "$item" == "$value" ]] && return 0
    done
    eval "${list_name}+=(\"\$value\")"
}

check_config_append_module_item() {
    local module="$1"
    local item="$2"
    local var_name

    [[ -z "$module" || -z "$item" ]] && return 0
    var_name="$(check_config_var_name "$CHECK_MODULE_ITEMS_PREFIX" "$module")"
    if [[ -n "${!var_name:-}" ]]; then
        printf -v "$var_name" '%s\n%s' "${!var_name}" "$item"
    else
        printf -v "$var_name" '%s' "$item"
    fi
}

find_check_config_file() {
    if [[ -n "${CHECK_CONFIG_FILE:-}" && -f "$CHECK_CONFIG_FILE" ]]; then
        echo "$CHECK_CONFIG_FILE"
        return 0
    fi

    if [[ -n "${BUILDSCRIPTS_DIR:-}" && -f "${BUILDSCRIPTS_DIR}/module_config.yaml" ]]; then
        echo "${BUILDSCRIPTS_DIR}/module_config.yaml"
        return 0
    fi

    if [[ -n "${SCRIPT_DIR:-}" && -f "${SCRIPT_DIR}/tools/build_scripts/module_config.yaml" ]]; then
        echo "${SCRIPT_DIR}/tools/build_scripts/module_config.yaml"
        return 0
    fi

    if [[ -f "module_config.yaml" ]]; then
        echo "module_config.yaml"
        return 0
    fi

    return 1
}

strip_yaml_value() {
    local value="$1"
    value="${value%%#*}"
    value="$(trim "$value")"
    value="${value%\"}"
    value="${value#\"}"
    value="${value%\'}"
    value="${value#\'}"
    printf '%s' "$value"
}

check_item_set_field() {
    local item_var="$1"
    local key="$2"
    local value="$3"
    local item

    eval "item=\"\${${item_var}}\""
    item="${item};${key}=${value}"
    printf -v "$item_var" '%s' "$item"
}

flush_check_item() {
    local module="$1"
    local item="$2"

    [[ -z "$module" || -z "$item" ]] && return 0
    check_config_append_module_item "$module" "$item"
}

load_check_config() {
    [[ $CHECK_CONFIG_LOADED -eq 1 ]] && return 0

    local config_file
    config_file="$(find_check_config_file)" || {
        print_error "依赖检查配置未找到: module_config.yaml"
        return 1
    }
    CHECK_CONFIG_FILE="$config_file"

    local section=""
    local current_module=""
    local current_field=""
    local current_item=""
    local raw line indent key value item_value

    while IFS= read -r raw || [[ -n "$raw" ]]; do
        line="${raw%%#*}"
        [[ -z "$(trim "$line")" ]] && continue

        indent=$(( $(expr match "$line" ' *') ))
        line="$(trim "$line")"

        if [[ $indent -eq 0 && "$line" == "modules:" ]]; then
            flush_check_item "$current_module" "$current_item"
            section="modules"
            current_module=""
            current_field=""
            current_item=""
            continue
        fi

        [[ "$section" == "modules" ]] || continue

        if [[ $indent -eq 2 && "$line" =~ ^([A-Za-z_][A-Za-z0-9_]*):[[:space:]]*$ ]]; then
            flush_check_item "$current_module" "$current_item"
            current_module="${BASH_REMATCH[1]}"
            current_field=""
            current_item=""
            check_config_append_unique CHECK_MODULES "$current_module"
            continue
        fi

        if [[ $indent -eq 4 && "$line" =~ ^([A-Za-z_][A-Za-z0-9_]*):[[:space:]]*$ ]]; then
            flush_check_item "$current_module" "$current_item"
            current_field="${BASH_REMATCH[1]}"
            current_item=""
            continue
        fi

        if [[ $indent -eq 6 && -n "$current_module" && -n "$current_field" && "$line" == "- "* ]]; then
            flush_check_item "$current_module" "$current_item"
            item_value="${line#- }"
            item_value="$(strip_yaml_value "$item_value")"

            if [[ "$item_value" == *:* ]]; then
                key="${item_value%%:*}"
                value="${item_value#*:}"
                value="$(strip_yaml_value "$value")"
                current_item="type=${current_field}"
                check_item_set_field current_item "$key" "$value"
            else
                current_item="type=${current_field};name=${item_value}"
            fi
            continue
        fi

        if [[ $indent -eq 8 && -n "$current_item" && "$line" == *:* ]]; then
            key="${line%%:*}"
            value="${line#*:}"
            value="$(strip_yaml_value "$value")"
            check_item_set_field current_item "$key" "$value"
            continue
        fi
    done < "$config_file"

    flush_check_item "$current_module" "$current_item"
    CHECK_CONFIG_LOADED=1
    return 0
}

module_has_check_config() {
    local module="$1"
    local var_name
    var_name="$(check_config_var_name "$CHECK_MODULE_ITEMS_PREFIX" "$module")"
    [[ -n "${!var_name:-}" ]]
}

get_module_check_items() {
    local module="$1"
    local var_name
    var_name="$(check_config_var_name "$CHECK_MODULE_ITEMS_PREFIX" "$module")"
    [[ -n "${!var_name:-}" ]] || return 0
    printf '%s\n' "${!var_name}"
}

check_item_get_field() {
    local item="$1"
    local field="$2"
    local part key value

    IFS=';' read -ra parts <<< "$item"
    for part in "${parts[@]}"; do
        key="${part%%=*}"
        value="${part#*=}"
        if [[ "$key" == "$field" ]]; then
            printf '%s' "$value"
            return 0
        fi
    done
}
