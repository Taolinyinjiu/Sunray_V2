#!/bin/bash
# Dependency check execution layer

CHECK_TOTAL=0
CHECK_PASS=0
CHECK_FAIL=0
CHECK_WARN=0
CHECK_SKIP=0
CHECK_FAILED_ITEMS=()
CHECK_WARN_ITEMS=()
CHECK_VERBOSE=false
CHECK_TMP_ROOT=""
CHECK_GLOBAL_ROS_READY=false

ensure_check_tmp_root() {
    if [[ -z "$CHECK_TMP_ROOT" ]]; then
        CHECK_TMP_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/sunray_check.XXXXXX")"
    fi
}

cleanup_check_tmp_root() {
    if [[ -n "$CHECK_TMP_ROOT" && -d "$CHECK_TMP_ROOT" ]]; then
        rm -rf "$CHECK_TMP_ROOT"
    fi
}

print_check_line() {
    local status="$1"
    local title="$2"
    local detail="${3:-}"

    case "$status" in
        pass)
            printf "  %b[通过]%b %-32s %s\n" "$SUCCESS_COLOR" "$NC" "$title" "$detail"
            ;;
        fail)
            printf "  %b[缺失]%b %-32s %s\n" "$ERROR_COLOR" "$NC" "$title" "$detail"
            ;;
        warn)
            printf "  %b[警告]%b %-32s %s\n" "$WARNING_COLOR" "$NC" "$title" "$detail"
            ;;
        skip)
            printf "  %b[跳过]%b %-32s %s\n" "$DIM" "$NC" "$title" "$detail"
            ;;
        *)
            printf "  [%s] %-32s %s\n" "$status" "$title" "$detail"
            ;;
    esac
}

record_check_result() {
    local status="$1"
    local module="$2"
    local check_id="$3"
    local title="$4"
    local detail="${5:-}"
    local hint="${6:-}"

    CHECK_TOTAL=$((CHECK_TOTAL + 1))

    case "$status" in
        pass)
            CHECK_PASS=$((CHECK_PASS + 1))
            print_check_line pass "$title" "$detail"
            ;;
        fail)
            CHECK_FAIL=$((CHECK_FAIL + 1))
            print_check_line fail "$title" "$detail"
            CHECK_FAILED_ITEMS+=("$module|$check_id|$title|$hint")
            ;;
        warn)
            CHECK_WARN=$((CHECK_WARN + 1))
            print_check_line warn "$title" "$detail"
            CHECK_WARN_ITEMS+=("$module|$check_id|$title|$hint")
            ;;
        skip)
            CHECK_SKIP=$((CHECK_SKIP + 1))
            print_check_line skip "$title" "$detail"
            ;;
    esac
}

run_shell_check() {
    local command_text="$1"
    bash -lc "$command_text" >/dev/null 2>&1
}

run_pkg_config_check() {
    local packages="$1"
    command -v pkg-config >/dev/null 2>&1 || return 1
    local package
    for package in $packages; do
        pkg-config --exists "$package" || return 1
    done
    return 0
}

run_ros_package_check() {
    local package="$1"
    if command -v rospack >/dev/null 2>&1; then
        rospack find "$package" >/dev/null 2>&1
        return $?
    fi

    [[ -n "${ROS_PACKAGE_PATH:-}" ]] || return 1
    local path
    IFS=':' read -ra ros_paths <<< "$ROS_PACKAGE_PATH"
    for path in "${ros_paths[@]}"; do
        [[ -d "$path/$package" ]] && return 0
    done
    return 1
}

run_cmake_package_check() {
    local package="$1"
    local version="$2"
    local components="$3"

    command -v cmake >/dev/null 2>&1 || return 1
    ensure_check_tmp_root

    local check_dir="$CHECK_TMP_ROOT/cmake_${package}_$RANDOM"
    mkdir -p "$check_dir"

    {
        echo "cmake_minimum_required(VERSION 3.10)"
        echo "project(SunrayDependencyCheck LANGUAGES CXX)"
        if [[ -n "$components" ]]; then
            if [[ -n "$version" ]]; then
                echo "find_package(${package} ${version} REQUIRED COMPONENTS ${components})"
            else
                echo "find_package(${package} REQUIRED COMPONENTS ${components})"
            fi
        else
            if [[ -n "$version" ]]; then
                echo "find_package(${package} ${version} REQUIRED)"
            else
                echo "find_package(${package} REQUIRED)"
            fi
        fi
    } > "$check_dir/CMakeLists.txt"

    cmake -S "$check_dir" -B "$check_dir/build" >/dev/null 2>&1
}

run_header_check() {
    local header="$1"
    command -v c++ >/dev/null 2>&1 || command -v g++ >/dev/null 2>&1 || return 1
    ensure_check_tmp_root

    local source_file="$CHECK_TMP_ROOT/header_${RANDOM}.cpp"
    echo "#include <${header}>" > "$source_file"
    echo "int main() { return 0; }" >> "$source_file"

    if command -v c++ >/dev/null 2>&1; then
        c++ -fsyntax-only "$source_file" >/dev/null 2>&1
    else
        g++ -fsyntax-only "$source_file" >/dev/null 2>&1
    fi
}

run_library_check() {
    local library="$1"
    command -v ldconfig >/dev/null 2>&1 || return 1
    ldconfig -p 2>/dev/null | grep -Eq "lib${library}\\.so"
}

run_path_exists_check() {
    local path="$1"
    [[ -e "$path" ]]
}

run_global_ros_check() {
    local check_id="ros_environment"
    local title="ROS Noetic environment"
    local hint="Install ROS Noetic and source /opt/ros/noetic/setup.bash."
    local command_text="command -v roscore >/dev/null 2>&1 && command -v catkin_make >/dev/null 2>&1"


    echo
    echo -e "${BOLD}${BRIGHT_CYAN}[ 全局环境检查 ]${NC}"

    if run_shell_check "$command_text"; then
        record_check_result pass "__global__" "$check_id" "$title" "$command_text" "$hint"
        CHECK_GLOBAL_ROS_READY=true
        return 0
    fi

    record_check_result fail "__global__" "$check_id" "$title" "$command_text" "$hint"
    return 1
}

run_one_check() {
    local module="$1"
    local item="$2"
    local type name title hint optional version components path detail

    type="$(check_item_get_field "$item" type)"
    name="$(check_item_get_field "$item" name)"
    title="$(check_item_get_field "$item" title)"
    hint="$(check_item_get_field "$item" hint)"
    optional="$(check_item_get_field "$item" optional)"
    version="$(check_item_get_field "$item" version)"
    components="$(check_item_get_field "$item" components)"
    path="$(check_item_get_field "$item" path)"
    [[ -z "$title" ]] && title="$name"
    if [[ -z "$type" ]]; then
        record_check_result fail "$module" "__invalid__" "无效检查项" "缺少 type" "$hint"
        return 1
    fi

    local ok=false
    case "$type" in
        ros_packages)
            detail="$name"
            run_ros_package_check "$name" && ok=true
            ;;
        pkg_config)
            detail="$name"
            run_pkg_config_check "$name" && ok=true
            ;;
        cmake_packages)
            detail="$name"
            [[ -n "$version" ]] && detail+=" ${version}"
            [[ -n "$components" ]] && detail+=" COMPONENTS ${components}"
            run_cmake_package_check "$name" "$version" "$components" && ok=true
            ;;
        paths)
            [[ -z "$path" ]] && path="$name"
            detail="$path"
            run_path_exists_check "$path" && ok=true
            ;;
        headers)
            detail="$name"
            run_header_check "$name" && ok=true
            ;;
        libraries)
            detail="lib${name}.so"
            run_library_check "$name" && ok=true
            ;;
        *)
            detail="未知检查类型: $type"
            ok=false
            ;;
    esac

    if [[ "$ok" == true ]]; then
        record_check_result pass "$module" "$type:$name" "$title" "$detail" "$hint"
        return 0
    fi

    if [[ "$optional" == true ]]; then
        record_check_result warn "$module" "$type:$name" "$title" "$detail" "$hint"
        return 0
    fi

    record_check_result fail "$module" "$type:$name" "$title" "$detail" "$hint"
    return 1
}

run_module_checks() {
    local module="$1"
    local description item had_config=false

    description="$(get_module_description "$module")"
    echo
    echo -e "${BOLD}${BRIGHT_CYAN}[ 检查模块: ${module} ]${NC} ${DIM}${description}${NC}"

    if ! module_has_check_config "$module"; then
        record_check_result skip "$module" "__none__" "无额外依赖" "module_config.yaml 未声明额外依赖"
        return 0
    fi

    while IFS= read -r item; do
        [[ -z "$item" ]] && continue
        had_config=true
        run_one_check "$module" "$item" || true
    done < <(get_module_check_items "$module")

    if [[ "$had_config" != true ]]; then
        record_check_result skip "$module" "__empty__" "无额外依赖" "module_config.yaml 未声明额外依赖"
    fi
}

print_dependency_summary() {
    echo
    print_double_separator
    echo -e "${BOLD}${BRIGHT_CYAN}依赖检查结果${NC}"
    print_double_separator
    echo -e "  ${BRIGHT_GREEN}通过: ${CHECK_PASS}${NC}"
    echo -e "  ${BRIGHT_RED}缺失: ${CHECK_FAIL}${NC}"
    echo -e "  ${BRIGHT_YELLOW}警告: ${CHECK_WARN}${NC}"
    echo -e "  ${DIM}跳过: ${CHECK_SKIP}${NC}"
    echo -e "  ${BRIGHT_BLUE}总计: ${CHECK_TOTAL}${NC}"

    if [[ ${#CHECK_FAILED_ITEMS[@]} -gt 0 ]]; then
        echo
        echo -e "${BRIGHT_RED}${BOLD}缺失依赖安装建议:${NC}"
        local item module check_id title hint
        for item in "${CHECK_FAILED_ITEMS[@]}"; do
            IFS='|' read -r module check_id title hint <<< "$item"
            echo "  - ${module}: ${title}"
            [[ -n "$hint" ]] && echo "    ${hint}"
        done
    fi

    if [[ ${#CHECK_WARN_ITEMS[@]} -gt 0 ]]; then
        echo
        echo -e "${BRIGHT_YELLOW}${BOLD}可选依赖提示:${NC}"
        local item module check_id title hint
        for item in "${CHECK_WARN_ITEMS[@]}"; do
            IFS='|' read -r module check_id title hint <<< "$item"
            echo "  - ${module}: ${title}"
            [[ -n "$hint" ]] && echo "    ${hint}"
        done
    fi
    print_double_separator
}

run_dependency_checks_for_modules() {
    local modules=("$@")
    local module

    CHECK_TOTAL=0
    CHECK_PASS=0
    CHECK_FAIL=0
    CHECK_WARN=0
    CHECK_SKIP=0
    CHECK_FAILED_ITEMS=()
    CHECK_WARN_ITEMS=()

    trap cleanup_check_tmp_root EXIT

    run_global_ros_check || {
        print_dependency_summary
        return 1
    }

    for module in "${modules[@]}"; do
        run_module_checks "$module"
    done

    print_dependency_summary

    [[ $CHECK_FAIL -eq 0 ]]
}
