#!/bin/sh

set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname "$0")" && pwd)

BUILD_DIR="${SCRIPT_DIR}/build"
BUILD_TYPE="Release"
JOBS=""
ENABLE_EXAMPLES="ON"
ENABLE_GRAPHICAL_EXAMPLES="ON"
ENABLE_RSUSB="OFF"
DO_CLEAN="OFF"
DO_INSTALL="ON"
RUN_LDCONFIG="ON"
RUN_UDEV_SETUP="ON"
ASK_REBOOT="ON"
INSTALL_PREFIX=""
SUDO_KEEPALIVE_PID=""

print_usage() {
    cat <<'EOF'
Usage: ./build.sh [options]

Options:
  --debug                 Build with Debug
  --release               Build with Release (default)
  --build-dir <path>      Build directory (default: ./build)
  --jobs <n>              Parallel build jobs
  --no-examples           Disable BUILD_EXAMPLES
  --no-graphical          Disable BUILD_GRAPHICAL_EXAMPLES
  --rsusb                 Enable FORCE_RSUSB_BACKEND
  --clean                 Remove build directory before configure
  --no-install            Skip cmake --install after build
  --no-ldconfig           Skip ldconfig after install
  --no-udev               Skip udev rules setup
  --no-reboot-prompt      Do not ask whether to reboot at the end
  --install-prefix <dir>  Set CMAKE_INSTALL_PREFIX
  -h, --help              Show this help

Examples:
  ./build.sh
  ./build.sh --clean --jobs 8 --no-graphical
  ./build.sh --rsusb --install-prefix /usr/local
EOF
}

is_root() {
    [ "$(id -u)" -eq 0 ]
}

cleanup() {
    if [ -n "${SUDO_KEEPALIVE_PID}" ]; then
        kill "${SUDO_KEEPALIVE_PID}" >/dev/null 2>&1 || true
    fi
}

ensure_sudo() {
    if is_root; then
        return
    fi

    if ! command -v sudo >/dev/null 2>&1; then
        echo "sudo is required for install, ldconfig and udev setup." >&2
        exit 1
    fi

    echo "==> Requesting sudo privileges upfront"
    sudo -v

    (
        while true; do
            sudo -n true >/dev/null 2>&1 || exit
            sleep 30
        done
    ) &
    SUDO_KEEPALIVE_PID=$!
}

run_privileged() {
    if is_root; then
        "$@"
    else
        sudo "$@"
    fi
}

prompt_reboot() {
    if [ "${ASK_REBOOT}" != "ON" ]; then
        return
    fi

    if [ ! -t 0 ]; then
        echo "==> Non-interactive shell detected; skipping reboot prompt"
        return
    fi

    printf "==> Reboot system now? [y/N]: "
    if ! read -r answer; then
        echo
        return
    fi

    case "${answer}" in
        y|Y|yes|YES)
            echo "==> Rebooting"
            run_privileged reboot
            ;;
        *)
            echo "==> Reboot skipped"
            ;;
    esac
}

detect_jobs() {
    if command -v nproc >/dev/null 2>&1; then
        nproc
        return
    fi

    if command -v getconf >/dev/null 2>&1; then
        getconf _NPROCESSORS_ONLN
        return
    fi

    echo 4
}

while [ $# -gt 0 ]; do
    case "$1" in
        --debug)
            BUILD_TYPE="Debug"
            ;;
        --release)
            BUILD_TYPE="Release"
            ;;
        --build-dir)
            [ $# -ge 2 ] || { echo "Missing value for --build-dir" >&2; exit 1; }
            BUILD_DIR="$2"
            shift
            ;;
        --jobs)
            [ $# -ge 2 ] || { echo "Missing value for --jobs" >&2; exit 1; }
            JOBS="$2"
            shift
            ;;
        --no-examples)
            ENABLE_EXAMPLES="OFF"
            ENABLE_GRAPHICAL_EXAMPLES="OFF"
            ;;
        --no-graphical)
            ENABLE_GRAPHICAL_EXAMPLES="OFF"
            ;;
        --rsusb)
            ENABLE_RSUSB="ON"
            ;;
        --clean)
            DO_CLEAN="ON"
            ;;
        --no-install)
            DO_INSTALL="OFF"
            ;;
        --install-prefix)
            [ $# -ge 2 ] || { echo "Missing value for --install-prefix" >&2; exit 1; }
            INSTALL_PREFIX="$2"
            shift
            ;;
        --no-ldconfig)
            RUN_LDCONFIG="OFF"
            ;;
        --no-udev)
            RUN_UDEV_SETUP="OFF"
            ;;
        --no-reboot-prompt)
            ASK_REBOOT="OFF"
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            print_usage >&2
            exit 1
            ;;
    esac
    shift
done

trap cleanup EXIT INT TERM

ensure_sudo

if [ -z "${JOBS}" ]; then
    JOBS=$(detect_jobs)
fi

if [ "${DO_CLEAN}" = "ON" ]; then
    rm -rf "${BUILD_DIR}"
fi

mkdir -p "${BUILD_DIR}"

set -- \
    -S "${SCRIPT_DIR}" \
    -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DBUILD_EXAMPLES="${ENABLE_EXAMPLES}" \
    -DBUILD_GRAPHICAL_EXAMPLES="${ENABLE_GRAPHICAL_EXAMPLES}" \
    -DFORCE_RSUSB_BACKEND="${ENABLE_RSUSB}"

if [ -n "${INSTALL_PREFIX}" ]; then
    set -- "$@" -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}"
fi

echo "==> Configuring librealsense"
echo "    source: ${SCRIPT_DIR}"
echo "    build:  ${BUILD_DIR}"
echo "    type:   ${BUILD_TYPE}"
echo "    jobs:   ${JOBS}"
echo "    examples: ${ENABLE_EXAMPLES}"
echo "    graphical: ${ENABLE_GRAPHICAL_EXAMPLES}"
echo "    rsusb: ${ENABLE_RSUSB}"
echo "    install: ${DO_INSTALL}"
echo "    ldconfig: ${RUN_LDCONFIG}"
echo "    udev: ${RUN_UDEV_SETUP}"

cmake "$@"

echo "==> Building librealsense"
cmake --build "${BUILD_DIR}" -j"${JOBS}"

if [ "${DO_INSTALL}" = "ON" ]; then
    echo "==> Installing librealsense"
    run_privileged cmake --install "${BUILD_DIR}"
fi

if [ "${DO_INSTALL}" = "ON" ] && [ "${RUN_LDCONFIG}" = "ON" ]; then
    echo "==> Running ldconfig"
    run_privileged ldconfig
fi

if [ "${RUN_UDEV_SETUP}" = "ON" ]; then
    echo "==> Setting up udev rules"
    if is_root; then
        /bin/bash "${SCRIPT_DIR}/scripts/setup_udev_rules.sh"
    else
        /bin/bash "${SCRIPT_DIR}/scripts/setup_udev_rules.sh"
    fi
fi

prompt_reboot
