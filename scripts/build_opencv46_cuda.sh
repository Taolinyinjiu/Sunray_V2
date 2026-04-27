#!/usr/bin/env bash
set -euo pipefail

# Build an isolated OpenCV 4.6.0 + opencv_contrib + CUDA installation for
# VINS-Fusion-GPU / C++ YOLO on Jetson Orin NX.
#
# Defaults are intentionally isolated from system ROS OpenCV:
#   install prefix: /opt/opencv-4.6.0-cuda
#   source/build:   $HOME/third_party/opencv-4.6.0-cuda-build
#
# Common overrides:
#   INSTALL_PREFIX=$HOME/opt/opencv-4.6.0-cuda ./scripts/build_opencv46_cuda.sh
#   WORKDIR=/data/opencv-build ./scripts/build_opencv46_cuda.sh
#   JOBS=6 ./scripts/build_opencv46_cuda.sh
#   BUILD_PYTHON=ON ./scripts/build_opencv46_cuda.sh
#   SKIP_APT=1 ./scripts/build_opencv46_cuda.sh

OPENCV_VERSION="${OPENCV_VERSION:-4.6.0}"
INSTALL_PREFIX="${INSTALL_PREFIX:-/opt/opencv-4.6.0-cuda}"
WORKDIR="${WORKDIR:-$HOME/third_party/opencv-4.6.0-cuda-build}"
CUDA_ARCH_BIN="${CUDA_ARCH_BIN:-8.7}"
JOBS="${JOBS:-$(nproc)}"
WITH_CUDNN="${WITH_CUDNN:-ON}"
OPENCV_DNN_CUDA="${OPENCV_DNN_CUDA:-ON}"
BUILD_PYTHON="${BUILD_PYTHON:-OFF}"
BUILD_CUDACODEC="${BUILD_CUDACODEC:-OFF}"
SKIP_APT="${SKIP_APT:-0}"
CLEAN_BUILD="${CLEAN_BUILD:-0}"

OPENCV_SRC="$WORKDIR/opencv"
CONTRIB_SRC="$WORKDIR/opencv_contrib"
BUILD_DIR="$WORKDIR/build"

log() {
    printf '\n[%s] %s\n' "$(date '+%H:%M:%S')" "$*"
}

require_cmd() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "Missing required command: $1" >&2
        exit 1
    fi
}

ensure_repo() {
    local dir="$1"
    local url="$2"
    local tag="$3"

    if [ ! -d "$dir/.git" ]; then
        git clone --branch "$tag" --depth 1 "$url" "$dir"
        return
    fi

    git -C "$dir" fetch --tags --depth 1 origin "$tag"
    git -C "$dir" checkout "$tag"
}

if [ "$(uname -m)" != "aarch64" ]; then
    log "Warning: this script is tuned for Jetson aarch64, current arch is $(uname -m)."
fi

export PATH="/usr/local/cuda/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"

require_cmd git
require_cmd cmake
require_cmd make

if ! command -v nvcc >/dev/null 2>&1; then
    echo "nvcc was not found. Make sure JetPack CUDA is installed and /usr/local/cuda exists." >&2
    exit 1
fi

if [ "$SKIP_APT" != "1" ]; then
    log "Installing build dependencies"
    sudo apt-get update
    sudo apt-get install -y \
        build-essential cmake git pkg-config unzip yasm \
        libgtk-3-dev libcanberra-gtk3-dev \
        libavcodec-dev libavformat-dev libswscale-dev libavutil-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
        libv4l-dev v4l-utils \
        libjpeg-dev libpng-dev libtiff-dev \
        libopenblas-dev liblapack-dev libatlas-base-dev gfortran \
        libtbb2 libtbb-dev libeigen3-dev \
        python3-dev python3-numpy
else
    log "Skipping apt dependency installation"
fi

mkdir -p "$WORKDIR"

log "Fetching OpenCV $OPENCV_VERSION"
ensure_repo "$OPENCV_SRC" "https://github.com/opencv/opencv.git" "$OPENCV_VERSION"
ensure_repo "$CONTRIB_SRC" "https://github.com/opencv/opencv_contrib.git" "$OPENCV_VERSION"

if [ "$CLEAN_BUILD" = "1" ]; then
    log "Cleaning build directory: $BUILD_DIR"
    rm -rf "$BUILD_DIR"
fi
mkdir -p "$BUILD_DIR"

log "Configuring OpenCV"
cmake -S "$OPENCV_SRC" -B "$BUILD_DIR" \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
    -D OPENCV_EXTRA_MODULES_PATH="$CONTRIB_SRC/modules" \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D WITH_CUDA=ON \
    -D CUDA_ARCH_BIN="$CUDA_ARCH_BIN" \
    -D CUDA_ARCH_PTX="" \
    -D WITH_CUDNN="$WITH_CUDNN" \
    -D OPENCV_DNN_CUDA="$OPENCV_DNN_CUDA" \
    -D ENABLE_FAST_MATH=ON \
    -D CUDA_FAST_MATH=ON \
    -D WITH_CUBLAS=ON \
    -D WITH_GSTREAMER=ON \
    -D WITH_V4L=ON \
    -D BUILD_opencv_cudacodec="$BUILD_CUDACODEC" \
    -D BUILD_opencv_python2=OFF \
    -D BUILD_opencv_python3="$BUILD_PYTHON" \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D BUILD_EXAMPLES=OFF \
    2>&1 | tee "$BUILD_DIR/configure.log"

log "Important CMake summary"
grep -E "NVIDIA CUDA|cuDNN|To be built|Unavailable" "$BUILD_DIR/configure.log" || true

log "Building OpenCV with $JOBS jobs"
cmake --build "$BUILD_DIR" --parallel "$JOBS"

log "Installing to $INSTALL_PREFIX"
sudo cmake --install "$BUILD_DIR"

log "Writing isolated environment script"
sudo tee "$INSTALL_PREFIX/setup.sh" >/dev/null <<EOF
export OpenCV_DIR=$INSTALL_PREFIX/lib/cmake/opencv4
export CMAKE_PREFIX_PATH=$INSTALL_PREFIX:\${CMAKE_PREFIX_PATH:-}
export PKG_CONFIG_PATH=$INSTALL_PREFIX/lib/pkgconfig:\${PKG_CONFIG_PATH:-}
export LD_LIBRARY_PATH=$INSTALL_PREFIX/lib:\${LD_LIBRARY_PATH:-}
EOF

log "Verifying installation"
if [ -x "$INSTALL_PREFIX/bin/opencv_version" ]; then
    "$INSTALL_PREFIX/bin/opencv_version"
fi

PKG_CONFIG_PATH="$INSTALL_PREFIX/lib/pkgconfig:${PKG_CONFIG_PATH:-}" \
    pkg-config --modversion opencv4 || true

for header in cudaoptflow.hpp cudaimgproc.hpp cudaarithm.hpp; do
    if [ ! -f "$INSTALL_PREFIX/include/opencv4/opencv2/$header" ]; then
        echo "Missing expected CUDA header: $INSTALL_PREFIX/include/opencv4/opencv2/$header" >&2
        exit 1
    fi
done

cat <<EOF

Done.

Use this OpenCV only when needed:
  source $INSTALL_PREFIX/setup.sh

For CMake projects:
  cmake ... -DOpenCV_DIR=$INSTALL_PREFIX/lib/cmake/opencv4

For ROS VINS-Fusion-GPU, rebuild cv_bridge in the same overlay before rebuilding VINS.
Then verify all ROS/VINS binaries point to this prefix:
  ldd <your_binary_or_lib.so> | grep opencv

EOF
