#!/bin/bash
# Build the common library (libfranka_matlab.a)
# Supports both native (amd64) and cross-compilation (arm64)

set -e

source /scripts/common.sh

log_info "Building common library for ${ARCH}..."

COMMON_PATH="${WORKSPACE}/common"
COMMON_BUILD_PATH="${COMMON_PATH}/build"
LIBFRANKA_PATH="${WORKSPACE}/${FRANKA_FOLDER}"

# Verify libfranka exists
if [[ ! -d "$LIBFRANKA_PATH/build" ]]; then
    log_error "libfranka not built. Run build_libfranka.sh first."
    exit 1
fi

# Clean and create build directory
rm -rf "$COMMON_BUILD_PATH"
mkdir -p "$COMMON_BUILD_PATH"
cd "$COMMON_BUILD_PATH"

# Configure CMake based on architecture
if [[ "$ARCH" == "arm64" ]]; then
    log_info "Configuring common library for ARM64 cross-compilation..."
    cmake \
        -DCMAKE_TOOLCHAIN_FILE=/scripts/toolchain-aarch64.cmake \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        -DFranka_DIR="${LIBFRANKA_PATH}/build" \
        -DFRANKA_FOLDER="${FRANKA_FOLDER}" \
        ..
else
    log_info "Configuring common library for native AMD64 build..."
    cmake \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        -DFranka_DIR="${LIBFRANKA_PATH}/build" \
        -DFRANKA_FOLDER="${FRANKA_FOLDER}" \
        ..
fi

# Build
log_info "Building common library..."
cmake --build . -j$(nproc)

# Copy output to bin directory
log_info "Copying build artifacts..."
BIN_PATH="${COMMON_PATH}/${BIN_FOLDER}"
mkdir -p "$BIN_PATH"
cp "${COMMON_BUILD_PATH}/libfranka_matlab.a" "$BIN_PATH/"

log_success "Common library build completed!"
log_info "Output: ${BIN_PATH}/libfranka_matlab.a"


