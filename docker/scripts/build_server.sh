#!/bin/bash
# Build the franka_robot_server executable
# Supports both native (amd64) and cross-compilation (arm64)

set -e

source /scripts/common.sh

log_info "Building franka_robot_server for ${ARCH}..."

SERVER_PATH="${WORKSPACE}/franka_robot_server"
SERVER_BUILD_PATH="${SERVER_PATH}/build"
LIBFRANKA_PATH="${WORKSPACE}/${FRANKA_FOLDER}"
COMMON_BIN_PATH="${WORKSPACE}/common/${BIN_FOLDER}"

# Verify dependencies
if [[ ! -d "$LIBFRANKA_PATH/build" ]]; then
    log_error "libfranka not built. Run build_libfranka.sh first."
    exit 1
fi

if [[ ! -f "$COMMON_BIN_PATH/libfranka_matlab.a" ]]; then
    log_error "Common library not built. Run build_common.sh first."
    exit 1
fi

# Clean and create build directory
rm -rf "$SERVER_BUILD_PATH"
mkdir -p "$SERVER_BUILD_PATH"
cd "$SERVER_BUILD_PATH"

# Configure CMake based on architecture
if [[ "$ARCH" == "arm64" ]]; then
    log_info "Configuring franka_robot_server for ARM64 cross-compilation..."
    
    # For cross-compilation, we need to use the host's capnp compiler
    # but link against ARM64 libraries
    cmake \
        -DCMAKE_TOOLCHAIN_FILE=/scripts/toolchain-aarch64.cmake \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        -DFranka_DIR="${LIBFRANKA_PATH}/build" \
        -DFRANKA_FOLDER="${FRANKA_FOLDER}" \
        -DBIN_FOLDER="${BIN_FOLDER}" \
        -DCapnProto_DIR="/opt/sysroot-aarch64/usr/lib/cmake/CapnProto" \
        -DCAPNP_EXECUTABLE="/usr/local/bin/capnp" \
        -DCAPNPC_CXX_EXECUTABLE="/usr/local/bin/capnpc-c++" \
        ..
else
    log_info "Configuring franka_robot_server for native AMD64 build..."
    cmake \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        -DFranka_DIR="${LIBFRANKA_PATH}/build" \
        -DFRANKA_FOLDER="${FRANKA_FOLDER}" \
        -DBIN_FOLDER="${BIN_FOLDER}" \
        ..
fi

# Build
log_info "Building franka_robot_server..."
cmake --build . --config "${BUILD_TYPE}" -j$(nproc)

# Set executable permissions
chmod +x franka_robot_server

# Copy output to bin directory
log_info "Copying build artifacts..."
BIN_PATH="${SERVER_PATH}/${BIN_FOLDER}"
mkdir -p "$BIN_PATH"
cp "${SERVER_BUILD_PATH}/franka_robot_server" "$BIN_PATH/"

log_success "franka_robot_server build completed!"
log_info "Output: ${BIN_PATH}/franka_robot_server"


