#!/bin/bash
# Build libfranka from source
# Supports both native (amd64) and cross-compilation (arm64)

set -e

source /scripts/common.sh

log_info "Building libfranka ${LIBFRANKA_VERSION} for ${ARCH}..."

LIBFRANKA_PATH="${WORKSPACE}/${FRANKA_FOLDER}"
LIBFRANKA_BUILD_PATH="${LIBFRANKA_PATH}/build"

# Check for corrupt or incomplete git repository
if [[ -d "$LIBFRANKA_PATH" ]]; then
    if [[ ! -d "$LIBFRANKA_PATH/.git" ]]; then
        log_warn "libfranka directory exists but is not a git repository. Removing..."
        rm -rf "$LIBFRANKA_PATH"
    else
        cd "$LIBFRANKA_PATH"
        if ! git rev-parse --git-dir > /dev/null 2>&1; then
            log_warn "libfranka git repository appears corrupt. Removing and re-cloning..."
            cd "$WORKSPACE"
            rm -rf "$LIBFRANKA_PATH"
        fi
    fi
fi

# Clone libfranka if not present
if [[ ! -d "$LIBFRANKA_PATH" ]]; then
    log_info "Cloning libfranka repository..."
    cd "$WORKSPACE"
    git clone --recursive https://github.com/frankarobotics/libfranka "$FRANKA_FOLDER"
    cd "$LIBFRANKA_PATH"
    git checkout "$LIBFRANKA_VERSION"
    git submodule update --init --recursive
else
    log_info "Using existing libfranka directory"
    cd "$LIBFRANKA_PATH"
    # Ensure we're on the correct version
    # If previous clone failed/was interrupted, git operations might fail.
    # We'll try to recover, otherwise fail.
    git reset --hard
    git fetch --all
    git checkout "$LIBFRANKA_VERSION"
    git submodule update --init --recursive
fi

# Patch libfranka for tinyxml2 case sensitivity issue on Linux
if grep -q "find_package(tinyxml2 REQUIRED)" "$LIBFRANKA_PATH/CMakeLists.txt"; then
    log_info "Patching libfranka CMakeLists.txt for tinyxml2 case sensitivity..."
    sed -i 's/find_package(tinyxml2 REQUIRED)/find_package(TinyXML2 REQUIRED)/' "$LIBFRANKA_PATH/CMakeLists.txt"
    sed -i 's/tinyxml2/TinyXML2::TinyXML2/' "$LIBFRANKA_PATH/CMakeLists.txt"
fi

# Clean and create build directory
rm -rf "$LIBFRANKA_BUILD_PATH"
mkdir -p "$LIBFRANKA_BUILD_PATH"
cd "$LIBFRANKA_BUILD_PATH"

# Configure CMake based on architecture
if [[ "$ARCH" == "arm64" ]]; then
    log_info "Configuring libfranka for ARM64 cross-compilation..."
    cmake \
        -DCMAKE_TOOLCHAIN_FILE=/scripts/toolchain-aarch64.cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DCMAKE_PREFIX_PATH="/opt/sysroot-aarch64/usr;/usr/lib/aarch64-linux-gnu;/usr/aarch64-linux-gnu/usr" \
        -Dpinocchio_DIR="/opt/sysroot-aarch64/usr/lib/cmake/pinocchio" \
        ..
else
    log_info "Configuring libfranka for native AMD64 build..."
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DCMAKE_PREFIX_PATH="/usr/local" \
        ..
fi

# Build
log_info "Building libfranka..."
cmake --build . -j$(nproc)

# Bundle runtime dependencies using linuxdeploy
log_info "Bundling libfranka runtime dependencies..."
bundle_libfranka_deps

log_success "libfranka build completed!"
