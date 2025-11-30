#!/bin/bash
# Entrypoint script for Franka Toolbox Docker build container
# This script orchestrates the entire build process

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configure git to treat the workspace as safe (avoids dubious ownership errors)
git config --global --add safe.directory '*'

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Default values
ARCH="amd64"
BUILD_TYPE="Release"
LIBFRANKA_VERSION=""
OUTPUT_DIR="/output"
WORKSPACE="/workspace"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --arch)
            ARCH="$2"
            shift 2
            ;;
        --build-type)
            BUILD_TYPE="$2"
            shift 2
            ;;
        --libfranka-version)
            LIBFRANKA_VERSION="$2"
            shift 2
            ;;
        --output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        all)
            BUILD_TARGET="all"
            shift
            ;;
        libfranka)
            BUILD_TARGET="libfranka"
            shift
            ;;
        common)
            BUILD_TARGET="common"
            shift
            ;;
        server)
            BUILD_TARGET="server"
            shift
            ;;
        help|--help|-h)
            echo "Usage: $0 [target] [options]"
            echo ""
            echo "Targets:"
            echo "  all         Build everything (default)"
            echo "  libfranka   Build only libfranka"
            echo "  common      Build only common library"
            echo "  server      Build only franka_robot_server"
            echo ""
            echo "Options:"
            echo "  --arch <arch>             Target architecture (amd64 or arm64, default: amd64)"
            echo "  --build-type <type>       Build type (Release or Debug, default: Release)"
            echo "  --libfranka-version <ver> Libfranka version (default: from config/libfranka_ver.csv)"
            echo "  --output <dir>            Output directory (default: /output)"
            exit 0
            ;;
        *)
            log_error "Unknown argument: $1"
            exit 1
            ;;
    esac
done

# Set default build target
BUILD_TARGET=${BUILD_TARGET:-all}

# Read libfranka version from config if not provided
if [[ -z "$LIBFRANKA_VERSION" ]]; then
    if [[ -f "$WORKSPACE/config/libfranka_ver.csv" ]]; then
        LIBFRANKA_VERSION=$(cat "$WORKSPACE/config/libfranka_ver.csv" | head -1 | tr -d '\r\n')
    else
        log_error "libfranka version not specified and config/libfranka_ver.csv not found"
        exit 1
    fi
fi

# Set architecture-specific variables
if [[ "$ARCH" == "arm64" ]]; then
    ARCH_SUFFIX="_arm"
    BIN_SUFFIX="_arm"
    FRANKA_FOLDER="libfranka_arm"
    BIN_FOLDER="bin_arm"
    export CMAKE_TOOLCHAIN_FILE="/scripts/toolchain-aarch64.cmake"
else
    ARCH_SUFFIX=""
    BIN_SUFFIX=""
    FRANKA_FOLDER="libfranka"
    BIN_FOLDER="bin"
    unset CMAKE_TOOLCHAIN_FILE
fi

# Export variables for sub-scripts
export ARCH
export ARCH_SUFFIX
export BIN_SUFFIX
export FRANKA_FOLDER
export BIN_FOLDER
export BUILD_TYPE
export LIBFRANKA_VERSION
export OUTPUT_DIR
export WORKSPACE

log_info "============================================"
log_info "Franka Toolbox Build System"
log_info "============================================"
log_info "Target:            $BUILD_TARGET"
log_info "Architecture:      $ARCH"
log_info "Build Type:        $BUILD_TYPE"
log_info "Libfranka Version: $LIBFRANKA_VERSION"
log_info "Output Directory:  $OUTPUT_DIR"
log_info "============================================"

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Run the appropriate build script(s)
case $BUILD_TARGET in
    all)
        /scripts/build_libfranka.sh
        /scripts/build_common.sh
        /scripts/build_server.sh
        /scripts/package.sh
        ;;
    libfranka)
        /scripts/build_libfranka.sh
        ;;
    common)
        /scripts/build_common.sh
        ;;
    server)
        /scripts/build_server.sh
        ;;
esac

log_success "Build completed successfully!"


