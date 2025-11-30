#!/bin/bash
# Main build script for Franka Toolbox target binaries
# This script orchestrates Docker builds for both amd64 and arm64 architectures
#
# Usage:
#   ./build.sh              # Build for both architectures
#   ./build.sh amd64        # Build for x86_64 only
#   ./build.sh arm64        # Build for ARM64 only (cross-compilation)
#   ./build.sh --help       # Show help

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

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

log_header() {
    echo -e "${CYAN}=====================================================${NC}"
    echo -e "${CYAN} $1${NC}"
    echo -e "${CYAN}=====================================================${NC}"
}

show_help() {
    echo "Franka Toolbox Docker Build System"
    echo ""
    echo "Usage: $0 [architecture] [options]"
    echo ""
    echo "Architectures:"
    echo "  amd64       Build for x86_64 (native Linux build)"
    echo "  arm64       Build for ARM64 (cross-compilation for Jetson)"
    echo "  all         Build for both architectures (default)"
    echo ""
    echo "Options:"
    echo "  --no-cache          Build without Docker cache"
    echo "  --libfranka <ver>   Specify libfranka version (default: from config)"
    echo "  --build-type <type> Build type: Release or Debug (default: Release)"
    echo "  --help, -h          Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                          # Build everything"
    echo "  $0 amd64                    # Build x86_64 binaries only"
    echo "  $0 arm64                    # Build ARM64 binaries only"
    echo "  $0 amd64 --no-cache         # Rebuild x86_64 without cache"
    echo "  $0 --libfranka 0.16.1       # Build with specific libfranka version"
    echo ""
    echo "Output:"
    echo "  common/bin.zip              # x86_64 common library"
    echo "  common/bin_arm.zip          # ARM64 common library"
    echo "  franka_robot_server/bin.tar.gz      # x86_64 server"
    echo "  franka_robot_server/bin_arm.tar.gz  # ARM64 server"
    echo "  dependencies/libfranka.zip          # x86_64 libfranka"
    echo "  dependencies/libfranka_arm.zip      # ARM64 libfranka"
}

# Default values
BUILD_ARCH="all"
NO_CACHE=""
LIBFRANKA_VERSION=""
BUILD_TYPE="Release"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        amd64|arm64|all)
            BUILD_ARCH="$1"
            shift
            ;;
        --no-cache)
            NO_CACHE="--no-cache"
            shift
            ;;
        --libfranka)
            LIBFRANKA_VERSION="$2"
            shift 2
            ;;
        --build-type)
            BUILD_TYPE="$2"
            shift 2
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
        *)
            log_error "Unknown argument: $1"
            show_help
            exit 1
            ;;
    esac
done

# Check Docker is installed and running
if ! command -v docker &> /dev/null; then
    log_error "Docker is not installed. Please install Docker first."
    exit 1
fi

if ! docker info &> /dev/null; then
    log_error "Docker daemon is not running. Please start Docker."
    exit 1
fi

# Read libfranka version from config if not provided
if [[ -z "$LIBFRANKA_VERSION" ]]; then
    if [[ -f "${PROJECT_ROOT}/config/libfranka_ver.csv" ]]; then
        LIBFRANKA_VERSION=$(cat "${PROJECT_ROOT}/config/libfranka_ver.csv" | head -1 | tr -d '\r\n')
    else
        log_error "libfranka version not specified and config/libfranka_ver.csv not found"
        exit 1
    fi
fi

log_header "Franka Toolbox Docker Build"
log_info "Project Root:      ${PROJECT_ROOT}"
log_info "Architecture:      ${BUILD_ARCH}"
log_info "Build Type:        ${BUILD_TYPE}"
log_info "Libfranka Version: ${LIBFRANKA_VERSION}"
echo ""

# Create output directories
mkdir -p "${PROJECT_ROOT}/common"
mkdir -p "${PROJECT_ROOT}/franka_robot_server"
mkdir -p "${PROJECT_ROOT}/dependencies"

# Build function for a specific architecture
build_for_arch() {
    local arch=$1
    local dockerfile="Dockerfile.${arch}"
    local image_name="franka-toolbox-builder-${arch}"
    local container_output="/tmp/franka-build-output-${arch}"
    
    log_header "Building for ${arch}"
    
    # Fix any existing root-owned files in project directories (from previous failed runs)
    log_info "Fixing existing file permissions..."
    docker run --rm -v "${PROJECT_ROOT}:/workspace:rw" alpine sh -c \
        "chown -R $(id -u):$(id -g) /workspace/common /workspace/franka_robot_server /workspace/dependencies 2>/dev/null || true"
    
    # Build Docker image
    log_info "Building Docker image: ${image_name}..."
    docker build \
        ${NO_CACHE} \
        -f "${SCRIPT_DIR}/${dockerfile}" \
        -t "${image_name}" \
        "${PROJECT_ROOT}"
    
    # Create temp output directory
    rm -rf "${container_output}"
    mkdir -p "${container_output}"
    
    # Run the build container
    log_info "Running build container..."
    docker run --rm \
        -v "${PROJECT_ROOT}:/workspace:rw" \
        -v "${container_output}:/output:rw" \
        --privileged \
        "${image_name}" \
        all \
        --arch "${arch}" \
        --build-type "${BUILD_TYPE}" \
        --libfranka-version "${LIBFRANKA_VERSION}"
    
    # Fix ownership of output files using docker (avoids needing sudo password)
    log_info "Fixing file ownership..."
    docker run --rm \
        -v "${container_output}:/output:rw" \
        alpine chown -R "$(id -u):$(id -g)" /output
    
    # Copy output files to project
    log_info "Copying build artifacts..."
    
    if [[ "$arch" == "amd64" ]]; then
        [[ -f "${container_output}/bin.zip" ]] && \
            cp "${container_output}/bin.zip" "${PROJECT_ROOT}/common/"
        [[ -f "${container_output}/bin.tar.gz" ]] && \
            cp "${container_output}/bin.tar.gz" "${PROJECT_ROOT}/franka_robot_server/"
        [[ -f "${container_output}/libfranka.zip" ]] && \
            cp "${container_output}/libfranka.zip" "${PROJECT_ROOT}/dependencies/"
    else
        [[ -f "${container_output}/bin_arm.zip" ]] && \
            cp "${container_output}/bin_arm.zip" "${PROJECT_ROOT}/common/"
        [[ -f "${container_output}/bin_arm.tar.gz" ]] && \
            cp "${container_output}/bin_arm.tar.gz" "${PROJECT_ROOT}/franka_robot_server/"
        [[ -f "${container_output}/libfranka_arm.zip" ]] && \
            cp "${container_output}/libfranka_arm.zip" "${PROJECT_ROOT}/dependencies/"
    fi
    
    # Cleanup
    rm -rf "${container_output}"
    
    log_success "Build for ${arch} completed!"
}

# Execute builds
case $BUILD_ARCH in
    amd64)
        build_for_arch "amd64"
        ;;
    arm64)
        build_for_arch "arm64"
        ;;
    all)
        build_for_arch "amd64"
        build_for_arch "arm64"
        ;;
esac

log_header "Build Summary"
echo ""
log_info "Output files:"
echo ""

if [[ "$BUILD_ARCH" == "amd64" ]] || [[ "$BUILD_ARCH" == "all" ]]; then
    echo "  x86_64 (amd64):"
    [[ -f "${PROJECT_ROOT}/common/bin.zip" ]] && \
        echo "    ✓ common/bin.zip" || echo "    ✗ common/bin.zip (missing)"
    [[ -f "${PROJECT_ROOT}/franka_robot_server/bin.tar.gz" ]] && \
        echo "    ✓ franka_robot_server/bin.tar.gz" || echo "    ✗ franka_robot_server/bin.tar.gz (missing)"
    [[ -f "${PROJECT_ROOT}/dependencies/libfranka.zip" ]] && \
        echo "    ✓ dependencies/libfranka.zip" || echo "    ✗ dependencies/libfranka.zip (missing)"
    echo ""
fi

if [[ "$BUILD_ARCH" == "arm64" ]] || [[ "$BUILD_ARCH" == "all" ]]; then
    echo "  ARM64 (arm64):"
    [[ -f "${PROJECT_ROOT}/common/bin_arm.zip" ]] && \
        echo "    ✓ common/bin_arm.zip" || echo "    ✗ common/bin_arm.zip (missing)"
    [[ -f "${PROJECT_ROOT}/franka_robot_server/bin_arm.tar.gz" ]] && \
        echo "    ✓ franka_robot_server/bin_arm.tar.gz" || echo "    ✗ franka_robot_server/bin_arm.tar.gz (missing)"
    [[ -f "${PROJECT_ROOT}/dependencies/libfranka_arm.zip" ]] && \
        echo "    ✓ dependencies/libfranka_arm.zip" || echo "    ✗ dependencies/libfranka_arm.zip (missing)"
    echo ""
fi

log_success "All builds completed successfully!"

