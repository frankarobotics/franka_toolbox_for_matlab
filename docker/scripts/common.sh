#!/bin/bash
# Common functions and variables for build scripts

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
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

# Bundle libfranka runtime dependencies using linuxdeploy
bundle_libfranka_deps() {
    local libfranka_build="${WORKSPACE}/${FRANKA_FOLDER}/build"
    
    # Read libfranka version for the shared library name
    local libfranka_ver=$(cat "${WORKSPACE}/config/libfranka_ver.csv" | head -1 | tr -d '\r\n')
    local libfranka_major_minor=$(echo "$libfranka_ver" | cut -d. -f1,2)
    local libfranka_so="libfranka.so.${libfranka_major_minor}"
    
    if [[ "$ARCH" == "arm64" ]]; then
        # For ARM64, we manually bundle ONLY our custom dependencies (Pinocchio, hpp-fcl, CapnProto)
        # We DO NOT bundle system libraries (Boost, Poco, etc.) to avoid conflicts with target system.
        # The user is expected to apt install those on the Jetson.
        log_info "Manually bundling custom ARM64 dependencies..."
        
        local dest_lib="${libfranka_build}/usr/lib"
        mkdir -p "${dest_lib}"
        
        # Copy libfranka itself
        cp "${libfranka_build}/${libfranka_so}" "${dest_lib}/" 2>/dev/null || true
        
        # Copy ONLY custom built libraries from our sysroot
        # These are safe to bundle because they are not in standard Ubuntu repos (or we use custom versions)
        log_info "Copying custom built libraries (Pinocchio, hpp-fcl, CapnProto)..."
        cp -P /opt/sysroot-aarch64/usr/lib/*.so* "${dest_lib}/" 2>/dev/null || true
        
        log_success "Custom dependencies bundled manually."
    else
        log_info "Running linuxdeploy to bundle dependencies..."
        
        local linuxdeploy="/opt/tools/linuxdeploy/AppRun"
        
        if [[ -f "$linuxdeploy" ]]; then
            "$linuxdeploy" \
                --appdir="${libfranka_build}" \
                --executable="${libfranka_build}/libfranka.so" \
                --library="${libfranka_build}/${libfranka_so}" \
                || log_warn "linuxdeploy completed with warnings"
        else
            log_warn "linuxdeploy not found, skipping dependency bundling"
        fi
    fi
}


