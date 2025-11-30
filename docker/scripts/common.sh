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

# Smart dependency bundler for ARM64 (mimics linuxdeploy using readelf)
bundle_arm64_deps_smart() {
    local binary="$1"
    local dest_lib="$2"
    local search_paths=("/opt/sysroot-aarch64/usr/lib" "/usr/lib/aarch64-linux-gnu")
    
    # Excludelist: base system libs that should NOT be bundled
    local exclude_pattern="^(libc\.so|libm\.so|libdl\.so|libpthread\.so|librt\.so|libgcc_s\.so|libstdc\+\+\.so|ld-linux.*\.so|libresolv\.so|libnss_.*\.so|libutil\.so)"
    
    # Already processed libraries (to avoid duplicates and infinite loops)
    declare -A processed
    
    # Queue of libraries to process
    local queue=("$binary")
    
    while [[ ${#queue[@]} -gt 0 ]]; do
        local current="${queue[0]}"
        queue=("${queue[@]:1}")  # Pop first element
        
        # Skip if already processed
        [[ -n "${processed[$current]}" ]] && continue
        processed[$current]=1
        
        # Get NEEDED libraries using readelf
        local needed_libs=$(readelf -d "$current" 2>/dev/null | grep 'NEEDED' | sed -n 's/.*\[\(.*\)\]/\1/p')
        
        for lib in $needed_libs; do
            # Skip if matches exclude pattern
            if [[ "$lib" =~ $exclude_pattern ]]; then
                continue
            fi
            
            # Find the library in search paths
            local lib_path=""
            for search_path in "${search_paths[@]}"; do
                if [[ -f "$search_path/$lib" ]]; then
                    lib_path="$search_path/$lib"
                    break
                fi
            done
            
            # If found, copy it and add to queue for recursive processing
            if [[ -n "$lib_path" ]]; then
                # Copy library and its symlinks
                cp -P "$lib_path"* "$dest_lib/" 2>/dev/null || true
                
                # Add to queue if not already processed
                [[ -z "${processed[$lib_path]}" ]] && queue+=("$lib_path")
            fi
        done
    done
}

# Bundle libfranka runtime dependencies using linuxdeploy
bundle_libfranka_deps() {
    local libfranka_build="${WORKSPACE}/${FRANKA_FOLDER}/build"
    
    # Read libfranka version for the shared library name
    local libfranka_ver=$(cat "${WORKSPACE}/config/libfranka_ver.csv" | head -1 | tr -d '\r\n')
    local libfranka_major_minor=$(echo "$libfranka_ver" | cut -d. -f1,2)
    local libfranka_so="libfranka.so.${libfranka_major_minor}"
    
    if [[ "$ARCH" == "arm64" ]]; then
        # For ARM64, intelligently bundle dependencies like linuxdeploy does
        log_info "Analyzing ARM64 dependencies with readelf (like linuxdeploy)..."
        
        local dest_lib="${libfranka_build}/usr/lib"
        mkdir -p "${dest_lib}"
        
        # Copy libfranka itself
        cp -P "${libfranka_build}/${libfranka_so}"* "${dest_lib}/" 2>/dev/null || true
        
        # Smart dependency bundling
        bundle_arm64_deps_smart "${libfranka_build}/${libfranka_so}" "${dest_lib}"
        
        log_success "Dependencies bundled intelligently ($(ls -1 ${dest_lib} | wc -l) libraries)."
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


