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
    
    # Track copied files by their real path (to avoid duplicates)
    declare -A copied_files
    
    # Already processed libraries (to avoid infinite loops)
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
            
            # If found, copy it intelligently (as the NEEDED name, no symlinks)
            if [[ -n "$lib_path" ]]; then
                # Resolve to the real file content
                local real_file=$(readlink -f "$lib_path")
                
                # Check if we already have this library file (by content/path) provided as this name
                # But wait, we want to ensure 'lib' (NEEDED name) exists in dest.
                
                if [[ ! -f "$dest_lib/$lib" ]]; then
                    # Copy the real file content to the destination AS the NEEDED name
                    cp "$real_file" "$dest_lib/$lib" 2>/dev/null || true
                fi
                
                # Add real file to queue for recursive dependency analysis
                [[ -z "${processed[$real_file]}" ]] && queue+=("$real_file")
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
        
        # Copy libfranka itself (copy real content to the SONAME, no symlinks)
        local libfranka_path="${libfranka_build}/${libfranka_so}"
        if [[ -f "$libfranka_path" ]]; then
            local real_libfranka=$(readlink -f "$libfranka_path")
            # Copy content to destination with the versioned name (e.g. libfranka.so.0.16)
            cp "$real_libfranka" "${dest_lib}/${libfranka_so}" 2>/dev/null || true
        fi
        
        # Smart dependency bundling (analyze the real file)
        local real_libfranka=$(readlink -f "$libfranka_path")
        bundle_arm64_deps_smart "$real_libfranka" "${dest_lib}"
        
        # Post-processing:
        # 1. Create usr/bin and copy libfranka.so there
        local dest_bin="${libfranka_build}/usr/bin"
        mkdir -p "${dest_bin}"
        cp "$real_libfranka" "${dest_bin}/libfranka.so" 2>/dev/null || true
        
        # 2. Set RPATHs using patchelf
        # For libfranka.so in bin: needs to look in ../lib
        if [[ -f "${dest_bin}/libfranka.so" ]]; then
            patchelf --set-rpath '$ORIGIN/../lib' "${dest_bin}/libfranka.so" 2>/dev/null || true
        fi
        
        # For libraries in lib: need to look in . (to find each other)
        for lib in "${dest_lib}"/*.so*; do
             patchelf --set-rpath '$ORIGIN' "$lib" 2>/dev/null || true
        done
        
        log_success "Dependencies bundled and RPATHs set."
        
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


