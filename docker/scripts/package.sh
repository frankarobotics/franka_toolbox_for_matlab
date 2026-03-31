#!/bin/bash
# Package build artifacts into distributable archives
# Creates bin.zip/bin_arm.zip for common and bin.tar.gz/bin_arm.tar.gz for server

set -e

source /scripts/common.sh

log_info "Packaging build artifacts for ${ARCH}..."

COMMON_BIN_PATH="${WORKSPACE}/common/${BIN_FOLDER}"
SERVER_BIN_PATH="${WORKSPACE}/franka_robot_server/${BIN_FOLDER}"

# Verify build artifacts exist
if [[ ! -f "$COMMON_BIN_PATH/libfranka_matlab.a" ]]; then
    log_error "Common library not found at ${COMMON_BIN_PATH}/libfranka_matlab.a"
    exit 1
fi

if [[ ! -f "$SERVER_BIN_PATH/franka_robot_server" ]]; then
    log_error "Server executable not found at ${SERVER_BIN_PATH}/franka_robot_server"
    exit 1
fi

# Create temp directory for packaging (avoids permission issues with mounted volumes)
TEMP_PKG_DIR="/tmp/franka_package"
rm -rf "$TEMP_PKG_DIR"
mkdir -p "$TEMP_PKG_DIR"

# Package common library (bin.zip or bin_arm.zip)
log_info "Creating common library archive..."
COMMON_ARCHIVE="bin${BIN_SUFFIX}.zip"
cd "${WORKSPACE}/common"
zip -r -y "${TEMP_PKG_DIR}/${COMMON_ARCHIVE}" "${BIN_FOLDER}"
log_info "Created: ${COMMON_ARCHIVE}"

# Copy to output directory
cp "${TEMP_PKG_DIR}/${COMMON_ARCHIVE}" "${OUTPUT_DIR}/"

# Package server executable (bin.tar.gz or bin_arm.tar.gz)
log_info "Creating server archive..."
SERVER_ARCHIVE="bin${BIN_SUFFIX}.tar.gz"
cd "${WORKSPACE}/franka_robot_server"
tar -czvf "${TEMP_PKG_DIR}/${SERVER_ARCHIVE}" "${BIN_FOLDER}"
log_info "Created: ${SERVER_ARCHIVE}"

# Copy to output directory
cp "${TEMP_PKG_DIR}/${SERVER_ARCHIVE}" "${OUTPUT_DIR}/"

# Also package libfranka dependencies if they exist
LIBFRANKA_PATH="${WORKSPACE}/${FRANKA_FOLDER}"
if [[ -d "$LIBFRANKA_PATH/build/usr" ]] || [[ -f "$LIBFRANKA_PATH/build/libfranka.so" ]]; then
    log_info "Creating libfranka dependencies archive..."
    
    # Use temp directory to avoid permission issues
    LIBFRANKA_TEMP="${TEMP_PKG_DIR}/${FRANKA_FOLDER}"
    mkdir -p "${LIBFRANKA_TEMP}/build"
    
    # Copy necessary libfranka components
    cp -r "${LIBFRANKA_PATH}/build/usr" "${LIBFRANKA_TEMP}/build/" 2>/dev/null || true
    cp -r "${LIBFRANKA_PATH}/include" "${LIBFRANKA_TEMP}/" 2>/dev/null || true
    cp -r "${LIBFRANKA_PATH}/common" "${LIBFRANKA_TEMP}/" 2>/dev/null || true
    
    # Copy libfranka.so files
    cp "${LIBFRANKA_PATH}/build/libfranka.so"* "${LIBFRANKA_TEMP}/build/" 2>/dev/null || true
    
    LIBFRANKA_ARCHIVE="${FRANKA_FOLDER}.zip"
    cd "${TEMP_PKG_DIR}"
    zip -r -y "${LIBFRANKA_ARCHIVE}" "${FRANKA_FOLDER}"
    
    # Copy to output directory
    cp "${LIBFRANKA_ARCHIVE}" "${OUTPUT_DIR}/"
    
    log_info "Created: ${LIBFRANKA_ARCHIVE}"
fi

# Clean up build directories
log_info "Cleaning up build directories..."
rm -rf "${COMMON_BIN_PATH}"
rm -rf "${SERVER_BIN_PATH}"
rm -rf "${WORKSPACE}/common/build"
rm -rf "${WORKSPACE}/franka_robot_server/build"
rm -rf "$TEMP_PKG_DIR"

log_success "Packaging completed!"
log_info ""
log_info "Output files in ${OUTPUT_DIR}:"
ls -la "${OUTPUT_DIR}/"


