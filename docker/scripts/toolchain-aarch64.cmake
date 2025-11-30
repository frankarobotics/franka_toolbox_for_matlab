# CMake toolchain file for cross-compiling to aarch64 (ARM64)
# Used for Jetson Orin and other ARM64 Linux targets

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Specify the cross compiler
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_AR aarch64-linux-gnu-ar)
set(CMAKE_RANLIB aarch64-linux-gnu-ranlib)
set(CMAKE_STRIP aarch64-linux-gnu-strip)

# Search paths configuration
# 1. /opt/sysroot-aarch64: Custom builds (Pinocchio, hpp-fcl, CapnProto)
# 2. /usr/lib/aarch64-linux-gnu: Apt-installed arm64 libraries
# 3. /usr/aarch64-linux-gnu: Cross-compiler sysroot
set(CMAKE_FIND_ROOT_PATH 
    /opt/sysroot-aarch64 
    /opt/sysroot-aarch64/usr 
    /usr/lib/aarch64-linux-gnu
    /usr/aarch64-linux-gnu
)

# Relax these modes to allow finding things in the multiarch paths
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

# Additional include/library paths
set(CMAKE_PREFIX_PATH 
    /opt/sysroot-aarch64/usr
    /usr/lib/aarch64-linux-gnu
)

# Custom paths for things we built manually
set(CapnProto_DIR /opt/sysroot-aarch64/usr/lib/cmake/CapnProto)
set(pinocchio_DIR /opt/sysroot-aarch64/usr/lib/cmake/pinocchio)
set(hpp-fcl_DIR /opt/sysroot-aarch64/usr/lib/cmake/hpp-fcl)

# Position independent code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Linker flags
# We need -rpath-link so the linker can find transitive dependencies (like libpinocchio needed by libfranka)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L/opt/sysroot-aarch64/usr/lib -L/usr/lib/aarch64-linux-gnu -Wl,-rpath-link,/opt/sysroot-aarch64/usr/lib -Wl,-rpath-link,/usr/lib/aarch64-linux-gnu")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -L/opt/sysroot-aarch64/usr/lib -L/usr/lib/aarch64-linux-gnu -Wl,-rpath-link,/opt/sysroot-aarch64/usr/lib -Wl,-rpath-link,/usr/lib/aarch64-linux-gnu")
