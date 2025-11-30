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

# Specify the sysroot
set(CMAKE_SYSROOT /opt/sysroot-aarch64)

# Search paths configuration
set(CMAKE_FIND_ROOT_PATH /opt/sysroot-aarch64 /opt/sysroot-aarch64/usr)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Additional include/library paths
set(CMAKE_PREFIX_PATH /opt/sysroot-aarch64/usr)
set(CMAKE_LIBRARY_PATH /opt/sysroot-aarch64/usr/lib)
set(CMAKE_INCLUDE_PATH /opt/sysroot-aarch64/usr/include)

# Set paths for specific packages
set(EIGEN3_INCLUDE_DIR /opt/sysroot-aarch64/usr/include)
set(Eigen3_DIR /opt/sysroot-aarch64/usr/share/eigen3/cmake)

# Set Cap'n Proto paths for cross-compilation
set(CapnProto_DIR /opt/sysroot-aarch64/usr/lib/cmake/CapnProto)

# Pinocchio paths
set(pinocchio_DIR /opt/sysroot-aarch64/usr/lib/cmake/pinocchio)

# Poco paths
set(Poco_DIR /opt/sysroot-aarch64/usr/lib/cmake/Poco)

# FCL paths
set(fcl_DIR /opt/sysroot-aarch64/usr/lib/cmake/fcl)

# urdfdom paths
set(urdfdom_DIR /opt/sysroot-aarch64/usr/lib/cmake/urdfdom)
set(urdfdom_headers_DIR /opt/sysroot-aarch64/usr/lib/cmake/urdfdom_headers)

# console_bridge paths
set(console_bridge_DIR /opt/sysroot-aarch64/usr/lib/cmake/console_bridge)

# Boost settings for cross-compilation
set(Boost_NO_SYSTEM_PATHS ON)
set(BOOST_ROOT /opt/sysroot-aarch64/usr)
set(Boost_INCLUDE_DIR /opt/sysroot-aarch64/usr/include)
set(Boost_LIBRARY_DIR /opt/sysroot-aarch64/usr/lib)

# Position independent code (needed for shared libraries)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Linker flags to find libraries in sysroot
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L/opt/sysroot-aarch64/usr/lib")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -L/opt/sysroot-aarch64/usr/lib")
