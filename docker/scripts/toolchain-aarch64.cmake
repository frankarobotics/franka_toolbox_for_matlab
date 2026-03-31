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

# Search modes for cross-compilation
# NEVER for programs (use host tools like capnp compiler)
# ONLY for libraries (use target sysroot only)
# BOTH for includes/packages to allow finding headers in /usr/include and configs in /usr/share
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

# Additional include/library paths
set(CMAKE_PREFIX_PATH 
    /opt/sysroot-aarch64/usr
    /usr/lib/aarch64-linux-gnu
    /usr/share
)

# Custom paths for things we built manually
set(CapnProto_DIR /opt/sysroot-aarch64/usr/lib/cmake/CapnProto)
set(pinocchio_DIR /opt/sysroot-aarch64/usr/lib/cmake/pinocchio)
set(hpp-fcl_DIR /opt/sysroot-aarch64/usr/lib/cmake/hpp-fcl)

# Eigen3 from apt multiarch install (libeigen3-dev:arm64)
# This prevents CMake from finding the host's Eigen3 and causing recursion
# Use CACHE variables with FORCE to ensure they survive find_package() calls
# libfranka's FindEigen3.cmake requires EIGEN3_INCLUDE_DIRS to be set
set(Eigen3_DIR "/usr/lib/aarch64-linux-gnu/cmake/eigen3" CACHE PATH "Eigen3 config dir" FORCE)
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3" CACHE PATH "Eigen3 include dir" FORCE)
set(EIGEN3_INCLUDE_DIRS "/usr/include/eigen3" CACHE PATH "Eigen3 include dirs" FORCE)
set(Eigen3_FOUND TRUE CACHE BOOL "Eigen3 found" FORCE)

# Poco from apt multiarch install (libpoco-dev:arm64)
set(Poco_INCLUDE_DIR "/usr/include" CACHE PATH "Poco include dir" FORCE)
set(Poco_Net_LIBRARY "/usr/lib/aarch64-linux-gnu/libPocoNet.so" CACHE FILEPATH "Poco Net library" FORCE)
set(Poco_Foundation_LIBRARY "/usr/lib/aarch64-linux-gnu/libPocoFoundation.so" CACHE FILEPATH "Poco Foundation library" FORCE)
set(Poco_LIBRARIES "/usr/lib/aarch64-linux-gnu/libPocoNet.so;/usr/lib/aarch64-linux-gnu/libPocoFoundation.so" CACHE STRING "Poco libraries" FORCE)
set(Poco_FOUND TRUE CACHE BOOL "Poco found" FORCE)

# Boost from apt multiarch install (libboost-*-dev:arm64)
set(BOOST_ROOT "/usr" CACHE PATH "Boost root" FORCE)
set(BOOST_INCLUDEDIR "/usr/include" CACHE PATH "Boost include dir" FORCE)
set(BOOST_LIBRARYDIR "/usr/lib/aarch64-linux-gnu" CACHE PATH "Boost library dir" FORCE)
set(Boost_LIBRARY_DIR "/usr/lib/aarch64-linux-gnu" CACHE PATH "Boost library dir" FORCE)
set(Boost_NO_SYSTEM_PATHS OFF CACHE BOOL "Boost no system paths" FORCE)
set(Boost_INCLUDE_DIR "/usr/include" CACHE PATH "Boost include dir" FORCE)
set(Boost_FILESYSTEM_LIBRARY "/usr/lib/aarch64-linux-gnu/libboost_filesystem.so" CACHE FILEPATH "Boost filesystem" FORCE)
set(Boost_SERIALIZATION_LIBRARY "/usr/lib/aarch64-linux-gnu/libboost_serialization.so" CACHE FILEPATH "Boost serialization" FORCE)
set(Boost_SYSTEM_LIBRARY "/usr/lib/aarch64-linux-gnu/libboost_system.so" CACHE FILEPATH "Boost system" FORCE)

# urdfdom from apt multiarch install (liburdfdom-dev:arm64, liburdfdom-headers-dev:arm64)
# Ubuntu 22.04 puts urdfdom_headers config in /usr/share/urdfdom_headers/cmake
set(urdfdom_headers_DIR "/usr/share/urdfdom_headers/cmake" CACHE PATH "urdfdom_headers config" FORCE)
set(urdfdom_DIR "/usr/lib/aarch64-linux-gnu/urdfdom/cmake" CACHE PATH "urdfdom config" FORCE)

# console_bridge from apt multiarch install
set(console_bridge_DIR "/usr/lib/aarch64-linux-gnu/console_bridge/cmake" CACHE PATH "console_bridge config" FORCE)

# ZLIB from apt multiarch install (zlib1g-dev:arm64)
set(ZLIB_LIBRARY "/usr/lib/aarch64-linux-gnu/libz.so" CACHE FILEPATH "ZLIB library" FORCE)
set(ZLIB_INCLUDE_DIR "/usr/include" CACHE PATH "ZLIB include" FORCE)
set(ZLIB_FOUND TRUE CACHE BOOL "ZLIB found" FORCE)

# PCRE from apt multiarch install (libpcre3-dev:arm64)
set(PCRE_LIBRARY "/usr/lib/aarch64-linux-gnu/libpcre.so" CACHE FILEPATH "PCRE library" FORCE)
set(PCRE_INCLUDE_DIR "/usr/include" CACHE PATH "PCRE include" FORCE)
set(PCRE_FOUND TRUE CACHE BOOL "PCRE found" FORCE)

# TinyXML2 from apt multiarch install (libtinyxml2-dev:arm64)
set(TinyXML2_INCLUDE_DIR "/usr/include" CACHE PATH "TinyXML2 include dir" FORCE)
set(TinyXML2_LIBRARY "/usr/lib/aarch64-linux-gnu/libtinyxml2.so" CACHE FILEPATH "TinyXML2 library" FORCE)
set(TinyXML2_FOUND TRUE CACHE BOOL "TinyXML2 found" FORCE)

# Position independent code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Linker flags
# We need -rpath-link so the linker can find transitive dependencies (like libpinocchio needed by libfranka)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L/opt/sysroot-aarch64/usr/lib -L/usr/lib/aarch64-linux-gnu -Wl,-rpath-link,/opt/sysroot-aarch64/usr/lib -Wl,-rpath-link,/usr/lib/aarch64-linux-gnu")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -L/opt/sysroot-aarch64/usr/lib -L/usr/lib/aarch64-linux-gnu -Wl,-rpath-link,/opt/sysroot-aarch64/usr/lib -Wl,-rpath-link,/usr/lib/aarch64-linux-gnu")
