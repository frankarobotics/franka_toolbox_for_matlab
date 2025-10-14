cmake_minimum_required(VERSION 3.10)
project(FrankaRobotServer)

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default libfranka folder name
set(FRANKA_FOLDER "libfranka" CACHE STRING "Path to libfranka directory")

# Define Franka paths explicitly (similar to common/CMakeLists.txt)
set(Franka_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../${FRANKA_FOLDER}/include)
set(Franka_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/../${FRANKA_FOLDER}/build/libfranka${CMAKE_SHARED_LIBRARY_SUFFIX})

# Set RPATH
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "\$ORIGIN/../../${FRANKA_FOLDER}/build/usr/lib")

# Prefer static libraries
set(CMAKE_FIND_LIBRARY_SUFFIXES ".lib")
set(CMAKE_FIND_LIBRARY_SUFFIXES ".a") 
set(BUILD_SHARED_LIBS OFF)

# Find required packages
find_package(Eigen3 3.3 REQUIRED)
find_package(CapnProto REQUIRED COMPONENTS rpc)

if(NOT CapnProto_FOUND)
    message(FATAL_ERROR "Cap'n Proto not found. Please install Cap'n Proto and try again.")
endif()

# Include directories
include_directories(${CAPNP_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../common/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../${FRANKA_FOLDER}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../${FRANKA_FOLDER}/common/include)

# Set default bin directory
set(BIN_FOLDER "bin" CACHE STRING "Binary directory (bin or bin_arm)")

# Add path to libfranka_matlab.a
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../common/${BIN_FOLDER})

# Locate the Cap'n Proto schema
set(CAPNP_SCHEMA "${CMAKE_CURRENT_SOURCE_DIR}/interface/rpc.capnp")

# Generate C++ source files from the schema
capnp_generate_cpp(RPC_SRCS RPC_HDRS ${CAPNP_SCHEMA})

# Define source files
set(SOURCES
    src/franka_robot_server.cpp
    src/franka_robot_rpc_service.cpp
    src/automatic_error_recovery.cpp
    src/get_robot_state.cpp
    src/gripper_control.cpp
    src/vacuum_gripper_control.cpp
    src/get_joint_poses.cpp
    src/joint_point_to_point_motion.cpp
    src/joint_trajectory_motion.cpp
    src/collision_behavior.cpp
    src/load_inertia.cpp
    ${RPC_SRCS}
    ${RPC_HDRS}
)

# Add the server executable
add_executable(franka_robot_server ${SOURCES})
target_include_directories(franka_robot_server PRIVATE 
    "${CMAKE_CURRENT_BINARY_DIR}/interface"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_link_libraries(franka_robot_server PRIVATE 
    CapnProto::capnp 
    CapnProto::capnp-rpc 
    ${Franka_LIBRARIES}
    franka_matlab
)


# # Optionally, add the example_mex executable if needed
# add_executable(example_mex cpp/example_mex.cpp ${RPC_SRCS} ${RPC_HDRS})
# target_include_directories(example_mex PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
# target_link_libraries(example_mex PRIVATE CapnProto::capnp CapnProto::capnp-rpc)

# # Install targets (optional)
# install(TARGETS franka_robot_server
#         RUNTIME DESTINATION bin) z