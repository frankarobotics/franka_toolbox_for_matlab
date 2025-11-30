# Franka Toolbox Docker Build System

This directory contains Docker-based build infrastructure for building the Franka Toolbox target binaries (server-side components).

## Overview

The Docker build system allows you to:
- Build x86_64 (amd64) Linux binaries in a consistent environment
- Cross-compile ARM64 binaries for Jetson Orin without needing access to the physical device
- Eliminate the need for manual dependency installation on your host system

## Prerequisites

- **Docker**: Install Docker Engine or Docker Desktop
  - Linux: https://docs.docker.com/engine/install/
  - macOS: https://docs.docker.com/desktop/install/mac-install/
  - Windows: https://docs.docker.com/desktop/install/windows-install/

- On Linux, ensure your user is in the `docker` group:
  ```bash
  sudo usermod -aG docker $USER
  # Log out and back in for changes to take effect
  ```

## Quick Start

### Build All Target Binaries (Both Architectures)

```bash
cd docker
./build.sh
```

This will build:
- `common/bin.zip` - x86_64 common library
- `common/bin_arm.zip` - ARM64 common library  
- `franka_robot_server/bin.tar.gz` - x86_64 server executable
- `franka_robot_server/bin_arm.tar.gz` - ARM64 server executable
- `dependencies/libfranka.zip` - x86_64 libfranka
- `dependencies/libfranka_arm.zip` - ARM64 libfranka

### Build for Specific Architecture

```bash
# Build only x86_64 binaries
./build.sh amd64

# Build only ARM64 binaries (cross-compilation)
./build.sh arm64
```

### Using from MATLAB

```matlab
% Build all target binaries using Docker (recommended)
franka_toolbox_binaries_target_docker_build('all');

% Build only x86_64 target binaries
franka_toolbox_binaries_target_docker_build('amd64');

% Build only ARM64 target binaries
franka_toolbox_binaries_target_docker_build('arm64');

% Or use the simplified wrapper
franka_toolbox_binaries_target_local_build();    % Uses Docker by default
franka_toolbox_binaries_target_remote_build();   % Uses Docker for ARM64 cross-compile
```

## Build Options

```bash
./build.sh [architecture] [options]

Architectures:
  amd64       Build for x86_64 (native Linux build)
  arm64       Build for ARM64 (cross-compilation for Jetson)
  all         Build for both architectures (default)

Options:
  --no-cache          Build without Docker cache
  --libfranka <ver>   Specify libfranka version (default: from config)
  --build-type <type> Build type: Release or Debug (default: Release)
  --help, -h          Show help message
```

## Directory Structure

```
docker/
├── Dockerfile.amd64          # Docker image for x86_64 builds
├── Dockerfile.arm64          # Docker image for ARM64 cross-compilation
├── build.sh                  # Main build orchestration script
├── .dockerignore             # Files to exclude from Docker context
├── README.md                 # This file
└── scripts/
    ├── entrypoint.sh         # Container entrypoint
    ├── common.sh             # Shared functions
    ├── build_libfranka.sh    # Build libfranka
    ├── build_common.sh       # Build common library
    ├── build_server.sh       # Build franka_robot_server
    ├── package.sh            # Package build artifacts
    └── toolchain-aarch64.cmake  # CMake toolchain for ARM64 cross-compilation
```

## What Gets Built

### Common Library (`libfranka_matlab.a`)
A static library containing shared MATLAB/Franka integration code used by the server.

### Franka Robot Server (`franka_robot_server`)
An RPC server executable that runs on the target Linux machine and communicates with MATLAB/Simulink via Cap'n Proto.

### libfranka
The Franka C++ library is built from source and bundled with its runtime dependencies using linuxdeploy.

## Troubleshooting

### Docker Permission Denied
```bash
# Add your user to the docker group
sudo usermod -aG docker $USER
# Log out and back in
```

### Build Cache Issues
```bash
# Rebuild without cache
./build.sh --no-cache
```

### Insufficient Disk Space
Docker builds require approximately 5GB of disk space. Clean up unused images:
```bash
docker system prune -a
```

### Cross-compilation Issues
ARM64 cross-compilation uses:
- `gcc-aarch64-linux-gnu` toolchain
- Pre-built ARM64 Cap'n Proto libraries
- CMake toolchain file for proper cross-compilation setup

If you encounter linking issues, ensure the Docker image was built successfully.

## Legacy Build Methods

The Docker build system replaces the need for:
- Manual installation of build dependencies on your host
- SSH access to a remote Jetson device
- Running MATLAB scripts that invoke remote builds

However, the legacy methods are still available:
```matlab
% Legacy local build (requires native dependencies)
franka_toolbox_binaries_target_local_build(false);

% Legacy remote build (requires SSH to Jetson)
franka_toolbox_binaries_target_remote_build(user, ip, port, false);
```

## License

Copyright (c) 2025 Franka Robotics GmbH - All Rights Reserved

