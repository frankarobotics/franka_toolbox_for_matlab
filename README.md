# Franka Toolbox for MATLAB: Matlab & Simulink library for Franka Robotics research robots

A Simulink & Matlab library and tools for the Franka Robotics Robots based on the [Franka Control Interface (FCI)](https://frankarobotics.github.io/docs/). See the [documentation page](https://frankarobotics.github.io/docs/franka_toolbox_for_matlab/docs/franka_matlab/index.html) for more information.

The repository includes a complete franka.mtlbx distribution (including pre-built binaries) which you can find either under the `dist` folder or in the github release page.

You can alternatively build the project for your own machine configuration following the instructions below.

## Generating a franka.mtlbx distribution

### Option 1: Docker Build (Recommended)

The simplest way to build target binaries is using Docker. This method:
- Requires no manual dependency installation on your host
- Cross-compiles ARM64 binaries locally (no Jetson required)
- Provides consistent, reproducible builds

**Requirements:**
- Docker installed and running
- MATLAB R2022a or newer (for host MEX files only)
- On Windows: Visual Studio 2019+ and Cap'n Proto for host builds

**Build Steps:**

1. Set the desired libfranka version in `config/libfranka_ver.csv`

2. Build target binaries using Docker (from Linux or via WSL):
   ```bash
   cd docker
   ./build.sh          # Builds both x86_64 and ARM64
   # Or: ./build.sh amd64    # x86_64 only
   # Or: ./build.sh arm64    # ARM64 only
   ```

3. Build host MEX files from MATLAB:
   ```matlab
   % On Linux:
   >> franka_toolbox_simulink_library_mex();
   >> franka_robot_mex();
   
   % On Windows:
   >> franka_toolbox_binaries_all_build();
   ```

4. Generate the distribution:
   ```matlab
   >> franka_toolbox_dist_make();
   ```

**Or do it all from MATLAB (Linux):**
```matlab
>> franka_toolbox_binaries_all_build();  % Uses Docker automatically
>> franka_toolbox_dist_make();
```

See [docker/README.md](docker/README.md) for detailed Docker build documentation.

---

### Option 2: Native/Remote Build (Legacy)

For those who prefer not to use Docker or need to build directly on target hardware.

#### Requirements

**Host PC: Windows**
- MATLAB R2022a or newer
- Microsoft Visual Studio 2019 or newer
- CMake 3.15 or newer
- Git
- Cap'n Proto: check https://capnproto.org/install.html#installation-windows

**Host PC: Linux**
- MATLAB R2022a or newer
- Build essentials (gcc, g++, make)
- CMake 3.15 or newer
- Git
- Cap'n Proto (build static library):
    ```bash
    # Install build dependencies
    sudo apt-get update
    sudo apt-get install -y build-essential cmake autoconf pkg-config libtool

    # Download and build Cap'n Proto from source
    curl -O https://capnproto.org/capnproto-c++-1.1.0.tar.gz
    tar zxf capnproto-c++-1.1.0.tar.gz
    cd capnproto-c++-1.1.0
    mkdir build && 
    cd build && 
    cmake -DBUILD_TESTING=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON .. && 
    make -j10 && 
    sudo make install && 
    cd ../.. && 
    rm -rf capnproto-c++-1.1.0* && 
    sudo ldconfig
    ```

**Target PC: Linux x86_64 or Jetson (ARM64)**
- libfranka's 3rd party dependencies (based on libfranka's version). Refer to the [Documentation](https://github.com/frankarobotics/libfranka/blob/main/README.md) for more information.
- SSH keys setup for connection without password request (for remote builds).
- Cap'n Proto (build static library):
    ```bash
    # Install build dependencies
    sudo apt-get update
    sudo apt-get install -y build-essential cmake autoconf pkg-config libtool

    # Install capnproto
    curl -O https://capnproto.org/capnproto-c++-1.1.0.tar.gz && 
    tar zxf capnproto-c++-1.1.0.tar.gz && 
    cd capnproto-c++-1.1.0 && 
    mkdir build && 
    cd build && 
    cmake -DBUILD_TESTING=OFF .. && 
    make -j3 && 
    sudo make install && 
    cd ../.. && 
    rm -rf capnproto-c++-1.1.0* && 
    sudo ldconfig
    ```

#### Build Steps (Legacy)

1. Set the desired libfranka version in `config/libfranka_ver.csv`

2. Build the binaries:

    - **Windows**: Builds host MEX files only
      ```matlab
      >> franka_toolbox_binaries_all_build();
      ```

    - **Linux (local target only)**:
      ```matlab
      >> franka_toolbox_binaries_target_local_build(false);  % false = native build
      ```

    - **Linux with remote Jetson**:
      ```matlab
      >> franka_toolbox_binaries_all_build(user, ip, port);
      ```

    For generating all binaries for the final distribution, you must:
    1. Build on Ubuntu with Jetson connected: `franka_toolbox_binaries_all_build(user, ip, port);`
    2. Build on Windows: `franka_toolbox_binaries_all_build();`

3. Generate the distribution (from Linux):
    ```matlab
    >> franka_toolbox_dist_make();
    ```

---

## Output Files

After building, the following files are generated:

| File | Description |
|------|-------------|
| `common/bin.zip` | x86_64 common library (`libfranka_matlab.a`) |
| `common/bin_arm.zip` | ARM64 common library |
| `franka_robot_server/bin.tar.gz` | x86_64 server executable |
| `franka_robot_server/bin_arm.tar.gz` | ARM64 server executable |
| `dependencies/libfranka.zip` | x86_64 libfranka with dependencies |
| `dependencies/libfranka_arm.zip` | ARM64 libfranka with dependencies |
| `dist/franka.mltbx` | Final MATLAB toolbox package |