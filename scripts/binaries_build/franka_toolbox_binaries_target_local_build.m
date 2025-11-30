function franka_toolbox_binaries_target_local_build(use_docker)
    %FRANKA_TOOLBOX_BINARIES_TARGET_LOCAL_BUILD Build target binaries for local x86_64 Linux
    %
    %   franka_toolbox_binaries_target_local_build() - Build using Docker (recommended)
    %   franka_toolbox_binaries_target_local_build(true) - Build using Docker
    %   franka_toolbox_binaries_target_local_build(false) - Build natively (legacy)
    %
    %   This function builds the target binaries (franka_robot_server and
    %   common library) for the local Linux x86_64 system.
    %
    %   Copyright (c) 2025 Franka Robotics GmbH - All Rights Reserved
    
    if nargin < 1
        use_docker = true;  % Default to Docker build
    end
    
    if use_docker
        fprintf('Building target binaries using Docker...\n');
        franka_toolbox_binaries_target_docker_build('amd64');
        return;
    end
    
    % Legacy native build (requires local dependencies)
    fprintf('Building target binaries natively (legacy mode)...\n');
    
    libfranka_ver = readcell('libfranka_ver.csv');
    libfranka_ver = libfranka_ver{1};

    if isunix()
        %% local target x86
        % libfranka
        franka_toolbox_libfranka_build(libfranka_ver,true);
    
        % common 
        franka_toolbox_common_build();
    
        % FrankaRobot() server
        franka_robot_server_build();
    else
        error('Native build is only supported on Unix systems. Use Docker instead.');
    end

end