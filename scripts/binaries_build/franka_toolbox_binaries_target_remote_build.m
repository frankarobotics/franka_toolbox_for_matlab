function franka_toolbox_binaries_target_remote_build(user, ip, port, use_docker)
    %FRANKA_TOOLBOX_BINARIES_TARGET_REMOTE_BUILD Build target binaries for ARM64 (Jetson)
    %
    %   franka_toolbox_binaries_target_remote_build() - Build using Docker (recommended)
    %   franka_toolbox_binaries_target_remote_build(user, ip, port) - Build on remote machine (legacy)
    %   franka_toolbox_binaries_target_remote_build(user, ip, port, false) - Build on remote machine (legacy)
    %   franka_toolbox_binaries_target_remote_build('', '', '', true) - Build using Docker
    %
    %   For Docker builds, the user/ip/port arguments are ignored.
    %   Docker cross-compilation builds ARM64 binaries locally without needing
    %   a remote Jetson machine.
    %
    %   Copyright (c) 2025 Franka Robotics GmbH - All Rights Reserved
    %   This file is subject to the terms and conditions defined in the file
    %   'LICENSE', which is part of this package
    
    % Determine if Docker should be used
    if nargin == 0
        % No arguments = use Docker
        use_docker = true;
    elseif nargin == 1 && islogical(user)
        % Single logical argument = docker flag
        use_docker = user;
    elseif nargin < 4
        % Legacy call with user/ip/port but no docker flag
        use_docker = false;
    end
    
    if use_docker
        fprintf('Building ARM64 target binaries using Docker cross-compilation...\n');
        fprintf('Note: No remote machine required - building locally in Docker.\n\n');
        franka_toolbox_binaries_target_docker_build('arm64');
        return;
    end
    
    % Legacy remote build (requires SSH access to Jetson)
    if nargin < 3 || isempty(user) || isempty(ip)
        error(['Remote build requires user, ip, and port arguments.\n' ...
               'Usage: franka_toolbox_binaries_target_remote_build(user, ip, port)\n' ...
               'Or use Docker build: franka_toolbox_binaries_target_remote_build()']);
    end
    
    if nargin < 3 || isempty(port)
        port = '22';
    end

    fprintf('Starting remote build process...\n');
    fprintf('Target: %s@%s:%s\n', user, ip, port);

    libfranka_ver = readcell('libfranka_ver.csv');
    libfranka_ver = libfranka_ver{1};
    fprintf('Using libfranka version: %s\n', libfranka_ver);

    %% remote target arm
    % libfranka 
    fprintf('\n=== Building libfranka ===\n');
    franka_toolbox_libfranka_remote_build(user,ip,port,libfranka_ver,true);

    % common 
    fprintf('\n=== Building common components ===\n');
    franka_toolbox_common_build(user,ip,port);

    % FrankaRobot() server
    fprintf('\n=== Building FrankaRobot server ===\n');
    franka_robot_server_build(user,ip,port);
    
    fprintf('\nRemote build completed successfully!\n');
end