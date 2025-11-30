function franka_toolbox_binaries_all_build(varargin)
    %FRANKA_TOOLBOX_BINARIES_ALL_BUILD Build all Franka Toolbox binaries
    %
    %   franka_toolbox_binaries_all_build() 
    %       Build host MEX files. On Linux, also builds target binaries using Docker.
    %
    %   franka_toolbox_binaries_all_build('docker')
    %       Build host MEX files and target binaries using Docker (Linux only).
    %
    %   franka_toolbox_binaries_all_build(user, ip, port)
    %       Legacy mode: Build host MEX files and target binaries using remote Jetson.
    %
    %   This function orchestrates the complete build process:
    %     1. Builds Simulink S-function MEX files (host)
    %     2. Builds MATLAB FrankaRobot MEX files (host)
    %     3. On Linux: Builds target server binaries (using Docker or remote)
    %
    %   Docker Build (Recommended):
    %     - Builds both x86_64 and ARM64 target binaries locally
    %     - No remote Jetson machine required
    %     - Requires Docker to be installed and running
    %
    %   Legacy Remote Build:
    %     - Requires SSH access to a Jetson device
    %     - Builds ARM64 binaries on the remote machine
    %
    %   After building, run franka_toolbox_dist_make() for packaging.
    %
    %   Copyright (c) 2025 Franka Robotics GmbH - All Rights Reserved
    %   This file is subject to the terms and conditions defined in the file
    %   'LICENSE', which is part of this package

    % Parse arguments
    use_docker = true;  % Default to Docker
    user = '';
    ip = '';
    port = '22';
    
    if nargin == 1
        if ischar(varargin{1}) && strcmpi(varargin{1}, 'docker')
            use_docker = true;
        elseif islogical(varargin{1})
            use_docker = varargin{1};
        else
            error('Invalid argument. Use ''docker'' or franka_toolbox_binaries_all_build(user, ip, port)');
        end
    elseif nargin == 3
        use_docker = false;
        user = varargin{1};
        ip = varargin{2};
        port = varargin{3};
    elseif nargin > 0 && nargin ~= 3
        error('Invalid number of arguments. Use franka_toolbox_binaries_all_build() or franka_toolbox_binaries_all_build(user, ip, port)');
    end

    fprintf('\n');
    fprintf('==============================================\n');
    fprintf('Franka Toolbox Build - All Binaries\n');
    fprintf('==============================================\n');
    fprintf('\n');

    % Build Simulink & MATLAB libs (host)
    fprintf('=== Building Host MEX Files ===\n\n');
    
    fprintf('Building Simulink library MEX files...\n');
    franka_toolbox_simulink_library_mex();
    
    fprintf('\nBuilding FrankaRobot MEX files...\n');
    franka_robot_mex();
    
    % Build target binaries (Linux only)
    if isunix()
        fprintf('\n=== Building Target Binaries ===\n\n');
        
        if use_docker
            fprintf('Using Docker for target builds (both amd64 and arm64)...\n\n');
            franka_toolbox_binaries_target_docker_build('all');
        else
            % Legacy mode with remote Jetson
            fprintf('Building local x86_64 target...\n');
            franka_toolbox_binaries_target_local_build(false);
            
            if ~isempty(user) && ~isempty(ip)
                fprintf('\nBuilding remote ARM64 target...\n');
                franka_toolbox_binaries_target_remote_build(user, ip, port, false);
            end
        end
    else
        fprintf('\n');
        fprintf('Note: Target binaries can only be built on Linux.\n');
        fprintf('Run this function on a Linux machine to build target binaries,\n');
        fprintf('or use the Docker build script directly.\n');
    end
    
    fprintf('\n');
    fprintf('==============================================\n');
    fprintf('Build Complete!\n');
    fprintf('==============================================\n');
    fprintf('\n');
    fprintf('Next step: Run franka_toolbox_dist_make() to create the distribution package.\n');
    fprintf('\n');
    
end