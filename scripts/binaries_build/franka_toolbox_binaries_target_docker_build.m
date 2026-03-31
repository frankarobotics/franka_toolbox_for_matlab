function franka_toolbox_binaries_target_docker_build(arch)
    %FRANKA_TOOLBOX_BINARIES_TARGET_DOCKER_BUILD Build target binaries using Docker
    %
    %   franka_toolbox_binaries_target_docker_build() - Build for both architectures
    %   franka_toolbox_binaries_target_docker_build('amd64') - Build for x86_64 only
    %   franka_toolbox_binaries_target_docker_build('arm64') - Build for ARM64 only
    %   franka_toolbox_binaries_target_docker_build('all') - Build for both architectures
    %
    %   This function uses Docker containers to build the target binaries
    %   (franka_robot_server and common library) for Linux targets.
    %
    %   Output files:
    %     - common/bin.zip and common/bin_arm.zip
    %     - franka_robot_server/bin.tar.gz and franka_robot_server/bin_arm.tar.gz
    %     - dependencies/libfranka.zip and dependencies/libfranka_arm.zip
    %
    %   Prerequisites:
    %     - Docker must be installed and running
    %     - On Linux, user must be in docker group or use sudo
    %
    %   Copyright (c) 2025 Franka Robotics GmbH - All Rights Reserved
    %   This file is subject to the terms and conditions defined in the file
    %   'LICENSE', which is part of this package
    
    if nargin < 1
        arch = 'all';
    end
    
    % Validate architecture argument
    valid_archs = {'amd64', 'arm64', 'all'};
    if ~ismember(arch, valid_archs)
        error('Invalid architecture: %s. Must be one of: %s', arch, strjoin(valid_archs, ', '));
    end
    
    % Get installation path
    installation_path = franka_toolbox_installation_path_get();
    docker_dir = fullfile(installation_path, 'docker');
    build_script = fullfile(docker_dir, 'build.sh');
    
    % Check Docker is available
    fprintf('Checking Docker availability...\n');
    [status, ~] = system('docker --version');
    if status ~= 0
        error(['Docker is not installed or not in PATH.\n' ...
               'Please install Docker and ensure it is running.']);
    end
    
    [status, ~] = system('docker info');
    if status ~= 0
        error(['Docker daemon is not running.\n' ...
               'Please start the Docker service.']);
    end
    
    % Check build script exists
    if ~isfile(build_script)
        error('Docker build script not found at: %s', build_script);
    end
    
    % Make build script executable (Unix only)
    if isunix()
        system(['chmod +x "', build_script, '"']);
        % Also make all scripts executable
        system(['chmod +x "', fullfile(docker_dir, 'scripts', '*.sh'), '"']);
    end
    
    % Read libfranka version
    libfranka_ver = readcell(fullfile(installation_path, 'config', 'libfranka_ver.csv'));
    libfranka_ver = libfranka_ver{1};
    
    fprintf('\n');
    fprintf('==============================================\n');
    fprintf('Franka Toolbox Docker Build\n');
    fprintf('==============================================\n');
    fprintf('Architecture:      %s\n', arch);
    fprintf('Libfranka Version: %s\n', libfranka_ver);
    fprintf('Docker Directory:  %s\n', docker_dir);
    fprintf('==============================================\n');
    fprintf('\n');
    
    % Build command
    cmd = sprintf('cd "%s" && bash build.sh %s --libfranka %s', ...
                  docker_dir, arch, libfranka_ver);
    
    fprintf('Running Docker build...\n');
    fprintf('Command: %s\n\n', cmd);
    
    % Execute build
    [status, output] = system(cmd, '-echo');
    
    if status ~= 0
        error('Docker build failed with exit code %d:\n%s', status, output);
    end
    
    fprintf('\n');
    fprintf('==============================================\n');
    fprintf('Build completed successfully!\n');
    fprintf('==============================================\n');
    
    % List output files
    fprintf('\nOutput files:\n');
    
    if strcmp(arch, 'amd64') || strcmp(arch, 'all')
        fprintf('\n  x86_64 (amd64):\n');
        check_and_print_file(fullfile(installation_path, 'common', 'bin.zip'));
        check_and_print_file(fullfile(installation_path, 'franka_robot_server', 'bin.tar.gz'));
        check_and_print_file(fullfile(installation_path, 'dependencies', 'libfranka.zip'));
    end
    
    if strcmp(arch, 'arm64') || strcmp(arch, 'all')
        fprintf('\n  ARM64 (arm64):\n');
        check_and_print_file(fullfile(installation_path, 'common', 'bin_arm.zip'));
        check_and_print_file(fullfile(installation_path, 'franka_robot_server', 'bin_arm.tar.gz'));
        check_and_print_file(fullfile(installation_path, 'dependencies', 'libfranka_arm.zip'));
    end
    
    fprintf('\n');
end

function check_and_print_file(filepath)
    if isfile(filepath)
        info = dir(filepath);
        fprintf('    ✓ %s (%.1f KB)\n', filepath, info.bytes/1024);
    else
        fprintf('    ✗ %s (not found)\n', filepath);
    end
end


