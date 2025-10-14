function franka_robot_server_build(varargin)
    %  Copyright (c) 2025 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package

    % Script to build the Franka Robot Server

    % Parse optional remote parameters
    p = inputParser;
    addOptional(p, 'user', '', @ischar);
    addOptional(p, 'ip', '', @ischar);
    addOptional(p, 'port', '22', @ischar);
    addOptional(p, 'build_type', 'Release', @ischar);
    parse(p, varargin{:});
    
    is_remote = ~isempty(p.Results.user) && ~isempty(p.Results.ip);
    build_type = p.Results.build_type;

    installation_path = franka_toolbox_installation_path_get();
    server_path = fullfile(installation_path, 'franka_robot_server');

    if is_remote

        installation_path = franka_toolbox_installation_path_get();

        % Check if libfranka_arm exists, if not unzip from dependencies
        if ~isfolder(fullfile(installation_path, 'libfranka_arm'))
            libfranka_arm_zip = fullfile(installation_path, 'dependencies', 'libfranka_arm.zip');
            if isfile(libfranka_arm_zip)
                unzip(libfranka_arm_zip, installation_path);
            else
                error('libfranka_arm.zip not found in dependencies folder');
            end
        end

        unzip(fullfile(installation_path,'common','bin_arm.zip'), fullfile(installation_path,'common'));

        % Define remote installation path
        remote_installation_path = '~/franka_matlab';
        
        % Check if remote directory exists before removing
        [~, cmdout] = franka_toolbox_remote_system_cmd('ls -d ~/franka_matlab 2>/dev/null || echo "Directory does not exist"', '', p.Results.user, p.Results.ip, p.Results.port,true);
        if ~contains(cmdout, 'Directory does not exist')
            franka_toolbox_remote_system_cmd('rm -rf ~/franka_matlab', '', p.Results.user, p.Results.ip, p.Results.port,true);
        end
        franka_toolbox_remote_system_cmd(...
            ['mkdir -p ', remote_installation_path], ...
            '', ...  % No working directory needed for mkdir
            p.Results.user, ...
            p.Results.ip, ...
            p.Results.port,true);

        % Copy only the necessary folders to remote machine
        folders_to_copy = {'common', 'franka_robot_server','libfranka_arm'};
        
        % Remove build-related folders from franka_robot_server if they exist
        folders_to_remove = {'build'};
        for i = 1:length(folders_to_remove)
            folder_path = fullfile(installation_path, 'franka_robot_server', folders_to_remove{i});
            if exist(folder_path, 'dir')
                rmdir(folder_path, 's');
            end
        end
        
        for i = 1:length(folders_to_copy)
            source_path = fullfile(installation_path, folders_to_copy{i});
            remote_path = [remote_installation_path, '/', folders_to_copy{i}];
            franka_toolbox_foder_remote_cp(source_path, p.Results.user, p.Results.ip, remote_path, p.Results.port,true);
        end
        
        % Execute remote build
        remote_franka_dir = fullfile(remote_installation_path,'libfranka_arm');
        remote_build_path = [remote_installation_path, '/franka_robot_server'];
        fprintf('Starting remote build process...\n');
        
        % Create and navigate to build directory
        franka_toolbox_remote_system_cmd('mkdir -p build', remote_build_path, p.Results.user, p.Results.ip, p.Results.port, true);
        
        % Configure CMake
        cmake_cmd = sprintf('cmake -DCMAKE_BUILD_TYPE=%s -DFranka_DIR="%s" -DFRANKA_FOLDER="libfranka_arm" -DBIN_FOLDER="bin_arm" ..', ...
            build_type, remote_franka_dir);
        franka_toolbox_remote_system_cmd(cmake_cmd, fullfile(remote_build_path, 'build'), ...
            p.Results.user, p.Results.ip, p.Results.port, true);
        
        % Build the project
        franka_toolbox_remote_system_cmd('cmake --build . --config Release -j$(nproc)', ...
            fullfile(remote_build_path, 'build'), p.Results.user, p.Results.ip, p.Results.port, true);

        % Add executable permissions to the built server
        franka_toolbox_remote_system_cmd('chmod +x build/franka_robot_server', ...
            remote_build_path, p.Results.user, p.Results.ip, p.Results.port, true);

        if ~isfolder(fullfile(installation_path,'franka_robot_server','bin_arm'))
            mkdir(fullfile(installation_path,'franka_robot_server','bin_arm'));
        end

        if isfile(fullfile(installation_path,'franka_robot_server','bin_arm.zip'))
            unzip(fullfile(installation_path,'franka_robot_server','bin_arm.zip'),fullfile(installation_path,'franka_robot_server'));
        end
    
        franka_toolbox_foder_from_remote_cp(fullfile('~','franka_matlab','franka_robot_server','build','franka_robot_server'),fullfile(installation_path,'franka_robot_server','bin_arm'),p.Results.user, p.Results.ip, p.Results.port,true);
    
        % Replace system tar with MATLAB tar for remote build
        tar(fullfile(installation_path,'franka_robot_server','bin_arm.tar.gz'), ...
            fullfile(installation_path,'franka_robot_server','bin_arm'));

        rmdir(fullfile(installation_path,'franka_robot_server','bin_arm'),'s');
        rmdir(fullfile(installation_path,'common','bin_arm'),'s');
    else

        installation_path = franka_toolbox_installation_path_get();
    
        % Check if libfranka exists, if not unzip from dependencies
        if ~isfolder(fullfile(installation_path, 'libfranka'))
            libfranka_zip = fullfile(installation_path, 'dependencies', 'libfranka.zip');
            if isfile(libfranka_zip)
                unzip(libfranka_zip, installation_path);
            else
                error('libfranka.zip not found in dependencies folder');
            end
        end

        unzip(fullfile(installation_path,'common','bin.zip'), fullfile(installation_path,'common'));

        % Get the Franka directory path
        frankaDir = fullfile(installation_path,'libfranka','build');

        % Create and navigate to build directory
        build_dir = fullfile(server_path, 'build');
        if ~exist(build_dir, 'dir')
            mkdir(build_dir);
        end

        % Configure CMake
        fprintf('Configuring CMake...\n');
        cmake_cmd = sprintf('cmake -DCMAKE_BUILD_TYPE=%s -DFranka_DIR="%s" -DFRANKA_FOLDER="libfranka" -DBIN_FOLDER="bin" ..', ...
            build_type, frankaDir);
        [status, cmdout] = franka_toolbox_system_cmd(cmake_cmd, build_dir);
        
        if status ~= 0
            error('CMake configuration failed:\n%s', cmdout);
        end

        % Build the project
        fprintf('Building project...\n');
        [status, cmdout] = franka_toolbox_system_cmd('cmake --build . --config Release', build_dir);
        
        if status ~= 0
            error('Build failed:\n%s', cmdout);
        end

        % Add executable permissions to the built server
        [status, cmdout] = franka_toolbox_system_cmd('chmod +x franka_robot_server', build_dir);
        if status ~= 0
            error('Failed to set executable permissions:\n%s', cmdout);
        end

        % Pack
        if ~isfolder(fullfile(installation_path,'franka_robot_server','bin'))
            mkdir(fullfile(installation_path,'franka_robot_server','bin'));
        end
    
        if isfile(fullfile(installation_path,'franka_robot_server','bin.zip'))
            unzip(fullfile(installation_path,'franka_robot_server','bin.zip'),fullfile(installation_path,'franka_robot_server'));
        end
    
        copyfile(fullfile(installation_path, 'franka_robot_server','build','franka_robot_server'),fullfile(installation_path,'franka_robot_server','bin')); 
        
        % Replace system tar with MATLAB tar for local build
        tar(fullfile(installation_path,'franka_robot_server','bin.tar.gz'), ...
            fullfile(installation_path,'franka_robot_server','bin'));

        rmdir(fullfile(installation_path,'franka_robot_server','bin'),'s');
        rmdir(fullfile(installation_path,'franka_robot_server','build'),'s');
        rmdir(fullfile(installation_path,'common','bin'),'s');
    end

    disp('=== Build completed successfully ===');

end