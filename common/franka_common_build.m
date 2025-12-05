function franka_common_build(user,ip,port)

    if nargin == 0

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
        
        buildDir = fullfile(installation_path, 'common', 'build');
        
        % Prep the build folder
        fprintf('Preparing build directory...\n');
        if isfolder(buildDir)
            fprintf('Removing existing build directory...\n');
            rmdir(buildDir,'s');
        end
        fprintf('Creating new build directory...\n');
        opts = struct('nothrow', false);
        franka_toolbox_local_exec('mkdir build', fullfile(installation_path, 'common'), opts);
        
        % Build common library for the Franka Toolbox for MATLAB
        fprintf('Configuring CMake build...\n');
        cmake_cmd = ['cmake -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=', ...
            fullfile(installation_path,'libfranka', 'build'), ' ..'];
        franka_toolbox_local_exec(cmake_cmd, buildDir, opts);
        
        fprintf('Building library...\n');
        franka_toolbox_local_exec('cmake --build .', buildDir, opts);
    
        % common
        if ~isfolder(fullfile(installation_path,'common','bin'))
    	    mkdir(fullfile(installation_path,'common','bin'));
        end
        
        if isfile(fullfile(installation_path,'common','bin.zip'))
            unzip(fullfile(installation_path,'common','bin.zip'),fullfile(installation_path,'common'));
        end
    
        copyfile(fullfile(installation_path, 'common','build','libfranka_matlab.a'),fullfile(installation_path,'common','bin'));
        
        zip(fullfile(installation_path,'common','bin.zip'),fullfile(installation_path,'common','bin'));
    
        % clean up
        if isfolder(fullfile(installation_path, 'common', 'build'))
            rmdir(fullfile(installation_path, 'common', 'build'), 's');
        end
        if isfolder(fullfile(installation_path, 'common', 'bin'))
            rmdir(fullfile(installation_path, 'common', 'bin'), 's');
        end

    elseif nargin == 3
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

        % common
        fprintf('Preparing remote build...\n');
        sshOpts = struct('verbose', true, 'nothrow', false);
        scpOpts = struct('recursive', true, 'nothrow', false);
        
        % Check if remote directory exists before removing
        [status, ~] = franka_toolbox_ssh_exec('ls -d ~/franka_matlab 2>/dev/null', user, ip, port);
        if status == 0
            franka_toolbox_ssh_exec('rm -rf ~/franka_matlab', user, ip, port, sshOpts);
        end
        franka_toolbox_ssh_exec('mkdir franka_matlab', user, ip, port, sshOpts);
        
        % Copy folders to remote
        franka_toolbox_scp(fullfile(franka_toolbox_installation_path_get(),'common'), ...
            ':franka_matlab/', user, ip, port, scpOpts);
        franka_toolbox_scp(fullfile(franka_toolbox_installation_path_get(),'libfranka_arm'), ...
            ':franka_matlab/', user, ip, port, scpOpts);
        
        % Build on remote
        franka_toolbox_ssh_exec('mkdir build', user, ip, port, sshOpts);
        
        cmake_cmd = ['cd ~/franka_matlab/common/build && cmake -DCMAKE_BUILD_TYPE=Release ' ...
            '-DFranka_DIR:PATH=~/franka_matlab/libfranka_arm/build ' ...
            '-DFRANKA_FOLDER:STRING=libfranka_arm ..'];
        franka_toolbox_ssh_exec(cmake_cmd, user, ip, port, sshOpts);
        
        franka_toolbox_ssh_exec('cd ~/franka_matlab/common/build && cmake --build .', user, ip, port, sshOpts);
        
        if ~isfolder(fullfile(installation_path,'common','bin_arm'))
            mkdir(fullfile(installation_path,'common','bin_arm'));
        end
        
        if isfile(fullfile(installation_path,'common','bin_arm.zip'))
            unzip(fullfile(installation_path,'common','bin_arm.zip'),fullfile(installation_path,'common'));
        end
    
        % Copy built library from remote
        franka_toolbox_scp(':~/franka_matlab/common/build/libfranka_matlab.a', ...
            fullfile(installation_path,'common','bin_arm','libfranka_matlab.a'), ...
            user, ip, port, scpOpts);
            
        zip(fullfile(installation_path,'common','bin_arm.zip'),fullfile(installation_path,'common','bin_arm'));
        
        % Clean up
        if isfolder(fullfile(installation_path, 'common', 'build'))
            rmdir(fullfile(installation_path, 'common', 'build'), 's');
        end
        if isfolder(fullfile(installation_path, 'common', 'bin'))
            rmdir(fullfile(installation_path, 'common', 'bin'), 's');
        end
    end

end
