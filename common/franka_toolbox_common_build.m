function franka_toolbox_common_build(user,ip,port)

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
        [status, cmdout] = franka_toolbox_system_cmd('mkdir build',fullfile(installation_path, 'common'));
        if status ~= 0
            error('Failed to create build directory: %s', cmdout);
        end
        
        % Build common library for the Franka Toolbox for MATLAB
        fprintf('Configuring CMake build...\n');
        [status, cmdout] = franka_toolbox_system_cmd(['cmake -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=',fullfile(installation_path,'libfranka', 'build'),' ..'],buildDir);
        if status ~= 0
            error('CMake configuration failed: %s', cmdout);
        end
        
        fprintf('Building library...\n');
        [status, cmdout] = franka_toolbox_system_cmd('cmake --build .',buildDir);
        if status ~= 0
            error('CMake build failed: %s', cmdout);
        end
    
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
        % Check if remote directory exists before removing
        [~, cmdout] = franka_toolbox_remote_system_cmd('ls -d ~/franka_matlab 2>/dev/null || echo "Directory does not exist"', '', user, ip, port,true);
        if ~contains(cmdout, 'Directory does not exist')
            franka_toolbox_remote_system_cmd('rm -rf ~/franka_matlab', '', user, ip, port,true);
        end
        franka_toolbox_remote_system_cmd('mkdir franka_matlab','~',user,ip,port,true);
        franka_toolbox_foder_remote_cp(fullfile(franka_toolbox_installation_path_get(),'common'),user,ip,'franka_matlab',port,true);
        franka_toolbox_foder_remote_cp(fullfile(franka_toolbox_installation_path_get(),'libfranka_arm'),user,ip,'franka_matlab',port,true);
        franka_toolbox_remote_system_cmd('mkdir build',fullfile('~','franka_matlab','common'),user,ip,port,true);
        franka_toolbox_remote_system_cmd([
            'cmake -DCMAKE_BUILD_TYPE=Release ' ...
            '-DFranka_DIR:PATH=', fullfile('~','franka_matlab','libfranka_arm','build'), ' ' ...
            '-DFRANKA_FOLDER:STRING=libfranka_arm ' ...
            '..'
        ], fullfile('~','franka_matlab','common','build'), user, ip, port, true);
        franka_toolbox_remote_system_cmd('cmake --build .',fullfile('~','franka_matlab', 'common', 'build'),user,ip,port,true);
        
        if ~isfolder(fullfile(installation_path,'common','bin_arm'))
            mkdir(fullfile(installation_path,'common','bin_arm'));
        end
        
        if isfile(fullfile(installation_path,'common','bin_arm.zip'))
            unzip(fullfile(installation_path,'common','bin_arm.zip'),fullfile(installation_path,'common'));
        end
    
        franka_toolbox_foder_from_remote_cp(fullfile('~','franka_matlab','common','build','libfranka_matlab.a'),fullfile(installation_path,'common','bin_arm','libfranka_matlab.a'),user,ip,port,true);
            
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
