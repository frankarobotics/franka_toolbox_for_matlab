function wontdo = franka_toolbox_libfranka_build(libfranka_version,force_install,clone_only,folder_name)
    %  Copyright (c) 2025 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package
    
    wontdo = true;
    if nargin == 0
        libfranka_ver = readcell('libfranka_ver.csv');
        libfranka_version = libfranka_ver{1};
        force_install = false;
        clone_only = false;
        folder_name = 'libfranka';
    end

    if nargin == 1
        force_install = false;
        clone_only = false;
        folder_name = 'libfranka';
    end
    
    if nargin == 2
        clone_only = false;
        folder_name = 'libfranka';
    end

    if nargin == 3
        folder_name = 'libfranka';
    end

    installation_path = franka_toolbox_installation_path_get();

    % Deploy the preinstalled libfranka first, just in case
    deps_path = fullfile(installation_path, 'dependencies');
    libfranka_zip = fullfile(deps_path, 'libfranka.zip');
    if exist(libfranka_zip, 'file')
        unzip(libfranka_zip, installation_path);
    else
        force_install = true;
    end
    
    libfranka_version_local = franka_toolbox_libfranka_build_local_check();
    
    if ~isempty(libfranka_version_local)
        % Proceed with installation if:
        % 1. force_install is true OR
        % 2. versions don't match
        if ~force_install && strcmp(libfranka_version_local, libfranka_version)
            return;
        else 
            wontdo = false;
            franka_toolbox_libfranka_local_uninstall();
        end
    end
    
    libfranka_path = fullfile(installation_path, folder_name);
    libfranka_build_path = fullfile(libfranka_path,'build');
    franka_toolbox_system_cmd(['git clone --recursive https://github.com/frankaemika/libfranka ',folder_name],installation_path,true);
    franka_toolbox_system_cmd(['git checkout ',libfranka_version],libfranka_path,true);
    franka_toolbox_system_cmd('git submodule update',libfranka_path,true);
    
    % Check if build directory exists and remove it
    if exist(libfranka_build_path, 'dir')
        rmdir(libfranka_build_path, 's');
    end
    franka_toolbox_system_cmd('mkdir build',libfranka_path);
    
    if ~clone_only
        franka_toolbox_system_cmd('cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DCMAKE_PREFIX_PATH="/opt/openrobots/lib/cmake" ..',libfranka_build_path,true);
        franka_toolbox_system_cmd('cmake --build .',libfranka_build_path,true);

        % libfranka runtime dependencies bundle
        franka_toolbox_libfranka_deps_bundle();

        % Pack libfranka
        franka_toolbox_libfranka_pack();
    end
    
end