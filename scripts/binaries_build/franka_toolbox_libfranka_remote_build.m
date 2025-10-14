function wontdo = franka_toolbox_libfranka_remote_build(user,ip,port,libfranka_version,force_install)
    %  Copyright (c) 2025 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package

    wontdo = true;

    if nargin < 5
        force_install = 'false';
        if nargin < 4
            libfranka_ver = readcell('libfranka_ver.csv');
            libfranka_version = libfranka_ver{1};
        end
    end
    
    libfranka_version_remote = '';
    if ~force_install
        fprintf('Checking remote libfranka version...\n');
        libfranka_version_remote = franka_toolbox_libfranka_remote_build_check(user, ip, port, '~');
        fprintf('Remote libfranka version: %s\n', libfranka_version_remote);
    end

    libfranka_path = fullfile(franka_toolbox_installation_path_get(),'libfranka_arm');

    if isempty(libfranka_version_remote) || force_install || ~strcmp(libfranka_version_remote,libfranka_version)  
        wontdo = false;
        
        if isfolder(libfranka_path)
            fprintf('Removing existing libfranka_arm directory...\n');
            rmdir(libfranka_path,'s'); 
        end
        
        fprintf('Building libfranka locally...\n');
        franka_toolbox_libfranka_build(libfranka_version,true,true,'libfranka_arm');
        
        fprintf('Cleaning remote libfranka directory...\n');
        franka_toolbox_remote_system_cmd('rm -rf libfranka','~',user,ip,port,true);
        
        fprintf('Copying libfranka to remote machine...\n');
        franka_toolbox_foder_remote_cp(['"',libfranka_path,'"'],user,ip,'~/libfranka',port,true);
        
        fprintf('Running CMake configuration on remote machine...\n');
        franka_toolbox_remote_system_cmd('cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DCMAKE_PREFIX_PATH="/opt/openrobots/lib/cmake" ..','~/libfranka/build',user,ip,port,true);
        
        fprintf('Building libfranka on remote machine...\n');
        franka_toolbox_remote_system_cmd('cmake --build .','~/libfranka/build',user,ip,port,true);
        
        fprintf('Cleaning up local libfranka_arm directory...\n');
        rmdir(libfranka_path,'s');
    end

    fprintf('Copying built libfranka from remote machine...\n');
    franka_toolbox_foder_from_remote_cp(fullfile('~','libfranka'),['"',libfranka_path,'"'],user,ip,port,true);

    fprintf('Bundling libfranka runtime dependencies...\n');
    franka_toolbox_libfranka_deps_bundle(user,ip,port);

    fprintf('Packing libfranka...\n');
    franka_toolbox_libfranka_pack(true);
        
end
