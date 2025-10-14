function franka_toolbox_libfranka_deps_bundle(user,ip,port)
    %  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package

    if nargin == 2
        port = '22';
    end

    libfranka_ver = readcell('libfranka_ver.csv');
    libfranka_ver = split(libfranka_ver{1}, '.');
    libfranka = ['libfranka.so.',strjoin(libfranka_ver(1:2),'.')];

    linuxdeploy_appimage = franka_toolbox_linuxdeploy_get();

    if nargin == 0
        if isunix()
            
            libfranka_path = fullfile(franka_toolbox_installation_path_get(),'libfranka','build');
            
            cmd = [linuxdeploy_appimage{1},...
                ' --appdir=',libfranka_path,...
                ' --executable=',fullfile(libfranka_path,'libfranka.so'),...
                ' --library=',fullfile(libfranka_path,libfranka)];

            franka_toolbox_system_cmd(cmd,'.',true);
        end
    else
        % Check if linuxdeploy has been extracted on remote system
        % If not, install it first
        [~, cmdout] = franka_toolbox_remote_system_cmd('ls -d ~/franka-dev-tools/squashfs-root 2>/dev/null || echo "Directory does not exist"', '', user, ip, port);
        if contains(cmdout, 'Directory does not exist')
            % linuxdeploy not extracted, install it
            franka_toolbox_linuxdeploy_install(user, ip, port);
        end
        
        libfranka_remote_path = '\$HOME/libfranka/build';
        cmd = ['./AppRun',...
            ' --appdir=',libfranka_remote_path,...
            ' --executable=',fullfile(libfranka_remote_path,'libfranka.so'),...
            ' --library=',fullfile(libfranka_remote_path,libfranka)];

        franka_toolbox_remote_system_cmd(cmd,'~/franka-dev-tools/squashfs-root',user,ip,port,true);
        franka_toolbox_foder_from_remote_cp(fullfile(libfranka_remote_path,'usr'),fullfile(franka_toolbox_installation_path_get(),'libfranka_arm','build'),user,ip,port,true);
    end

end