function franka_toolbox_linuxdeploy_install(user,ip,port)
    %  Copyright (c) 2025 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package

    if nargin == 2
        port = '22';
    end

    if isunix()

        linuxdeploy_appimage = franka_toolbox_linuxdeploy_get();
        aarch64_path = linuxdeploy_appimage{2};

        sshOpts = struct('nothrow', false);
        franka_toolbox_ssh_exec('rm -rf ~/franka-dev-tools && mkdir -p ~/franka-dev-tools', user, ip, port, sshOpts);
        
        scpOpts = struct('nothrow', false);
        franka_toolbox_scp(aarch64_path, ':~/franka-dev-tools/linuxdeploy-aarch64.AppImage', user, ip, port, scpOpts);
        
        franka_toolbox_ssh_exec('cd ~/franka-dev-tools && ./linuxdeploy-aarch64.AppImage --appimage-extract', user, ip, port, sshOpts);

    end

end
