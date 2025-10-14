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

        franka_toolbox_remote_system_cmd(['rm -rf ','~/franka-dev-tools',' && mkdir -p ','~/franka-dev-tools'],'~',user,ip,port);
        franka_toolbox_foder_remote_cp(aarch64_path,user,ip,'~/franka-dev-tools/linuxdeploy-aarch64.AppImage',port);
        franka_toolbox_remote_system_cmd('./linuxdeploy-aarch64.AppImage --appimage-extract','~/franka-dev-tools',user,ip,port);

    end

end