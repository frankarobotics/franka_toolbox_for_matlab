function appimage_path = franka_toolbox_linuxdeploy_get()
    %  Copyright (c) 2025 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package

    % Determine local temp destination inside toolbox installation path
    % (user-space, avoids noexec /tmp mounts)
    base_tmp = fullfile(franka_toolbox_installation_path_get(), 'tmp', 'linuxdeploy');
    if ~isfolder(base_tmp)
        mkdir(base_tmp);
    end

    % Detect host architecture and select appropriate AppImage name and URL
    is_unix_host = isunix();
    if ~is_unix_host
        error('linuxdeploy is only relevant on UNIX-like systems.');
    end

    linux_deploy_release_url = 'https://github.com/linuxdeploy/linuxdeploy/releases/download/1-alpha-20250213-2';

    % Download both x86_64 and aarch64 AppImages
    appimage_names = {'linuxdeploy-x86_64.AppImage', 'linuxdeploy-aarch64.AppImage'};
    appimage_path = cell(1, 2);
    
    for i = 1:2
        appimage_name = appimage_names{i};
        url = [linux_deploy_release_url, '/', appimage_name];
        appimage_path{i} = fullfile(base_tmp, appimage_name);
        
        % Download on-the-fly into tmp if missing
        if ~isfile(appimage_path{i})
            cmd = ['wget -q -O ', appimage_name, ' ', url, ' && chmod +x ', appimage_name];
            franka_toolbox_system_cmd(cmd, base_tmp);
        end
    end
end


