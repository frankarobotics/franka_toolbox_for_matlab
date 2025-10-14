function libfranka_version = franka_toolbox_libfranka_system_installation_check(user,ip,port)
    %  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package
    libfranka_version = '';
    remote = true;

    if nargin < 1
        remote = false;
    end

    if ~remote && ispc()
        error('Windows are not supported as target PC');
    end

    if nargin < 3
        port = '22';
    end
    
    libfranka_version = '';

    ld_search_cmd = 'ls /usr/lib';
    
    if ~remote
        [~, r] = system(ld_search_cmd);
    else 
        [~, r] = franka_toolbox_remote_system_cmd(ld_search_cmd,'~',user,ip,port);
    end

    if ~isempty(r)
        libfranka_version = regexp(strtrim(r),'libfranka.so.(\d*\.\d*\.\d*)','tokens');
        if ~isempty(libfranka_version)
            libfranka_version = libfranka_version{1}{1};
        end
    end
    
end