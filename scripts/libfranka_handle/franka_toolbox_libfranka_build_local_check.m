function libfranka_version = franka_toolbox_libfranka_build_local_check()
    %  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package
    
    path = franka_toolbox_installation_path_get();
    
    libfranka_version = '';

    if isfile(fullfile(path,'libfranka','build', 'libfranka.so'))
        libfranka_found = dir(fullfile(path,'libfranka','build', 'libfranka.*'));
        libfranka_version = regexp([libfranka_found(:).name],'\d+\.\d+\.\d+','match');
        libfranka_version = libfranka_version{1};
    end

end