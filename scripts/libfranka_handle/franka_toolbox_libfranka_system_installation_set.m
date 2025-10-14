function franka_toolbox_libfranka_system_installation_set(flag)
    %  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package
    
    if nargin == 0
        flag = true;
    end

    if ~ispref('franka_toolbox','libfranka_system_installation')
        addpref('franka_toolbox','libfranka_system_installation',flag);
    else
        setpref('franka_toolbox','libfranka_system_installation',flag);
    end
end
