function libfranka_system_installation = franka_toolbox_libfranka_system_installation_get()
    %  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package

    if ~ispref('franka_toolbox','libfranka_system_installation')
        addpref('franka_toolbox','libfranka_system_installation',false);
    end
    
    libfranka_system_installation = getpref('franka_toolbox').libfranka_system_installation;

end
