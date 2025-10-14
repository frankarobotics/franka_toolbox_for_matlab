function franka_toolbox_libfranka_local_uninstall()
    %  Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package
    
    rmdir(fullfile(franka_toolbox_installation_path_get(),'libfranka'),'s');
    
end