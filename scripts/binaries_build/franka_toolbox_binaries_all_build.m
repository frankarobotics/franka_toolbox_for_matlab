function franka_toolbox_binaries_all_build(user,ip,port)
    %  Copyright (c) 2025 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package

    % This needs to be run x1 by Ubuntu Host PC 
    % with Jetson connected and user,ip, port provided
    % and 1x with Windows Host PC (no Jetson connected)
    % run franka_toolbox_dist_make(); when done for packaging the
    % distribution

    % Build Simulink & MATLAB libs
    franka_toolbox_simulink_library_mex();
    franka_robot_mex();
    
    % Additionally if in linux build targets & dependencies

    if isunix()
        franka_toolbox_binaries_target_local_build();

        if nargin == 3
            franka_toolbox_binaries_target_remote_build(user,ip,port);
        end
        
    end
    
end